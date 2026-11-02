/* Copyright 2026, Robotec.ai sp. z o.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "GaussianSplatFeatureProcessor.h"
#include "GaussianSplatProjectCullGroupPass.h"
#include "GaussianSplatTileGrid.h"
#include <Atom/RPI.Public/Buffer/BufferSystemInterface.h>
#include <Atom/RPI.Public/Pass/ParentPass.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Reflect/Asset/AssetUtils.h>
#include <Atom/RPI.Reflect/Pass/PassRequest.h>
#include <Atom/RPI.Reflect/System/AnyAsset.h>
#include <AzCore/Math/Vector2.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/std/algorithm.h>
#include <AzCore/std/limits.h>
#include <AzCore/std/string/string_view.h>
#include <Utils/GaussianSplatPacking.h>
#include <cstdint>

namespace GaussianSplat
{
    using namespace AZStd::string_view_literals;

    namespace
    {
        //! Match AZSL's NumDigits (GaussianSplatCommon.azsli): one look-back status bin per digit value, and
        //! one thread per digit value in a radix-sort group.
        constexpr uint32_t NumDigits = 256;
        //! Match AZSL's RadixKeysPerThread (GaussianSplatCommon.azsli): entries each thread of such a group owns.
        constexpr uint32_t RadixKeysPerThread = 16;
        //! Entries per block for the radix-sort GlobalHistogram/Onesweep passes. Must match
        //! GaussianSplatCommon.azsli's RadixBlockSize, since this is what sizes the one look-back status row
        //! per block that those shaders index. Derived from the two constants above exactly as AZSL derives
        //! it, so the two halves cannot drift apart the way a restated literal can.
        constexpr uint32_t RadixBlockSize = NumDigits * RadixKeysPerThread;
        //! A dense ~300k-splat cloud has been observed needing ~2.1M entries in a frame; this default gives
        //! headroom while keeping each ping-pong entry buffer to ~192MB.
        constexpr uint64_t DefaultEntryCapacity = static_cast<uint64_t>(16) * 1024 * 1024;
        //! Largest entry count GaussianSplatRadixOnesweep.azsl's look-back can carry: its status words keep the
        //! running prefix in their low 30 bits, and the prefix reaches the entry count.
        constexpr uint64_t MaxEntryCapacity = (static_cast<uint64_t>(1) << 30) - 1;
        constexpr auto RadixEntryCapacityRegistryKey = "/Robotec/GaussianSplat/RadixEntryCapacity"_sv;

        //! Accumulated coverage at which a pixel's splat depth is taken to be settled. Small enough to mean
        //! "the nearest splat that contributes at all" while still stepping over sub-2% dust in front of a
        //! surface; see GaussianSplatTileShading.azsl's m_depthCoverageThreshold.
        constexpr float DefaultDepthCoverageThreshold = 0.02f;
        constexpr auto DepthCoverageThresholdRegistryKey = "/Robotec/GaussianSplat/DepthCoverageThreshold"_sv;

        //! Creates buffer at byteCount, or resizes it if it already exists at a different size. Warns and
        //! returns false if it could not be created, in which case the caller must not record it as allocated
        //! or uploaded. byteCount is 64-bit because the radix entry buffers are the largest allocations in the
        //! gem - hundreds of MB at the default capacity - and their size is a product the caller computes.
        bool EnsureBuffer(
            AZ::Data::Instance<AZ::RPI::Buffer>& buffer,
            const char* bufferName,
            AZ::RPI::CommonBufferPoolType poolType,
            uint32_t elementSize,
            AZ::u64 byteCount)
        {
            if (buffer == nullptr)
            {
                AZ::RPI::CommonBufferDescriptor desc;
                desc.m_poolType = poolType;
                desc.m_bufferName = bufferName;
                desc.m_elementSize = elementSize;
                desc.m_byteCount = byteCount;
                buffer = AZ::RPI::BufferSystemInterface::Get()->CreateBufferFromCommonPool(desc);
                if (buffer == nullptr)
                {
                    AZ_Warning(
                        "GaussianSplatFeatureProcessor",
                        false,
                        "Could not allocate '%s' (%llu bytes).",
                        bufferName,
                        static_cast<unsigned long long>(byteCount));
                    return false;
                }
            }
            else if (buffer->GetBufferSize() != byteCount)
            {
                buffer->Resize(byteCount);
            }
            return true;
        }

        //! The frame size a pipeline's tile-indexed buffers are sized from: chainFrameSize - what the chain
        //! resolved for itself once it has run - and the pipeline's own root output before that.
        [[nodiscard]] AZ::Vector2 FrameSizeForBuffers(AZ::RPI::RenderPipeline* pipeline, const AZ::Vector2& chainFrameSize)
        {
            if (!chainFrameSize.IsZero())
            {
                return chainFrameSize;
            }
            return TileGrid::TryGetPipelineFrameSize(pipeline).value_or(AZ::Vector2(1920.0f, 1080.0f));
        }

        //! Uploads asset's splats, packed by Packing::PackCloudGeometry, into buffer.
        void UploadCloudGeometry(AZ::Data::Instance<AZ::RPI::Buffer>& buffer, const GaussianSplatAsset& asset)
        {
            const AZStd::vector<float> packed = Packing::PackCloudGeometry(asset.m_data);

            const auto byteCount = static_cast<uint32_t>(packed.size() * sizeof(float));
            if (!EnsureBuffer(
                    buffer,
                    "GaussianSplatFeatureProcessor, SourceGeometry",
                    AZ::RPI::CommonBufferPoolType::ReadOnly,
                    Packing::GeometryElementSize,
                    byteCount))
            {
                return;
            }
            buffer->UpdateData(packed.data(), byteCount);
        }

        //! Uploads asset's SH coefficients, packed by Packing::PackCloudSh at uploadDegree, into buffer.
        void UploadCloudSh(AZ::Data::Instance<AZ::RPI::Buffer>& buffer, const GaussianSplatAsset& asset, uint32_t uploadDegree)
        {
            const AZStd::vector<uint32_t> packed = Packing::PackCloudSh(asset, uploadDegree);

            const auto byteCount = static_cast<uint32_t>(packed.size() * sizeof(uint32_t));
            if (!EnsureBuffer(
                    buffer,
                    "GaussianSplatFeatureProcessor, SourceSh",
                    AZ::RPI::CommonBufferPoolType::ReadOnly,
                    sizeof(uint32_t),
                    byteCount))
            {
                return;
            }
            buffer->UpdateData(packed.data(), byteCount);
        }

    } // namespace

    void GaussianSplatFeatureProcessor::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GaussianSplatFeatureProcessor, FeatureProcessor>();
        }
    }

    GaussianSplatFeatureProcessor* GaussianSplatFeatureProcessor::GetInstance(AZ::RPI::RenderPipeline* pipeline)
    {
        if (pipeline == nullptr)
        {
            return nullptr;
        }
        auto* scene = pipeline->GetScene();
        if (scene == nullptr)
        {
            return nullptr;
        }
        return scene->GetFeatureProcessor<GaussianSplatFeatureProcessor>();
    }

    void GaussianSplatFeatureProcessor::Activate()
    {
        EnableSceneNotification();

        // Every setting is read once, here, from the Settings Registry (see
        // Registry/gaussiansplat_settings.setreg); an absent key leaves the default in place.
        uint64_t entryCapacity = DefaultEntryCapacity;
        m_depthCoverageThreshold = DefaultDepthCoverageThreshold;
        if (auto* registry = AZ::SettingsRegistry::Get())
        {
            AZ::u64 registryCapacity = 0;
            if (registry->Get(registryCapacity, RadixEntryCapacityRegistryKey))
            {
                entryCapacity = static_cast<uint64_t>(registryCapacity);
            }

            double coverageThreshold = 0.0;
            if (registry->Get(coverageThreshold, DepthCoverageThresholdRegistryKey))
            {
                m_depthCoverageThreshold = static_cast<float>(coverageThreshold);
            }
        }

        if (entryCapacity == 0)
        {
            AZ_Warning(
                "GaussianSplatFeatureProcessor",
                false,
                "%.*s is 0, which disables sorting; falling back to the default of %llu.",
                AZ_STRING_ARG(RadixEntryCapacityRegistryKey),
                static_cast<unsigned long long>(DefaultEntryCapacity));
            entryCapacity = DefaultEntryCapacity;
        }
        else if (entryCapacity > MaxEntryCapacity)
        {
            AZ_Warning(
                "GaussianSplatFeatureProcessor",
                false,
                "%.*s is %llu, more than the radix sort's look-back can count; clamping to %llu.",
                AZ_STRING_ARG(RadixEntryCapacityRegistryKey),
                static_cast<unsigned long long>(entryCapacity),
                static_cast<unsigned long long>(MaxEntryCapacity));
            entryCapacity = MaxEntryCapacity;
        }

        m_entryCapacity = static_cast<uint32_t>(entryCapacity);
        m_blockCapacity = static_cast<uint32_t>((entryCapacity + RadixBlockSize - 1) / RadixBlockSize);

        if (!(m_depthCoverageThreshold > 0.0f) || m_depthCoverageThreshold > 1.0f)
        {
            AZ_Warning(
                "GaussianSplatFeatureProcessor",
                false,
                "%.*s is %f, outside (0, 1]; falling back to the default of %f.",
                AZ_STRING_ARG(DepthCoverageThresholdRegistryKey),
                m_depthCoverageThreshold,
                DefaultDepthCoverageThreshold);
            m_depthCoverageThreshold = DefaultDepthCoverageThreshold;
        }

        AZ_Info("GaussianSplatFeatureProcessor", "Radix-sort entry capacity: %u\n", m_entryCapacity);
    }

    void GaussianSplatFeatureProcessor::Deactivate()
    {
        DisableSceneNotification();
        for (auto& [_, data] : m_pipelineData)
        {
            RemoveSharedPasses(data);
        }
        m_pipelineData.clear();
        AZ::Data::AssetBus::MultiHandler::BusDisconnect();
        m_gaussianSplatAssets.clear();
    }

    AZStd::vector<GaussianSplatFeatureProcessorInterface::GaussianSplatHandle> GaussianSplatFeatureProcessor::GetCloudHandles() const
    {
        AZStd::vector<GaussianSplatHandle> handles;
        handles.reserve(m_splatCloudData.size());
        for (const auto& [handle, _] : m_splatCloudData)
        {
            handles.push_back(handle);
        }
        return handles;
    }

    const GaussianSplatFeatureProcessor::CloudRenderData* GaussianSplatFeatureProcessor::FindRenderableCloud(
        GaussianSplatHandle handle) const
    {
        const auto it = m_splatCloudData.find(handle);
        if (it == m_splatCloudData.end() || !it->second.m_visible)
        {
            return nullptr;
        }
        const auto& renderData = it->second.m_renderData;
        if (renderData.m_sourceGeometry == nullptr || renderData.m_sourceSh == nullptr)
        {
            return nullptr;
        }

        return &renderData;
    }

    bool GaussianSplatFeatureProcessor::IsCloudVisible(GaussianSplatHandle handle) const
    {
        const auto it = m_splatCloudData.find(handle);
        return it != m_splatCloudData.end() && it->second.m_visible;
    }

    bool GaussianSplatFeatureProcessor::IsCloudRenderable(GaussianSplatHandle handle) const
    {
        return FindRenderableCloud(handle) != nullptr;
    }

    AZStd::optional<GaussianSplatFeatureProcessor::CloudRenderData> GaussianSplatFeatureProcessor::GetCloudRenderData(
        GaussianSplatHandle handle) const
    {
        if (const CloudRenderData* renderData = FindRenderableCloud(handle))
        {
            return *renderData;
        }

        return AZStd::nullopt;
    }

    void GaussianSplatFeatureProcessor::InjectSharedPasses(AZ::RPI::RenderPipeline* pipeline, PipelineData& data)
    {
        if (data.m_parentPass || m_splatCloudData.empty())
        {
            return;
        }

        // The chain goes in only once every buffer it binds exists. A pass that cannot attach one leaves its
        // slot empty, which makes AZ::RPI::PassValidationResults::IsValid() reject the pipeline's entire pass
        // tree and AZ::RPI::RenderPipeline::UpdatePasses discard it - taking the rest of the pipeline's
        // rendering down with the splats.
        if (!EnsurePipelineBuffers(pipeline, data))
        {
            return;
        }

        auto parentRequestAsset = AZ::RPI::AssetUtils::LoadAssetByProductPath<AZ::RPI::AnyAsset>(
            "Passes/GaussianSplatParentRequest.azasset", AZ::RPI::AssetUtils::TraceLevel::Warning);
        if (!parentRequestAsset.IsReady())
        {
            // Retried automatically - InjectSharedPasses is called every Simulate() until it succeeds.
            return;
        }

        const auto* parentTemplateRequest = parentRequestAsset->GetDataAs<AZ::RPI::PassRequest>();
        if (parentTemplateRequest == nullptr)
        {
            AZ_Error("GaussianSplatFeatureProcessor", false, "Failed to load GaussianSplat parent pass request asset.");
            return;
        }

        // Each pipeline gets its own separate pass tree, so the template-authored pass name is used as-is
        // with no per-pipeline uniquing needed.
        AZ::RPI::PassRequest parentRequest = *parentTemplateRequest;

        AZ::RPI::Ptr<AZ::RPI::Pass> parentPass = AZ::RPI::PassSystemInterface::Get()->CreatePassFromRequest(&parentRequest);
        if (!parentPass)
        {
            AZ_Error("GaussianSplatFeatureProcessor", false, "Failed to create GaussianSplat parent pass.");
            return;
        }
        if (!pipeline->AddPassAfter(parentPass, AZ::Name("PostProcessPass")))
        {
            AZ_Warning("GaussianSplatFeatureProcessor", false, "Failed to add GaussianSplat parent pass to the render pipeline.");
            return;
        }

        // Storing the parent is what marks the chain as injected: every child pass derives its own
        // configuration and per-frame state directly, so nothing needs to be pushed into any specific child
        // here.
        data.m_parentPass = parentPass;
        // A freshly created chain's GaussianSplatProjectCullGroupPass builds its per-cloud children from the
        // current cloud set, so there is nothing pending for SyncCloudPasses to push into it.
        data.m_cloudPassesDirty = false;
    }

    const GaussianSplatFeatureProcessor::SplatBuffers* GaussianSplatFeatureProcessor::GetPipelineBuffers(
        const AZ::RPI::RenderPipeline* pipeline) const
    {
        auto it = m_pipelineData.find(const_cast<AZ::RPI::RenderPipeline*>(pipeline));
        return it != m_pipelineData.end() && it->second.m_buffers.m_entriesA ? &it->second.m_buffers : nullptr;
    }

    bool GaussianSplatFeatureProcessor::ArePipelineBuffersStale(AZ::RPI::RenderPipeline* pipeline, const PipelineData& data) const
    {
        if (!data.m_buffers.m_entriesA)
        {
            return true;
        }
        // Compared by tile grid rather than pixel size: m_tileRanges is the only buffer the frame size reaches,
        // so a resize that keeps the grid leaves the set valid.
        const AZ::Vector2 frameSize = FrameSizeForBuffers(pipeline, data.m_chainFrameSize);
        const SplatBuffers& buffers = data.m_buffers;
        return buffers.m_entryCapacity != GetRadixEntryCapacity() || buffers.m_splatCapacity != GetSharedSplatCapacity() ||
            buffers.m_tileCountX != TileGrid::TileCountX(frameSize) || buffers.m_tileCountY != TileGrid::TileCountY(frameSize);
    }

    bool GaussianSplatFeatureProcessor::EnsurePipelineBuffers(AZ::RPI::RenderPipeline* pipeline, PipelineData& data)
    {
        if (!ArePipelineBuffersStale(pipeline, data))
        {
            return true;
        }

        // Released before the replacements are requested, so a resize asks the driver for one set rather than
        // two. The RHI still hands the memory back a few frames later, by which point one of the following
        // Simulate()s has retried the allocation.
        data.m_buffers = {};

        const uint32_t entryCapacity = GetRadixEntryCapacity();
        const uint32_t blockCapacity = AZStd::max(m_blockCapacity, 1u);
        const uint32_t splatCapacity = GetSharedSplatCapacity();
        const AZ::Vector2 frameSize = FrameSizeForBuffers(pipeline, data.m_chainFrameSize);
        const uint32_t tileCountX = TileGrid::TileCountX(frameSize);
        const uint32_t tileCountY = TileGrid::TileCountY(frameSize);
        const uint32_t tileCount = AZStd::max(tileCountX * tileCountY, 1u);

        // GpuRadixEntry: m_key (2 words), m_visibleSlot - see GaussianSplatCommon.azsli.
        constexpr uint32_t RadixEntrySize = sizeof(uint32_t) * 3;

        // Written as "EnsureBuffer(...) && allocated" rather than "allocated && EnsureBuffer(...)" so every
        // buffer is attempted even once one has failed: a shortfall then reports every allocation it affected
        // rather than just the first.
        bool allocated = true;
        using PoolType = AZ::RPI::CommonBufferPoolType;
        SplatBuffers& buffers = data.m_buffers;
        allocated = EnsureBuffer(
                        buffers.m_entriesA,
                        "GaussianSplat, EntriesA",
                        PoolType::ReadWrite,
                        RadixEntrySize,
                        AZ::u64{ RadixEntrySize } * entryCapacity) &&
            allocated;
        allocated = EnsureBuffer(
                        buffers.m_entriesB,
                        "GaussianSplat, EntriesB",
                        PoolType::ReadWrite,
                        RadixEntrySize,
                        AZ::u64{ RadixEntrySize } * entryCapacity) &&
            allocated;
        allocated = EnsureBuffer(
                        buffers.m_tileSplatIndices,
                        "GaussianSplat, TileSplatIndices",
                        PoolType::ReadWrite,
                        sizeof(uint32_t),
                        AZ::u64{ sizeof(uint32_t) } * entryCapacity) &&
            allocated;
        allocated = EnsureBuffer(
                        buffers.m_tileStatus,
                        "GaussianSplat, TileStatus",
                        PoolType::ReadWrite,
                        sizeof(uint32_t),
                        AZ::u64{ sizeof(uint32_t) } * blockCapacity * NumDigits) &&
            allocated;
        allocated =
            EnsureBuffer(buffers.m_tileCounter, "GaussianSplat, TileCounter", PoolType::ReadWrite, sizeof(uint32_t), sizeof(uint32_t)) &&
            allocated;
        // SplatProjection2D / GpuSplatProjection - see GaussianSplatCommon.azsli.
        allocated =
            EnsureBuffer(
                buffers.m_splatProjection, "GaussianSplat, SplatProjection", PoolType::ReadWrite, 40, AZ::u64{ 40 } * splatCapacity) &&
            allocated;
        // GpuTileRange: m_offset, m_count.
        allocated = EnsureBuffer(
                        buffers.m_tileRanges,
                        "GaussianSplat, TileRanges",
                        PoolType::ReadWrite,
                        sizeof(uint32_t) * 2,
                        AZ::u64{ sizeof(uint32_t) * 2 } * tileCount) &&
            allocated;
        // [entryCount, rawEntryCount] - see GaussianSplatRadixPrepareSortDispatch.azsl.
        allocated = EnsureBuffer(buffers.m_counts, "GaussianSplat, Counts", PoolType::ReadWrite, sizeof(uint32_t), sizeof(uint32_t) * 2) &&
            allocated;

        if (!allocated)
        {
            data.m_buffers = {};
            return false;
        }

        buffers.m_entryCapacity = entryCapacity;
        buffers.m_splatCapacity = splatCapacity;
        buffers.m_tileCountX = tileCountX;
        buffers.m_tileCountY = tileCountY;
        return true;
    }

    void GaussianSplatFeatureProcessor::RemoveSharedPasses(PipelineData& data)
    {
        if (data.m_parentPass != nullptr)
        {
            // Queuing the parent for removal also removes (and destroys) its children - see ParentPass::~ParentPass.
            data.m_parentPass->QueueForRemoval();
            data.m_parentPass = nullptr;
        }
        // m_chainFrameSize deliberately survives: it is the size the replacement chain must be built for, and
        // the pipeline's root output - the only other source - is what got it wrong if they ever diverged.
        // Likewise, the replacement group pass reads the cloud set directly when it first builds its children.
        data.m_cloudPassesDirty = false;
    }

    void GaussianSplatFeatureProcessor::SyncCloudPasses(PipelineData& data)
    {
        if (!data.m_cloudPassesDirty || !data.m_parentPass)
        {
            return;
        }
        auto* parentPass = azrtti_cast<AZ::RPI::ParentPass*>(data.m_parentPass.get());
        if (parentPass == nullptr)
        {
            return;
        }
        // Only reachable once the parent pass has been built at least once, so a still-unbuilt chain simply
        // leaves the flag set and is retried on a later Simulate().
        auto groupPass = parentPass->FindChildPass(AZ::Name("GaussianSplatProjectCullGroup"));
        if (auto* cullGroupPass = azrtti_cast<GaussianSplatProjectCullGroupPass*>(groupPass.get()))
        {
            cullGroupPass->QueueRebuildChildren();
            data.m_cloudPassesDirty = false;
        }
    }

    void GaussianSplatFeatureProcessor::MarkCloudPassesDirty()
    {
        for (auto& [_, data] : m_pipelineData)
        {
            data.m_cloudPassesDirty = true;
        }
    }

    void GaussianSplatFeatureProcessor::AddRenderPasses(AZ::RPI::RenderPipeline* pipeline)
    {
        auto& data = m_pipelineData[pipeline];

        // A pass hot-reload has AZ::RPI::RenderPipeline::UpdatePasses rebuild the pipeline's root pass from its
        // descriptor, which the injected chain is no part of - so the chain is left hanging off the discarded
        // root, kept alive only by m_parentPass. Treating that as uninjected is what gets it back into the
        // pipeline that AddRenderPasses is being called for.
        //
        // Dropping the reference is the whole removal here, rather than RemoveSharedPasses: the old root was
        // detached from this pipeline and queued for removal along with everything under it, so the orphan is
        // already on its way out. Queuing it again would route it through the pass system's own tree - its
        // pipeline pointer having been cleared with the rest of that branch - where AZ::RPI::ParentPass's
        // removal guard reads the state of a pipeline pass tree its parent no longer has, and reports the
        // removal as out of phase.
        if (data.m_parentPass && data.m_parentPass->GetRenderPipeline() != pipeline)
        {
            data.m_parentPass = nullptr;
            // As in RemoveSharedPasses: the replacement group pass reads the cloud set directly when it first
            // builds its children, so there is nothing left to sync into it.
            data.m_cloudPassesDirty = false;
        }

        InjectSharedPasses(pipeline, data);
        FeatureProcessor::AddRenderPasses(pipeline);
    }

    void GaussianSplatFeatureProcessor::OnRenderPipelineChanged(
        AZ::RPI::RenderPipeline* renderPipeline, AZ::RPI::SceneNotification::RenderPipelineChangeType changeType)
    {
        if (changeType != AZ::RPI::SceneNotification::RenderPipelineChangeType::Removed)
        {
            return;
        }
        if (auto it = m_pipelineData.find(renderPipeline); it != m_pipelineData.end())
        {
            RemoveSharedPasses(it->second);
            m_pipelineData.erase(it);
        }
    }

    GaussianSplatFeatureProcessorInterface::GaussianSplatHandle GaussianSplatFeatureProcessor::AcquireGaussianSplatFromAsset(
        AZ::Data::Asset<GaussianSplatAsset> gaussianSplatAsset)
    {
        const GaussianSplatHandle handle = ++m_currentGaussianSplatDataIndex;
        const AZ::Data::AssetId assetId = gaussianSplatAsset.GetId();
        auto& cloudData = m_splatCloudData[handle];
        cloudData.m_index = handle;
        cloudData.m_assetId = assetId;
        m_cloudSweepPending = true;
        MarkCloudPassesDirty();

        auto& assetData = m_gaussianSplatAssets[assetId];
        assetData.m_handles.emplace_back(handle);
        if (assetData.m_handles.size() == 1)
        {
            if (!(gaussianSplatAsset.IsLoading() || gaussianSplatAsset.IsReady()))
            {
                gaussianSplatAsset.QueueLoad();
            }
            assetData.m_asset = gaussianSplatAsset;
            // Connecting also delivers OnAssetReady for an asset that is already Ready (AZ::Data::AssetBus's
            // connection policy), and one still in ReadyPreNotify gets the queued notification once connected.
            // Either replaces m_asset with the asset manager's current data, which the caller's copy may not
            // be if the asset was reloaded since. A later cloud of the same asset just shares the entry.
            AZ::Data::AssetBus::MultiHandler::BusConnect(assetId);
        }

        for (auto& [pipeline, data] : m_pipelineData)
        {
            InjectSharedPasses(pipeline, data);
        }
        return handle;
    }

    void GaussianSplatFeatureProcessor::UpdateCloudSourceBuffers()
    {
        m_cloudSweepPending = false;

        AZ::u64 totalCount = 0;
        for (auto& [_, assetData] : m_gaussianSplatAssets)
        {
            const auto* asset = assetData.m_asset.GetAs<GaussianSplatAsset>();
            const bool loaded = asset != nullptr && asset->IsReady();
            const auto splatCount = loaded ? static_cast<uint32_t>(asset->m_data.size()) : 0u;
            totalCount += AZ::u64{ splatCount } * assetData.m_handles.size();

            // The SH degree each cloud renders at, in m_handles order; nullopt for a cloud that renders nothing.
            // Release keeps m_handles in step with m_splatCloudData, so every handle here names a cloud.
            AZStd::vector<AZStd::optional<uint32_t>> renderedShDegrees(assetData.m_handles.size());
            bool geometryNeeded = false;
            AZStd::array<bool, GaussianSplatAsset::MaxShDegree + 1> shDegreeNeeded{};
            for (size_t i = 0; i < assetData.m_handles.size(); ++i)
            {
                const auto& cloudData = m_splatCloudData.at(assetData.m_handles[i]);
                auto& degree = renderedShDegrees[i];
                if (loaded && cloudData.m_visible)
                {
                    // The loader rejects assets above MaxShDegree, so this indexes shDegreeNeeded in bounds.
                    degree = AZStd::min(asset->m_shDegree, cloudData.m_maxShDegreeOverride);
                    geometryNeeded = true;
                    shDegreeNeeded[*degree] = true;
                }
            }

            if (!geometryNeeded)
            {
                assetData.m_sourceGeometry = nullptr;
            }
            else if (assetData.m_sourceGeometry == nullptr)
            {
                UploadCloudGeometry(assetData.m_sourceGeometry, *asset);
            }
            for (uint32_t degree = 0; degree < shDegreeNeeded.size(); ++degree)
            {
                if (!shDegreeNeeded[degree])
                {
                    assetData.m_sourceSh[degree] = nullptr;
                }
                else if (assetData.m_sourceSh[degree] == nullptr)
                {
                    UploadCloudSh(assetData.m_sourceSh[degree], *asset, degree);
                }
            }

            // A failed upload leaves its buffer null, which FindRenderableCloud treats as not renderable.
            for (size_t i = 0; i < assetData.m_handles.size(); ++i)
            {
                auto& renderData = m_splatCloudData.at(assetData.m_handles[i]).m_renderData;
                renderData.m_splatCount = splatCount;
                const auto degree = renderedShDegrees[i];
                renderData.m_sourceGeometry = degree ? assetData.m_sourceGeometry : nullptr;
                renderData.m_sourceSh = degree ? assetData.m_sourceSh[*degree] : nullptr;
                renderData.m_shDegree = degree.value_or(0);
            }
        }

        // The shared splat buffers and the shaders index splats with 32-bit counters, so that is as far as the
        // capacity can go; the projection buffer allocation at that size will fail and be reported by
        // EnsureBuffer rather than silently come out too small.
        constexpr AZ::u64 MaxTotalSplatCount = AZStd::numeric_limits<uint32_t>::max();
        AZ_Warning(
            "GaussianSplatFeatureProcessor",
            totalCount <= MaxTotalSplatCount,
            "Total splat count across all clouds (%llu) exceeds %llu; clamping.",
            static_cast<unsigned long long>(totalCount),
            static_cast<unsigned long long>(MaxTotalSplatCount));
        m_totalSplatCount = static_cast<uint32_t>(AZStd::min(totalCount, MaxTotalSplatCount));
    }

    void GaussianSplatFeatureProcessor::OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        if (auto it = m_gaussianSplatAssets.find(asset.GetId()); it != m_gaussianSplatAssets.end())
        {
            auto& assetData = it->second;
            AZ_Assert(asset.GetAs<GaussianSplatAsset>(), "Asset is not of the expected type.");
            assetData.m_asset = asset;
            // Repacked from the new data on the next sweep, which also repoints every cloud at the new buffers.
            assetData.m_sourceGeometry = nullptr;
            assetData.m_sourceSh = {};
            m_cloudSweepPending = true;

            // From a copy of the handles: a change handler runs component code that may release or acquire
            // clouds, which edits both maps under us.
            const AZStd::vector<GaussianSplatHandle> handles = assetData.m_handles;
            for (const auto handle : handles)
            {
                if (auto cloudIt = m_splatCloudData.find(handle); cloudIt != m_splatCloudData.end())
                {
                    cloudIt->second.m_changedEvent.Signal();
                }
            }
        }
    }

    void GaussianSplatFeatureProcessor::OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        OnAssetReloaded(asset);
    }

    void GaussianSplatFeatureProcessor::Simulate([[maybe_unused]] const FeatureProcessor::SimulatePacket& packet)
    {
        for (auto& [pipeline, data] : m_pipelineData)
        {
            if (m_splatCloudData.empty())
            {
                // Nothing to render, all buffers can be cleared.
                data.m_buffers = {};
                continue;
            }

            if (ArePipelineBuffersStale(pipeline, data))
            {
                // Out before the buffers are rebuilt: AZ::RPI::Pass::AttachBufferToSlot snapshots a buffer's
                // descriptor and rejects a slot that already holds an attachment, so a resized buffer only
                // reaches the shader through a fresh Reset()+Build() of the chain.
                RemoveSharedPasses(data);
                if (!EnsurePipelineBuffers(pipeline, data))
                {
                    continue;
                }
            }
            if (!data.m_parentPass)
            {
                InjectSharedPasses(pipeline, data);
            }
        }

        if (m_cloudSweepPending)
        {
            UpdateCloudSourceBuffers();
        }

        // Everything else a cull pass needs about its cloud (transform, multipliers, SH degree, visibility) is
        // read straight off this feature processor in that pass's own CompileResources, so no per-frame upload
        // or dirty tracking is needed for it here.
        for (auto& [_, data] : m_pipelineData)
        {
            SyncCloudPasses(data);
        }

        // Must happen here in Simulate(), not from a per-frame pass hook: PassSystem::FrameUpdate processes
        // every pass's queued rebuild before rendering any pipeline in that same call, but Simulate() runs
        // earlier in the same frame, before FrameUpdate.
        for (auto& [_, data] : m_pipelineData)
        {
            if (!data.m_parentPass)
            {
                continue;
            }
            // What the chain renders at, which a viewport resize moves and which a pipeline rescaling after
            // post-processing makes differ from the root output the first allocation was sized against. Either
            // way the next Simulate() sees stale buffers and rebuilds them around this size.
            if (auto resolvedFrameSize = TileGrid::TryGetResolvedFrameSize(data.m_parentPass.get()))
            {
                data.m_chainFrameSize = *resolvedFrameSize;
            }
        }
    }

    void GaussianSplatFeatureProcessor::SetTransform(const GaussianSplatHandle& handle, const AZ::Transform& transform)
    {
        if (auto it = m_splatCloudData.find(handle); it != m_splatCloudData.end())
        {
            it->second.m_renderData.m_transform = transform;
        }
    }

    void GaussianSplatFeatureProcessor::SetOpacityMultiplier(const GaussianSplatHandle& handle, float opacityMultiplier)
    {
        if (auto it = m_splatCloudData.find(handle); it != m_splatCloudData.end())
        {
            it->second.m_renderData.m_opacityMultiplier = opacityMultiplier;
        }
    }

    void GaussianSplatFeatureProcessor::SetScaleMultiplier(const GaussianSplatHandle& handle, float scaleMultiplier)
    {
        if (auto it = m_splatCloudData.find(handle); it != m_splatCloudData.end())
        {
            it->second.m_renderData.m_scaleMultiplier = scaleMultiplier;
        }
    }

    void GaussianSplatFeatureProcessor::SetMaxShDegree(const GaussianSplatHandle& handle, uint32_t maxShDegree)
    {
        if (auto it = m_splatCloudData.find(handle); it != m_splatCloudData.end())
        {
            it->second.m_maxShDegreeOverride = maxShDegree;
            // Unlike the transform/opacity/scale setters, this one changes the SH buffer's stride, so it needs a repack -
            // which is also what makes a lowered Max SH Degree actually shrink GPU memory rather than just
            // skipping evaluation. UpdateCloudSourceBuffers decides whether the effective degree really moved.
            m_cloudSweepPending = true;
        }
    }

    void GaussianSplatFeatureProcessor::SetVisibility(const GaussianSplatHandle& handle, bool visible)
    {
        auto it = m_splatCloudData.find(handle);
        if (it == m_splatCloudData.end() || it->second.m_visible == visible)
        {
            return;
        }

        it->second.m_visible = visible;
        m_cloudSweepPending = true;
        MarkCloudPassesDirty();
    }

    void GaussianSplatFeatureProcessor::ReleaseGaussianSplat(const GaussianSplatHandle& handle)
    {
        if (auto it = m_splatCloudData.find(handle); it != m_splatCloudData.end())
        {
            const AZ::Data::AssetId assetId = it->second.m_assetId;
            m_splatCloudData.erase(it);
            // The remaining clouds keep their source buffers; the total splat count (and therefore the shared
            // output buffers' capacity), the set of cull passes, and which SH degrees the asset still needs change.
            m_cloudSweepPending = true;
            MarkCloudPassesDirty();
            if (m_splatCloudData.empty())
            {
                for (auto& [_, data] : m_pipelineData)
                {
                    RemoveSharedPasses(data);
                }
            }

            // The asset's other clouds still need its data and reloads, so only its last user lets it go.
            if (auto assetIt = m_gaussianSplatAssets.find(assetId); assetIt != m_gaussianSplatAssets.end())
            {
                auto& handles = assetIt->second.m_handles;
                AZStd::erase(handles, handle);
                if (handles.empty())
                {
                    AZ::Data::AssetBus::MultiHandler::BusDisconnect(assetId);
                    m_gaussianSplatAssets.erase(assetIt);
                }
            }
        }
    }

    uint32_t GaussianSplatFeatureProcessor::GetSplatCount(const GaussianSplatHandle& handle) const
    {
        if (auto it = m_splatCloudData.find(handle); it != m_splatCloudData.end())
        {
            // From the asset rather than m_renderData, which the sweep only updates on the next Simulate.
            const auto* asset = FindLoadedAsset(it->second);
            return asset ? static_cast<uint32_t>(asset->m_data.size()) : 0;
        }
        return 0;
    }

    AZStd::optional<AZ::Aabb> GaussianSplatFeatureProcessor::GetBounds(const GaussianSplatHandle& handle) const
    {
        if (auto it = m_splatCloudData.find(handle); it != m_splatCloudData.end())
        {
            const auto* asset = FindLoadedAsset(it->second);
            return asset ? asset->m_bounds : AZ::Aabb::CreateNull();
        }
        return AZStd::nullopt;
    }

    const GaussianSplatAsset* GaussianSplatFeatureProcessor::FindLoadedAsset(const SplatCloudData& cloudData) const
    {
        const auto it = m_gaussianSplatAssets.find(cloudData.m_assetId);
        if (it == m_gaussianSplatAssets.end())
        {
            return nullptr;
        }
        const auto* asset = it->second.m_asset.GetAs<GaussianSplatAsset>();
        return asset != nullptr && asset->IsReady() ? asset : nullptr;
    }

    void GaussianSplatFeatureProcessor::ConnectChangeEventHandler(
        const GaussianSplatHandle& handle, GaussianSplatChangedEvent::Handler& handler)
    {
        auto it = m_splatCloudData.find(handle);
        if (it == m_splatCloudData.end())
        {
            AZ_Warning(
                "GaussianSplat",
                false,
                "Handle %d names no acquired cloud, so the handler will never be invoked. Connect after "
                "AcquireGaussianSplatFromAsset returns.",
                handle);
            return;
        }
        handler.Connect(it->second.m_changedEvent);
    }

    const GaussianSplatFeatureProcessor::SplatBuffers* GetPipelineBuffers(AZ::RPI::RenderPipeline* pipeline)
    {
        if (auto* featureProcessor = GaussianSplatFeatureProcessor::GetInstance(pipeline))
        {
            return featureProcessor->GetPipelineBuffers(pipeline);
        }
        return nullptr;
    }
} // namespace GaussianSplat
