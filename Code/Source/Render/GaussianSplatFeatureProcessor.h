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

#pragma once

#include <Atom/RPI.Public/Buffer/Buffer.h>
#include <Atom/RPI.Public/Pass/Pass.h>
#include <Atom/RPI.Reflect/Base.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector2.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/optional.h>
#include <GaussianSplat/GaussianSplatFeatureProcessorInterface.h>

namespace AZ::RPI
{
    class RenderPipeline;
}

namespace GaussianSplat
{
    class GaussianSplatFeatureProcessor
        : public GaussianSplatFeatureProcessorInterface
        , protected AZ::Data::AssetBus::MultiHandler
    {
    public:
        AZ_RTTI(GaussianSplatFeatureProcessor, GaussianSplatFeatureProcessorTypeId, GaussianSplatFeatureProcessorInterface);
        AZ_CLASS_ALLOCATOR(GaussianSplatFeatureProcessor, AZ::SystemAllocator)

        static void Reflect(AZ::ReflectContext* context);

        static GaussianSplatFeatureProcessor* GetInstance(AZ::RPI::RenderPipeline* pipeline);

        GaussianSplatFeatureProcessor() = default;
        ~GaussianSplatFeatureProcessor() override = default;
        GaussianSplatFeatureProcessor(const GaussianSplatFeatureProcessor&) = delete;
        GaussianSplatFeatureProcessor& operator=(const GaussianSplatFeatureProcessor&) = delete;
        GaussianSplatFeatureProcessor(GaussianSplatFeatureProcessor&&) = delete;
        GaussianSplatFeatureProcessor& operator=(GaussianSplatFeatureProcessor&&) = delete;

        // GaussianSplatFeatureProcessorInterface overrides
        void SetTransform(const GaussianSplatHandle& handle, const AZ::Transform& transform) override;
        void SetOpacityMultiplier(const GaussianSplatHandle& handle, float opacityMultiplier) override;
        void SetScaleMultiplier(const GaussianSplatHandle& handle, float scaleMultiplier) override;
        void SetMaxShDegree(const GaussianSplatHandle& handle, uint32_t maxShDegree) override;
        GaussianSplatHandle AcquireGaussianSplatFromAsset(AZ::Data::Asset<GaussianSplatAsset> gaussianSplatAsset) override;
        void SetVisibility(const GaussianSplatHandle& handle, bool visible) override;
        void ReleaseGaussianSplat(const GaussianSplatHandle& handle) override;
        [[nodiscard]] uint32_t GetSplatCount(const GaussianSplatHandle& handle) const override;
        [[nodiscard]] AZStd::optional<AZ::Aabb> GetBounds(const GaussianSplatHandle& handle) const override;

        void ConnectChangeEventHandler(const GaussianSplatHandle& handle, GaussianSplatChangedEvent::Handler& handler) override;

        //! Every GPU buffer the splat pass chain reads and writes, owned per render pipeline because the
        //! contents are per view - two cameras sort and shade their own splats. Allocated in full before the
        //! chain is injected, so a pass only ever builds against buffers that already exist.
        struct SplatBuffers
        {
            //! Ping-pong pair the radix rounds alternate between; see GaussianSplatRadixOnesweepPass.
            AZ::Data::Instance<AZ::RPI::Buffer> m_entriesA;
            AZ::Data::Instance<AZ::RPI::Buffer> m_entriesB;
            AZ::Data::Instance<AZ::RPI::Buffer> m_tileSplatIndices;
            AZ::Data::Instance<AZ::RPI::Buffer> m_tileStatus;
            AZ::Data::Instance<AZ::RPI::Buffer> m_tileCounter;
            AZ::Data::Instance<AZ::RPI::Buffer> m_splatProjection;
            AZ::Data::Instance<AZ::RPI::Buffer> m_tileRanges;
            AZ::Data::Instance<AZ::RPI::Buffer> m_counts;

            //! The sizes the buffers above were allocated for, and the only sizes a pass may bound its shader by.
            //! The feature processor's live splat count and the chain's live frame size move a frame or more
            //! before Simulate sees the set is stale and rebuilds it, and a shader bounded by them in the meantime
            //! would index past the end of the buffers it actually has. Simulate compares these against the
            //! live values to decide that; a change means the chain comes out, the buffers are rebuilt, and the
            //! chain goes back in - AZ::RPI::Pass::AttachBufferToSlot snapshots a buffer's descriptor, so only
            //! a fresh Reset()+Build() shows the shader a new size.
            uint32_t m_entryCapacity = 0; //!< Elements in m_entriesA, m_entriesB and m_tileSplatIndices.
            uint32_t m_splatCapacity = 0; //!< Elements in m_splatProjection.
            uint32_t m_tileCountX = 0; //!< Tile grid m_tileRanges is sized for, m_tileCountX * m_tileCountY entries.
            uint32_t m_tileCountY = 0;
        };

        //! pipeline's buffer set, or nullptr while it has none - which is also every moment its pass chain is
        //! not injected, since the chain only goes in once the set is complete.
        [[nodiscard]] const SplatBuffers* GetPipelineBuffers(const AZ::RPI::RenderPipeline* pipeline) const;

        //! Accumulated coverage (1 - transmittance) at which GaussianSplatTileShading.azsl settles a pixel's
        //! splat depth, validated to (0, 1] in Activate.
        [[nodiscard]] float GetDepthCoverageThreshold() const
        {
            return m_depthCoverageThreshold;
        }

        //! Everything one GaussianSplatProjectCullPass instance needs about the single cloud it projects.
        struct CloudRenderData
        {
            AZ::Data::Instance<AZ::RPI::Buffer> m_sourceGeometry;
            AZ::Data::Instance<AZ::RPI::Buffer> m_sourceSh;
            uint32_t m_splatCount = 0;
            uint32_t m_shDegree = 0;
            AZ::Transform m_transform = AZ::Transform::CreateIdentity();
            float m_opacityMultiplier = 1.0f;
            float m_scaleMultiplier = 1.0f;
        };

        //! Handles of every registered cloud, renderable this frame or not, in unspecified order.
        [[nodiscard]] AZStd::vector<GaussianSplatHandle> GetCloudHandles() const;

        //! Whether this cloud is shown. A hidden cloud gets no cull pass at all: the pass SRG holds a
        //! reference to the source buffers, so leaving one alive would pin the memory SetVisibility frees.
        [[nodiscard]] bool IsCloudVisible(GaussianSplatHandle handle) const;

        //! Whether this cloud contributes anything this frame. False for an unknown handle, a hidden cloud,
        //! or one whose source buffers are not uploaded yet. Cheaper than GetCloudRenderData, which copies
        //! the struct; GaussianSplatProjectCullPass::IsEnabled calls this every frame.
        [[nodiscard]] bool IsCloudRenderable(GaussianSplatHandle handle) const;

        //! This cloud's current render data, or AZStd::nullopt when IsCloudRenderable is false.
        [[nodiscard]] AZStd::optional<CloudRenderData> GetCloudRenderData(GaussianSplatHandle handle) const;

    protected:
        // FeatureProcessor overrides
        //! Injects the shared ParentPass (tile-binning/radix-sort/shading/composite children) into pipeline. See
        //! Docs/Rasterizer.md's "Drawing" section for the pass graph this injects.
        void AddRenderPasses(AZ::RPI::RenderPipeline* pipeline) override;
        //! Tears down and forgets pipeline's PipelineData entry when the pipeline it was injected into is removed.
        void OnRenderPipelineChanged(
            AZ::RPI::RenderPipeline* pipeline, AZ::RPI::SceneNotification::RenderPipelineChangeType changeType) override;

        // Data::AssetBus overrides
        void OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        void OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset) override;

    private:
        //! This cloud's render data, or nullptr when it contributes nothing this frame.
        [[nodiscard]] const CloudRenderData* FindRenderableCloud(GaussianSplatHandle handle) const;

        struct SplatCloudData
        {
            GaussianSplatHandle m_index = 0;
            CloudRenderData m_renderData;

            uint32_t m_maxShDegreeOverride = 3;
            //! False drops this cloud's source buffers until it is shown again - see SetVisibility.
            bool m_visible = true;
            AZ::Aabb m_bounds = AZ::Aabb::CreateNull(); //!< local-space bounds, from the asset
            //! Raised when an async load or reload changes this cloud's payload.
            GaussianSplatChangedEvent m_changedEvent;
            AZ::Data::AssetId m_assetId; //! AssetId of the gaussian splat asset, if acquired from an asset
            //! The source asset, held both to keep a ref count on it and as the single storage for this
            //! cloud's vertices and SH coefficients, which UpdateCloudSourceBuffers uploads from.
            //! m_splatCount and m_bounds are cached copies that stay valid while the asset is not resident.
            AZ::Data::Asset<AZ::Data::AssetData> m_assetData;
        };

        // FeatureProcessor overrides
        void Activate() override;
        void Deactivate() override;
        void Simulate(const FeatureProcessor::SimulatePacket& packet) override;

        //! Repacks each visible cloud that needs it into that cloud's own GPU buffers, and recomputes
        //! m_totalSplatCount over every cloud, hidden ones included. A cloud needs its geometry repacked when
        //! m_sourceGeometry is null, and its SH repacked when that buffer is null or the degree its asset and
        //! Max-SH-Degree override call for differs from the uploaded one, so a cloud already up to date costs
        //! nothing here. A hidden cloud is skipped entirely - SetVisibility drops its buffers, and this is
        //! what would otherwise hand them straight back.
        //!
        //! Both buffers are built by a per-splat CPU pass rather than a byte copy: Packing::PackCloudGeometry
        //! deinterleaves CloudVertex, and Packing::PackCloudSh transposes the asset's channel-major SH blocks
        //! into the coefficient-major, RGB-interleaved layout GaussianSplatProjectCull.azsl walks linearly.
        //! Repacks are rare (asset load, or a Max-SH-Degree change), so paying that pass here is cheaper than
        //! the per-thread work it removes.
        //!
        //! Runs only on m_cloudSweepPending, not every frame. A changed total splat count reaches the shared
        //! buffers indirectly: Simulate's ArePipelineBuffersStale check tears the pass tree down on a later
        //! frame, since the shared projection buffer can't grow in place.
        void UpdateCloudSourceBuffers();

        //! Per-pipeline state for one RenderPipeline's own instance of the shared GaussianSplat pass chain.
        struct PipelineData
        {
            //! This pipeline's injected chain, or null while it has none.
            AZ::RPI::Ptr<AZ::RPI::Pass> m_parentPass;
            //! The size the injected chain resolved for itself, or zero before it has ever reported one.
            AZ::Vector2 m_chainFrameSize = AZ::Vector2::CreateZero();
            //! Set when the cloud set changes, so this pipeline's GaussianSplatProjectCullGroupPass is asked to
            //! re-create its per-cloud children. Stays set until that group pass is actually reachable, since
            //! a parent pass's children only exist after its first Build().
            bool m_cloudPassesDirty = false;
            //! This pipeline's GPU buffers, allocated before its chain is injected and released with it.
            SplatBuffers m_buffers;
        };

        //! Radix-sort entry capacity a pipeline's buffer set is allocated at: m_entryCapacity, floored to 1.
        [[nodiscard]] uint32_t GetRadixEntryCapacity() const
        {
            return AZStd::max(m_entryCapacity, 1u);
        }
        //! Splat capacity a pipeline's shared projection buffer is allocated at: the total splat count across
        //! every registered cloud (not just visible ones), floored to 1.
        [[nodiscard]] uint32_t GetSharedSplatCapacity() const
        {
            return AZStd::max(m_totalSplatCount, 1u);
        }

        //! Creates this pipeline's shared ParentPass and injects it into pipeline, if not already injected and at
        //! least one cloud exists. Safe to call redundantly.
        void InjectSharedPasses(AZ::RPI::RenderPipeline* pipeline, PipelineData& data);
        //! Removes data's shared parent pass (and its children) from whatever pipeline it was injected into, if any.
        void RemoveSharedPasses(PipelineData& data);
        //! Allocates data's buffer set, or confirms the existing one still matches the current capacities and
        //! frame size. Releases and rebuilds the set when any of those moved, which requires the caller to have
        //! taken the chain out first. False means an allocation failed and the chain has to stay out.
        bool EnsurePipelineBuffers(AZ::RPI::RenderPipeline* pipeline, PipelineData& data);
        //! Whether data's buffer set still matches the sizes it was built from.
        [[nodiscard]] bool ArePipelineBuffersStale(AZ::RPI::RenderPipeline* pipeline, const PipelineData& data) const;
        //! If data's cloud set changed since its per-cloud cull passes were last created, asks its
        //! GaussianSplatProjectCullGroupPass to re-create them. No-op (and leaves the dirty flag set for a
        //! later retry) while that group pass isn't reachable yet.
        void SyncCloudPasses(PipelineData& data);
        //! Marks every pipeline's per-cloud cull passes as needing to be re-created. Called on cloud
        //! acquire/release - the one thing that changes how many cull passes a group pass should hold.
        void MarkCloudPassesDirty();

        //! Keyed by raw pointer since RenderPipeline instances outlive the map entries referencing them (an
        //! entry is erased in OnRenderPipelineChanged before the pipeline itself is destroyed).
        AZStd::unordered_map<AZ::RPI::RenderPipeline*, PipelineData> m_pipelineData;

        AZStd::unordered_map<GaussianSplatHandle, SplatCloudData> m_splatCloudData; //!< Map of gaussian splat cloud data
        GaussianSplatHandle m_currentGaussianSplatDataIndex = 0; //!< Index to the next cloud data to be created
        AZStd::unordered_map<AZ::Data::AssetId, GaussianSplatHandle> m_gaussianSplatAssets;

        uint32_t m_totalSplatCount = 0; //!< total splat count across every registered cloud; see GetSharedSplatCapacity
        //! Schedules one UpdateCloudSourceBuffers sweep over every cloud; deliberately coarse, since the sweep
        //! has to walk them all to recompute m_totalSplatCount anyway. It does not mean every cloud repacks:
        //! the sweep checks each cloud's buffers against its asset and skips the ones already up to date.
        //!
        //! Set on cloud add/remove/asset-(re)load, by SetVisibility (hiding drops a cloud's source buffers and
        //! showing it re-uploads them), and unconditionally by SetMaxShDegree (whether or not the degree it
        //! resolves to actually moved - the sweep is what compares that against m_uploadedShDegree, and leaves
        //! a cloud's SH buffer alone if it didn't), because the degree also determines the SH buffer's stride;
        //! forces UpdateCloudSourceBuffers on the next Simulate(). The transform/opacity/scale setters do *not*
        //! set it - those reach the GPU as PassSrg constants each cull pass reads straight off this feature
        //! processor in its own CompileResources, so they need no upload at all.
        bool m_cloudSweepPending = true;

        //! Fixed GPU radix-sort entries buffer capacity, set once in Activate. See SplatBuffers::m_entryCapacity.
        uint32_t m_entryCapacity = 0;
        //! Number of RadixBlockSize-entry scatter/histogram blocks m_entryCapacity is sized for; sizes SplatBuffers::m_tileStatus.
        uint32_t m_blockCapacity = 0;
        //! Validated coverage threshold for the per-pixel splat depth, set once in Activate. See
        //! GetDepthCoverageThreshold.
        float m_depthCoverageThreshold = 0.0f;
    };

    //! The buffers GaussianSplatFeatureProcessor allocated for pipeline, or nullptr if it has none yet.
    //!
    //! Every splat pass binds its slots from this set rather than allocating: the feature processor puts the
    //! whole set in place before it injects the chain, so a pass that is building always finds it. A null
    //! result therefore means the chain is being built outside the feature processor's control, and the pass
    //! leaves its slots alone rather than half-attaching.
    const GaussianSplatFeatureProcessor::SplatBuffers* GetPipelineBuffers(AZ::RPI::RenderPipeline* pipeline);
} // namespace GaussianSplat
