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

#include "GaussianSplatRadixPrepareSortDispatchPass.h"
#include "GaussianSplatFeatureProcessor.h"
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/Shader/ShaderResourceGroup.h>
#include <AzCore/Name/Name.h>
#include <AzCore/std/smart_ptr/make_shared.h>

namespace GaussianSplat
{
    namespace
    {
        //! Diagnostic readback cadence: this only needs to catch a sustained overflow, not react within a
        //! single frame, so a large interval keeps its GPU/CPU sync overhead negligible.
        constexpr uint32_t FramesBetweenCountsReadback = 60;
    } // namespace

    AZ::RPI::Ptr<GaussianSplatRadixPrepareSortDispatchPass> GaussianSplatRadixPrepareSortDispatchPass::Create(
        const AZ::RPI::PassDescriptor& descriptor)
    {
        return aznew GaussianSplatRadixPrepareSortDispatchPass(descriptor);
    }

    GaussianSplatRadixPrepareSortDispatchPass::GaussianSplatRadixPrepareSortDispatchPass(const AZ::RPI::PassDescriptor& descriptor)
        : AZ::RPI::ComputePass(descriptor)
    {
    }

    void GaussianSplatRadixPrepareSortDispatchPass::BuildInternal()
    {
        if (const auto* buffers = GetPipelineBuffers(GetRenderPipeline()))
        {
            AttachBufferToSlot(AZ::Name("CountsOut"), buffers->m_counts);
        }
    }

    void GaussianSplatRadixPrepareSortDispatchPass::FrameBeginInternal(FramePrepareParams params)
    {
        AZ::RPI::ComputePass::FrameBeginInternal(params);

        if (m_readbackInFlight)
        {
            return;
        }
        if (m_framesSinceLastReadback < FramesBetweenCountsReadback)
        {
            ++m_framesSinceLastReadback;
            return;
        }

        const auto* buffers = GetPipelineBuffers(GetRenderPipeline());
        if (buffers == nullptr)
        {
            return;
        }

        if (!m_countsReadback)
        {
            m_countsReadback = AZStd::make_shared<AZ::RPI::AttachmentReadback>(
                AZ::RHI::ScopeId{ "GaussianSplatRadixPrepareSortDispatchPass_CountsReadback" });
            m_countsReadback->SetCallback(
                [this](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
                {
                    OnCountsReadback(result);
                });
        }

        m_lastRequestedEntryCapacity = buffers->m_entryCapacity;
        if (ReadbackAttachment(m_countsReadback, 0, AZ::Name("CountsOut")))
        {
            m_readbackInFlight = true;
            m_framesSinceLastReadback = 0;
        }
    }

    void GaussianSplatRadixPrepareSortDispatchPass::OnCountsReadback(const AZ::RPI::AttachmentReadback::ReadbackResult& result)
    {
        m_readbackInFlight = false;
        if (result.m_state != AZ::RPI::AttachmentReadback::ReadbackState::Success || !result.m_dataBuffer ||
            result.m_dataBuffer->size() < sizeof(uint32_t) * 2)
        {
            return;
        }

        const auto* counts = reinterpret_cast<const uint32_t*>(result.m_dataBuffer->data());
        [[maybe_unused]] const uint32_t rawEntryCount = counts[1];
        AZ_Warning(
            "GaussianSplatRadixPrepareSortDispatchPass",
            rawEntryCount <= m_lastRequestedEntryCapacity,
            "Radix-sort entry count (%u) exceeded the fixed entry capacity (%u) - excess splat/tile-touch "
            "entries were silently dropped this frame, which can make visible splats disappear (e.g. when a "
            "wide/dense view causes many splat-tile touches). Consider raising the "
            "/Robotec/GaussianSplat/RadixEntryCapacity Settings Registry key.",
            rawEntryCount,
            m_lastRequestedEntryCapacity);
    }

    void GaussianSplatRadixPrepareSortDispatchPass::CompileResources(const AZ::RHI::FrameGraphCompileContext& context)
    {
        // entryCapacity clamps the entry counter before it sizes every later stage's dispatch - see
        // GaussianSplatRadixPrepareSortDispatch.azsl's comment on why the counter can overshoot it.
        if (auto srg = GetShaderResourceGroup())
        {
            const auto* buffers = GetPipelineBuffers(GetRenderPipeline());
            srg->SetConstant(m_entryCapacityIndex, buffers ? buffers->m_entryCapacity : 0u);
        }

        // Last: compiles the pass SRG, so every SetConstant above has to precede it.
        AZ::RPI::ComputePass::CompileResources(context);
    }
} // namespace GaussianSplat
