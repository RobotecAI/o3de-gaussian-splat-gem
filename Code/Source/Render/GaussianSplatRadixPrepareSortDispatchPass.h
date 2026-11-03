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

#include <Atom/RPI.Public/Pass/AttachmentReadback.h>
#include <Atom/RPI.Public/Pass/ComputePass.h>

namespace GaussianSplat
{
    //! Runs after GaussianSplatRadixBuildKeysPass and before the first GaussianSplatRadixOnesweepPass round and
    //! GaussianSplatRadixUnpackPass: turns the GPU-only entry counter into the entry count
    //! those later passes read, plus DispatchIndirectCommand buffers so each dispatches only as many thread
    //! groups as this frame's real entry count needs (GlobalHistogram and every round's Onesweep share the
    //! same dispatch-args buffer, since they all launch one 256-thread group per block).
    class GaussianSplatRadixPrepareSortDispatchPass final : public AZ::RPI::ComputePass
    {
        AZ_RPI_PASS(GaussianSplatRadixPrepareSortDispatchPass);

    public:
        AZ_RTTI(GaussianSplatRadixPrepareSortDispatchPass, "{7B4E1A9C-3D6F-4C2A-9E8B-2A5D7F1C4E9B}", AZ::RPI::ComputePass);
        AZ_CLASS_ALLOCATOR(GaussianSplatRadixPrepareSortDispatchPass, AZ::SystemAllocator);
        ~GaussianSplatRadixPrepareSortDispatchPass() override = default;

        static AZ::RPI::Ptr<GaussianSplatRadixPrepareSortDispatchPass> Create(const AZ::RPI::PassDescriptor& descriptor);

    protected:
        explicit GaussianSplatRadixPrepareSortDispatchPass(const AZ::RPI::PassDescriptor& descriptor);
        //! Attaches "CountsOut" from GaussianSplatFeatureProcessor's per-pipeline set. The two dispatch-args
        //! buffers are fixed-size transient attachments declared in the pass template.
        void BuildInternal() override;
        void FrameBeginInternal(FramePrepareParams params) override;
        //! Stages this frame's constants, then lets the base compile the pass SRG. The dispatch is a
        //! single thread, set as a constant in the pass template rather than here.
        //! Must happen here rather than in BuildCommandListInternal, which runs in the execute phase - after
        //! AZ::RPI::ComputePass::CompileResources has already compiled the SRG.
        void CompileResources(const AZ::RHI::FrameGraphCompileContext& context) override;

    private:
        //! Called once the async readback of "CountsOut" (requested from FrameBeginInternal) completes.
        //! Logs an AZ_Warning every time the raw, unclamped entry count exceeds the entry capacity.
        void OnCountsReadback(const AZ::RPI::AttachmentReadback::ReadbackResult& result);

        AZ::RHI::ShaderInputNameIndex m_entryCapacityIndex = "m_entryCapacity";

        //! Diagnostic-only readback of "CountsOut"'s raw entry count, so an entry-capacity overflow (silently
        //! dropped splat/tile-touch entries) can be logged instead of just making splats disappear.
        AZStd::shared_ptr<AZ::RPI::AttachmentReadback> m_countsReadback;
        bool m_readbackInFlight = false;
        uint32_t m_framesSinceLastReadback = 0;
        uint32_t m_lastRequestedEntryCapacity = 0;
    };
} // namespace GaussianSplat
