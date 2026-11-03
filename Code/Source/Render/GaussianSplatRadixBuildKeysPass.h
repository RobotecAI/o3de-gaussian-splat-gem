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

#include <Atom/RPI.Public/Pass/ComputePass.h>

namespace GaussianSplat
{
    //! First stage of GPU tile binning: expands each visible splat into one (sort key, visibleSlot) entry per
    //! screen tile its bounding circle touches, atomically reserving its own slice of the flat entry buffer.
    //! Dispatches indirectly off a visible-count buffer GaussianSplatRadixPrepareBuildKeysDispatchPass computes
    //! each frame, since the visible count is only known GPU-side.
    //!
    //! The sort key packs (tileId, quantized depth) into 64 bits (two uint32 words) so the radix-sort chain that
    //! follows produces both per-tile grouping and near-to-far order in one combined-key sort: the low word is
    //! `0xFFFFFFFF - round(deviceDepth * 4294967295)` (reverse-Z, so ascending key = near-to-far), the high word
    //! is the exact tile index - see PackRadixKey in GaussianSplatCommon.azsli.
    class GaussianSplatRadixBuildKeysPass final : public AZ::RPI::ComputePass
    {
        AZ_RPI_PASS(GaussianSplatRadixBuildKeysPass);

    public:
        AZ_RTTI(GaussianSplatRadixBuildKeysPass, "{8C6C4BC2-909F-4C53-B1F6-EEF5FCFD412C}", AZ::RPI::ComputePass);
        AZ_CLASS_ALLOCATOR(GaussianSplatRadixBuildKeysPass, AZ::SystemAllocator);
        ~GaussianSplatRadixBuildKeysPass() override = default;

        static AZ::RPI::Ptr<GaussianSplatRadixBuildKeysPass> Create(const AZ::RPI::PassDescriptor& descriptor);

    protected:
        explicit GaussianSplatRadixBuildKeysPass(const AZ::RPI::PassDescriptor& descriptor);
        //! Chains to AZ::RPI::ComputePass::BuildInternal to resolve the indirect-dispatch buffer binding, then
        //! attaches "EntriesOut" - one half of the ping-pong pair GaussianSplatFeatureProcessor owns for this
        //! pipeline, the half round 0 reads from.
        void BuildInternal() override;
        //! Stages this frame's constants, then lets the base compile the pass SRG. Must happen here rather
        //! than in BuildCommandListInternal, which runs in the execute phase - after
        //! AZ::RPI::ComputePass::CompileResources has already compiled the SRG.
        void CompileResources(const AZ::RHI::FrameGraphCompileContext& context) override;

    private:
        AZ::RHI::ShaderInputNameIndex m_tileCountXIndex = "m_tileCountX";
        AZ::RHI::ShaderInputNameIndex m_tileCountYIndex = "m_tileCountY";
        AZ::RHI::ShaderInputNameIndex m_entryCapacityIndex = "m_entryCapacity";
        AZ::RHI::ShaderInputNameIndex m_visibleCapacityIndex = "m_visibleCapacity";
    };
} // namespace GaussianSplat
