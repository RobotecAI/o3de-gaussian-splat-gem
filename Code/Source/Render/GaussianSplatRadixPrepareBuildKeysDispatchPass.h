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
    //! Runs after GaussianSplatProjectCullPass and before GaussianSplatRadixBuildKeysPass: turns the GPU-only
    //! visible-splat counter into a DispatchIndirectCommand, so that GaussianSplatRadixBuildKeysPass
    //! can dispatch off the real visible count instead of the total splat count.
    class GaussianSplatRadixPrepareBuildKeysDispatchPass final : public AZ::RPI::ComputePass
    {
        AZ_RPI_PASS(GaussianSplatRadixPrepareBuildKeysDispatchPass);

    public:
        AZ_RTTI(GaussianSplatRadixPrepareBuildKeysDispatchPass, "{2F6B7E4A-1C9D-4E3F-8A2B-6D5C9F1E7A4B}", AZ::RPI::ComputePass);
        AZ_CLASS_ALLOCATOR(GaussianSplatRadixPrepareBuildKeysDispatchPass, AZ::SystemAllocator);
        ~GaussianSplatRadixPrepareBuildKeysDispatchPass() override = default;

        static AZ::RPI::Ptr<GaussianSplatRadixPrepareBuildKeysDispatchPass> Create(const AZ::RPI::PassDescriptor& descriptor);

    protected:
        explicit GaussianSplatRadixPrepareBuildKeysDispatchPass(const AZ::RPI::PassDescriptor& descriptor);
        //! Stages this frame's constants and the dispatch size, then lets the base compile the pass SRG.
        //! Must happen here rather than in BuildCommandListInternal, which runs in the execute phase - after
        //! AZ::RPI::ComputePass::CompileResources has already compiled the SRG.
        void CompileResources(const AZ::RHI::FrameGraphCompileContext& context) override;

    private:
        AZ::RHI::ShaderInputNameIndex m_totalSplatCountIndex = "m_totalSplatCount";
    };
} // namespace GaussianSplat
