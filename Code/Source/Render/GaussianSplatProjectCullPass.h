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
#include <GaussianSplat/GaussianSplatFeatureProcessorInterface.h>

namespace GaussianSplat
{
    //! Expands one cloud's raw per-splat asset payload, projects it to screen space, culls it, and evaluates its
    //! SH color - once per splat per frame, entirely on the GPU, one thread per splat of that cloud.
    //! GaussianSplatProjectCullGroupPass creates one instance per cloud registered at
    //! GaussianSplatFeatureProcessor, so this pass's cloud is fixed for the pass's lifetime and all of its
    //! per-cloud data (source buffer, model matrix, multipliers, SH degree) is plain PassSrg state rather than a
    //! GPU-side descriptor table every thread has to search.
    class GaussianSplatProjectCullPass final : public AZ::RPI::ComputePass
    {
        AZ_RPI_PASS(GaussianSplatProjectCullPass);

    public:
        AZ_RTTI(GaussianSplatProjectCullPass, "{3D8A6C1E-4F7B-4E9A-9C2D-8A5B1E6F3D7C}", AZ::RPI::ComputePass);
        AZ_CLASS_ALLOCATOR(GaussianSplatProjectCullPass, AZ::SystemAllocator);
        ~GaussianSplatProjectCullPass() override = default;

        static AZ::RPI::Ptr<GaussianSplatProjectCullPass> Create(const AZ::RPI::PassDescriptor& descriptor);

        //! Set once by GaussianSplatProjectCullGroupPass right after creation, before this pass is ever built.
        void SetCloudHandle(GaussianSplatFeatureProcessorInterface::GaussianSplatHandle handle);

        //! False while this pass's cloud contributes nothing, so AZ::RPI::Pass::FrameBegin skips the pass
        //! whole. Overridden rather than toggled by the parent because FrameBegin early-outs ahead of
        //! FrameBeginInternal, leaving a self-disabled pass no hook from which to re-enable itself.
        bool IsEnabled() const override;

    protected:
        explicit GaussianSplatProjectCullPass(const AZ::RPI::PassDescriptor& descriptor);
        //! Stages this frame's constants/buffer views and the dispatch size, then lets the base compile the
        //! pass SRG. Must happen here rather than in BuildCommandListInternal, which runs in the execute
        //! phase - after AZ::RPI::ComputePass::CompileResources has already compiled the SRG.
        void CompileResources(const AZ::RHI::FrameGraphCompileContext& context) override;

    private:
        //! Which cloud this instance projects. Fixed for the pass's lifetime: the group pass discards and
        //! re-creates its children whenever the feature processor's cloud set changes.
        GaussianSplatFeatureProcessorInterface::GaussianSplatHandle m_cloudHandle =
            GaussianSplatFeatureProcessorInterface::InvalidGaussianSplatHandle;

        AZ::RHI::ShaderInputNameIndex m_modelMatrixIndex = "m_modelMatrix";
        AZ::RHI::ShaderInputNameIndex m_viewportSizeIndex = "m_viewportSize";
        AZ::RHI::ShaderInputNameIndex m_opacityMultiplierIndex = "m_opacityMultiplier";
        AZ::RHI::ShaderInputNameIndex m_scaleMultiplierIndex = "m_scaleMultiplier";
        AZ::RHI::ShaderInputNameIndex m_splatCountIndex = "m_splatCount";
        AZ::RHI::ShaderInputNameIndex m_shUintsPerSplatIndex = "m_shUintsPerSplat";
        AZ::RHI::ShaderInputNameIndex m_shDegreeIndex = "m_shDegree";
        AZ::RHI::ShaderInputNameIndex m_visibleCapacityIndex = "m_visibleCapacity";
        //! Bound via SRG::SetBufferView, not Pass::AttachBufferToSlot, since these are CPU-written buffers that
        //! never get an RHI::AttachmentId assigned. Only safe to bind from CompileResources: the pass SRG is
        //! compiled at the end of that call, so a bind issued any later would not reach the GPU until the
        //! following frame.
        AZ::RHI::ShaderInputNameIndex m_sourceGeometryIndex = "m_sourceGeometry";
        AZ::RHI::ShaderInputNameIndex m_sourceShIndex = "m_sourceSh";
    };
} // namespace GaussianSplat
