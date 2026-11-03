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
#include <Atom/RPI.Public/Pass/ComputePass.h>

namespace GaussianSplat
{
    //! One 16x16-thread-group compute dispatch per screen tile, consuming the per-frame splat/tile buffers the
    //! GPU tile-binning pipeline (GaussianSplatProjectCullPass through GaussianSplatRadixUnpackPass) builds every
    //! frame. See Docs/Rasterizer.md's "Drawing" section for the overall pass graph (this pass writes an
    //! offscreen accumulation image and a per-pixel splat depth image; GaussianSplatComposite.pass then blends
    //! the former into the scene color and GaussianSplatDepthComposite.pass merges the latter into the
    //! pipeline's linear depth). A single
    //! shared instance serves every visible splat cloud in the scene (see
    //! GaussianSplatFeatureProcessor::InjectSharedPasses): the pipeline merges every visible cloud's splats into
    //! one tile-sorted buffer set before this pass ever dispatches, so there is exactly one dispatch writing the
    //! accumulation image per frame, not one sequential dispatch per cloud into the same UAV.
    class GaussianSplatTileShadingPass final : public AZ::RPI::ComputePass
    {
        AZ_RPI_PASS(GaussianSplatTileShadingPass);

    public:
        AZ_RTTI(GaussianSplatTileShadingPass, "{6C6E9F5A-6C9A-4C60-9B3E-6A2E5C6D9F35}", AZ::RPI::ComputePass);
        AZ_CLASS_ALLOCATOR(GaussianSplatTileShadingPass, AZ::SystemAllocator);
        ~GaussianSplatTileShadingPass() override = default;

        static AZ::RPI::Ptr<GaussianSplatTileShadingPass> Create(const AZ::RPI::PassDescriptor& descriptor);

    protected:
        explicit GaussianSplatTileShadingPass(const AZ::RPI::PassDescriptor& descriptor);
        //! Pushes the current tile grid dimensions as a shader constant, then defers to
        //! ComputePass::CompileResources, which compiles the pass SRG and sizes the dispatch from the
        //! OpaqueDepth attachment ("FullscreenDispatch": true) rather than a CPU-pushed thread count. The
        //! constant has to be staged in this call: the frame graph runs it before the execute phase, and the
        //! base compiles the SRG at the end of it.
        void CompileResources(const AZ::RHI::FrameGraphCompileContext& context) override;

    private:
        AZ::RHI::ShaderInputNameIndex m_tileCountIndex = "m_tileCount";
        AZ::RHI::ShaderInputNameIndex m_depthCoverageThresholdIndex = "m_depthCoverageThreshold";
    };
} // namespace GaussianSplat
