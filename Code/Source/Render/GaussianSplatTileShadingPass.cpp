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

#include "GaussianSplatTileShadingPass.h"
#include "GaussianSplatFeatureProcessor.h"
#include <Atom/RPI.Public/Pass/ParentPass.h>
#include <Atom/RPI.Public/Shader/ShaderResourceGroup.h>
#include <AzCore/Math/Vector2.h>

namespace GaussianSplat
{
    AZ::RPI::Ptr<GaussianSplatTileShadingPass> GaussianSplatTileShadingPass::Create(const AZ::RPI::PassDescriptor& descriptor)
    {
        return aznew GaussianSplatTileShadingPass(descriptor);
    }

    GaussianSplatTileShadingPass::GaussianSplatTileShadingPass(const AZ::RPI::PassDescriptor& descriptor)
        : AZ::RPI::ComputePass(descriptor)
    {
    }

    void GaussianSplatTileShadingPass::CompileResources(const AZ::RHI::FrameGraphCompileContext& context)
    {
        if (auto srg = GetShaderResourceGroup())
        {
            // The grid m_tileRanges is allocated for - see SplatBuffers::m_tileCountX. The dispatch itself
            // follows the live OpaqueDepth size, so the shader treats any group outside this grid as an
            // empty tile.
            const auto* buffers = GetPipelineBuffers(GetRenderPipeline());
            const uint32_t tileCountX = buffers ? buffers->m_tileCountX : 0u;
            const uint32_t tileCountY = buffers ? buffers->m_tileCountY : 0u;
            // Sent as a float2 (exact for any realistic tile-grid dimension) rather than a uint2, since
            // AZ::RHI::ShaderResourceGroup::SetConstant memcpy's the C++ type's bit pattern into the shader
            // constant slot with no int<->float conversion - the shader casts this back to uint2 on read.
            srg->SetConstant(m_tileCountIndex, AZ::Vector2(aznumeric_cast<float>(tileCountX), aznumeric_cast<float>(tileCountY)));

            if (auto* featureProcessor = GaussianSplatFeatureProcessor::GetInstance(GetRenderPipeline()))
            {
                srg->SetConstant(m_depthCoverageThresholdIndex, featureProcessor->GetDepthCoverageThreshold());
            }
        }

        AZ::RPI::ComputePass::CompileResources(context);
    }
} // namespace GaussianSplat
