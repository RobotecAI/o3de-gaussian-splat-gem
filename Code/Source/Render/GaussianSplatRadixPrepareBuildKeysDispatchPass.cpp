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

#include "GaussianSplatRadixPrepareBuildKeysDispatchPass.h"
#include "GaussianSplatFeatureProcessor.h"
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/Shader/ShaderResourceGroup.h>

namespace GaussianSplat
{
    AZ::RPI::Ptr<GaussianSplatRadixPrepareBuildKeysDispatchPass> GaussianSplatRadixPrepareBuildKeysDispatchPass::Create(
        const AZ::RPI::PassDescriptor& descriptor)
    {
        return aznew GaussianSplatRadixPrepareBuildKeysDispatchPass(descriptor);
    }

    GaussianSplatRadixPrepareBuildKeysDispatchPass::GaussianSplatRadixPrepareBuildKeysDispatchPass(
        const AZ::RPI::PassDescriptor& descriptor)
        : AZ::RPI::ComputePass(descriptor)
    {
    }

    void GaussianSplatRadixPrepareBuildKeysDispatchPass::CompileResources(const AZ::RHI::FrameGraphCompileContext& context)
    {
        if (auto srg = GetShaderResourceGroup())
        {
            // The capacity of the projection buffer BuildKeys reads, as allocated - see SplatBuffers::m_splatCapacity.
            const auto* buffers = GetPipelineBuffers(GetRenderPipeline());
            srg->SetConstant(m_totalSplatCountIndex, buffers ? buffers->m_splatCapacity : 0u);
        }

        AZ::RPI::ComputePass::CompileResources(context);
    }
} // namespace GaussianSplat
