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

#include "GaussianSplatProjectCullPass.h"
#include "GaussianSplatFeatureProcessor.h"
#include "GaussianSplatTileGrid.h"
#include "Utils/GaussianSplatPacking.h"
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/Shader/ShaderResourceGroup.h>

namespace GaussianSplat
{
    AZ::RPI::Ptr<GaussianSplatProjectCullPass> GaussianSplatProjectCullPass::Create(const AZ::RPI::PassDescriptor& descriptor)
    {
        return aznew GaussianSplatProjectCullPass(descriptor);
    }

    GaussianSplatProjectCullPass::GaussianSplatProjectCullPass(const AZ::RPI::PassDescriptor& descriptor)
        : AZ::RPI::ComputePass(descriptor)
    {
    }

    void GaussianSplatProjectCullPass::SetCloudHandle(GaussianSplatFeatureProcessorInterface::GaussianSplatHandle handle)
    {
        m_cloudHandle = handle;
    }

    bool GaussianSplatProjectCullPass::IsEnabled() const
    {
        if (!AZ::RPI::ComputePass::IsEnabled())
        {
            return false;
        }

        const GaussianSplatFeatureProcessor* featureProcessor = GaussianSplatFeatureProcessor::GetInstance(GetRenderPipeline());
        return featureProcessor != nullptr && featureProcessor->IsCloudRenderable(m_cloudHandle);
    }

    void GaussianSplatProjectCullPass::CompileResources(const AZ::RHI::FrameGraphCompileContext& context)
    {
        uint32_t splatCount = 0;
        if (auto srg = GetShaderResourceGroup())
        {
            if (const GaussianSplatFeatureProcessor* featureProcessor = GaussianSplatFeatureProcessor::GetInstance(GetRenderPipeline()))
            {
                if (const auto cloud = featureProcessor->GetCloudRenderData(m_cloudHandle))
                {
                    splatCount = cloud->m_splatCount;
                    AZStd::array<float, 16> modelMatrix{};
                    AZ::Matrix4x4::CreateFromTransform(cloud->m_transform).StoreToRowMajorFloat16(modelMatrix.data());
                    srg->SetConstantRaw(m_modelMatrixIndex, modelMatrix.data(), static_cast<uint32_t>(sizeof(float) * 16));
                    srg->SetConstant(m_opacityMultiplierIndex, cloud->m_opacityMultiplier);
                    srg->SetConstant(m_scaleMultiplierIndex, cloud->m_scaleMultiplier);
                    srg->SetConstant(m_splatCountIndex, cloud->m_splatCount);
                    srg->SetConstant(m_shUintsPerSplatIndex, Packing::ShUintsPerSplat(cloud->m_shDegree));
                    srg->SetConstant(m_shDegreeIndex, cloud->m_shDegree);
                    srg->SetBufferView(m_sourceGeometryIndex, cloud->m_sourceGeometry->GetBufferView());
                    srg->SetBufferView(m_sourceShIndex, cloud->m_sourceSh->GetBufferView());
                }
            }

            // Bounds the visible slot this cloud's splats reserve in the shared output buffer, which holds
            // every registered cloud's splats - not this cloud's own count. This is the capacity the bound
            // buffer was allocated with, which can lag the feature processor's live total by a frame.
            const auto* buffers = GetPipelineBuffers(GetRenderPipeline());
            srg->SetConstant(m_visibleCapacityIndex, buffers ? buffers->m_splatCapacity : 0u);

            srg->SetConstant(m_viewportSizeIndex, TileGrid::GetFrameSize(GetParent()));
        }

        // One thread per splat of this cloud alone, not a fullscreen/image-sized dispatch.
        SetTargetThreadCounts(splatCount, 1, 1);

        // Last: compiles the pass SRG, so every SetConstant/SetBufferView above has to precede it.
        AZ::RPI::ComputePass::CompileResources(context);
    }
} // namespace GaussianSplat
