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

#include "GaussianSplatRadixBuildKeysPass.h"
#include "GaussianSplatFeatureProcessor.h"
#include <Atom/RPI.Public/Pass/ParentPass.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Public/Shader/ShaderResourceGroup.h>
#include <AzCore/Name/Name.h>

namespace GaussianSplat
{
    namespace
    {
        //! The sort's six 8-bit rounds cover the key's low 48 bits, leaving 16 for tileId above the 32-bit
        //! depth - see GaussianSplatCommon.azsli's NumRadixRounds and PackRadixKey.
        constexpr uint32_t MaxSortableTileCount = 0xFFFF;
    } // namespace

    AZ::RPI::Ptr<GaussianSplatRadixBuildKeysPass> GaussianSplatRadixBuildKeysPass::Create(const AZ::RPI::PassDescriptor& descriptor)
    {
        return aznew GaussianSplatRadixBuildKeysPass(descriptor);
    }

    GaussianSplatRadixBuildKeysPass::GaussianSplatRadixBuildKeysPass(const AZ::RPI::PassDescriptor& descriptor)
        : AZ::RPI::ComputePass(descriptor)
    {
    }

    void GaussianSplatRadixBuildKeysPass::BuildInternal()
    {
        AZ::RPI::ComputePass::BuildInternal();

        if (const auto* buffers = GetPipelineBuffers(GetRenderPipeline()))
        {
            AttachBufferToSlot(AZ::Name("EntriesOut"), buffers->m_entriesA);
        }
    }

    void GaussianSplatRadixBuildKeysPass::CompileResources(const AZ::RHI::FrameGraphCompileContext& context)
    {
        if (auto srg = GetShaderResourceGroup())
        {
            // The grid m_tileRanges is allocated for, not the chain's live frame size, so every tile id this
            // pass keys an entry with indexes a range slot that exists. GaussianSplatRadixUnpackPass and
            // GaussianSplatTileShadingPass use the same grid, so all three agree on the tile id layout.
            const auto* buffers = GetPipelineBuffers(GetRenderPipeline());
            const uint32_t tileCountX = buffers ? buffers->m_tileCountX : 0u;
            srg->SetConstant(m_tileCountXIndex, tileCountX);

            const uint32_t tileCountY = buffers ? buffers->m_tileCountY : 0u;
            srg->SetConstant(m_tileCountYIndex, tileCountY);

            // Bounds the visible slots this pass reads from the projection buffer and the entries it writes -
            // see SplatBuffers::m_splatCapacity and m_entryCapacity.
            srg->SetConstant(m_visibleCapacityIndex, buffers ? buffers->m_splatCapacity : 0u);
            srg->SetConstant(m_entryCapacityIndex, buffers ? buffers->m_entryCapacity : 0u);

            AZ_WarningOnce(
                "GaussianSplat",
                tileCountX * tileCountY <= MaxSortableTileCount,
                "Frame is %ux%u = %u tiles, more than the %u the radix sort's 16-bit tileId can order; splats "
                "will sort incorrectly.",
                tileCountX,
                tileCountY,
                tileCountX * tileCountY,
                MaxSortableTileCount);
        }

        AZ::RPI::ComputePass::CompileResources(context);
    }
} // namespace GaussianSplat
