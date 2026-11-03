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

#include "GaussianSplatRadixOnesweepPass.h"
#include "GaussianSplatFeatureProcessor.h"
#include <Atom/RPI.Public/Shader/ShaderResourceGroup.h>
#include <AzCore/Name/Name.h>
#include <AzCore/std/string/string_view.h>

namespace GaussianSplat
{
    AZ::RPI::Ptr<GaussianSplatRadixOnesweepPass> GaussianSplatRadixOnesweepPass::Create(const AZ::RPI::PassDescriptor& descriptor)
    {
        return aznew GaussianSplatRadixOnesweepPass(descriptor);
    }

    GaussianSplatRadixOnesweepPass::GaussianSplatRadixOnesweepPass(const AZ::RPI::PassDescriptor& descriptor)
        : AZ::RPI::ComputePass(descriptor)
    {
    }

    void GaussianSplatRadixOnesweepPass::BuildInternal()
    {
        AZ::RPI::ComputePass::BuildInternal();

        // This instance's template-authored name is "GaussianSplatRadixOnesweep" followed by its round index
        // (0..5 - see GaussianSplatParent.pass), so the round is simply this name's trailing character.
        const AZStd::string_view name = GetName().GetStringView();
        AZ_Assert(
            !name.empty() && name.back() >= '0' && name.back() <= '9',
            "GaussianSplatRadixOnesweepPass '%.*s' has no trailing digit round index in its name.",
            AZ_STRING_ARG(name));
        m_round = static_cast<uint32_t>(name.back() - '0');

        const auto* buffers = GetPipelineBuffers(GetRenderPipeline());
        if (buffers == nullptr)
        {
            return;
        }

        // GaussianSplatRadixBuildKeysPass fills m_entriesA, so even rounds read A and scatter into B, odd
        // rounds the reverse. NumRadixRounds is even, leaving the finished sort back in m_entriesA.
        const bool readsEntriesA = (m_round % 2) == 0;
        AttachBufferToSlot(AZ::Name("Entries"), readsEntriesA ? buffers->m_entriesA : buffers->m_entriesB);
        AttachBufferToSlot(AZ::Name("EntriesOut"), readsEntriesA ? buffers->m_entriesB : buffers->m_entriesA);
        // Shared by every round; the Clear load action on both slots re-zeroes them before each dispatch.
        AttachBufferToSlot(AZ::Name("TileStatus"), buffers->m_tileStatus);
        AttachBufferToSlot(AZ::Name("TileCounter"), buffers->m_tileCounter);
        AttachBufferToSlot(AZ::Name("Counts"), buffers->m_counts);
    }

    void GaussianSplatRadixOnesweepPass::CompileResources(const AZ::RHI::FrameGraphCompileContext& context)
    {
        if (auto srg = GetShaderResourceGroup())
        {
            srg->SetConstant(m_roundIndex, m_round);
        }

        // Last: compiles the pass SRG, so the SetConstant above has to precede it.
        AZ::RPI::ComputePass::CompileResources(context);
    }
} // namespace GaussianSplat
