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

#include "GaussianSplatProjectCullGroupPass.h"
#include "GaussianSplatFeatureProcessor.h"
#include "GaussianSplatProjectCullPass.h"
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <Atom/RPI.Reflect/Pass/PassRequest.h>
#include <Atom/RPI.Reflect/Pass/PassTemplate.h>
#include <AzCore/std/string/string.h>

namespace GaussianSplat
{
    AZ::RPI::Ptr<GaussianSplatProjectCullGroupPass> GaussianSplatProjectCullGroupPass::Create(const AZ::RPI::PassDescriptor& descriptor)
    {
        return aznew GaussianSplatProjectCullGroupPass(descriptor);
    }

    GaussianSplatProjectCullGroupPass::GaussianSplatProjectCullGroupPass(const AZ::RPI::PassDescriptor& descriptor)
        : AZ::RPI::ParentPass(descriptor)
    {
    }

    void GaussianSplatProjectCullGroupPass::QueueRebuildChildren()
    {
        m_flags.m_createChildren = true;
        QueueForBuildAndInitialization();
    }

    void GaussianSplatProjectCullGroupPass::BuildInternal()
    {
        // Ahead of the base call, which builds the children that resolve "Parent" against this binding.
        // SetupOutputsFromTemplate runs later, but declares nothing for this slot, so it overwrites nothing.
        if (const auto* buffers = GetPipelineBuffers(GetRenderPipeline()))
        {
            AttachBufferToSlot(AZ::Name("SplatProjectionOut"), buffers->m_splatProjection);
        }

        AZ::RPI::ParentPass::BuildInternal();
    }

    void GaussianSplatProjectCullGroupPass::CreateChildPassesInternal()
    {
        auto* featureProcessor = GaussianSplatFeatureProcessor::GetInstance(GetRenderPipeline());
        if (featureProcessor == nullptr)
        {
            return;
        }

        const AZStd::shared_ptr<const AZ::RPI::PassTemplate> cullTemplate =
            AZ::RPI::PassSystemInterface::Get()->GetPassTemplate(AZ::Name("GaussianSplatProjectCullPassTemplate"));
        if (!cullTemplate)
        {
            AZ_Error(
                "GaussianSplatProjectCullGroupPass",
                false,
                "GaussianSplatProjectCullPassTemplate is not registered - cannot create per-cloud cull passes.");
            return;
        }

        // Constructed here rather than as file-scope statics: AZ::Name needs the name dictionary, which does
        // not exist yet during static initialization.
        const AZ::Name parentPassName{ "Parent" };
        const AZ::Name clearPassName{ "GaussianSplatVisibleCounterClear" };
        const AZ::Name splatProjectionSlotName{ "SplatProjectionOut" };
        const AZ::Name cullVisibleCounterSlotName{ "VisibleCounter" };
        const AZ::Name clearVisibleCounterSlotName{ "VisibleCounterOut" };

        for (const auto handle : featureProcessor->GetCloudHandles())
        {
            // A hidden cloud gets no pass, so nothing holds a buffer view keeping its source buffers alive.
            if (!featureProcessor->IsCloudVisible(handle))
            {
                continue;
            }

            // Every cull pass appends into the one merged buffer set the downstream sort chain consumes.
            // Projection comes from this group's own slot; the counter from the clear sibling, which owns it
            // and, as children[0], is built first. The group's own VisibleCounterOut is unusable here -
            // SetupOutputsFromTemplate fills it only after every child is built.
            AZ::RPI::PassRequest childRequest;
            childRequest.m_templateName = AZ::Name("GaussianSplatProjectCullPassTemplate");
            childRequest.m_passName = AZ::Name(AZStd::string::format("GaussianSplatProjectCull.%d", handle));

            AZ::RPI::PassConnection connection;

            connection.m_attachmentRef.m_pass = parentPassName;
            connection.m_localSlot = splatProjectionSlotName;
            connection.m_attachmentRef.m_attachment = splatProjectionSlotName;
            childRequest.m_connections.emplace_back(connection);

            connection.m_attachmentRef.m_pass = clearPassName;
            connection.m_localSlot = cullVisibleCounterSlotName;
            connection.m_attachmentRef.m_attachment = clearVisibleCounterSlotName;
            childRequest.m_connections.emplace_back(connection);

            const AZ::RPI::PassDescriptor childDescriptor{ childRequest.m_passName, cullTemplate, &childRequest };
            AZ::RPI::Ptr<GaussianSplatProjectCullPass> cullPass = GaussianSplatProjectCullPass::Create(childDescriptor);
            if (!cullPass)
            {
                continue;
            }
            cullPass->SetCloudHandle(handle);
            AddChild(cullPass);
        }
    }
} // namespace GaussianSplat
