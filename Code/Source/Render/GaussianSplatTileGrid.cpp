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

#include "GaussianSplatTileGrid.h"
#include <Atom/RPI.Public/Image/AttachmentImage.h>
#include <Atom/RPI.Public/Pass/ParentPass.h>
#include <Atom/RPI.Public/Pass/Pass.h>
#include <Atom/RPI.Public/Pass/PassAttachment.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <AzCore/Name/Name.h>

namespace GaussianSplat::TileGrid
{
    AZStd::optional<AZ::Vector2> TryGetResolvedFrameSize(AZ::RPI::Pass* gaussianSplatParentPass)
    {
        if (gaussianSplatParentPass == nullptr)
        {
            return AZStd::nullopt;
        }

        const AZ::RPI::PassAttachmentBinding* binding = gaussianSplatParentPass->FindAttachmentBinding(AZ::Name("ColorInputOutput"));
        if (binding == nullptr)
        {
            return AZStd::nullopt;
        }

        const AZ::RPI::Ptr<AZ::RPI::PassAttachment>& attachment = binding->GetAttachment();
        if (!attachment || attachment->m_descriptor.m_image.m_size.m_width == 0 || attachment->m_descriptor.m_image.m_size.m_height == 0)
        {
            return AZStd::nullopt;
        }

        return AZ::Vector2(
            aznumeric_cast<float>(attachment->m_descriptor.m_image.m_size.m_width),
            aznumeric_cast<float>(attachment->m_descriptor.m_image.m_size.m_height));
    }

    AZStd::optional<AZ::Vector2> TryGetPipelineFrameSize(AZ::RPI::RenderPipeline* pipeline)
    {
        if (pipeline == nullptr || !pipeline->GetRootPass())
        {
            return AZStd::nullopt;
        }

        for (const AZ::RPI::PassAttachmentBinding& binding : pipeline->GetRootPass()->GetAttachmentBindings())
        {
            if (binding.m_slotType != AZ::RPI::PassSlotType::Output && binding.m_slotType != AZ::RPI::PassSlotType::InputOutput)
            {
                continue;
            }
            const AZ::RPI::Ptr<AZ::RPI::PassAttachment>& attachment = binding.GetAttachment();
            if (!attachment || attachment->GetAttachmentType() != AZ::RHI::AttachmentType::Image)
            {
                continue;
            }

            // An imported attachment carries its size on the resource rather than on the descriptor, which for
            // a swap chain is the only place the real backbuffer extent appears.
            const AZ::RHI::ImageDescriptor imageDescriptor = attachment->m_importedResource
                ? static_cast<AZ::RPI::AttachmentImage*>(attachment->m_importedResource.get())->GetDescriptor()
                : attachment->m_descriptor.m_image;
            if (imageDescriptor.m_size.m_width == 0 || imageDescriptor.m_size.m_height == 0)
            {
                continue;
            }

            return AZ::Vector2(
                aznumeric_cast<float>(imageDescriptor.m_size.m_width), aznumeric_cast<float>(imageDescriptor.m_size.m_height));
        }
        return AZStd::nullopt;
    }

    AZStd::optional<AZ::Vector2> TryGetResolvedFrameSizeFromAncestor(AZ::RPI::Pass* pass)
    {
        for (AZ::RPI::Pass* current = pass; current != nullptr; current = current->GetParent())
        {
            if (auto resolved = TryGetResolvedFrameSize(current))
            {
                return resolved;
            }
        }
        return AZStd::nullopt;
    }

    AZ::Vector2 GetFrameSize(AZ::RPI::Pass* gaussianSplatParentPass)
    {
        if (auto resolved = TryGetResolvedFrameSizeFromAncestor(gaussianSplatParentPass))
        {
            return *resolved;
        }
        return AZ::Vector2(1920.0f, 1080.0f);
    }
} // namespace GaussianSplat::TileGrid
