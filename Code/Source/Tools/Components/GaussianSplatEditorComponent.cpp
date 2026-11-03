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

#include "GaussianSplatEditorComponent.h"

#include <AzCore/Component/TransformBus.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/Entity/EditorEntityInfoBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ViewportSelection/EditorSelectionUtil.h>

namespace GaussianSplat
{

    void GaussianSplatEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void GaussianSplatEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        GaussianSplatEditorComponentBase::Reflect(context);
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GaussianSplatEditorComponent, GaussianSplatEditorComponentBase>()->Version(0)->Field(
                "Splat Count", &GaussianSplatEditorComponent::m_splatCount);
            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<GaussianSplatEditorComponent>("Gaussian Splat", "Visualize a 3D Gaussian Splatting scene")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GaussianSplatEditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Graphics/Mesh")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GaussianSplatEditorComponent::m_splatCount,
                        "Splat Count",
                        "Number of splats in the currently loaded gaussian splat asset")
                    ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &GaussianSplatEditorComponent::HasAssignedAsset);
            }
        }
    }

    GaussianSplatEditorComponent::GaussianSplatEditorComponent(const GaussianSplatComponentConfig& configuration)
        : GaussianSplatEditorComponentBase(configuration)
    {
    }

    bool GaussianSplatEditorComponent::ReadInConfig(const AZ::ComponentConfig* baseConfig)
    {
        const auto* config = azrtti_cast<const GaussianSplatComponentConfig*>(baseConfig);
        if (config == nullptr)
        {
            AZ_Error("GaussianSplatEditorComponent", false, "Failed to set new configuration.");
            return false;
        }
        m_controller.SetConfiguration(*config);
        return true;
    }

    void GaussianSplatEditorComponent::Activate()
    {
        GaussianSplatEditorComponentBase::Activate();
        AzToolsFramework::EditorEntityInfoNotificationBus::Handler::BusConnect();
        bool visible = true;
        AzToolsFramework::EditorEntityInfoRequestBus::EventResult(
            visible, GetEntityId(), &AzToolsFramework::EditorEntityInfoRequestBus::Events::IsVisible);
        m_controller.SetVisibility(visible);
        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusConnect(GetEntityId());
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
        m_controller.SetOnAssetDataChanged(
            [this]()
            {
                RefreshSplatCount();
            });
        // Covers an asset that is already loaded: its change event fires inside the base Activate, ahead of
        // the callback above, so this is the only refresh it gets.
        RefreshSplatCount();
    }

    void GaussianSplatEditorComponent::Deactivate()
    {
        m_controller.SetOnAssetDataChanged(nullptr);
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        AzToolsFramework::EditorEntityInfoNotificationBus::Handler::BusDisconnect();
        AzToolsFramework::EditorComponentSelectionRequestsBus::Handler::BusDisconnect();
        GaussianSplatEditorComponentBase::Deactivate();
    }

    AZ::Aabb GaussianSplatEditorComponent::GetEditorSelectionBoundsViewport([[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo)
    {
        return m_controller.GetWorldBounds();
    }

    bool GaussianSplatEditorComponent::EditorSelectionIntersectRayViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, const AZ::Vector3& src, const AZ::Vector3& dir, float& distance)
    {
        return AzToolsFramework::AabbIntersectRay(src, dir, m_controller.GetWorldBounds(), distance);
    }

    bool GaussianSplatEditorComponent::SupportsEditorRayIntersect()
    {
        return true;
    }

    bool GaussianSplatEditorComponent::SupportsEditorRayIntersectViewport([[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo)
    {
        return true;
    }

    void GaussianSplatEditorComponent::OnEntityInfoUpdatedVisibility(AZ::EntityId entityId, bool visible)
    {
        m_controller.OnEntityInfoUpdatedVisibility(entityId, visible);
    }

    void GaussianSplatEditorComponent::RefreshSplatCount()
    {
        m_splatCount = m_controller.GetSplatCount();
        InvalidatePropertyDisplay(AzToolsFramework::Refresh_Values);
    }

    bool GaussianSplatEditorComponent::HasAssignedAsset() const
    {
        return m_controller.GetConfiguration().m_gaussianSplatAsset.GetId().IsValid();
    }

    void GaussianSplatEditorComponent::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        if (!IsSelected())
        {
            return;
        }

        const AZ::Aabb bounds = m_controller.GetLocalBounds();
        if (!bounds.IsValid())
        {
            return;
        }

        AZ::Transform worldTM = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTM, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        debugDisplay.PushMatrix(worldTM);
        debugDisplay.SetColor(AZ::Colors::White);
        debugDisplay.DrawWireBox(bounds.GetMin(), bounds.GetMax());
        debugDisplay.PopMatrix();
    }

} // namespace GaussianSplat
