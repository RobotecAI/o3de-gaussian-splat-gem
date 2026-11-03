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

#include "Clients/GaussianSplatComponent.h"
#include "Clients/GaussianSplatComponentController.h"

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzToolsFramework/API/ComponentEntitySelectionBus.h>
#include <AzToolsFramework/Entity/EditorEntityInfoBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentAdapter.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <GaussianSplat/GaussianSplatTypeIds.h>

namespace GaussianSplat
{

    using GaussianSplatEditorComponentBase = AzToolsFramework::Components::
        EditorComponentAdapter<GaussianSplatComponentController, GaussianSplatComponent, GaussianSplatComponentConfig>;

    class GaussianSplatEditorComponent
        : public GaussianSplatEditorComponentBase
        , private AzToolsFramework::EditorEntityInfoNotificationBus::Handler
        , private AzFramework::EntityDebugDisplayEventBus::Handler
        , public AzToolsFramework::EditorComponentSelectionRequestsBus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(
            GaussianSplatEditorComponent, GaussianSplatEditorComponentTypeId, AzToolsFramework::Components::EditorComponentBase);
        GaussianSplatEditorComponent() = default;
        explicit GaussianSplatEditorComponent(const GaussianSplatComponentConfig& configuration);
        ~GaussianSplatEditorComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase interface overrides ...
        void Activate() override;
        void Deactivate() override;

        // AZ::Component overrides ...
        bool ReadInConfig(const AZ::ComponentConfig* baseConfig) override;

        // AzToolsFramework::EditorComponentSelectionRequestsBus overrides ...
        AZ::Aabb GetEditorSelectionBoundsViewport(const AzFramework::ViewportInfo& viewportInfo) override;
        bool EditorSelectionIntersectRayViewport(
            const AzFramework::ViewportInfo& viewportInfo, const AZ::Vector3& src, const AZ::Vector3& dir, float& distance) override;

        bool SupportsEditorRayIntersect() override;
        bool SupportsEditorRayIntersectViewport(const AzFramework::ViewportInfo& viewportInfo) override;

        // AzFramework::EntityDebugDisplayEventBus overrides ...
        void DisplayEntityViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

    private:
        // AzToolsFramework::EditorEntityInfoNotificationBus overrides ...
        void OnEntityInfoUpdatedVisibility(AZ::EntityId entityId, bool visible) override;

        //! Re-reads m_splatCount from the controller and asks the property grid to redraw with it.
        void RefreshSplatCount();
        //! EditContext visibility callback for the read-only Splat Count field.
        bool HasAssignedAsset() const;

        //! Number of splats in the currently loaded asset, for read-only display in the EditContext. Kept out of
        //! GaussianSplatComponentController since it's Editor-only display state, not data to persist.
        uint32_t m_splatCount = 0;
    };
} // namespace GaussianSplat
