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

#include "GaussianSplatEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <GaussianSplat/GaussianSplatTypeIds.h>

namespace GaussianSplat
{
    AZ_COMPONENT_IMPL(
        GaussianSplatEditorSystemComponent,
        "GaussianSplatEditorSystemComponent",
        GaussianSplatEditorSystemComponentTypeId,
        BaseSystemComponent);

    void GaussianSplatEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GaussianSplatEditorSystemComponent, GaussianSplatSystemComponent>()->Version(0);
        }
    }

    GaussianSplatEditorSystemComponent::GaussianSplatEditorSystemComponent() = default;

    GaussianSplatEditorSystemComponent::~GaussianSplatEditorSystemComponent() = default;

    void GaussianSplatEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("GaussianSplatSystemEditorService"));
    }

    void GaussianSplatEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("GaussianSplatSystemEditorService"));
    }

    void GaussianSplatEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void GaussianSplatEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void GaussianSplatEditorSystemComponent::Activate()
    {
        GaussianSplatSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void GaussianSplatEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        GaussianSplatSystemComponent::Deactivate();
    }

} // namespace GaussianSplat
