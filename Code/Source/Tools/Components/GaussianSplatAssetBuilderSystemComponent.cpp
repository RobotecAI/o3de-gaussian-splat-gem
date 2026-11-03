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

#include "GaussianSplatAssetBuilderSystemComponent.h"
#include "Tools/Builders/GaussianSplatBuilder.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

namespace GaussianSplat
{
    // Required for forward declare of GaussianSplatAsset
    GaussianSplatAssetBuilderSystemComponent::GaussianSplatAssetBuilderSystemComponent() = default;
    GaussianSplatAssetBuilderSystemComponent::~GaussianSplatAssetBuilderSystemComponent() = default;

    void GaussianSplatAssetBuilderSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        GaussianSplatAsset::Reflect(context);
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GaussianSplatAssetBuilderSystemComponent, AZ::Component>()->Version(0)->Attribute(
                AZ::Edit::Attributes::SystemComponentTags, AssetBuilderSDK::ComponentTags::AssetBuilder);
        }
    }

    void GaussianSplatAssetBuilderSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GaussianSplatAssetBuilderService"));
    }

    void GaussianSplatAssetBuilderSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GaussianSplatAssetBuilderService"));
    }

    void GaussianSplatAssetBuilderSystemComponent::GetRequiredServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        // This doesn't require any services to exist before startup.
    }

    void GaussianSplatAssetBuilderSystemComponent::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        // If the asset services exist at all, they should be started first so that this builder can register
        // with them correctly.
        dependent.push_back(AZ_CRC_CE("AssetDatabaseService"));
        dependent.push_back(AZ_CRC_CE("AssetCatalogService"));
    }

    void GaussianSplatAssetBuilderSystemComponent::Activate()
    {
        m_gaussianSplatAssetHandler = AZStd::make_unique<GaussianSplatAssetHandler>();
        m_gaussianSplatAssetHandler->Register();
        m_gaussianSplatBuilder = AZStd::make_unique<GaussianSplatBuilder>();
    }

    void GaussianSplatAssetBuilderSystemComponent::Deactivate()
    {
        m_gaussianSplatAssetHandler->Unregister();
        m_gaussianSplatAssetHandler.reset();
        m_gaussianSplatBuilder.reset();
    }

} // namespace GaussianSplat
