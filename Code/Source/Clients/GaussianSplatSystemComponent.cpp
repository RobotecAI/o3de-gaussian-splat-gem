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

#include "GaussianSplatSystemComponent.h"
#include "GaussianSplat/GaussianSplatTypeIds.h"
#include "Render/GaussianSplatFeatureProcessor.h"
#include "Render/GaussianSplatProjectCullGroupPass.h"
#include "Render/GaussianSplatProjectCullPass.h"
#include "Render/GaussianSplatRadixBuildKeysPass.h"
#include "Render/GaussianSplatRadixOnesweepPass.h"
#include "Render/GaussianSplatRadixPrepareBuildKeysDispatchPass.h"
#include "Render/GaussianSplatRadixPrepareSortDispatchPass.h"
#include "Render/GaussianSplatRadixUnpackPass.h"
#include "Render/GaussianSplatTileShadingPass.h"
#include <Atom/RPI.Public/FeatureProcessorFactory.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>

namespace GaussianSplat
{
    AZ_COMPONENT_IMPL(GaussianSplatSystemComponent, "GaussianSplatSystemComponent", GaussianSplatSystemComponentTypeId);

    void GaussianSplatSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        GaussianSplatFeatureProcessor::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GaussianSplatSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void GaussianSplatSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GaussianSplatService"));
    }

    void GaussianSplatSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GaussianSplatService"));
    }

    void GaussianSplatSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("RPISystem"));
    }

    void GaussianSplatSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void GaussianSplatSystemComponent::Init()
    {
    }

    void GaussianSplatSystemComponent::LoadPassTemplateMappings()
    {
        AZ::RPI::PassSystemInterface::Get()->LoadPassTemplateMappings("Passes/GaussianSplatPassTemplates.azasset");
    }

    void GaussianSplatSystemComponent::Activate()
    {
        // Register the feature processor before anyone requests it.
        AZ::RPI::FeatureProcessorFactory::Get()->RegisterFeatureProcessor<GaussianSplatFeatureProcessor>();

        auto* passSystem = AZ::RPI::PassSystemInterface::Get();
        AZ_Assert(passSystem, "PassSystemInterface is null");
        m_loadTemplatesHandler = AZ::RPI::PassSystemInterface::OnReadyLoadTemplatesEvent::Handler(
            []()
            {
                LoadPassTemplateMappings();
            });
        passSystem->ConnectEvent(m_loadTemplatesHandler);
        passSystem->AddPassCreator(AZ::Name("GaussianSplatTileShadingPass"), &GaussianSplatTileShadingPass::Create);
        passSystem->AddPassCreator(AZ::Name("GaussianSplatProjectCullPass"), &GaussianSplatProjectCullPass::Create);
        passSystem->AddPassCreator(AZ::Name("GaussianSplatProjectCullGroupPass"), &GaussianSplatProjectCullGroupPass::Create);
        passSystem->AddPassCreator(AZ::Name("GaussianSplatRadixBuildKeysPass"), &GaussianSplatRadixBuildKeysPass::Create);
        passSystem->AddPassCreator(
            AZ::Name("GaussianSplatRadixPrepareBuildKeysDispatchPass"), &GaussianSplatRadixPrepareBuildKeysDispatchPass::Create);
        passSystem->AddPassCreator(
            AZ::Name("GaussianSplatRadixPrepareSortDispatchPass"), &GaussianSplatRadixPrepareSortDispatchPass::Create);
        passSystem->AddPassCreator(AZ::Name("GaussianSplatRadixOnesweepPass"), &GaussianSplatRadixOnesweepPass::Create);
        passSystem->AddPassCreator(AZ::Name("GaussianSplatRadixUnpackPass"), &GaussianSplatRadixUnpackPass::Create);

        m_gaussianSplatAssetHandler = AZStd::make_unique<GaussianSplatAssetHandler>();
        m_gaussianSplatAssetHandler->Register();
    }

    void GaussianSplatSystemComponent::Deactivate()
    {
        m_gaussianSplatAssetHandler->Unregister();
        m_gaussianSplatAssetHandler.reset();

        m_loadTemplatesHandler.Disconnect();

        AZ::RPI::FeatureProcessorFactory::Get()->UnregisterFeatureProcessor<GaussianSplatFeatureProcessor>();
    }

} // namespace GaussianSplat
