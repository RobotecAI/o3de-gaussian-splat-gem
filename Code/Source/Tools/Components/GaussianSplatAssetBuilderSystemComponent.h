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

#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <GaussianSplat/GaussianSplatAsset.h>
#include <GaussianSplat/GaussianSplatTypeIds.h>

namespace GaussianSplat
{
    class GaussianSplatBuilder;

    class GaussianSplatAssetBuilderSystemComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(GaussianSplatAssetBuilderSystemComponent, GaussianSplatAssetBuilderSystemComponentTypeId);
        static void Reflect(AZ::ReflectContext* context);

        GaussianSplatAssetBuilderSystemComponent();
        ~GaussianSplatAssetBuilderSystemComponent() override;

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // Component overrides ...
        void Activate() override;
        void Deactivate() override;

        AZStd::unique_ptr<GaussianSplatBuilder> m_gaussianSplatBuilder;

    private:
        AZStd::unique_ptr<GaussianSplatAssetHandler> m_gaussianSplatAssetHandler;
    };
} // namespace GaussianSplat
