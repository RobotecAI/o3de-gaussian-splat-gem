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

#include "Components/GaussianSplatAssetBuilderSystemComponent.h"
#include "Components/GaussianSplatEditorComponent.h"
#include "GaussianSplatEditorSystemComponent.h"
#include <GaussianSplat/GaussianSplatTypeIds.h>
#include <GaussianSplatModuleInterface.h>

namespace GaussianSplat
{
    class GaussianSplatEditorModule : public GaussianSplatModuleInterface
    {
    public:
        AZ_RTTI(GaussianSplatEditorModule, GaussianSplatEditorModuleTypeId, GaussianSplatModuleInterface);
        AZ_CLASS_ALLOCATOR(GaussianSplatEditorModule, AZ::SystemAllocator);

        GaussianSplatEditorModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    GaussianSplatEditorSystemComponent::CreateDescriptor(),
                    GaussianSplatEditorComponent::CreateDescriptor(),
                    GaussianSplatAssetBuilderSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<GaussianSplatEditorSystemComponent>(),
            };
        }
    };
} // namespace GaussianSplat

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), GaussianSplat::GaussianSplatEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_GaussianSplat_Editor, GaussianSplat::GaussianSplatEditorModule)
#endif
