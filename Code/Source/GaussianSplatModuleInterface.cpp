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

#include "GaussianSplatModuleInterface.h"
#include <AzCore/Memory/Memory.h>
#include <GaussianSplat/GaussianSplatTypeIds.h>

#include <Clients/GaussianSplatComponent.h>
#include <Clients/GaussianSplatSystemComponent.h>

namespace GaussianSplat
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(GaussianSplatModuleInterface, "GaussianSplatModuleInterface", GaussianSplatModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(GaussianSplatModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(GaussianSplatModuleInterface, AZ::SystemAllocator);

    GaussianSplatModuleInterface::GaussianSplatModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(),
            {
                GaussianSplatSystemComponent::CreateDescriptor(),
                GaussianSplatComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList GaussianSplatModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<GaussianSplatSystemComponent>(),
        };
    }
} // namespace GaussianSplat
