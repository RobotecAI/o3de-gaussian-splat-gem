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

#include "GaussianSplatComponentController.h"
#include <AzFramework/Components/ComponentAdapter.h>
#include <GaussianSplat/GaussianSplatTypeIds.h>

namespace GaussianSplat
{
    using GaussianSplatComponentBase =
        AzFramework::Components::ComponentAdapter<GaussianSplatComponentController, GaussianSplatComponentConfig>;

    class GaussianSplatComponent : public GaussianSplatComponentBase
    {
    public:
        AZ_COMPONENT(GaussianSplatComponent, GaussianSplatComponentTypeId);
        GaussianSplatComponent() = default;
        explicit GaussianSplatComponent(const GaussianSplatComponentConfig& config);
        ~GaussianSplatComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);

        // Component interface overrides ...
        void Activate() override;
        void Deactivate() override;
    };
} // namespace GaussianSplat
