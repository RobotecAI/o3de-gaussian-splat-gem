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

#include "GaussianSplatComponent.h"

namespace GaussianSplat
{
    GaussianSplatComponent::GaussianSplatComponent(const GaussianSplatComponentConfig& config)
        : GaussianSplatComponentBase(config)
    {
    }

    void GaussianSplatComponent::Reflect(AZ::ReflectContext* context)
    {
        GaussianSplatComponentBase::Reflect(context);
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GaussianSplatComponent, GaussianSplatComponentBase>()->Version(0);
        }
    }

    void GaussianSplatComponent::Activate()
    {
        GaussianSplatComponentBase::Activate();
    }

    void GaussianSplatComponent::Deactivate()
    {
        GaussianSplatComponentBase::Deactivate();
    }
} // namespace GaussianSplat
