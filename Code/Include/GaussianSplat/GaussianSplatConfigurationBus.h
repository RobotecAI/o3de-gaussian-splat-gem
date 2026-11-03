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

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/EBus/EBus.h>
#include <GaussianSplat/GaussianSplatAsset.h>
#include <GaussianSplat/GaussianSplatTypeIds.h>

namespace GaussianSplat
{
    class GaussianSplatConfigurationRequests : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(GaussianSplatConfigurationRequests, GaussianSplatConfigurationRequestsTypeId);
        ~GaussianSplatConfigurationRequests() override = default;

        //! Set the Asset to be used by the GaussianSplatComponent.
        virtual void SetGaussianSplatAsset(AZ::Data::Asset<GaussianSplatAsset> asset) = 0;

        //! Retrieve the current Asset used by the GaussianSplatComponent.
        [[nodiscard]] virtual AZ::Data::Asset<GaussianSplatAsset> GetGaussianSplatAsset() const = 0;

        //! Set the opacity multiplier for the GaussianSplat rendering.
        //! @note default value is 1.0
        virtual void SetOpacityMultiplier(float opacityMultiplier) = 0;

        //! Retrieve the current opacity multiplier.
        [[nodiscard]] virtual float GetOpacityMultiplier() const = 0;

        //! Set the scale multiplier for the GaussianSplat rendering.
        //
        //! This is different than transform scale as it is a per-splat basis multiplier
        //! and does not affect positions between splats in the same asset.
        virtual void SetScaleMultiplier(float scaleMultiplier) = 0;

        //! Retrieve the current scale multiplier
        [[nodiscard]] virtual float GetScaleMultiplier() const = 0;

        //! Limit the the degree of the spherical harmonic (SH) expansion
        //! that will be used for rendering.
        virtual void SetMaxShDegree(uint32_t maxShDegree) = 0;

        //! Retrieve the current max SH degree.
        [[nodiscard]] virtual uint32_t GetMaxShDegree() const = 0;

        //! Set whether the Gaussian Splat should be visible.
        virtual void SetVisibility(bool visible) = 0;

        //! Retrieve the current visibility setting.
        [[nodiscard]] virtual bool GetVisibility() const = 0;
    };

    using GaussianSplatConfigurationBus = AZ::EBus<GaussianSplatConfigurationRequests>;
} // namespace GaussianSplat
