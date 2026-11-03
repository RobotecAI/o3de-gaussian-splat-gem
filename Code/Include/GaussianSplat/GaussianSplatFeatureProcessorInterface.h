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

#include <Atom/RPI.Public/FeatureProcessor.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Math/Aabb.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/base.h>
#include <GaussianSplat/GaussianSplatAsset.h>
#include <GaussianSplat/GaussianSplatTypeIds.h>

namespace GaussianSplat
{
    // GaussianSplatFeatureProcessorInterface provides an interface to the feature processor for code outside of Atom
    class GaussianSplatFeatureProcessorInterface : public AZ::RPI::FeatureProcessor
    {
    public:
        using GaussianSplatHandle = int32_t;
        using GaussianSplatChangedEvent = AZ::Event<>;
        constexpr static GaussianSplatHandle InvalidGaussianSplatHandle = -1;
        AZ_RTTI(GaussianSplatFeatureProcessorInterface, GaussianSplatFeatureProcessorInterfaceTypeId, AZ::RPI::FeatureProcessor);

        virtual void SetTransform(const GaussianSplatHandle& handle, const AZ::Transform& transform) = 0;
        virtual void SetOpacityMultiplier(const GaussianSplatHandle& handle, float opacityMultiplier) = 0;
        virtual void SetScaleMultiplier(const GaussianSplatHandle& handle, float scaleMultiplier) = 0;

        //! Clamp spherical-harmonics evaluation to at most this degree.
        //! 0 renders flat (DC-only) color.
        virtual void SetMaxShDegree(const GaussianSplatHandle& handle, uint32_t maxShDegree) = 0;

        virtual GaussianSplatHandle AcquireGaussianSplatFromAsset(AZ::Data::Asset<GaussianSplatAsset> gaussianSplatAsset) = 0;
        virtual void SetVisibility(const GaussianSplatHandle& handle, bool visible) = 0;
        virtual void ReleaseGaussianSplat(const GaussianSplatHandle& handle) = 0;
        [[nodiscard]] virtual uint32_t GetSplatCount(const GaussianSplatHandle& handle) const = 0;
        [[nodiscard]] virtual AZStd::optional<AZ::Aabb> GetBounds(const GaussianSplatHandle& handle) const = 0;

        //! Connects a handler to load/reload changes to the cloud named by handle.
        virtual void ConnectChangeEventHandler(const GaussianSplatHandle& handle, GaussianSplatChangedEvent::Handler& handler) = 0;
    };
} // namespace GaussianSplat
