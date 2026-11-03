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

#include "GaussianSplat/GaussianSplatConfigurationBus.h"
#include "GaussianSplat/GaussianSplatFeatureProcessorInterface.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Aabb.h>
#include <AzCore/std/functional.h>
#include <AzFramework/Visibility/BoundsBus.h>

namespace GaussianSplat
{
    //! Configuration for a GaussianSplat component.
    class GaussianSplatComponentConfig final : public AZ::ComponentConfig
    {
    public:
        AZ_RTTI(GaussianSplatComponentConfig, GaussianSplatComponentConfigTypeId);

        GaussianSplatComponentConfig() = default;
        ~GaussianSplatComponentConfig() override = default;

        static void Reflect(AZ::ReflectContext* context);

        //! Global opacity multiplier applied on top of each splat's stored opacity.
        float m_opacityMultiplier = 1.0f;

        //! Global scale multiplier applied on top of each splat's stored scale.
        float m_scaleMultiplier = 1.0f;

        //! Clamps SH evaluation to at most this degree; 0 renders flat (DC-only) color. Defaults to the maximum
        //! supported degree, i.e. unclamped (see GaussianSplatAsset::MaxShDegree).
        uint32_t m_maxShDegreeOverride = GaussianSplatAsset::MaxShDegree;

        //! Runtime handle to the gaussian splat resource managed by the feature processor.
        GaussianSplatFeatureProcessorInterface::GaussianSplatHandle m_gaussianSplatHandle =
            GaussianSplatFeatureProcessorInterface::InvalidGaussianSplatHandle;

        //! Asset reference to the gaussian splat data to be loaded and rendered.
        AZ::Data::Asset<GaussianSplatAsset> m_gaussianSplatAsset;
    };

    class GaussianSplatComponentController
        : public GaussianSplatConfigurationBus::Handler
        , public AzFramework::BoundsRequestBus::Handler
        , private AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_TYPE_INFO(GaussianSplatComponentController, GaussianSplatComponentControllerTypeId);
        GaussianSplatComponentController();
        explicit GaussianSplatComponentController(const GaussianSplatComponentConfig& config);
        ~GaussianSplatComponentController() override = default;
        //! Not copyable or movable: m_changeEventHandler's callback is bound to this instance.
        GaussianSplatComponentController(const GaussianSplatComponentController&) = delete;
        GaussianSplatComponentController& operator=(const GaussianSplatComponentController&) = delete;
        GaussianSplatComponentController(GaussianSplatComponentController&&) = delete;
        GaussianSplatComponentController& operator=(GaussianSplatComponentController&&) = delete;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // Controller component
        void Activate(AZ::EntityId entityId);
        void Deactivate();
        void SetConfiguration(const GaussianSplatComponentConfig& config);
        const GaussianSplatComponentConfig& GetConfiguration() const;

        // AzFramework::BoundsRequestBus overrides ...
        AZ::Aabb GetWorldBounds() const override;
        AZ::Aabb GetLocalBounds() const override;

        // AZ::TransformNotificationBus::Handler overrides ...
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        // AzToolsFramework::EditorEntityInfoNotificationBus overrides ...
        void OnEntityInfoUpdatedVisibility(AZ::EntityId entityId, bool visible);

        // GaussianSplatConfigurationBus::Handler overrides ...
        void SetGaussianSplatAsset(AZ::Data::Asset<GaussianSplatAsset> asset) override;
        void SetOpacityMultiplier(float opacityMultiplier) override;
        void SetScaleMultiplier(float scaleMultiplier) override;
        void SetMaxShDegree(uint32_t maxShDegree) override;
        void SetVisibility(bool visible) override;
        AZ::Data::Asset<GaussianSplatAsset> GetGaussianSplatAsset() const override;
        float GetOpacityMultiplier() const override;
        float GetScaleMultiplier() const override;
        uint32_t GetMaxShDegree() const override;
        bool GetVisibility() const override;

        uint32_t GetSplatCount() const;

        //! Invoked when the loaded asset's data changes outside a property-grid edit (e.g. async load/reload).
        //! Set by the Editor component to request a property grid refresh, since this class can't include AzToolsFramework itself.
        void SetOnAssetDataChanged(AZStd::function<void()> callback)
        {
            m_onAssetDataChanged = AZStd::move(callback);
        }

    private:
        AZ::Crc32 OnAssetChanged();

        void HandleChange();

        AZ::EntityId m_entityId;
        GaussianSplatFeatureProcessorInterface* m_featureProcessor = nullptr;
        GaussianSplatComponentConfig m_config;
        bool m_visibility = true;
        AZStd::function<void()> m_onAssetDataChanged;
        GaussianSplatFeatureProcessorInterface::GaussianSplatChangedEvent::Handler m_changeEventHandler;
    };
} // namespace GaussianSplat
