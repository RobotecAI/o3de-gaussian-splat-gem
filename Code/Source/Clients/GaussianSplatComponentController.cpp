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

#include "GaussianSplatComponentController.h"
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Visibility/EntityBoundsUnionBus.h>
#include <cstdint>

namespace GaussianSplat
{
    void GaussianSplatComponentConfig::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GaussianSplatComponentConfig, AZ::ComponentConfig>()
                ->Version(0)
                ->Field("GaussianSplatAsset", &GaussianSplatComponentConfig::m_gaussianSplatAsset)
                ->Field("Scale Multiplier", &GaussianSplatComponentConfig::m_scaleMultiplier)
                ->Field("Max SH Degree Override", &GaussianSplatComponentConfig::m_maxShDegreeOverride)
                ->Field("Opacity Multiplier", &GaussianSplatComponentConfig::m_opacityMultiplier);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<GaussianSplatComponentConfig>("GaussianSplatComponentConfig", "Config for GaussianSplatComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GaussianSplatComponentConfig::m_gaussianSplatAsset,
                        "Gaussian Splat Asset",
                        "Asset containing the gaussian splat cloud data")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GaussianSplatComponentConfig::m_scaleMultiplier,
                        "Scale Multiplier",
                        "Grows or shrinks each splat's individual blob size in place, without moving splat "
                        "positions or changing the cloud's overall extent (unlike the entity Transform's scale, "
                        "which resizes the whole cloud including splat spacing)")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 10.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Slider,
                        &GaussianSplatComponentConfig::m_maxShDegreeOverride,
                        "Max SH Degree",
                        "Clamp spherical-harmonics evaluation to at most this degree; 0 renders flat (DC-only) "
                        "color, higher values allow more directional detail")
                    ->Attribute(AZ::Edit::Attributes::Max, GaussianSplatAsset::MaxShDegree)
                    ->Attribute(AZ::Edit::Attributes::Step, 1)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Slider,
                        &GaussianSplatComponentConfig::m_opacityMultiplier,
                        "Opacity Multiplier",
                        "Global opacity multiplier applied on top of each splat's stored opacity")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 1.0f);
            }
        }
    }

    // Every constructor binds the change handler to this instance. Serialization and "Add Component" build the
    // controller through the default constructor, and AZ::EventHandler refuses to connect a handler without a
    // callback, so one left unbound would never see an async load finish.
    GaussianSplatComponentController::GaussianSplatComponentController()
        : m_changeEventHandler(
              [this]()
              {
                  HandleChange();
              })
    {
    }

    GaussianSplatComponentController::GaussianSplatComponentController(const GaussianSplatComponentConfig& config)
        : GaussianSplatComponentController()
    {
        m_config = config;
    }

    void GaussianSplatComponentController::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void GaussianSplatComponentController::SetConfiguration(const GaussianSplatComponentConfig& config)
    {
        m_config = config;
        OnAssetChanged();
    }

    const GaussianSplatComponentConfig& GaussianSplatComponentController::GetConfiguration() const
    {
        return m_config;
    }

    void GaussianSplatComponentController::Reflect(AZ::ReflectContext* context)
    {
        GaussianSplatComponentConfig::Reflect(context);
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GaussianSplatComponentController>()->Version(0)->Field(
                "Configuration", &GaussianSplatComponentController::m_config);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<GaussianSplatComponentController>("GaussianSplatComponentController", "GaussianSplatComponentController")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GaussianSplatComponentController")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GaussianSplatComponentController::m_config,
                        "Configuration",
                        "Configuration of the gaussian splat cloud")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &GaussianSplatComponentController::OnAssetChanged);
            }
        }
    }

    void GaussianSplatComponentController::Activate(AZ::EntityId entityId)
    {
        m_entityId = entityId;
        m_featureProcessor = AZ::RPI::Scene::GetFeatureProcessorForEntity<GaussianSplatFeatureProcessorInterface>(m_entityId);
        AZ_Assert(m_featureProcessor, "Failed to get Gaussian Splat Feature Processor for entity.");
        AZ::TransformNotificationBus::Handler::BusConnect(m_entityId);
        GaussianSplatConfigurationBus::Handler::BusConnect(m_entityId);
        AzFramework::BoundsRequestBus::Handler::BusConnect(m_entityId);
        OnAssetChanged();
    }

    void GaussianSplatComponentController::Deactivate()
    {
        m_changeEventHandler.Disconnect();
        AzFramework::BoundsRequestBus::Handler::BusDisconnect();
        GaussianSplatConfigurationBus::Handler::BusDisconnect();
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        if (m_featureProcessor != nullptr)
        {
            m_featureProcessor->ReleaseGaussianSplat(m_config.m_gaussianSplatHandle);
        }
    }

    AZ::Crc32 GaussianSplatComponentController::OnAssetChanged()
    {
        if (m_featureProcessor != nullptr)
        {
            m_changeEventHandler.Disconnect();
            m_featureProcessor->ReleaseGaussianSplat(m_config.m_gaussianSplatHandle);
            m_config.m_gaussianSplatHandle = GaussianSplatFeatureProcessorInterface::InvalidGaussianSplatHandle;
            if (m_config.m_gaussianSplatAsset.GetId().IsValid())
            {
                m_config.m_gaussianSplatHandle = m_featureProcessor->AcquireGaussianSplatFromAsset(m_config.m_gaussianSplatAsset);
                m_featureProcessor->ConnectChangeEventHandler(m_config.m_gaussianSplatHandle, m_changeEventHandler);
                AZ::Transform transform = AZ::Transform::CreateIdentity();
                AZ::TransformBus::EventResult(transform, m_entityId, &AZ::TransformBus::Events::GetWorldTM);
                m_featureProcessor->SetTransform(m_config.m_gaussianSplatHandle, transform);
                m_featureProcessor->SetOpacityMultiplier(m_config.m_gaussianSplatHandle, m_config.m_opacityMultiplier);
                m_featureProcessor->SetScaleMultiplier(m_config.m_gaussianSplatHandle, m_config.m_scaleMultiplier);
                m_featureProcessor->SetMaxShDegree(m_config.m_gaussianSplatHandle, m_config.m_maxShDegreeOverride);
                m_featureProcessor->SetVisibility(m_config.m_gaussianSplatHandle, m_visibility);
            }
        }
        if (m_onAssetDataChanged)
        {
            m_onAssetDataChanged();
        }
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    void GaussianSplatComponentController::HandleChange()
    {
        AZ::Interface<AzFramework::IEntityBoundsUnion>::Get()->RefreshEntityLocalBoundsUnion(m_entityId);

        // Asset load/reload is async, so this is the only point that observes final data (e.g. splat count);
        // uses m_onAssetDataChanged since this class is shared with the runtime component and can't include AzToolsFramework.
        if (m_onAssetDataChanged)
        {
            m_onAssetDataChanged();
        }
    }

    uint32_t GaussianSplatComponentController::GetSplatCount() const
    {
        return m_featureProcessor ? m_featureProcessor->GetSplatCount(m_config.m_gaussianSplatHandle) : 0;
    }

    void GaussianSplatComponentController::OnEntityInfoUpdatedVisibility(AZ::EntityId entityId, bool visible)
    {
        if (entityId == m_entityId)
        {
            // Through SetVisibility so m_visibility follows too: OnAssetChanged re-applies it to every
            // re-acquired cloud, and a stale value would show a cloud the editor has hidden.
            SetVisibility(visible);
        }
    }

    AZ::Aabb GaussianSplatComponentController::GetWorldBounds() const
    {
        AZ::Transform transform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(transform, m_entityId, &AZ::TransformBus::Events::GetWorldTM);
        AZ::Aabb bounds = GetLocalBounds();
        if (bounds.IsValid())
        {
            bounds.ApplyTransform(transform);
        }
        return bounds;
    }

    AZ::Aabb GaussianSplatComponentController::GetLocalBounds() const
    {
        if (m_featureProcessor == nullptr)
        {
            return AZ::Aabb::CreateNull();
        }
        auto bounds = m_featureProcessor->GetBounds(m_config.m_gaussianSplatHandle);
        if (bounds.has_value())
        {
            return bounds.value();
        }
        return AZ::Aabb::CreateNull();
    }

    void GaussianSplatComponentController::OnTransformChanged([[maybe_unused]] const AZ::Transform& local, const AZ::Transform& world)
    {
        if (m_featureProcessor != nullptr)
        {
            m_featureProcessor->SetTransform(m_config.m_gaussianSplatHandle, world);
        }
    }

    void GaussianSplatComponentController::SetGaussianSplatAsset(AZ::Data::Asset<GaussianSplatAsset> asset)
    {
        m_config.m_gaussianSplatAsset = asset;
        OnAssetChanged();
    }

    void GaussianSplatComponentController::SetOpacityMultiplier(float opacityMultiplier)
    {
        m_config.m_opacityMultiplier = AZStd::clamp(opacityMultiplier, 0.0f, 1.0f);
        if (m_featureProcessor != nullptr)
        {
            m_featureProcessor->SetOpacityMultiplier(m_config.m_gaussianSplatHandle, m_config.m_opacityMultiplier);
        }
    }

    void GaussianSplatComponentController::SetScaleMultiplier(float scaleMultiplier)
    {
        m_config.m_scaleMultiplier = scaleMultiplier;
        if (m_featureProcessor != nullptr)
        {
            m_featureProcessor->SetScaleMultiplier(m_config.m_gaussianSplatHandle, m_config.m_scaleMultiplier);
        }
    }

    void GaussianSplatComponentController::SetMaxShDegree(uint32_t maxShDegree)
    {
        m_config.m_maxShDegreeOverride = AZStd::clamp(maxShDegree, uint32_t{ 0 }, GaussianSplatAsset::MaxShDegree);
        if (m_featureProcessor != nullptr)
        {
            m_featureProcessor->SetMaxShDegree(m_config.m_gaussianSplatHandle, m_config.m_maxShDegreeOverride);
        }
    }

    void GaussianSplatComponentController::SetVisibility(bool visible)
    {
        m_visibility = visible;
        if (m_featureProcessor != nullptr)
        {
            m_featureProcessor->SetVisibility(m_config.m_gaussianSplatHandle, visible);
        }
    }

    AZ::Data::Asset<GaussianSplatAsset> GaussianSplatComponentController::GetGaussianSplatAsset() const
    {
        return m_config.m_gaussianSplatAsset;
    }

    float GaussianSplatComponentController::GetOpacityMultiplier() const
    {
        return m_config.m_opacityMultiplier;
    }

    float GaussianSplatComponentController::GetScaleMultiplier() const
    {
        return m_config.m_scaleMultiplier;
    }

    uint32_t GaussianSplatComponentController::GetMaxShDegree() const
    {
        return m_config.m_maxShDegreeOverride;
    }

    bool GaussianSplatComponentController::GetVisibility() const
    {
        return m_visibility;
    }
} // namespace GaussianSplat
