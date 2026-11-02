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

#include "GaussianSplatTypeIds.h"
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Math/Aabb.h>
#include <AzFramework/Asset/GenericAssetHandler.h>

namespace GaussianSplat
{
    class GaussianSplatAsset final : public AZ::Data::AssetData
    {
    public:
        //! Maximum spherical-harmonics degree the runtime vertex layout can hold (matches the 3DGS reference: degree 3).
        static constexpr uint32_t MaxShDegree = 3;
        //! 15 "rest" coefficients per channel x 3 channels (R, G, B) at degree 3 - the widest m_shRest stride.
        static constexpr uint32_t MaxShRestCoeffs = 45;

        //! Per-splat data, at a fixed 56-byte stride independent of SH degree. Degree 1-3 "rest" coefficients
        //! live in the asset's own m_shRest array, keeping this struct at exactly 56 bytes.
        struct CloudVertex
        {
            AZ_TYPE_INFO(CloudVertex, SplatVertexAssetTypeId);

            static void Reflect(AZ::ReflectContext* context);

            AZStd::array<float, 3> m_position{};
            AZStd::array<float, 4> m_rotation{}; //!< normalized quaternion (x, y, z, w)
            AZStd::array<float, 3> m_scale{}; //!< linear (post-exp) scale
            float m_opacity = 0.0f; //!< linear (post-sigmoid) opacity, [0, 1]
            AZStd::array<float, 3> m_shDc{}; //!< degree-0 (DC) SH coefficient, one per color channel (R, G, B)
        };

        struct CloudHeader
        {
            uint32_t m_magicNumber{ 0x0 };
            uint32_t m_elementSize{ 0x0 };
            uint32_t m_numPoints{ 0x0 };
            uint32_t m_shDegree{ 0x0 }; //!< SH degree (0-3) actually populated by the source asset
        };

        static constexpr uint32_t GaussianSplatMagicNumber = 0x67735054; // 'gsPT'

        static constexpr inline const char* DisplayName = "GaussianSplatAsset";
        static constexpr inline const char* Extension = "gsplat";
        static constexpr inline const char* Group = "GaussianSplat";

        AZ_RTTI(GaussianSplatAsset, GaussianSplatAssetTypeId, AZ::Data::AssetData)
        AZ_CLASS_ALLOCATOR(GaussianSplatAsset, AZ::SystemAllocator);

        static void Reflect(AZ::ReflectContext* context);

        AZStd::vector<CloudVertex> m_data;
        //! Degree 1-3 "rest" SH coefficients for every splat, flat and tightly packed at
        //! ShRestStride() coefficients per splat, each one an IEEE-754 binary16 bit pattern (see
        //! Math::FloatToHalf in Utils/GaussianSplatMath.h). Within a splat the layout is channel-major,
        //! [R_0..R_(k-1), G_0..G_(k-1), B_0..B_(k-1)] where k = Math::ShRestCoeffsPerChannel(m_shDegree),
        //! matching the .ply's own f_rest_0.. ordering so GaussianSplatBuilder copies it across in order.
        //!
        //! Half precision: these coefficients are small, bounded residuals on top of the DC term, and
        //! binary16's ~3 significant decimal digits sit well below the 8-bit-per-channel color the shading
        //! pass resolves to. The DC term itself stays at full precision in CloudVertex::m_shDc.
        AZStd::vector<uint16_t> m_shRest;
        uint32_t m_shDegree = 0; //!< SH degree (0-3) actually populated across m_data
        //! Local-space bounds of every splat center in m_data.
        AZ::Aabb m_bounds = AZ::Aabb::CreateNull();

        //! "Rest" coefficients this asset stores per splat: the stride of m_shRest, and zero at degree 0.
        //! Out of line because it is Math::ShRestCoeffsPerSplat, which this gem keeps private to Source.
        [[nodiscard]] uint32_t ShRestStride() const;

        //! Pointer to splat index's run of ShRestStride() coefficients in m_shRest, or nullptr at degree 0
        //! (where m_shRest is empty) or if index is out of range.
        [[nodiscard]] const uint16_t* ShRestForSplat(size_t index) const
        {
            const size_t stride = ShRestStride();
            const size_t offset = index * stride;
            return (stride != 0 && offset + stride <= m_shRest.size()) ? m_shRest.data() + offset : nullptr;
        }
    };

    class GaussianSplatAssetHandler final : public AzFramework::GenericAssetHandler<GaussianSplatAsset>
    {
    public:
        GaussianSplatAssetHandler();

        // AZ::Data::AssetHandler overrides...
        AZ::Data::AssetHandler::LoadResult LoadAssetData(
            const AZ::Data::Asset<AZ::Data::AssetData>& asset,
            AZStd::shared_ptr<AZ::Data::AssetDataStream> stream,
            const AZ::Data::AssetFilterCB& assetLoadFilterCB) override;
        bool SaveAssetData(const AZ::Data::Asset<AZ::Data::AssetData>& asset, AZ::IO::GenericStream* stream) override;
    };
} // namespace GaussianSplat
