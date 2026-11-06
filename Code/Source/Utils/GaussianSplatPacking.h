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

#include <AzCore/std/containers/vector.h>
#include <GaussianSplat/GaussianSplatAsset.h>
#include <cstdint>

// CPU-side packing of a GaussianSplatAsset's geometry and spherical-harmonics coefficients into the buffer
// layouts the splat shaders read.
namespace GaussianSplat::Packing
{
    //! Floats one splat occupies in the geometry buffer. Matches SplatGeometry in GaussianSplatCommon.azsli
    //! exactly: float3 position, float opacity, float4 rotation, float3 scale, float padding - every float3
    //! followed by one scalar, so the struct needs no explicit HLSL packing rules to line up.
    inline constexpr uint32_t GeometryFloatsPerSplat = 12;

    //! Byte stride of one packed SplatGeometry element, and so the geometry buffer's element size.
    inline constexpr uint32_t GeometryElementSize = sizeof(float) * GeometryFloatsPerSplat;

    //! Deinterleaves splatVertices into the geometry buffer the splat shaders read: GeometryFloatsPerSplat
    //! floats per splat, in SplatGeometry's field order, with the trailing pad left at zero.
    AZStd::vector<float> PackCloudGeometry(const AZStd::vector<GaussianSplatAsset::CloudVertex>& splatVertices);

    //! binary16 values in one splat's SH run at this degree: an (R, G, B) triple for the DC term plus one
    //! per "rest" coefficient per channel. Matches GaussianSplatProjectCull.azsl's ShElement addressing.
    uint32_t ShHalvesPerSplat(uint32_t shDegree);

    //! uints one splat's SH run occupies: two binary16 values per uint, rounded up so each splat's run
    //! starts on a uint boundary. This is the shader's m_shUintsPerSplat stride.
    uint32_t ShUintsPerSplat(uint32_t shDegree);

    //! Writes one binary16 value into a splat's uint run at halfIndex, low half of a uint first - the
    //! layout GaussianSplatProjectCull.azsl's ShHalf reads back.
    void WriteShHalf(uint32_t* splatRun, uint32_t halfIndex, uint16_t value);

    //! Packs every splat of asset into the SH buffer the cull shader reads: ShUintsPerSplat
    //! (uploadDegree) uints per splat, holding the DC triple followed by the "rest" coefficients transposed
    //! from the asset's channel-major blocks into coefficient-major, RGB-interleaved order.
    //!
    //! uploadDegree is the degree to pack at.
    //! The source stride stays the asset's own m_shDegree throughout, since that - not uploadDegree -
    //! is what m_shRest's channel-major blocks are laid out for; reading with uploadDegree as the stride
    //! would pull the wrong coefficients out of the G and B blocks.
    //!
    //! The coefficients are already binary16 in the asset, so this moves them without converting; only the
    //! DC term (full precision in CloudVertex::m_shDc) is narrowed here.
    AZStd::vector<uint32_t> PackCloudSh(const GaussianSplatAsset& asset, uint32_t uploadDegree);
} // namespace GaussianSplat::Packing
