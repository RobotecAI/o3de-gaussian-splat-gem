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

#include "GaussianSplatPacking.h"
#include "GaussianSplatMath.h"
#include <GaussianSplat/GaussianSplatAsset.h>

namespace GaussianSplat::Packing
{
    AZStd::vector<float> PackCloudGeometry(const AZStd::vector<GaussianSplatAsset::CloudVertex>& splatVertices)
    {
        const auto splatCount = static_cast<uint32_t>(splatVertices.size());
        AZStd::vector<float> packed(static_cast<size_t>(splatCount) * GeometryFloatsPerSplat, 0.0f);
        for (uint32_t splat = 0; splat < splatCount; ++splat)
        {
            const auto& vertex = splatVertices[splat];
            float* out = packed.data() + static_cast<size_t>(splat) * GeometryFloatsPerSplat;
            out[0] = vertex.m_position[0];
            out[1] = vertex.m_position[1];
            out[2] = vertex.m_position[2];
            out[3] = vertex.m_opacity;
            out[4] = vertex.m_rotation[0];
            out[5] = vertex.m_rotation[1];
            out[6] = vertex.m_rotation[2];
            out[7] = vertex.m_rotation[3];
            out[8] = vertex.m_scale[0];
            out[9] = vertex.m_scale[1];
            out[10] = vertex.m_scale[2];
            // out[11] is SplatGeometry's trailing pad, left at zero.
        }

        return packed;
    }

    uint32_t ShHalvesPerSplat(uint32_t shDegree)
    {
        return 3 * (1 + Math::ShRestCoeffsPerChannel(shDegree));
    }

    uint32_t ShUintsPerSplat(uint32_t shDegree)
    {
        return (ShHalvesPerSplat(shDegree) + 1) / 2;
    }

    void WriteShHalf(uint32_t* splatRun, uint32_t halfIndex, uint16_t value)
    {
        uint32_t& word = splatRun[halfIndex >> 1];
        const uint32_t shift = (halfIndex & 1u) ? 16u : 0u;
        word = (word & ~(0xFFFFu << shift)) | (static_cast<uint32_t>(value) << shift);
    }

    AZStd::vector<uint32_t> PackCloudSh(const GaussianSplatAsset& asset, uint32_t uploadDegree)
    {
        const auto splatCount = static_cast<uint32_t>(asset.m_data.size());
        const uint32_t uploadCoeffs = Math::ShRestCoeffsPerChannel(uploadDegree);
        const uint32_t assetCoeffs = Math::ShRestCoeffsPerChannel(asset.m_shDegree);
        const uint32_t uintsPerSplat = ShUintsPerSplat(uploadDegree);

        // Every coefficient is one binary16, two to a uint - see GaussianSplatProjectCull.azsl's ShHalf. The
        // per-splat run is padded up to a whole uint so a splat's coefficients always start on a uint
        // boundary, which is what lets the shader address them as splatIndex * m_shUintsPerSplat.
        uint64_t packedSize = uint64_t{ splatCount } * uintsPerSplat;
        AZStd::vector<uint32_t> packed(packedSize, 0u);
        for (uint32_t splat = 0; splat < splatCount; ++splat)
        {
            const auto& vertex = asset.m_data[splat];
            const uint16_t* const assetRest = asset.ShRestForSplat(splat);
            uint32_t* const out = packed.data() + static_cast<size_t>(splat) * uintsPerSplat;

            // Element 0 of the run is the DC term, narrowed to binary16 to share the buffer's format; the
            // asset keeps it at full precision (CloudVertex::m_shDc).
            for (uint32_t channel = 0; channel < 3; ++channel)
            {
                WriteShHalf(out, channel, Math::FloatToHalf(vertex.m_shDc[channel]));
            }

            if (assetRest == nullptr)
            {
                // A degree-0 asset, or one whose SH array is short - the DC term is all there is to write.
                continue;
            }

            // Transpose channel-major [R_0..R_(k-1), G_0.., B_0..] into coefficient-major [R_i, G_i, B_i].
            for (uint32_t coeff = 0; coeff < uploadCoeffs; ++coeff)
            {
                for (uint32_t channel = 0; channel < 3; ++channel)
                {
                    WriteShHalf(out, 3 + coeff * 3 + channel, assetRest[channel * assetCoeffs + coeff]);
                }
            }
        }

        return packed;
    }
} // namespace GaussianSplat::Packing
