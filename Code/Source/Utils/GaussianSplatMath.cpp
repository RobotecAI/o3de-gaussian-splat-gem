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

#include "GaussianSplatMath.h"
#include <AzCore/Math/MathUtils.h>
#include <cmath>
#include <cstring>

namespace GaussianSplat::Math
{
    float ActivateOpacity(float rawOpacity)
    {
        return 1.0f / (1.0f + std::exp(-rawOpacity));
    }

    float ActivateScale(float rawScale)
    {
        return std::exp(rawScale);
    }

    AZ::Quaternion ActivateRotation(float rawX, float rawY, float rawZ, float rawW)
    {
        AZ::Quaternion rotation(rawX, rawY, rawZ, rawW);
        if (rotation.GetLengthSq() < AZ::Constants::FloatEpsilon)
        {
            return AZ::Quaternion::CreateIdentity();
        }
        return rotation.GetNormalized();
    }

    AZ::Vector3 ConvertPositionYUpToZUp(float x, float y, float z)
    {
        return { x, z, -y };
    }

    AZ::Quaternion ConvertRotationYUpToZUp(const AZ::Quaternion& rotation)
    {
        return AZ::Quaternion(rotation.GetX(), rotation.GetZ(), -rotation.GetY(), rotation.GetW());
    }

    AZ::Vector3 ConvertScaleYUpToZUp(float scaleX, float scaleY, float scaleZ)
    {
        return { scaleX, scaleZ, scaleY };
    }

    void ConvertShRestYUpToZUp(uint32_t shDegree, float* restCoeffs)
    {
        if (restCoeffs == nullptr || shDegree == 0)
        {
            return;
        }

        // Band 1 (l = 1, coefficients 0-2).
        {
            const float c0 = restCoeffs[0];
            const float c1 = restCoeffs[1];
            restCoeffs[0] = -c1;
            restCoeffs[1] = c0;
            // restCoeffs[2] is unchanged: its basis function depends only on the untouched X axis.
        }

        if (shDegree < 2)
        {
            return;
        }

        // Band 2 (l = 2, coefficients 3-7). Sqrt3Over2 mixes the two Y-even basis functions.
        {
            constexpr float sqrt3Over2 = 0.8660254037844386f;
            const float c3 = restCoeffs[3];
            const float c4 = restCoeffs[4];
            const float c5 = restCoeffs[5];
            const float c6 = restCoeffs[6];
            const float c7 = restCoeffs[7];
            restCoeffs[3] = -c6;
            restCoeffs[4] = -c4;
            restCoeffs[5] = -0.5f * c5 - sqrt3Over2 * c7;
            restCoeffs[6] = c3;
            restCoeffs[7] = -sqrt3Over2 * c5 + 0.5f * c7;
        }

        if (shDegree < 3)
        {
            return;
        }

        // Band 3 (l = 3, coefficients 8-14).
        {
            constexpr float sqrt10Over4 = 0.7905694150420949f;
            constexpr float sqrt6Over4 = 0.6123724356957945f;
            constexpr float sqrt15Over4 = 0.9682458365518543f;
            const float c8 = restCoeffs[8];
            const float c9 = restCoeffs[9];
            const float c10 = restCoeffs[10];
            const float c11 = restCoeffs[11];
            const float c12 = restCoeffs[12];
            const float c13 = restCoeffs[13];
            const float c14 = restCoeffs[14];
            restCoeffs[8] = sqrt10Over4 * c11 - sqrt6Over4 * c13;
            restCoeffs[9] = -c9;
            restCoeffs[10] = sqrt6Over4 * c11 + sqrt10Over4 * c13;
            restCoeffs[11] = -sqrt10Over4 * c8 - sqrt6Over4 * c10;
            restCoeffs[12] = -0.25f * c12 - sqrt15Over4 * c14;
            restCoeffs[13] = sqrt6Over4 * c8 - sqrt10Over4 * c10;
            restCoeffs[14] = -sqrt15Over4 * c12 + 0.25f * c14;
        }
    }

    uint32_t ShDegreeFromRestCount(size_t restCount)
    {
        if (restCount >= 45)
        {
            return 3;
        }
        if (restCount >= 24)
        {
            return 2;
        }
        if (restCount >= 9)
        {
            return 1;
        }
        return 0;
    }

    uint32_t ShRestCoeffsPerChannel(uint32_t shDegree)
    {
        switch (shDegree)
        {
        case 1:
            return 3;
        case 2:
            return 8;
        case 3:
            return 15;
        default:
            return 0;
        }
    }

    uint32_t ShRestCoeffsPerSplat(uint32_t shDegree)
    {
        return ShRestCoeffsPerChannel(shDegree) * 3;
    }

    uint16_t FloatToHalf(float value)
    {
        uint32_t bits = 0;
        static_assert(sizeof(bits) == sizeof(value), "float is not 32 bits wide.");
        memcpy(&bits, &value, sizeof(bits));

        const uint16_t sign = static_cast<uint16_t>((bits >> 16) & 0x8000u);
        // Unbiased exponent, then rebiased from binary32's 127 to binary16's 15.
        const int32_t exponent = static_cast<int32_t>((bits >> 23) & 0xFFu) - 127 + 15;
        const uint32_t mantissa = bits & 0x7FFFFFu;

        if (exponent >= 0x1F)
        {
            // Infinity, NaN, or an overflow of binary16's range. A NaN must stay a NaN (non-zero mantissa).
            const bool isNaN = ((bits >> 23) & 0xFFu) == 0xFFu && mantissa != 0;
            return static_cast<uint16_t>(sign | 0x7C00u | (isNaN ? 0x200u : 0u));
        }

        if (exponent <= 0)
        {
            if (exponent < -10)
            {
                // Smaller than half the smallest binary16 subnormal, so it rounds to zero either way.
                return sign;
            }
            // Subnormal: restore binary32's implicit leading 1 and shift it down into the subnormal range.
            // shift lands in [14, 24] here, so neither the shift nor the halfway mask can overflow.
            const uint32_t subnormalMantissa = mantissa | 0x800000u;
            const uint32_t shift = static_cast<uint32_t>(14 - exponent);
            uint32_t result = subnormalMantissa >> shift;
            const uint32_t dropped = subnormalMantissa & ((1u << shift) - 1);
            const uint32_t halfway = 1u << (shift - 1);
            if (dropped > halfway || (dropped == halfway && (result & 1) != 0))
            {
                ++result;
            }
            return static_cast<uint16_t>(sign | result);
        }

        // Normal: keep the top 10 mantissa bits, rounding the 13 dropped bits to nearest with ties to even.
        // A mantissa that carries past 10 bits rolls into the exponent field on its own, which is exactly the
        // increment binary16 needs - including at the top of the range, where it lands on infinity.
        uint32_t result = (static_cast<uint32_t>(exponent) << 10) | (mantissa >> 13);
        const uint32_t dropped = mantissa & 0x1FFFu;
        if (dropped > 0x1000u || (dropped == 0x1000u && (result & 1) != 0))
        {
            ++result;
        }
        return static_cast<uint16_t>(sign | result);
    }

    float HalfToFloat(uint16_t half)
    {
        const uint32_t sign = static_cast<uint32_t>(half & 0x8000u) << 16;
        const uint32_t exponent = (half >> 10) & 0x1Fu;
        const uint32_t mantissa = half & 0x3FFu;

        uint32_t bits = 0;
        if (exponent == 0)
        {
            if (mantissa != 0)
            {
                // Subnormal: normalize it by shifting the leading 1 up out of the mantissa.
                uint32_t shiftedMantissa = mantissa;
                int32_t shiftedExponent = -1;
                do
                {
                    shiftedMantissa <<= 1;
                    ++shiftedExponent;
                } while ((shiftedMantissa & 0x400u) == 0);
                bits = sign | (static_cast<uint32_t>(127 - 15 - shiftedExponent) << 23) | ((shiftedMantissa & 0x3FFu) << 13);
            }
            else
            {
                bits = sign; // +/- zero
            }
        }
        else if (exponent == 0x1F)
        {
            bits = sign | 0x7F800000u | (mantissa << 13); // infinity or NaN
        }
        else
        {
            bits = sign | ((exponent - 15 + 127) << 23) | (mantissa << 13);
        }

        float result = 0.0f;
        memcpy(&result, &bits, sizeof(result));
        return result;
    }
} // namespace GaussianSplat::Math
