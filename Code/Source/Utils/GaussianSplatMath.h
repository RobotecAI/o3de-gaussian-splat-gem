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

#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/base.h>
#include <cstdint>

// Pure, CPU-only math shared by the .ply importer (GaussianSplatBuilder) and the FeatureProcessor.
// Kept free of AzToolsFramework/AssetBuilderSDK/Atom dependencies so it can be exercised directly
// by GaussianSplat.Tests without pulling in editor-only or rendering-only build dependencies.
namespace GaussianSplat::Math
{
    //! Converts a raw (pre-activation) opacity value stored in a 3DGS-trained .ply into a linear [0, 1] opacity.
    float ActivateOpacity(float rawOpacity);

    //! Converts a raw (log-space) scale value stored in a 3DGS-trained .ply into a linear scale.
    float ActivateScale(float rawScale);

    //! Normalizes a raw (possibly unnormalized) rotation quaternion stored in a 3DGS-trained .ply.
    AZ::Quaternion ActivateRotation(float rawX, float rawY, float rawZ, float rawW);

    //! Converts a position from the .ply's Y-up, COLMAP/SfM convention into O3DE's Z-up, right-handed
    //! convention: a -90 degree rotation about X, i.e. ply-Y becomes engine-(-Z) and ply-Z becomes engine-Y.
    AZ::Vector3 ConvertPositionYUpToZUp(float x, float y, float z);

    //! Applies the same Y-up -> Z-up basis change (see ConvertPositionYUpToZUp) to a rotation quaternion, by
    //! conjugation (R' = B*R*B^T). Because this conjugates rather than composes, the per-axis scale must be
    //! permuted to match - see ConvertScaleYUpToZUp.
    AZ::Quaternion ConvertRotationYUpToZUp(const AZ::Quaternion& rotation);

    //! Applies the same Y-up -> Z-up basis change (see ConvertPositionYUpToZUp) to a splat's per-axis scale.
    //! Pairs with ConvertRotationYUpToZUp's conjugation of the rotation: the covariance Sigma = R*S^2*R^T must
    //! become B*Sigma*B^T, which for this basis change means the two non-X scale axes swap.
    AZ::Vector3 ConvertScaleYUpToZUp(float scaleX, float scaleY, float scaleZ);

    //! Applies the same Y-up -> Z-up basis change (see ConvertPositionYUpToZUp) to one color channel's "rest"
    //! (degree 1-3) SH coefficients, in-place, in the reference 3DGS basis order (matching
    //! GaussianSplatProjectCull.azsl's EvaluateShColor). Needed because the shader evaluates the SH basis in engine
    //! world space, but the coefficients were trained against the .ply's frame; only the DC term is rotation-invariant.
    void ConvertShRestYUpToZUp(uint32_t shDegree, float* restCoeffs);

    //! Maps a count of "f_rest_*" properties found in a 3DGS-trained .ply vertex element to an SH degree (0-3).
    //! 3DGS stores 3 * (SH coefficients per channel excluding DC) floats in f_rest, so 0/9/24/45 map to degree 0/1/2/3.
    uint32_t ShDegreeFromRestCount(size_t restCount);

    //! Number of "rest" (non-DC) SH coefficients per color channel for a given SH degree (0-3).
    uint32_t ShRestCoeffsPerChannel(uint32_t shDegree);

    //! Total "rest" (non-DC) SH coefficients stored per splat at a given degree: one per channel per
    //! coefficient. This is the stride of GaussianSplatAsset::m_shRest.
    uint32_t ShRestCoeffsPerSplat(uint32_t shDegree);

    //! Converts a float to IEEE-754 binary16, returned as its raw bit pattern, with round-to-nearest-even.
    //! Values too large for binary16 saturate to infinity and values too small flush through the subnormal
    //! range to zero, so no input produces a NaN that wasn't one already.
    //!
    //! Written here rather than taken from the engine: the only binary16 conversion in the tree lives in
    //! ImageProcessingAtom's private sources, which this gem cannot include (and which the builder, a
    //! separate module, could not link against either).
    uint16_t FloatToHalf(float value);

    //! Converts an IEEE-754 binary16 bit pattern back to float. Exact - every binary16 value is representable
    //! as a float - so this is the inverse of FloatToHalf for any value FloatToHalf did not have to round.
    float HalfToFloat(uint16_t half);
} // namespace GaussianSplat::Math
