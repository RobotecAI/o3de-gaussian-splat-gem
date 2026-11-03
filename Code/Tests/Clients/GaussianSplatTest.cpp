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

#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzCore/std/string/string.h>
#include <AzTest/AzTest.h>
#include <GaussianSplat/GaussianSplatAsset.h>
#include <Utils/GaussianSplatMath.h>
#include <cmath>

namespace UnitTest
{
    class GaussianSplatAssetTest : public LeakDetectionFixture
    {
        void SetUp() override
        {
            LeakDetectionFixture::SetUp();
            AZ::Data::AssetManager::Descriptor descriptor;
            AZ::Data::AssetManager::Create(descriptor);
            m_testHandler = AZStd::make_unique<GaussianSplat::GaussianSplatAssetHandler>();
            AZ::Data::AssetManager::Instance().RegisterHandler(
                m_testHandler.get(), AZ::AzTypeInfo<GaussianSplat::GaussianSplatAsset>::Uuid());
        }

        void TearDown() override
        {
            AZ::Data::AssetManager::Instance().DispatchEvents();
            AZ::Data::AssetManager::Instance().UnregisterHandler(m_testHandler.get());
            AZ::Data::AssetManager::Destroy();

            m_testHandler.reset();

            LeakDetectionFixture::TearDown();
        }

    protected:
        AZStd::unique_ptr<GaussianSplat::GaussianSplatAssetHandler> m_testHandler;
    };

    //! Test the ability to write and read a product gaussian splat asset with partial (degree 1) SH data, and
    //! that coefficients beyond what the asset's degree stores come back zero-filled.
    TEST_F(GaussianSplatAssetTest, WritingAndLoadingAssetRoundTrip)
    {
        AZ::Data::Asset<GaussianSplat::GaussianSplatAsset> testAsset =
            AZ::Data::AssetManager::Instance().CreateAsset<GaussianSplat::GaussianSplatAsset>(AZ::Data::AssetId(AZ::Uuid::CreateRandom()));
        GaussianSplat::GaussianSplatAsset& gold = *testAsset.Get();
        gold.m_shDegree = 1;
        gold.m_data.resize(2);
        gold.m_shRest.resize(gold.m_data.size() * gold.ShRestStride());

        gold.m_data[0].m_position = { 1.0f, 2.0f, 3.0f };
        gold.m_data[0].m_rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
        gold.m_data[0].m_scale = { 0.1f, 0.2f, 0.3f };
        gold.m_data[0].m_opacity = 0.75f;
        gold.m_data[0].m_shDc = { 0.5f, 0.4f, 0.3f };
        for (uint32_t i = 0; i < 9; ++i)
        {
            gold.m_shRest[i] = GaussianSplat::Math::FloatToHalf(static_cast<float>(i) * 0.1f);
        }

        gold.m_data[1].m_position = { -1.0f, -2.0f, -3.0f };
        gold.m_data[1].m_rotation = { 0.1f, 0.2f, 0.3f, 0.9f };
        gold.m_data[1].m_scale = { 1.0f, 1.0f, 1.0f };
        gold.m_data[1].m_opacity = 0.25f;
        gold.m_data[1].m_shDc = { 0.1f, 0.2f, 0.3f };
        for (uint32_t i = 0; i < 9; ++i)
        {
            gold.m_shRest[9 + i] = GaussianSplat::Math::FloatToHalf(static_cast<float>(i) * -0.1f);
        }

        AZStd::vector<AZ::u8> buffer(4096);
        AZ::IO::MemoryStream stream(buffer.data(), buffer.size(), buffer.size());
        ASSERT_TRUE(m_testHandler->SaveAssetData(testAsset, &stream));

        auto assetStream = AZStd::make_shared<AZ::Data::AssetDataStream>();
        assetStream->Open(buffer);
        AZ::Data::Asset<GaussianSplat::GaussianSplatAsset> loadedAsset =
            AZ::Data::AssetManager::Instance().CreateAsset<GaussianSplat::GaussianSplatAsset>(AZ::Data::AssetId(AZ::Uuid::CreateRandom()));
        ASSERT_EQ(m_testHandler->LoadAssetData(loadedAsset, assetStream, nullptr), AZ::Data::AssetHandler::LoadResult::LoadComplete);

        GaussianSplat::GaussianSplatAsset& loaded = *loadedAsset.Get();
        ASSERT_EQ(loaded.m_shDegree, 1);
        ASSERT_EQ(loaded.m_data.size(), 2);
        // A degree-1 asset stores 9 "rest" coefficients per splat (3 channels x 3 coefficients), and no more.
        ASSERT_EQ(loaded.ShRestStride(), 9);
        ASSERT_EQ(loaded.m_shRest.size(), 18);
        for (size_t i = 0; i < 2; ++i)
        {
            EXPECT_EQ(loaded.m_data[i].m_position, gold.m_data[i].m_position);
            EXPECT_EQ(loaded.m_data[i].m_rotation, gold.m_data[i].m_rotation);
            EXPECT_EQ(loaded.m_data[i].m_scale, gold.m_data[i].m_scale);
            EXPECT_FLOAT_EQ(loaded.m_data[i].m_opacity, gold.m_data[i].m_opacity);
            EXPECT_EQ(loaded.m_data[i].m_shDc, gold.m_data[i].m_shDc);
            const uint16_t* const loadedRest = loaded.ShRestForSplat(i);
            const uint16_t* const goldRest = gold.ShRestForSplat(i);
            ASSERT_NE(loadedRest, nullptr);
            ASSERT_NE(goldRest, nullptr);
            for (uint32_t c = 0; c < 9; ++c)
            {
                EXPECT_EQ(loadedRest[c], goldRest[c]);
            }
        }
    }

    TEST(GaussianSplatMathTest, HalfConversionRoundTripsRepresentableValues)
    {
        // Exactly representable in binary16, so the round trip has to be lossless.
        for (const float exact : { 0.0f, 1.0f, -1.0f, 0.5f, -0.25f, 2.0f, 1024.0f, 0.00006103515625f })
        {
            EXPECT_FLOAT_EQ(GaussianSplat::Math::HalfToFloat(GaussianSplat::Math::FloatToHalf(exact)), exact);
        }

        // Values needing rounding land within binary16's ~2^-11 relative precision. This is the range SH
        // "rest" coefficients actually occupy.
        for (const float rounded : { 0.1f, -0.1f, 0.3333f, 1.7f, -12.34f })
        {
            EXPECT_NEAR(
                GaussianSplat::Math::HalfToFloat(GaussianSplat::Math::FloatToHalf(rounded)), rounded, 0.001f * std::fabs(rounded) + 1e-7f);
        }

        // Signed zero survives, values past binary16's range saturate to infinity, and values below its
        // smallest subnormal flush to zero.
        EXPECT_EQ(GaussianSplat::Math::FloatToHalf(-0.0f), 0x8000);
        EXPECT_TRUE(std::isinf(GaussianSplat::Math::HalfToFloat(GaussianSplat::Math::FloatToHalf(1e30f))));
        EXPECT_FLOAT_EQ(GaussianSplat::Math::HalfToFloat(GaussianSplat::Math::FloatToHalf(1e-30f)), 0.0f);
    }

    TEST(GaussianSplatMathTest, ActivationFunctionsBehaveAsExpected)
    {
        EXPECT_NEAR(GaussianSplat::Math::ActivateOpacity(0.0f), 0.5f, 1e-5f);
        EXPECT_LT(GaussianSplat::Math::ActivateOpacity(-10.0f), 0.01f);
        EXPECT_GT(GaussianSplat::Math::ActivateOpacity(10.0f), 0.99f);
        EXPECT_LT(GaussianSplat::Math::ActivateOpacity(-1.0f), GaussianSplat::Math::ActivateOpacity(1.0f));

        EXPECT_NEAR(GaussianSplat::Math::ActivateScale(0.0f), 1.0f, 1e-5f);
        EXPECT_GT(GaussianSplat::Math::ActivateScale(-100.0f), 0.0f);
        EXPECT_GT(GaussianSplat::Math::ActivateScale(2.0f), GaussianSplat::Math::ActivateScale(1.0f));

        const AZ::Quaternion normalized = GaussianSplat::Math::ActivateRotation(0.0f, 0.0f, 0.0f, 2.0f);
        EXPECT_NEAR(normalized.GetLength(), 1.0f, 1e-5f);
        const AZ::Quaternion degenerate = GaussianSplat::Math::ActivateRotation(0.0f, 0.0f, 0.0f, 0.0f);
        EXPECT_TRUE(degenerate.IsClose(AZ::Quaternion::CreateIdentity()));
    }

    TEST(GaussianSplatMathTest, ConvertPositionYUpToZUpMapsPlyDownAxisToEngineUp)
    {
        // 3DGS .ply files store Y increasing downward, so +1 in ply-Y must land at -1 along O3DE's up (Z) axis.
        EXPECT_TRUE(GaussianSplat::Math::ConvertPositionYUpToZUp(0.0f, 1.0f, 0.0f).IsClose(-AZ::Vector3::CreateAxisZ()));
        EXPECT_TRUE(GaussianSplat::Math::ConvertPositionYUpToZUp(0.0f, 0.0f, 1.0f).IsClose(AZ::Vector3::CreateAxisY()));
        EXPECT_TRUE(GaussianSplat::Math::ConvertPositionYUpToZUp(1.0f, 0.0f, 0.0f).IsClose(AZ::Vector3::CreateAxisX()));
    }

    TEST(GaussianSplatMathTest, ConvertRotationYUpToZUpPreservesIdentityAndRemapsAxis)
    {
        EXPECT_TRUE(
            GaussianSplat::Math::ConvertRotationYUpToZUp(AZ::Quaternion::CreateIdentity()).IsClose(AZ::Quaternion::CreateIdentity()));

        // A rotation about ply-Y must become a rotation about engine-Z, reversed (ply-Y maps to engine-(-Z)).
        const AZ::Quaternion converted =
            GaussianSplat::Math::ConvertRotationYUpToZUp(AZ::Quaternion::CreateRotationY(AZ::Constants::HalfPi));
        EXPECT_TRUE(converted.IsClose(AZ::Quaternion::CreateRotationZ(-AZ::Constants::HalfPi)));
    }

    namespace
    {
        //! The Y-up -> Z-up basis change ConvertPositionYUpToZUp/ConvertRotationYUpToZUp implement, as a matrix:
        //! (x, y, z) -> (x, z, -y).
        AZ::Matrix3x3 PlyToEngineBasis()
        {
            return AZ::Matrix3x3::CreateFromRows(
                AZ::Vector3(1.0f, 0.0f, 0.0f), AZ::Vector3(0.0f, 0.0f, 1.0f), AZ::Vector3(0.0f, -1.0f, 0.0f));
        }

        //! A splat's shape: Sigma = R*S*(R*S)^T, the same covariance construction
        //! GaussianSplatProjectCull.azsl's ProjectSplatToScreen uses on the GPU.
        AZ::Matrix3x3 SplatCovariance(const AZ::Quaternion& rotation, const AZ::Vector3& scale)
        {
            const AZ::Matrix3x3 rs = AZ::Matrix3x3::CreateFromQuaternion(rotation) * AZ::Matrix3x3::CreateScale(scale);
            return rs * rs.GetTranspose();
        }

        //! Reference evaluation of the degree 1-3 SH "rest" terms for a single color channel, mirroring the basis
        //! functions and ordering in GaussianSplatProjectCull.azsl's EvaluateShColor (the DC term is omitted since it
        //! carries no directional dependence).
        float EvaluateShRest(const AZStd::vector<float>& c, const AZ::Vector3& dir, uint32_t degree)
        {
            const float x = dir.GetX();
            const float y = dir.GetY();
            const float z = dir.GetZ();

            float result = 0.4886025119029199f * (-y * c[0] + z * c[1] - x * c[2]);
            if (degree < 2)
            {
                return result;
            }

            const float xx = x * x;
            const float yy = y * y;
            const float zz = z * z;
            result += 1.0925484305920792f * (x * y) * c[3] + -1.0925484305920792f * (y * z) * c[4] +
                0.31539156525252005f * (2.0f * zz - xx - yy) * c[5] + -1.0925484305920792f * (x * z) * c[6] +
                0.5462742152960396f * (xx - yy) * c[7];
            if (degree < 3)
            {
                return result;
            }

            result += -0.5900435899266435f * y * (3.0f * xx - yy) * c[8] + 2.890611442640554f * (x * y * z) * c[9] +
                -0.4570457994644658f * y * (4.0f * zz - xx - yy) * c[10] +
                0.3731763325901154f * z * (2.0f * zz - 3.0f * xx - 3.0f * yy) * c[11] +
                -0.4570457994644658f * x * (4.0f * zz - xx - yy) * c[12] + 1.445305721320277f * z * (xx - yy) * c[13] +
                -0.5900435899266435f * x * (xx - 3.0f * yy) * c[14];
            return result;
        }
    } // namespace

    TEST(GaussianSplatMathTest, ConvertScaleYUpToZUpSwapsTheTwoNonXAxes)
    {
        const AZ::Vector3 converted = GaussianSplat::Math::ConvertScaleYUpToZUp(1.0f, 2.0f, 3.0f);
        EXPECT_TRUE(converted.IsClose(AZ::Vector3(1.0f, 3.0f, 2.0f)));
    }

    TEST(GaussianSplatMathTest, ConvertedRotationAndScaleTogetherPreserveTheSplatShape)
    {
        // Converted rotation/scale must rebuild the basis-changed covariance B*Sigma*B^T exactly.
        const AZ::Quaternion plyRotation =
            (AZ::Quaternion::CreateRotationZ(0.7f) * AZ::Quaternion::CreateRotationY(-0.4f) * AZ::Quaternion::CreateRotationX(1.1f))
                .GetNormalized();
        const AZ::Vector3 plyScale(0.5f, 2.0f, 3.5f);

        const AZ::Matrix3x3 basis = PlyToEngineBasis();
        const AZ::Matrix3x3 expected = basis * SplatCovariance(plyRotation, plyScale) * basis.GetTranspose();
        const AZ::Matrix3x3 actual = SplatCovariance(
            GaussianSplat::Math::ConvertRotationYUpToZUp(plyRotation),
            GaussianSplat::Math::ConvertScaleYUpToZUp(plyScale.GetX(), plyScale.GetY(), plyScale.GetZ()));

        for (int row = 0; row < 3; ++row)
        {
            EXPECT_TRUE(actual.GetRow(row).IsClose(expected.GetRow(row), 1e-4f))
                << "row " << row << " expected "
                << AZStd::string::format(
                       "(%f, %f, %f)", expected.GetRow(row).GetX(), expected.GetRow(row).GetY(), expected.GetRow(row).GetZ())
                       .c_str();
        }
    }

    TEST(GaussianSplatMathTest, ConvertShRestYUpToZUpMakesEngineSpaceEvaluationMatchPlySpace)
    {
        // Converted coefficients evaluated at an engine-space direction must match the original coefficients
        // evaluated at the corresponding ply-space direction, for every degree and a spread of directions.
        const AZ::Matrix3x3 basis = PlyToEngineBasis();
        const AZ::Vector3 plyDirections[] = { AZ::Vector3(1.0f, 0.0f, 0.0f),
                                              AZ::Vector3(0.0f, 1.0f, 0.0f),
                                              AZ::Vector3(0.0f, 0.0f, 1.0f),
                                              AZ::Vector3(0.6f, -0.48f, 0.64f),
                                              AZ::Vector3(-0.3f, 0.5f, -0.81f) };

        for (uint32_t degree = 1; degree <= 3; ++degree)
        {
            const uint32_t coeffCount = GaussianSplat::Math::ShRestCoeffsPerChannel(degree);

            // Non-degenerate coefficients: no two alike, mixed signs.
            AZStd::vector<float> plyCoeffs(coeffCount);
            for (uint32_t i = 0; i < coeffCount; ++i)
            {
                plyCoeffs[i] = 0.37f * static_cast<float>(i + 1) * ((i % 3 == 0) ? -1.0f : 1.0f);
            }

            AZStd::vector<float> engineCoeffs = plyCoeffs;
            GaussianSplat::Math::ConvertShRestYUpToZUp(degree, engineCoeffs.data());

            for (const AZ::Vector3& plyDirection : plyDirections)
            {
                const AZ::Vector3 engineDirection = basis * plyDirection;
                EXPECT_NEAR(EvaluateShRest(engineCoeffs, engineDirection, degree), EvaluateShRest(plyCoeffs, plyDirection, degree), 1e-4f)
                    << "degree " << degree;
            }
        }
    }

    TEST(GaussianSplatMathTest, ConvertShRestYUpToZUpLeavesHigherBandsUntouchedBelowTheirDegree)
    {
        // A degree-1 asset only stores 3 coefficients per channel, so the conversion must not read or write
        // past them.
        AZStd::vector<float> coeffs = { 1.0f, 2.0f, 3.0f, 99.0f, 99.0f };
        GaussianSplat::Math::ConvertShRestYUpToZUp(1, coeffs.data());
        EXPECT_FLOAT_EQ(coeffs[3], 99.0f);
        EXPECT_FLOAT_EQ(coeffs[4], 99.0f);
    }

    TEST(GaussianSplatMathTest, ShDegreeFromRestCountMapsKnownCounts)
    {
        EXPECT_EQ(GaussianSplat::Math::ShDegreeFromRestCount(0), 0u);
        EXPECT_EQ(GaussianSplat::Math::ShDegreeFromRestCount(9), 1u);
        EXPECT_EQ(GaussianSplat::Math::ShDegreeFromRestCount(24), 2u);
        EXPECT_EQ(GaussianSplat::Math::ShDegreeFromRestCount(45), 3u);
    }

    AZ_UNIT_TEST_HOOK(DEFAULT_UNIT_TEST_ENV);
} // namespace UnitTest
