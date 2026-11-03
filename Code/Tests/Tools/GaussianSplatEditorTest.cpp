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

#include <AzCore/UnitTest/TestTypes.h>
#include <AzTest/AzTest.h>
#include <Tools/Builders/GaussianSplatBuilder.h>

#include <cstdio>
#include <fstream>

namespace UnitTest
{
    class GaussianSplatBuilderTest : public LeakDetectionFixture
    {
    };

    namespace
    {
        //! Writes a minimal ASCII .ply header (no vertex payload - IsGaussianSplatPly only reads up to
        //! "end_header") either with the full set of 3DGS vertex properties (+ restCount f_rest_* properties),
        //! or with a plain point-cloud-style position+color layout.
        void WriteTestPlyHeader(const char* path, bool includeGaussianSplatProperties, uint32_t restCount)
        {
            std::ofstream file(path, std::ios::binary);
            file << "ply\n";
            file << "format ascii 1.0\n";
            file << "element vertex 1\n";
            file << "property float x\n";
            file << "property float y\n";
            file << "property float z\n";
            if (includeGaussianSplatProperties)
            {
                file << "property float opacity\n";
                file << "property float scale_0\n";
                file << "property float scale_1\n";
                file << "property float scale_2\n";
                file << "property float rot_0\n";
                file << "property float rot_1\n";
                file << "property float rot_2\n";
                file << "property float rot_3\n";
                file << "property float f_dc_0\n";
                file << "property float f_dc_1\n";
                file << "property float f_dc_2\n";
                for (uint32_t i = 0; i < restCount; ++i)
                {
                    file << "property float f_rest_" << i << "\n";
                }
            }
            else
            {
                file << "property uchar red\n";
                file << "property uchar green\n";
                file << "property uchar blue\n";
            }
            file << "end_header\n";
        }
    } // namespace

    TEST_F(GaussianSplatBuilderTest, RecognizesGaussianSplatPlyAndDetectsMaxShDegree)
    {
        const char* path = "test_gaussian_splat_degree3.ply";
        WriteTestPlyHeader(path, true, 45);

        uint32_t shDegree = 0;
        EXPECT_TRUE(GaussianSplat::GaussianSplatBuilder::IsGaussianSplatPly(path, shDegree));
        EXPECT_EQ(shDegree, 3u);

        std::remove(path);
    }

    TEST_F(GaussianSplatBuilderTest, RecognizesLowerShDegree)
    {
        const char* path = "test_gaussian_splat_degree1.ply";
        WriteTestPlyHeader(path, true, 9);

        uint32_t shDegree = 0;
        EXPECT_TRUE(GaussianSplat::GaussianSplatBuilder::IsGaussianSplatPly(path, shDegree));
        EXPECT_EQ(shDegree, 1u);

        std::remove(path);
    }

    TEST_F(GaussianSplatBuilderTest, RejectsPlainPointCloudPly)
    {
        const char* path = "test_plain_pointcloud.ply";
        WriteTestPlyHeader(path, false, 0);

        uint32_t shDegree = 0;
        EXPECT_FALSE(GaussianSplat::GaussianSplatBuilder::IsGaussianSplatPly(path, shDegree));

        std::remove(path);
    }

    TEST_F(GaussianSplatBuilderTest, RejectsMissingFile)
    {
        uint32_t shDegree = 0;
        EXPECT_FALSE(GaussianSplat::GaussianSplatBuilder::IsGaussianSplatPly("this_file_does_not_exist.ply", shDegree));
    }
    AZ_UNIT_TEST_HOOK(DEFAULT_UNIT_TEST_ENV);
} // namespace UnitTest
