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

#include <AzCore/IO/Path/Path.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzTest/AzTest.h>
#include <AzTest/Utils.h>
#include <Tools/Builders/GaussianSplatBuilder.h>

#include <array>
#include <string>
#include <vector>

namespace UnitTest
{
    namespace
    {
        //! Header of a binary little-endian .ply with one "vertex" element of vertexCount entries, each holding
        //! the given float properties.
        std::string PlyHeader(size_t vertexCount, const std::vector<std::string>& properties)
        {
            std::string header = "ply\nformat binary_little_endian 1.0\nelement vertex " + std::to_string(vertexCount) + "\n";
            for (const std::string& property : properties)
            {
                header += "property float " + property + "\n";
            }
            return header + "end_header\n";
        }

        //! Binary .ply data of floatCount float zeros.
        std::string ZeroFloats(size_t floatCount)
        {
            return std::string(floatCount * sizeof(float), '\0');
        }

        //! The vertex properties CreateJobs requires, and f_rest_0..(restCount-1).
        std::vector<std::string> GaussianSplatProperties(uint32_t restCount)
        {
            std::vector<std::string> properties(
                GaussianSplat::RequiredSplatVertexProperties.begin(), GaussianSplat::RequiredSplatVertexProperties.end());
            for (uint32_t i = 0; i < restCount; ++i)
            {
                properties.push_back("f_rest_" + std::to_string(i));
            }
            return properties;
        }

        //! The vertex properties CreateJobs requires, except the positions x/y/z.
        std::vector<std::string> GaussianSplatPropertiesWithoutPositions()
        {
            std::vector<std::string> properties = GaussianSplatProperties(0);
            std::erase_if(
                properties,
                [](const std::string& property)
                {
                    return property == "x" || property == "y" || property == "z";
                });
            return properties;
        }

        //! Platforms enabled in every CreateJobs request the tests send.
        constexpr std::array<const char*, 2> TestPlatforms = { "pc", "linux" };

        //! Runs CreateJobs on the given .ply with TestPlatforms enabled.
        AssetBuilderSDK::CreateJobsResponse RunCreateJobs(const AZ::IO::Path& path)
        {
            GaussianSplat::GaussianSplatBuilder builder;
            AssetBuilderSDK::CreateJobsRequest request;
            request.m_watchFolder = path.ParentPath().Native();
            request.m_sourceFile = path.Filename().Native();
            for (const char* platform : TestPlatforms)
            {
                request.m_enabledPlatforms.emplace_back(platform, AZStd::unordered_set<AZStd::string>{});
            }
            AssetBuilderSDK::CreateJobsResponse response;
            builder.CreateJobs(request, response);
            return response;
        }

        //! Runs ProcessJob on the given .ply. Expects it not to throw - an exception escaping ProcessJob
        //! terminates the AssetBuilder process.
        AssetBuilderSDK::ProcessJobResultCode RunProcessJob(const char* path)
        {
            GaussianSplat::GaussianSplatBuilder builder;
            AssetBuilderSDK::ProcessJobRequest request;
            request.m_fullPath = path;
            AssetBuilderSDK::ProcessJobResponse response;
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Success;
            EXPECT_NO_THROW(builder.ProcessJob(request, response));
            return response.m_resultCode;
        }
    } // namespace

    class GaussianSplatBuilderTest : public LeakDetectionFixture
    {
    protected:
        //! Unique per test, so test .ply files never land in the working directory; removed with its contents
        //! when the test ends.
        AZ::Test::ScopedAutoTempDirectory m_tempDirectory;

        //! Writes contents to fileName in m_tempDirectory and returns its path. Fails the test if the file cannot be
        //! written, since a missing file would also make ProcessJob fail.
        AZ::IO::Path WriteFile(const char* fileName, const std::string& contents)
        {
            EXPECT_TRUE(
                AZ::Test::CreateTestFile(m_tempDirectory, fileName, AZStd::string_view(contents.data(), contents.size())).has_value())
                << "cannot write " << fileName;
            return m_tempDirectory.Resolve(fileName);
        }

        //! Writes a complete binary .ply with a single vertex holding exactly the given float properties, so tests
        //! can produce both valid splat files and ones that pass CreateJobs' header check but lack properties
        //! ProcessJob reads.
        AZ::IO::Path WriteTestPly(const char* fileName, const std::vector<std::string>& properties)
        {
            return WriteFile(fileName, PlyHeader(1, properties) + ZeroFloats(properties.size()));
        }
    };

    TEST_F(GaussianSplatBuilderTest, CreatesJobPerPlatformForGaussianSplatPly)
    {
        const AZ::IO::Path path = WriteTestPly("test_gaussian_splat.ply", GaussianSplatProperties(45));

        const AssetBuilderSDK::CreateJobsResponse response = RunCreateJobs(path);
        EXPECT_EQ(response.m_result, AssetBuilderSDK::CreateJobsResultCode::Success);
        ASSERT_EQ(response.m_createJobOutputs.size(), TestPlatforms.size());
        for (size_t i = 0; i < TestPlatforms.size(); ++i)
        {
            EXPECT_EQ(response.m_createJobOutputs[i].GetPlatformIdentifier(), TestPlatforms[i]);
        }
    }

    TEST_F(GaussianSplatBuilderTest, CreatesNoJobsForPlainPointCloudPly)
    {
        const AZ::IO::Path path = WriteTestPly("test_plain_pointcloud.ply", { "x", "y", "z", "red", "green", "blue" });

        const AssetBuilderSDK::CreateJobsResponse response = RunCreateJobs(path);
        EXPECT_EQ(response.m_result, AssetBuilderSDK::CreateJobsResultCode::Success);
        EXPECT_TRUE(response.m_createJobOutputs.empty());
    }

    TEST_F(GaussianSplatBuilderTest, CreatesNoJobsForMissingFile)
    {
        const AZ::IO::Path path = m_tempDirectory.Resolve("this_file_does_not_exist.ply");

        const AssetBuilderSDK::CreateJobsResponse response = RunCreateJobs(path);
        EXPECT_EQ(response.m_result, AssetBuilderSDK::CreateJobsResultCode::Success);
        EXPECT_TRUE(response.m_createJobOutputs.empty());
    }

    TEST_F(GaussianSplatBuilderTest, CreatesNoJobsForPlyMissingPositions)
    {
        const AZ::IO::Path path = WriteTestPly("test_gaussian_splat_missing_xyz.ply", GaussianSplatPropertiesWithoutPositions());

        const AssetBuilderSDK::CreateJobsResponse response = RunCreateJobs(path);
        EXPECT_EQ(response.m_result, AssetBuilderSDK::CreateJobsResultCode::Success);
        EXPECT_TRUE(response.m_createJobOutputs.empty());
    }

    TEST_F(GaussianSplatBuilderTest, ProcessJobFailsOnPlyWithGapInFRest)
    {
        // Nine f_rest_* properties imply SH degree 1, but f_rest_8 is replaced by f_rest_9.
        std::vector<std::string> properties = GaussianSplatProperties(8);
        properties.emplace_back("f_rest_9");
        const AZ::IO::Path path = WriteTestPly("test_gaussian_splat_f_rest_gap.ply", properties);

        EXPECT_EQ(RunProcessJob(path.c_str()), AssetBuilderSDK::ProcessJobResult_Failed);
    }

    TEST_F(GaussianSplatBuilderTest, ProcessJobFailsOnNonPlyFile)
    {
        // happly rejects the first line by throwing from the PLYData constructor.
        const AZ::IO::Path path = WriteFile("test_not_a_ply.ply", "not a ply file\n");

        EXPECT_EQ(RunProcessJob(path.c_str()), AssetBuilderSDK::ProcessJobResult_Failed);
    }

    TEST_F(GaussianSplatBuilderTest, ProcessJobFailsOnTruncatedBinaryPly)
    {
        // The header declares two vertices but the data stops partway through the second; happly alone keeps
        // the short read's values.
        const std::vector<std::string> properties = GaussianSplatProperties(0);
        const AZ::IO::Path path = WriteFile(
            "test_gaussian_splat_truncated_binary.ply",
            PlyHeader(2, properties) + ZeroFloats(properties.size() * 3 / 2));

        EXPECT_EQ(RunProcessJob(path.c_str()), AssetBuilderSDK::ProcessJobResult_Failed);
    }

    AZ_UNIT_TEST_HOOK(DEFAULT_UNIT_TEST_ENV);
} // namespace UnitTest
