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

#include "GaussianSplatBuilder.h"
#include <AssetBuilderSDK/AssetBuilderSDK.h>
#include <AssetBuilderSDK/SerializationDependencies.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <GaussianSplat/GaussianSplatAsset.h>
#include <Utils/GaussianSplatMath.h>
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <happly.h>
#include <string>
#include <string_view>
#include <unordered_set>

namespace GaussianSplat
{
    constexpr const char* GaussianSplatBuilderJobKey = "GaussianSplatBuilder";

    // 3DGS-trained .ply exports store f_rest_* channel-major: the first ShRestCoeffsPerChannel(degree)
    // properties are the R channel's coefficients, the next block is G, the last block is B - the same
    // channel-major layout GaussianSplatAsset::m_shRest uses, so no reordering is needed here.
    namespace
    {
        //! Prefix of the SH "rest" coefficient properties; how many of them there are sets the SH degree.
        constexpr std::string_view FRestPrefix = "f_rest_";

        std::string FRestPropertyName(uint32_t index)
        {
            return std::string(FRestPrefix) + std::to_string(index);
        }

        //! The .ply "vertex" properties ProcessJob reads, one entry per vertex in each vector.
        struct PlyVertexProperties
        {
            uint32_t m_shDegree = 0;
            size_t m_count = 0;
            std::vector<float> m_posX, m_posY, m_posZ;
            std::vector<float> m_opacity;
            std::vector<float> m_scale0, m_scale1, m_scale2;
            std::vector<float> m_rot0, m_rot1, m_rot2, m_rot3;
            std::vector<float> m_shDc0, m_shDc1, m_shDc2;
            std::vector<std::vector<float>> m_shRest;
        };

        bool IsRestProperty(const std::string& propertyName)
        {
            return propertyName.starts_with(FRestPrefix);
        }

        //! True if the .ply's "vertex" element has every property in RequiredSplatVertexProperties. Reads only the
        //! header (cheap per CreateJobs call) to avoid claiming plain point-cloud .ply files that Pointcloud's
        //! builder also watches.
        bool IsGaussianSplatPly(const char* fullPath)
        {
            std::ifstream file(fullPath, std::ios::binary);
            if (!file.is_open())
            {
                return false;
            }

            std::unordered_set<std::string> properties;
            std::string line;
            bool inVertexElement = false;
            while (std::getline(file, line))
            {
                if (!line.empty() && line.back() == '\r')
                {
                    line.pop_back();
                }

                if (line.rfind("element ", 0) == 0)
                {
                    inVertexElement = line.rfind("element vertex ", 0) == 0;
                }
                else if (inVertexElement && line.rfind("property ", 0) == 0)
                {
                    const size_t lastSpace = line.find_last_of(' ');
                    if (lastSpace != std::string::npos)
                    {
                        properties.insert(line.substr(lastSpace + 1));
                    }
                }
                else if (line == "end_header")
                {
                    break;
                }
            }

            return std::ranges::all_of(
                RequiredSplatVertexProperties,
                [&](const char* property)
                {
                    return properties.contains(property);
                });
        }

        //! Reads every vertex property a splat needs from the .ply at fullPath, with the SH degree implied by its
        //! f_rest_* count. Fails with a description of the problem if the file is malformed or lacks any of
        //! those properties.
        AZ::Outcome<PlyVertexProperties, AZStd::string> ReadPlyVertexProperties(const char* fullPath)
        {
            std::ifstream file(fullPath, std::ios::binary);
            if (!file.is_open())
            {
                return AZ::Failure(AZStd::string("the file cannot be opened"));
            }

            // happly does not check its reads: a short read leaves whatever it got in the property's values.
            // Make a failed read throw instead, so a truncated file fails the job.
            file.exceptions(std::ios::failbit | std::ios::badbit);

            // happly reports other malformed input (bad header, missing "vertex" element, missing or uncoercible
            // properties) by throwing.
            try
            {
                happly::PLYData plyIn(file);
                auto& vertexElement = plyIn.getElement("vertex");

                const size_t restPropertyCount = std::ranges::count_if(vertexElement.getPropertyNames(), IsRestProperty);
                const uint32_t shDegree = Math::ShDegreeFromRestCount(restPropertyCount);
                const uint32_t restCount = Math::ShRestCoeffsPerSplat(shDegree);

                PlyVertexProperties out;
                out.m_shDegree = shDegree;
                out.m_count = vertexElement.count;
                out.m_posX = vertexElement.getProperty<float>("x");
                out.m_posY = vertexElement.getProperty<float>("y");
                out.m_posZ = vertexElement.getProperty<float>("z");
                out.m_opacity = vertexElement.getProperty<float>("opacity");
                out.m_scale0 = vertexElement.getProperty<float>("scale_0");
                out.m_scale1 = vertexElement.getProperty<float>("scale_1");
                out.m_scale2 = vertexElement.getProperty<float>("scale_2");
                out.m_rot0 = vertexElement.getProperty<float>("rot_0");
                out.m_rot1 = vertexElement.getProperty<float>("rot_1");
                out.m_rot2 = vertexElement.getProperty<float>("rot_2");
                out.m_rot3 = vertexElement.getProperty<float>("rot_3");
                out.m_shDc0 = vertexElement.getProperty<float>("f_dc_0");
                out.m_shDc1 = vertexElement.getProperty<float>("f_dc_1");
                out.m_shDc2 = vertexElement.getProperty<float>("f_dc_2");
                out.m_shRest.resize(restCount);
                for (uint32_t i = 0; i < restCount; ++i)
                {
                    out.m_shRest[i] = vertexElement.getProperty<float>(FRestPropertyName(i));
                }
                return AZ::Success(AZStd::move(out));
            } catch (const std::ios_base::failure&)
            {
                return AZ::Failure(
                    AZStd::string(file.eof() ? "the file ends before all the data its header declares" : "the file cannot be read"));
            } catch (const std::exception& e)
            {
                return AZ::Failure(AZStd::string(e.what()));
            }
        }
    } // namespace

    GaussianSplatBuilder::GaussianSplatBuilder()
    {
        AssetBuilderSDK::AssetBuilderDesc gaussianSplatAssetBuilderDescriptor;
        gaussianSplatAssetBuilderDescriptor.m_name = GaussianSplatBuilderJobKey;
        // 2: scale axes and SH "rest" coefficients go through the Y-up -> Z-up basis change.
        // 3: SH "rest" coefficients are stored as binary16, in a flat per-asset array rather than per vertex.
        gaussianSplatAssetBuilderDescriptor.m_version = 3;
        gaussianSplatAssetBuilderDescriptor.m_busId = azrtti_typeid<GaussianSplatBuilder>();
        gaussianSplatAssetBuilderDescriptor.m_analysisFingerprint = AZStd::string::format(
            "%u,%zu,%zu",
            GaussianSplatAsset::GaussianSplatMagicNumber,
            sizeof(GaussianSplatAsset::CloudHeader),
            sizeof(GaussianSplatAsset::CloudVertex));
        gaussianSplatAssetBuilderDescriptor.m_patterns.push_back(
            AssetBuilderSDK::AssetBuilderPattern("*.ply", AssetBuilderSDK::AssetBuilderPattern::PatternType::Wildcard));

        gaussianSplatAssetBuilderDescriptor.m_createJobFunction = [this](auto&& request, auto&& response)
        {
            return CreateJobs(request, response);
        };

        gaussianSplatAssetBuilderDescriptor.m_processJobFunction = [this](auto&& request, auto&& response)
        {
            return ProcessJob(request, response);
        };

        BusConnect(gaussianSplatAssetBuilderDescriptor.m_busId);

        AssetBuilderSDK::AssetBuilderBus::Broadcast(
            &AssetBuilderSDK::AssetBuilderBus::Handler::RegisterBuilderInformation, gaussianSplatAssetBuilderDescriptor);
    }

    GaussianSplatBuilder::~GaussianSplatBuilder()
    {
        BusDisconnect();
    }

    void GaussianSplatBuilder::CreateJobs(
        const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const
    {
        const auto fullSourcePath = AZ::IO::Path(request.m_watchFolder) / AZ::IO::Path(request.m_sourceFile);

        if (!IsGaussianSplatPly(fullSourcePath.String().c_str()))
        {
            // Not a Gaussian-Splat .ply (missing positions or opacity/scale/rot/SH-DC vertex properties)
            response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
            return;
        }

        for (const AssetBuilderSDK::PlatformInfo& platformInfo : request.m_enabledPlatforms)
        {
            AssetBuilderSDK::JobDescriptor jobDescriptor;
            jobDescriptor.m_critical = false;
            jobDescriptor.m_jobKey = "GaussianSplat Asset";
            jobDescriptor.SetPlatformIdentifier(platformInfo.m_identifier.c_str());

            response.m_createJobOutputs.push_back(jobDescriptor);
        }

        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
    }

    void GaussianSplatBuilder::ProcessJob(
        const AssetBuilderSDK::ProcessJobRequest& request, AssetBuilderSDK::ProcessJobResponse& response) const
    {
        response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;

        const auto readOutcome = ReadPlyVertexProperties(request.m_fullPath.c_str());
        if (!readOutcome.IsSuccess())
        {
            AZ_TracePrintf(
                AssetBuilderSDK::ErrorWindow,
                "Error: Failed to read %s: %s.\n",
                request.m_fullPath.c_str(),
                readOutcome.GetError().c_str());
            return;
        }
        const PlyVertexProperties& ply = readOutcome.GetValue();
        const uint32_t shDegree = ply.m_shDegree;
        const size_t numVertices = ply.m_count;
        const uint32_t restCoeffsPerChannel = Math::ShRestCoeffsPerChannel(shDegree);

        AZ::Data::Asset<GaussianSplatAsset> gaussianSplatAsset;
        gaussianSplatAsset.Create(AZ::Data::AssetId(AZ::Uuid::CreateRandom()));
        gaussianSplatAsset->m_shDegree = shDegree;
        gaussianSplatAsset->m_data.resize(numVertices);
        const uint32_t restCoeffsPerSplat = Math::ShRestCoeffsPerSplat(shDegree);
        gaussianSplatAsset->m_shRest.resize(numVertices * restCoeffsPerSplat);

        // Scratch for one splat's "rest" coefficients. The Y-up -> Z-up basis change mixes coefficients
        // together, so it runs at full precision here and only the result is narrowed to binary16.
        AZStd::vector<float> restScratch(restCoeffsPerSplat);

        for (size_t i = 0; i < numVertices; ++i)
        {
            GaussianSplatAsset::CloudVertex& vertex = gaussianSplatAsset->m_data[i];
            const AZ::Vector3 position = Math::ConvertPositionYUpToZUp(ply.m_posX[i], ply.m_posY[i], ply.m_posZ[i]);
            vertex.m_position = { position.GetX(), position.GetY(), position.GetZ() };

            // .ply stores the rotation quaternion scalar-first (rot_0=w, rot_1=x, rot_2=y, rot_3=z), per the
            // INRIA 3DGS training convention - ActivateRotation takes (x, y, z, w), so reorder accordingly.
            const AZ::Quaternion rotation =
                Math::ConvertRotationYUpToZUp(Math::ActivateRotation(ply.m_rot1[i], ply.m_rot2[i], ply.m_rot3[i], ply.m_rot0[i]));
            vertex.m_rotation = { rotation.GetX(), rotation.GetY(), rotation.GetZ(), rotation.GetW() };

            const AZ::Vector3 scale = Math::ConvertScaleYUpToZUp(
                Math::ActivateScale(ply.m_scale0[i]), Math::ActivateScale(ply.m_scale1[i]), Math::ActivateScale(ply.m_scale2[i]));
            vertex.m_scale = { scale.GetX(), scale.GetY(), scale.GetZ() };

            vertex.m_opacity = Math::ActivateOpacity(ply.m_opacity[i]);

            // The DC (degree-0) SH term is rotation-invariant, so the basis change leaves it alone.
            vertex.m_shDc = { ply.m_shDc0[i], ply.m_shDc1[i], ply.m_shDc2[i] };

            // f_rest_0..(3k-1) in the .ply is already channel-major (R block, then G block, then B block, each
            // of length k) - the same convention GaussianSplatAsset::m_shRest uses, so this is a straight copy...
            for (uint32_t j = 0; j < restCoeffsPerSplat; ++j)
            {
                restScratch[j] = ply.m_shRest[j][i];
            }

            // ...but the coefficients themselves are still expressed in the .ply's frame, so apply the same
            // Y-up -> Z-up basis change to each channel's block independently.
            for (uint32_t channel = 0; channel < 3; ++channel)
            {
                Math::ConvertShRestYUpToZUp(shDegree, restScratch.data() + static_cast<size_t>(channel * restCoeffsPerChannel));
            }

            uint16_t* const splatRest = gaussianSplatAsset->m_shRest.data() + i * restCoeffsPerSplat;
            for (uint32_t j = 0; j < restCoeffsPerSplat; ++j)
            {
                splatRest[j] = Math::FloatToHalf(restScratch[j]);
            }
        }

        if (auto* assetHandler = AZ::Data::AssetManager::Instance().GetHandler(azrtti_typeid<GaussianSplatAsset>()))
        {
            auto tempAssetOutputPath = AZ::IO::Path(request.m_tempDirPath) / request.m_sourceFile;
            tempAssetOutputPath.ReplaceExtension(GaussianSplatAsset::Extension);
            AZ::IO::FileIOStream outStream(
                tempAssetOutputPath.String().c_str(), AZ::IO::OpenMode::ModeWrite | AZ::IO::OpenMode::ModeCreatePath);
            if (!outStream.IsOpen())
            {
                AZ_TracePrintf(
                    AssetBuilderSDK::ErrorWindow, "Error: Failed job %s because file cannot be created.\n", request.m_fullPath.c_str());
                response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
                return;
            }
            if (assetHandler->SaveAssetData(gaussianSplatAsset, &outStream))
            {
                AssetBuilderSDK::JobProduct gaussianSplatJobProduct;
                gaussianSplatJobProduct.m_productFileName = tempAssetOutputPath.String();
                gaussianSplatJobProduct.m_productSubID = 0;
                gaussianSplatJobProduct.m_productAssetType = azrtti_typeid<GaussianSplatAsset>();
                gaussianSplatJobProduct.m_dependenciesHandled = true;
                response.m_outputProducts.push_back(AZStd::move(gaussianSplatJobProduct));
                response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Success;
                return;
            }
            AZ_Error("GaussianSplatBuilder", false, "The asset could not be saved to file: %s", tempAssetOutputPath.c_str());
        }
        response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
    }

} // namespace GaussianSplat
