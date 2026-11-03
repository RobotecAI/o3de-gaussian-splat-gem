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
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <GaussianSplat/GaussianSplatAsset.h>
#include <Utils/GaussianSplatMath.h>
#include <cstddef>
#include <fstream>
#include <happly.h>
#include <unordered_set>

namespace GaussianSplat
{
    constexpr const char* GaussianSplatBuilderJobKey = "GaussianSplatBuilder";

    // 3DGS-trained .ply exports store f_rest_* channel-major: the first ShRestCoeffsPerChannel(degree)
    // properties are the R channel's coefficients, the next block is G, the last block is B - the same
    // channel-major layout GaussianSplatAsset::m_shRest uses, so no reordering is needed here.
    namespace
    {
        AZStd::string FRestPropertyName(uint32_t index)
        {
            return AZStd::string::format("f_rest_%u", index);
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

    bool GaussianSplatBuilder::IsGaussianSplatPly(const char* fullPath, uint32_t& outShDegree)
    {
        std::ifstream file(fullPath, std::ios::binary);
        if (!file.is_open())
        {
            return false;
        }

        std::unordered_set<std::string> properties;
        std::string line;
        bool inVertexElement = false;
        size_t restCount = 0;
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
                    std::string propertyName = line.substr(lastSpace + 1);
                    if (propertyName.find("f_rest_") == 0)
                    {
                        ++restCount;
                    }
                    properties.insert(AZStd::move(propertyName));
                }
            }
            else if (line == "end_header")
            {
                break;
            }
        }

        constexpr AZStd::array<const char*, 11> requiredProperties = { "opacity", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1",
                                                                       "rot_2",   "rot_3",   "f_dc_0",  "f_dc_1",  "f_dc_2" };
        for (const char* required : requiredProperties)
        {
            if (properties.find(required) == properties.end())
            {
                return false;
            }
        }

        outShDegree = Math::ShDegreeFromRestCount(restCount);
        return true;
    }

    void GaussianSplatBuilder::CreateJobs(
        const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const
    {
        const auto fullSourcePath = AZ::IO::Path(request.m_watchFolder) / AZ::IO::Path(request.m_sourceFile);

        uint32_t shDegree = 0;
        if (!IsGaussianSplatPly(fullSourcePath.String().c_str(), shDegree))
        {
            // Not a Gaussian-Splat .ply (missing opacity/scale/rot/SH-DC vertex properties)
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

        uint32_t shDegree = 0;
        if (!IsGaussianSplatPly(request.m_fullPath.c_str(), shDegree))
        {
            AZ_TracePrintf(
                AssetBuilderSDK::ErrorWindow,
                "Error: %s is not a Gaussian-Splat .ply (missing opacity/scale/rot/SH-DC properties).\n",
                request.m_fullPath.c_str());
            return;
        }

        happly::PLYData plyIn(request.m_fullPath.c_str());
        auto& vertexElement = plyIn.getElement("vertex");
        const size_t numVertices = vertexElement.count;

        const std::vector<float> posX = vertexElement.getProperty<float>("x");
        const std::vector<float> posY = vertexElement.getProperty<float>("y");
        const std::vector<float> posZ = vertexElement.getProperty<float>("z");
        const std::vector<float> opacityRaw = vertexElement.getProperty<float>("opacity");
        const std::vector<float> scale0 = vertexElement.getProperty<float>("scale_0");
        const std::vector<float> scale1 = vertexElement.getProperty<float>("scale_1");
        const std::vector<float> scale2 = vertexElement.getProperty<float>("scale_2");
        const std::vector<float> rot0 = vertexElement.getProperty<float>("rot_0");
        const std::vector<float> rot1 = vertexElement.getProperty<float>("rot_1");
        const std::vector<float> rot2 = vertexElement.getProperty<float>("rot_2");
        const std::vector<float> rot3 = vertexElement.getProperty<float>("rot_3");
        const std::vector<float> shDc0 = vertexElement.getProperty<float>("f_dc_0");
        const std::vector<float> shDc1 = vertexElement.getProperty<float>("f_dc_1");
        const std::vector<float> shDc2 = vertexElement.getProperty<float>("f_dc_2");

        const uint32_t restCoeffsPerChannel = Math::ShRestCoeffsPerChannel(shDegree);
        std::vector<std::vector<float>> shRest(static_cast<size_t>(restCoeffsPerChannel) * 3);
        for (uint32_t i = 0; i < shRest.size(); ++i)
        {
            shRest[i] = vertexElement.getProperty<float>(FRestPropertyName(i).c_str());
        }

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
            const AZ::Vector3 position = Math::ConvertPositionYUpToZUp(posX[i], posY[i], posZ[i]);
            vertex.m_position = { position.GetX(), position.GetY(), position.GetZ() };

            // .ply stores the rotation quaternion scalar-first (rot_0=w, rot_1=x, rot_2=y, rot_3=z), per the
            // INRIA 3DGS training convention - ActivateRotation takes (x, y, z, w), so reorder accordingly.
            const AZ::Quaternion rotation = Math::ConvertRotationYUpToZUp(Math::ActivateRotation(rot1[i], rot2[i], rot3[i], rot0[i]));
            vertex.m_rotation = { rotation.GetX(), rotation.GetY(), rotation.GetZ(), rotation.GetW() };

            const AZ::Vector3 scale =
                Math::ConvertScaleYUpToZUp(Math::ActivateScale(scale0[i]), Math::ActivateScale(scale1[i]), Math::ActivateScale(scale2[i]));
            vertex.m_scale = { scale.GetX(), scale.GetY(), scale.GetZ() };

            vertex.m_opacity = Math::ActivateOpacity(opacityRaw[i]);

            // The DC (degree-0) SH term is rotation-invariant, so the basis change leaves it alone.
            vertex.m_shDc = { shDc0[i], shDc1[i], shDc2[i] };

            // f_rest_0..(3k-1) in the .ply is already channel-major (R block, then G block, then B block, each
            // of length k) - the same convention GaussianSplatAsset::m_shRest uses, so this is a straight copy...
            for (uint32_t j = 0; j < restCoeffsPerSplat; ++j)
            {
                restScratch[j] = shRest[j][i];
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
