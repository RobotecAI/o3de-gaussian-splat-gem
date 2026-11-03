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

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <GaussianSplat/GaussianSplatAsset.h>
#include <Utils/GaussianSplatMath.h>
#include <cstring>

namespace GaussianSplat
{
    namespace
    {
        // Fixed portion of an on-disk vertex record
        constexpr size_t PositionCount = 3;
        constexpr size_t RotationCount = 4;
        constexpr size_t ScaleCount = 3;
        constexpr size_t OpacityCount = 1;
        constexpr size_t ShDcCount = 3;
        constexpr size_t FixedPrefixFloatCount = PositionCount + RotationCount + ScaleCount + OpacityCount + ShDcCount;

        //! Size of one on-disk vertex record: the fixed float prefix, then this degree's "rest" coefficients
        //! as binary16. Recorded in CloudHeader::m_elementSize and checked on load.
        size_t DiskElementSize(uint32_t shDegree)
        {
            return sizeof(float) * FixedPrefixFloatCount + sizeof(uint16_t) * Math::ShRestCoeffsPerSplat(shDegree);
        }

        void PackVertex(const GaussianSplatAsset::CloudVertex& vertex, const uint16_t* shRest, uint32_t restCoeffsStored, uint8_t* cursor)
        {
            memcpy(cursor, vertex.m_position.data(), sizeof(float) * PositionCount);
            cursor += sizeof(float) * PositionCount;
            memcpy(cursor, vertex.m_rotation.data(), sizeof(float) * RotationCount);
            cursor += sizeof(float) * RotationCount;
            memcpy(cursor, vertex.m_scale.data(), sizeof(float) * ScaleCount);
            cursor += sizeof(float) * ScaleCount;
            memcpy(cursor, &vertex.m_opacity, sizeof(float) * OpacityCount);
            cursor += sizeof(float) * OpacityCount;
            memcpy(cursor, vertex.m_shDc.data(), sizeof(float) * ShDcCount);
            cursor += sizeof(float) * ShDcCount;
            if (restCoeffsStored != 0)
            {
                memcpy(cursor, shRest, sizeof(uint16_t) * restCoeffsStored);
            }
        }

        void UnpackVertex(GaussianSplatAsset::CloudVertex& vertex, uint16_t* shRest, uint32_t restCoeffsStored, const uint8_t* cursor)
        {
            memcpy(vertex.m_position.data(), cursor, sizeof(float) * PositionCount);
            cursor += sizeof(float) * PositionCount;
            memcpy(vertex.m_rotation.data(), cursor, sizeof(float) * RotationCount);
            cursor += sizeof(float) * RotationCount;
            memcpy(vertex.m_scale.data(), cursor, sizeof(float) * ScaleCount);
            cursor += sizeof(float) * ScaleCount;
            memcpy(&vertex.m_opacity, cursor, sizeof(float) * OpacityCount);
            cursor += sizeof(float) * OpacityCount;
            memcpy(vertex.m_shDc.data(), cursor, sizeof(float) * ShDcCount);
            cursor += sizeof(float) * ShDcCount;
            if (restCoeffsStored != 0)
            {
                memcpy(shRest, cursor, sizeof(uint16_t) * restCoeffsStored);
            }
        }
    } // namespace

    uint32_t GaussianSplatAsset::ShRestStride() const
    {
        return Math::ShRestCoeffsPerSplat(m_shDegree);
    }

    void GaussianSplatAsset::CloudVertex::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GaussianSplatAsset::CloudVertex>()
                ->Version(1)
                ->Field("m_position", &GaussianSplatAsset::CloudVertex::m_position)
                ->Field("m_rotation", &GaussianSplatAsset::CloudVertex::m_rotation)
                ->Field("m_scale", &GaussianSplatAsset::CloudVertex::m_scale)
                ->Field("m_opacity", &GaussianSplatAsset::CloudVertex::m_opacity)
                ->Field("m_shDc", &GaussianSplatAsset::CloudVertex::m_shDc);
        }
    }

    void GaussianSplatAsset::Reflect(AZ::ReflectContext* context)
    {
        CloudVertex::Reflect(context);
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GaussianSplatAsset, AZ::Data::AssetData>()
                ->Version(1)
                ->Field("m_data", &GaussianSplatAsset::m_data)
                ->Field("m_shRest", &GaussianSplatAsset::m_shRest)
                ->Field("m_shDegree", &GaussianSplatAsset::m_shDegree);
        }
    }

    GaussianSplatAssetHandler::GaussianSplatAssetHandler()
        : AzFramework::GenericAssetHandler<GaussianSplatAsset>(
              GaussianSplatAsset::DisplayName, GaussianSplatAsset::Group, GaussianSplatAsset::Extension)
    {
    }

    bool GaussianSplatAssetHandler::SaveAssetData(const AZ::Data::Asset<AZ::Data::AssetData>& asset, AZ::IO::GenericStream* stream)
    {
        auto* assetData = asset.GetAs<GaussianSplatAsset>();
        AZ_Assert(assetData, "Asset is not of the expected type.");
        AZ_Assert(assetData->m_shDegree <= GaussianSplatAsset::MaxShDegree, "Invalid SH degree on gaussian splat asset.");

        const uint32_t restCoeffsStored = Math::ShRestCoeffsPerSplat(assetData->m_shDegree);
        const size_t elementSize = DiskElementSize(assetData->m_shDegree);
        const size_t numElements = assetData->m_data.size();
        AZ_Assert(
            assetData->m_shRest.size() == numElements * restCoeffsStored,
            "Gaussian splat asset's SH coefficient array does not match its splat count and degree.");

        GaussianSplatAsset::CloudHeader header;
        header.m_magicNumber = GaussianSplatAsset::GaussianSplatMagicNumber;
        header.m_elementSize = static_cast<uint32_t>(elementSize);
        header.m_numPoints = static_cast<uint32_t>(numElements);
        header.m_shDegree = assetData->m_shDegree;

        size_t totalWritten = stream->Write(sizeof(header), &header);
        AZ_Assert(totalWritten == sizeof(header), "Failed to write header data to stream.");

        AZStd::vector<uint8_t> vertexBuffer(elementSize);
        for (size_t i = 0; i < numElements; ++i)
        {
            PackVertex(assetData->m_data[i], assetData->ShRestForSplat(i), restCoeffsStored, vertexBuffer.data());
            totalWritten += stream->Write(elementSize, vertexBuffer.data());
        }

        const size_t expectedSize = sizeof(header) + elementSize * numElements;
        AZ_Assert(totalWritten == expectedSize, "Failed to write all point data to stream.");
        return totalWritten == expectedSize;
    }

    AZ::Data::AssetHandler::LoadResult GaussianSplatAssetHandler::LoadAssetData(
        const AZ::Data::Asset<AZ::Data::AssetData>& asset,
        AZStd::shared_ptr<AZ::Data::AssetDataStream> stream,
        [[maybe_unused]] const AZ::Data::AssetFilterCB& assetLoadFilterCB)
    {
        auto* assetData = asset.GetAs<GaussianSplatAsset>();
        AZ_Assert(assetData, "Asset is not of the expected type.");

        if (assetData == nullptr || stream->GetLength() == 0)

        {
            return AZ::Data::AssetHandler::LoadResult::Error;
        }
        GaussianSplatAsset::CloudHeader header;
        stream->Read(sizeof(header), &header);
        AZ_Assert(
            header.m_magicNumber == GaussianSplatAsset::GaussianSplatMagicNumber, "Invalid magic number in gaussian splat asset file.");
        AZ_Assert(header.m_shDegree <= GaussianSplatAsset::MaxShDegree, "Invalid SH degree in gaussian splat asset file.");

        const uint32_t restCoeffsStored = Math::ShRestCoeffsPerSplat(header.m_shDegree);
        const size_t elementSize = DiskElementSize(header.m_shDegree);
        if (header.m_elementSize != elementSize)
        {
            // A file written against a different vertex layout. Reading it with this layout's stride would
            // silently produce garbage splats, so refuse it and let the AssetProcessor rebuild it instead.
            AZ_Error(
                "GaussianSplat",
                false,
                "Gaussian splat asset has element size %u, but this build reads %zu-byte vertices at SH degree %u.",
                header.m_elementSize,
                elementSize,
                header.m_shDegree);
            return AZ::Data::AssetHandler::LoadResult::Error;
        }

        AZ_Info("GaussianSplat", "Loading gaussian splat cloud with %u points, SH degree %u", header.m_numPoints, header.m_shDegree);

        assetData->m_shDegree = header.m_shDegree;
        assetData->m_data.resize(header.m_numPoints);
        assetData->m_shRest.resize(static_cast<size_t>(header.m_numPoints) * restCoeffsStored);

        AZStd::vector<uint8_t> vertexBuffer(elementSize);
        for (size_t i = 0; i < assetData->m_data.size(); ++i)
        {
            stream->Read(elementSize, vertexBuffer.data());
            UnpackVertex(assetData->m_data[i], assetData->m_shRest.data() + i * restCoeffsStored, restCoeffsStored, vertexBuffer.data());
        }
        return AZ::Data::AssetHandler::LoadResult::LoadComplete;
    }

} // namespace GaussianSplat
