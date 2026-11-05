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

#include <AssetBuilderSDK/AssetBuilderBusses.h>
#include <AssetBuilderSDK/AssetBuilderSDK.h>
#include <AzCore/std/containers/array.h>
#include <GaussianSplat/GaussianSplatTypeIds.h>

namespace GaussianSplat
{
    [[maybe_unused]] constexpr const char* GaussianSplatBuilderName = "GaussianSplatBuilder";

    //! The "vertex" properties a .ply must have for the builder to create jobs for it. Every property but the
    //! positions sets a 3DGS .ply apart from a plain point cloud. The f_rest_* properties are optional: how many
    //! there are sets the SH degree.
    inline constexpr AZStd::array<const char*, 14> RequiredSplatVertexProperties = {
        "x", "y", "z", "opacity", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3", "f_dc_0", "f_dc_1", "f_dc_2"
    };

    class GaussianSplatBuilder : public AssetBuilderSDK::AssetBuilderCommandBus::Handler
    {
    public:
        AZ_RTTI(GaussianSplatBuilder, GaussianSplatBuilderTypeId);

        GaussianSplatBuilder();
        ~GaussianSplatBuilder() override;

        // AssetBuilderSDK::AssetBuilderCommandBus overrides...
        void CreateJobs(const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const;
        void ProcessJob(const AssetBuilderSDK::ProcessJobRequest& request, AssetBuilderSDK::ProcessJobResponse& response) const;
        void ShutDown() override
        {
        }
    };
} // namespace GaussianSplat
