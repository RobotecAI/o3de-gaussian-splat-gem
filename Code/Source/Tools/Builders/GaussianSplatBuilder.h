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
#include <GaussianSplat/GaussianSplatTypeIds.h>

namespace GaussianSplat
{
    [[maybe_unused]] constexpr const char* GaussianSplatBuilderName = "GaussianSplatBuilder";

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

        //! True if the .ply's "vertex" element has 3DGS properties (opacity, scale_0..2, rot_0..3, f_dc_0..2);
        //! outputs the implied SH degree. Reads only the header (cheap per CreateJobs call) to avoid claiming
        //! plain point-cloud .ply files that Pointcloud's builder also watches. Public for testing.
        static bool IsGaussianSplatPly(const char* fullPath, uint32_t& outShDegree);
    };
} // namespace GaussianSplat
