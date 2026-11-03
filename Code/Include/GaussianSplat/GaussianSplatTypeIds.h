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

namespace GaussianSplat
{
    // System Component TypeIds
    inline constexpr const char* GaussianSplatSystemComponentTypeId = "{5B537173-FB1A-4DC5-8C39-76E5487BFE7D}";
    inline constexpr const char* GaussianSplatEditorSystemComponentTypeId = "{3581A0AD-C6DB-4DDB-A668-B8FD84C72597}";
    inline constexpr const char* GaussianSplatAssetBuilderSystemComponentTypeId = "{85927BCC-D08C-40F2-948B-BDF25B8C9EE6}";

    // Module derived classes TypeIds
    inline constexpr const char* GaussianSplatModuleInterfaceTypeId = "{404837E0-A0D8-446B-8D8B-4B701E620713}";
    inline constexpr const char* GaussianSplatModuleTypeId = "{0DF4947D-5C81-4941-831E-B226ECBB6667}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* GaussianSplatEditorModuleTypeId = GaussianSplatModuleTypeId;

    // Components TypeIds
    inline constexpr const char* GaussianSplatComponentTypeId = "{F6FB48DE-0D08-4996-8CE4-6722E43EDE4B}";
    inline constexpr const char* GaussianSplatEditorComponentTypeId = "{4119DE1C-61AE-493E-ACB6-4ECFB2033C23}";
    inline constexpr const char* GaussianSplatComponentConfigTypeId = "{3A291A36-726B-40E5-BFB6-5972517FAF59}";

    // Assets TypeIds
    inline constexpr const char* GaussianSplatAssetTypeId = "{3B48AF46-BDA8-4DDB-BB47-3F03E3DCDDB5}";
    inline constexpr const char* SplatVertexAssetTypeId = "{188EBCBB-AAB8-400F-8C00-E1ECC576E9C9}";

    // Interface / bus TypeIds
    inline constexpr const char* GaussianSplatConfigurationRequestsTypeId = "{E8DBDFBA-FD08-4DBB-A12B-92DB8E523028}";
    inline constexpr const char* GaussianSplatFeatureProcessorInterfaceTypeId = "{EFE1B8EB-6243-4689-9DE4-266E094F5EA3}";
    inline constexpr const char* GaussianSplatFeatureProcessorTypeId = "{11A21865-2C53-4BB2-A6D4-81EEDE24DA2D}";
    inline constexpr const char* GaussianSplatComponentControllerTypeId = "{65E8E499-BB5E-45A2-B13A-C73DF729C739}";
    inline constexpr const char* GaussianSplatBuilderTypeId = "{55C5AFD8-965D-467E-8397-29B69B745FAC}";
} // namespace GaussianSplat
