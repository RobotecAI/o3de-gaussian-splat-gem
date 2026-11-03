# Copyright 2026, Robotec.ai sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(FILES
    Source/GaussianSplatModuleInterface.cpp
    Source/GaussianSplatModuleInterface.h
    Source/Clients/GaussianSplatSystemComponent.cpp
    Source/Clients/GaussianSplatSystemComponent.h
    Source/Clients/GaussianSplatAsset.cpp
    Source/Clients/GaussianSplatComponent.cpp
    Source/Clients/GaussianSplatComponent.h
    Source/Clients/GaussianSplatComponentController.h
    Source/Clients/GaussianSplatComponentController.cpp
    Source/Utils/GaussianSplatMath.h
    Source/Utils/GaussianSplatMath.cpp
    Source/Utils/GaussianSplatPacking.h
    Source/Utils/GaussianSplatPacking.cpp
    Source/Render/GaussianSplatFeatureProcessor.h
    Source/Render/GaussianSplatFeatureProcessor.cpp
    Source/Render/GaussianSplatTileGrid.h
    Source/Render/GaussianSplatTileGrid.cpp
    Source/Render/GaussianSplatTileShadingPass.h
    Source/Render/GaussianSplatTileShadingPass.cpp
    Source/Render/GaussianSplatProjectCullPass.h
    Source/Render/GaussianSplatProjectCullGroupPass.cpp
    Source/Render/GaussianSplatProjectCullGroupPass.h
    Source/Render/GaussianSplatProjectCullPass.cpp
    Source/Render/GaussianSplatRadixBuildKeysPass.h
    Source/Render/GaussianSplatRadixBuildKeysPass.cpp
    Source/Render/GaussianSplatRadixPrepareBuildKeysDispatchPass.h
    Source/Render/GaussianSplatRadixPrepareBuildKeysDispatchPass.cpp
    Source/Render/GaussianSplatRadixPrepareSortDispatchPass.h
    Source/Render/GaussianSplatRadixPrepareSortDispatchPass.cpp
    Source/Render/GaussianSplatRadixOnesweepPass.h
    Source/Render/GaussianSplatRadixOnesweepPass.cpp
    Source/Render/GaussianSplatRadixUnpackPass.h
    Source/Render/GaussianSplatRadixUnpackPass.cpp
)
