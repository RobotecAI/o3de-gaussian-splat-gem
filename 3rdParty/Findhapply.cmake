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

# Fetches the "hapPLY" single-header PLY parsing library (nmwsharp/happly) at configure time.
if (TARGET 3rdParty::happly)
    return()
endif()

set(HAPPLY_COMMIT "ed5480454d15e5287370eadc3682e9d40ba395e1")
set(HAPPLY_URL "https://raw.githubusercontent.com/nmwsharp/happly/${HAPPLY_COMMIT}/happly.h")
set(HAPPLY_SHA256 "05a2bc71e9f3a48b2b996d06c5516254132d0f26f52b3fd845aff2492bf51f82")

include(FetchContent)
FetchContent_Declare(
    happly
    URL ${HAPPLY_URL}
    URL_HASH SHA256=${HAPPLY_SHA256}
    DOWNLOAD_NO_EXTRACT TRUE
)
FetchContent_MakeAvailable(happly)

message(STATUS "GaussianSplat Gem uses happly@${HAPPLY_COMMIT} (MIT License) ${HAPPLY_URL}")

add_library(happly INTERFACE)
target_include_directories(happly INTERFACE ${happly_SOURCE_DIR})

get_property(this_gem_root GLOBAL PROPERTY "@GEMROOT:${gem_name}@")
ly_get_engine_relative_source_dir(${this_gem_root} relative_this_gem_root)
set_property(TARGET happly PROPERTY FOLDER "${relative_this_gem_root}/External")

add_library(3rdParty::happly ALIAS happly)

set(happly_FOUND TRUE)
