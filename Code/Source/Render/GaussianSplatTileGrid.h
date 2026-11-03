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

#include <AzCore/Math/Vector2.h>
#include <AzCore/base.h>
#include <AzCore/std/optional.h>

namespace AZ::RPI
{
    class Pass;
    class RenderPipeline;
} // namespace AZ::RPI

namespace GaussianSplat::TileGrid
{
    // Size in both axis that defines a tile in pixels. Has to be the same as TileSizePixels in shader code.
    constexpr uint32_t TileSizePixels = 16;

    //! Reads gaussianSplatParentPass's own "ColorInputOutput" attachment size - the resolution the GaussianSplat
    //! pass chain injected into one particular RenderPipeline actually renders at this frame. Returns
    //! AZStd::nullopt if the binding or its attachment isn't resolved yet.
    AZStd::optional<AZ::Vector2> TryGetResolvedFrameSize(AZ::RPI::Pass* gaussianSplatParentPass);

    //! The size pipeline itself renders at, read from its root pass rather than from the GaussianSplat chain,
    //! so it resolves before that chain is injected. Mirrors AZ::RPI::RenderPipeline::UpdateViewportScissor:
    //! the first output or input/output binding on the root pass that carries an image attachment.
    //!
    //! That root attachment is the pipeline's presentation target, which a pipeline rescaling between its
    //! post-process output and its root would make larger than the chain's own "ColorInputOutput". Callers
    //! sizing anything tile-indexed must therefore re-check against TryGetResolvedFrameSize once the chain is
    //! up - see GaussianSplatFeatureProcessor::Simulate.
    AZStd::optional<AZ::Vector2> TryGetPipelineFrameSize(AZ::RPI::RenderPipeline* pipeline);

    //! Same as TryGetResolvedFrameSize, but searches pass and then each of its ancestors in turn, so a pass
    //! nested more than one level below the shared parent (every GaussianSplatProjectCullPass, under
    //! GaussianSplatProjectCullGroupPass) still resolves the same frame size as a direct child would.
    AZStd::optional<AZ::Vector2> TryGetResolvedFrameSizeFromAncestor(AZ::RPI::Pass* pass);

    //! Same as TryGetResolvedFrameSizeFromAncestor, but falls back to a 1920x1080 default instead of returning
    //! nullopt. Pass a pass's GetParent() - the search walks up from there.
    AZ::Vector2 GetFrameSize(AZ::RPI::Pass* gaussianSplatParentPass);

    inline uint32_t TileCountX(const AZ::Vector2& frameSize)
    {
        return (static_cast<uint32_t>(frameSize.GetX()) + TileSizePixels - 1) / TileSizePixels;
    }

    inline uint32_t TileCountY(const AZ::Vector2& frameSize)
    {
        return (static_cast<uint32_t>(frameSize.GetY()) + TileSizePixels - 1) / TileSizePixels;
    }
} // namespace GaussianSplat::TileGrid
