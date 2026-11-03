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

#include <Atom/RPI.Public/Pass/ComputePass.h>

namespace GaussianSplat
{
    //! Final stage of GPU tile binning, run after the fourth (and last) radix-sort digit round. The entry
    //! buffer is now globally sorted by the combined (tileId, depth) key, so every tile's entries form one
    //! contiguous run. One thread per sorted entry: unpacks that entry's visibleSlot into
    //! m_tileSplatIndices[entryIndex], and, only for threads at a tile-run boundary, writes that run's
    //! (offset, count) into m_tileRanges[tileId]. Tiles with no entries this frame must read back (0, 0)
    //! rather than a stale range from a previous frame; the "TileRangesOut" slot gets that for free by
    //! declaring a Clear load action, which has the RHI zero the buffer before this pass's dispatch.
    class GaussianSplatRadixUnpackPass final : public AZ::RPI::ComputePass
    {
        AZ_RPI_PASS(GaussianSplatRadixUnpackPass);

    public:
        AZ_RTTI(GaussianSplatRadixUnpackPass, "{59AA447A-6512-41C3-BD49-9FBED584792F}", AZ::RPI::ComputePass);
        AZ_CLASS_ALLOCATOR(GaussianSplatRadixUnpackPass, AZ::SystemAllocator);
        ~GaussianSplatRadixUnpackPass() override = default;

        static AZ::RPI::Ptr<GaussianSplatRadixUnpackPass> Create(const AZ::RPI::PassDescriptor& descriptor);

    protected:
        explicit GaussianSplatRadixUnpackPass(const AZ::RPI::PassDescriptor& descriptor);
        //! Attaches the tile-splat-index and tile-ranges buffers from GaussianSplatFeatureProcessor's
        //! per-pipeline set. "SortedEntries" is wired via a Connections entry in GaussianSplatParent.pass
        //! instead, to the last GaussianSplatRadixOnesweepPass round's output.
        void BuildInternal() override;
        //! Stages this frame's constants, then lets the base compile the pass SRG. Must happen here rather
        //! than in BuildCommandListInternal, which runs in the execute phase - after
        //! AZ::RPI::ComputePass::CompileResources has already compiled the SRG.
        void CompileResources(const AZ::RHI::FrameGraphCompileContext& context) override;

    private:
        AZ::RHI::ShaderInputNameIndex m_tileCountIndex = "m_tileCount";
    };
} // namespace GaussianSplat
