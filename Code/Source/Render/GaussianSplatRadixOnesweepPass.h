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
    //! The whole of one 8-bit LSD radix-sort digit round, in a single dispatch. Instantiated NumRadixRounds
    //! times (GaussianSplatRadixOnesweep0..5 in GaussianSplatParent.pass), once per digit shift
    //! 0/8/16/24/32/40 across the (depth, tileId) key - see GaussianSplatCommon.azsli's ExtractRadixDigit -
    //! chaining each instance's "EntriesOut" into the next instance's "Entries". The round's look-back state
    //! has to start zeroed, which the RHI does: the "TileStatus" and "TileCounter" slots declare a Clear load
    //! action, so both buffers are filled with zero before each round's dispatch.
    //!
    //! Entries are processed in blocks of RadixBlockSize, one 256-thread group per block, each thread owning
    //! RadixKeysPerThread entries. Each group reads its block once and, holding it in registers, ranks it with
    //! wave intrinsics, derives the block's own digit histogram from that ranking, resolves its output offsets
    //! by decoupled look-back against the other blocks, and scatters. Every part of that bookkeeping is per
    //! block rather than per entry, which is why a block spans many entries per thread: the status-buffer
    //! clear, the two status publishes and the look-back chain all shrink by RadixKeysPerThread. This is the single-pass structure of
    //! Merrill & Garland, "Single-pass Parallel Prefix Scan with Decoupled Look-back", applied to radix sort as in Adinets & Merrill's
    //! Onesweep: the scan is fused into the pass that produces its input and consumes its output, so neither
    //! the per-block histograms nor the per-block offsets are ever written to memory.
    //!
    //! Ranking is stable - an InterlockedAdd-based scatter would assign ranks in GPU execution order, silently
    //! breaking the stability LSD radix sort's correctness depends on across all of its rounds. Stability
    //! without serialization comes from SM6.0 wave intrinsics (WaveActiveBallot et al.), which requires
    //! RHI::DeviceFeatures::m_waveOperation; there is currently no non-wave-intrinsic fallback path.
    //!
    //! The look-back only yields a digit's prefix across blocks. Each output position also needs that digit's
    //! cross-digit base, which cannot travel along the look-back chain: it sums over all blocks, including
    //! tiles with higher ids than the tile that needs it, and a tile may only wait on lower tile ids. That base
    //! is the same for every block in the round, and the GaussianSplatRadixDigitBase pass computes every round's
    //! copy of it once per frame, so this pass just reads the precomputed table.
    class GaussianSplatRadixOnesweepPass final : public AZ::RPI::ComputePass
    {
        AZ_RPI_PASS(GaussianSplatRadixOnesweepPass);

    public:
        AZ_RTTI(GaussianSplatRadixOnesweepPass, "{7E0C69DD-223A-4B68-896C-63D592C999D4}", AZ::RPI::ComputePass);
        AZ_CLASS_ALLOCATOR(GaussianSplatRadixOnesweepPass, AZ::SystemAllocator);
        ~GaussianSplatRadixOnesweepPass() override = default;

        static AZ::RPI::Ptr<GaussianSplatRadixOnesweepPass> Create(const AZ::RPI::PassDescriptor& descriptor);

    protected:
        explicit GaussianSplatRadixOnesweepPass(const AZ::RPI::PassDescriptor& descriptor);
        //! Takes this instance's round index from the trailing digit of its own template-authored name
        //! (GaussianSplatRadixOnesweep0..5) and binds every GaussianSplatFeatureProcessor-owned buffer the
        //! round needs from it, so nothing CPU-side has to resolve or push per-round state. Only "DigitBase"
        //! and "IndirectArgs" are wired in GaussianSplatParent.pass, being transient attachments other
        //! passes own.
        void BuildInternal() override;
        //! Stages this frame's constants, then lets the base compile the pass SRG. Must happen here rather
        //! than in BuildCommandListInternal, which runs in the execute phase - after
        //! AZ::RPI::ComputePass::CompileResources has already compiled the SRG. Pushing the round every frame
        //! also keeps it correct across a shader hot-reload, which replaces the pass SRG with a freshly
        //! created (so zero-initialized) one without re-running BuildInternal.
        void CompileResources(const AZ::RHI::FrameGraphCompileContext& context) override;

    private:
        AZ::RHI::ShaderInputNameIndex m_roundIndex = "m_round";
        //! The only CPU-pushed constant (the shader derives its digit shift from it) - entryCount and the
        //! dispatch's thread-group count both come from buffers GaussianSplatRadixPrepareSortDispatchPass
        //! writes each frame. Fixed for the life of the instance; CompileResources stages it every frame.
        uint32_t m_round = 0;
    };
} // namespace GaussianSplat
