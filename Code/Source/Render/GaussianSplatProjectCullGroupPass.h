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

#include <Atom/RPI.Public/Pass/ParentPass.h>

namespace GaussianSplat
{
    //! Bundles one GaussianSplatProjectCullPass per cloud registered at GaussianSplatFeatureProcessor under a
    //! single named child of the shared parent pass, and exposes the two buffers they append into on its own
    //! output slots. Its GaussianSplatVisibleCounterClear child comes from the template (so it is always
    //! children[0], and therefore zeroes the visible counter before any cull pass appends to it); the per-cloud
    //! cull passes are created in CreateChildPassesInternal, which AZ::RPI::ParentPass::CreateChildPasses runs
    //! after the template children.
    //! Modeled on SkyAtmosphereParentPass, the engine's own "one child pass per feature-processor-registered
    //! object" pattern.
    class GaussianSplatProjectCullGroupPass final : public AZ::RPI::ParentPass
    {
        AZ_RPI_PASS(GaussianSplatProjectCullGroupPass);

    public:
        AZ_RTTI(GaussianSplatProjectCullGroupPass, "{5C2E8B91-73A4-4D06-9F1B-C48A2E7D5036}", AZ::RPI::ParentPass);
        AZ_CLASS_ALLOCATOR(GaussianSplatProjectCullGroupPass, AZ::SystemAllocator);
        ~GaussianSplatProjectCullGroupPass() = default;

        static AZ::RPI::Ptr<GaussianSplatProjectCullGroupPass> Create(const AZ::RPI::PassDescriptor& descriptor);

        //! Discards and re-creates the per-cloud children from the feature processor's current cloud set.
        //! Called by GaussianSplatFeatureProcessor whenever a cloud is acquired or released; the new children
        //! are built on the next AZ::RPI::PassSystem::FrameUpdate, not synchronously here.
        void QueueRebuildChildren();

    protected:
        explicit GaussianSplatProjectCullGroupPass(const AZ::RPI::PassDescriptor& descriptor);

        //! Attaches the feature processor's shared projection buffer to SplatProjectionOut, ahead of the base
        //! creating the children that connect to it. Feature-processor-owned because its size follows the
        //! runtime splat count, which a template attachment cannot express.
        void BuildInternal() override;

        //! Creates one GaussianSplatProjectCullPass per cloud currently registered at the feature processor,
        //! each connected to this group's SplatProjectionOut and to the GaussianSplatVisibleCounterClear sibling.
        void CreateChildPassesInternal() override;
    };
} // namespace GaussianSplat
