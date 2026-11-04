# Changelog

## [1.0.2] - 2026-11-04
This is the first open-source release of the Gaussian Splat Gem for [Open 3D Engine](https://o3de.org). It renders [3D Gaussian Splatting](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/) scenes in O3DE's Atom renderer, entirely on the GPU.

### Highlights
- .ply import: reads 3DGS-standard .ply files and converts them to .gsplat assets through the Asset Processor.
- Gaussian Splat component: add the component to an entity, pick an asset, and tune these settings:
  - Opacity Multiplier
  - Scale Multiplier
  - Max SH Degree
- GPU rendering: a compute pass projects, culls and colors the splats (including spherical-harmonics color). The splats are then sorted with a custom Onesweep radix sort and blended front to back, one screen tile at a time.
- Depth output: splat depth is merged into the pipeline's linear depth buffer, so passes that run later can use it.
- Configurable: the sort buffer capacity and the depth coverage threshold are set through the Settings Registry.

### Requirements
- O3DE 2.6.0 or newer (tested with 2605.0 and 2610.0)
- Linux with the Vulkan RHI
- GPU support for SM6.0 wave intrinsics

### License
Apache-2.0

See the [README](https://github.com/RobotecAI/o3de-gaussian-splat-gem) and Docs/ for usage and design details.

**Full Changelog**: https://github.com/RobotecAI/o3de-gaussian-splat-gem/commits/v1.0.2
