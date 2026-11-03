# GaussianSplat — Architecture & Design Decisions

This gem renders [3D Gaussian Splatting](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/) scenes in O3DE/Atom. This document records the design decisions behind its structure and the reasoning for each, for anyone extending or debugging it later. For the rendering pipeline specifically (the most involved part), see [`Rasterizer.md`](./Rasterizer.md).

## Asset format decisions

**Full spherical harmonics, not flat/DC-only color.** A Gaussian-Splat gem that can't reproduce view-dependent specular/reflective detail misses the point of the representation, and the SH evaluation math (see [`Rasterizer.md`](./Rasterizer.md)) is well-defined enough to implement once at full fidelity rather than as a flat-color placeholder needing a later rewrite.

**Variable-length SH on disk, channel-major.** The on-disk `.gsplat` record stores only as many "rest" coefficients as the source `.ply` actually has (0/9/24/45 coefficients for SH degree 0/1/2/3, via `Math::ShDegreeFromRestCount` and `GaussianSplatAsset.cpp`'s `DiskElementSize`), not always the full degree-3 allotment padded with zeros - this avoids wasting space on scenes exported with fewer bands. The layout is **channel-major** (`[R_0..R_(k-1), G_0..G_(k-1), B_0..B_(k-1)]`), matching the `.ply`'s own `f_rest_0..` ordering exactly, so `GaussianSplatBuilder` copies the values as-is with no interleaving math. A four-field `CloudHeader` (magic `'gsPT'`, element size, point count, SH degree) precedes the records; the loader asserts the magic number and rejects the file outright if the stored element size disagrees with what the header's degree implies, since reading a different layout at this stride yields garbage splats rather than a clean failure.

**"Rest" coefficients are binary16, everywhere.** They are small, bounded residuals on top of the DC term, and binary16's ~3 significant decimal digits sit well below the 8-bit-per-channel color the shading pass resolves to - so they are stored as halves on disk, in memory, and on the GPU, at a third of degree-3 float32's 180 bytes/splat. `Math::FloatToHalf`/`HalfToFloat` do the conversion (written in-gem: the engine's only binary16 helper lives in ImageProcessingAtom's private sources, which neither this gem nor the builder module can link against). The DC term stays float32 in the asset, being the term that carries the splat's base color. The Y-up → Z-up basis change runs at full precision in the builder and only its result is narrowed.

In memory the split matches disk: `GaussianSplatAsset::CloudVertex` is a fixed 56 bytes (position, rotation, scale, opacity, SH DC) and the degree 1-3 coefficients live in the asset's own flat `AZStd::vector<uint16_t> m_shRest`, at `ShRestStride()` halves per splat. Both arrays are plain fixed-stride blocks the loader can `resize` and index, and a low-degree cloud pays only for the bands it has - a degree-0 asset costs 56 bytes/splat rather than a degree-3 stride's 236. The GPU layout is derived from this by `Packing::PackCloudSh` rather than copied - see "GPU buffer layout" below.

**Activation functions applied at import time, not render time.** 3DGS training stores opacity as a pre-sigmoid logit and scale as a pre-exp log-value (both need a "raw → linear" activation before use). `GaussianSplatBuilder` applies `Math::ActivateOpacity`/`ActivateScale`/`ActivateRotation` (sigmoid / exp / quaternion-normalize) once, at import, rather than in the shader every frame - cheaper, and keeps the runtime data already in the units the renderer needs.

**Quaternion component order is fixed at import too.** 3DGS `.ply` files store the rotation quaternion scalar-first (`rot_0` = w, `rot_1..3` = x, y, z); `GaussianSplatBuilder` reorders it to the engine's `(x, y, z, w)` at the same point it normalizes.

## Coordinate-system conversion happens in the builder

3DGS training data is Y-up; O3DE is Z-up. The entire basis change is applied **once, at import**, by `GaussianSplatBuilder` - never in a shader and never at load time. Keeping it here means the runtime only ever sees engine-space data, and there is exactly one place to look when a scene comes in mirrored or rotated.

The conversion is `(x, y, z) → (x, z, -y)` (`Math::ConvertPositionYUpToZUp`), and it has to be applied consistently to four different kinds of quantity, which is the subtle part:

- **Position** - the basis change directly.
- **Rotation** (`ConvertRotationYUpToZUp`) - applied by _conjugation_ (`R' = B R Bᵀ`), which for this basis works out to `(x, y, z, w) → (x, z, -y, w)`.
- **Scale** (`ConvertScaleYUpToZUp`) - the two non-X axes **swap**. This pairs with the rotation being conjugated rather than composed: a splat's covariance is `Σ = R S² Rᵀ` and must become `B Σ Bᵀ`, which for this basis means permuting the per-axis scale to match. Converting the rotation without also permuting the scale produces subtly wrong ellipsoid orientations that are easy to mistake for a projection bug.
- **SH "rest" coefficients** (`ConvertShRestYUpToZUp`) - rotated per band, per channel, in place. The shader evaluates the SH basis in engine world space, but the coefficients were trained against the `.ply`'s frame, and only the DC term is rotation-invariant. Skipping this leaves view-dependent color that is _nearly_ right head-on and visibly wrong from the side. The builder applies it to each channel's block independently, which is why the asset's channel-major layout is convenient here.

## GPU buffer layout: geometry and SH are separate buffers

A cloud's per-splat payload reaches the GPU as **two** buffers, not one interleaved record (`GaussianSplatFeatureProcessor::UpdateCloudSourceBuffers`, consumed by `GaussianSplatProjectCull.azsl`):

- **Geometry** - `StructuredBuffer<SplatGeometry>`, 48 bytes/splat: position, opacity, rotation, scale. Every cull thread reads exactly one element.
- **Spherical harmonics** - `StructuredBuffer<uint>`, `3 * (1 + k)` binary16 values/splat packed two to a uint (an `(R, G, B)` triple for the DC term, then one triple per "rest" coefficient), where `k` follows the cloud's _uploaded_ SH degree. Each splat's run is padded up to a whole uint so it starts on a uint boundary, making its base address `splatIndex * m_shUintsPerSplat`; `ShHalf` in `GaussianSplatProjectCull.azsl` unpacks a value with `f16tof32`. At degree 3 this is 96 bytes/splat. Only a splat that survives culling reads it.

Splitting them is a cache-behavior decision. Projection and culling need geometry and nothing else; in one interleaved record two adjacent threads' geometry would sit a full record apart, so nearly every byte fetched is SH data the thread may be about to discard. Split, adjacent threads read adjacent elements, and a culled splat never touches its SH data at all. At degree 3 the two buffers total 144 bytes/splat.

The SH buffer is **coefficient-major and RGB-interleaved**, deliberately unlike the channel-major asset layout above: `EvaluateShColor` consumes one `(R, G, B)` triple at a time, which channel-major storage would scatter `k` coefficients apart. `Packing::PackCloudSh` transposes once, at upload. Both uploads are therefore a per-splat CPU pass rather than a byte copy - acceptable because uploads happen only on asset load or a Max-SH-Degree change, not per frame.

Both buffers are **per cloud**, allocated from `CommonBufferPoolType::ReadOnly` and bound into that cloud's own cull pass `PassSrg`. They are distinct from the pipeline-wide scratch buffers the sort chain uses, which the feature processor owns per render pipeline - see [`Rasterizer.md`](./Rasterizer.md)'s "Buffer ownership" section.

**Why `float4` arrays, not `float[N]`, in any GPU-visible struct.** HLSL/AZSL's default packing rules pad **scalar** array elements out to 16-byte slots, so a `float m_coeffs[48]` would consume 768 bytes rather than 192 and silently mismatch whatever the C++ side wrote; `float4` array elements are already exactly 16 bytes with no padding. The same reasoning drives field ordering in every GPU-visible struct here (`SplatGeometry`, `SplatProjection2D` in `GaussianSplatCommon.azsli`): a `float3` is always immediately followed by exactly one scalar, filling the remaining 4-byte slot of that 16-byte register - the one scalar-after-vector arrangement that needs no explicit padding.

## Runtime tunables

| `GaussianSplatComponentConfig` field | Editor label           | Purpose                                                                                                                                                                                                                                                  |
| ------------------------------------ | ---------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `m_gaussianSplatAsset`               | `Gaussian Splat Asset` | The `.gsplat` asset reference to render                                                                                                                                                                                                                  |
| `m_opacityMultiplier`                | `Opacity Multiplier`   | Global alpha multiplier on top of each splat's stored opacity. Slider, clamped to `[0, 1]`, so it can only attenuate                                                                                                                                     |
| `m_scaleMultiplier`                  | `Scale Multiplier`     | Grows or shrinks each splat's individual blob in place, without moving positions or changing the cloud's extent (unlike the entity Transform's scale, which resizes splat spacing too). Clamped to `[0, 10]`                                             |
| `m_maxShDegreeOverride`              | `Max SH Degree`        | Clamps SH evaluation below what the asset stores; `0` = flat (DC-only) color. Slider over `[0, GaussianSplatAsset::MaxShDegree]`. Also shrinks the GPU SH buffer, which is packed at this degree - degree 0 stores 12 bytes/splat against degree 3's 192 |

Only `Max SH Degree` triggers a GPU repack (`m_cloudSweepPending`); `Opacity Multiplier` and `Scale Multiplier` reach the GPU as `PassSrg` constants read per frame, with no dirty tracking at all.

## Known limitations

- **No `.splat` (antimatter15 compact format) import path** - only the 3DGS-standard `.ply` layout is recognized.
- **Wave intrinsics are required** with no fallback path - see [`Rasterizer.md`](./Rasterizer.md)'s "Wave intrinsics are a hard requirement".
- See [`Rasterizer.md`](./Rasterizer.md)'s "Known open risks" for the rendering pipeline's own limitations - chiefly the fixed radix-entry capacity and the full chain rebuild a tile-grid or total-splat-count change forces - and its "The sort key" for the 65535-tile sort-key ceiling.
