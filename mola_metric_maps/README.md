# `mola_metric_maps`

Advanced metric map classes for LiDAR odometry and SLAM, built on the
generic `mrpt::maps::CMetricMap` interface.  All maps are serializable,
visualizable, and integrate with the [mp2p_icp](https://github.com/MOLAorg/mp2p_icp)
registration library.

---

## Map classes

### Production-quality maps

| Class | Header | Description |
|-------|--------|-------------|
| `mola::HashedVoxelPointCloud` | `HashedVoxelPointCloud.h` | Flat sparse hash map of cubic voxels. Up to 32 points per voxel stored without heap allocation (fixed array SSO). Backend: `tsl::robin_map`. Implements `NearestNeighborsCapable`. |
| `mola::SparseVoxelPointCloud` | `SparseVoxelPointCloud.h` | Two-level voxel map: sparse outer blocks + 32³ inner `FixedDenseGrid3D`. Tracks per-voxel point means; supports voxel-mean ICP matching. Implements `NearestNeighborsCapable`. The primary workhorse map in MOLA LO/SLAM. |
| `mola::NDT` | `NDT.h` | Normal Distributions Transform map (Magnusson 2007). Fits a Gaussian per voxel; planar voxels expose `NearestPlaneCapable`, non-planar ones expose `NearestNeighborsCapable`. Enables automatic point-to-point vs. point-to-plane pairing selection. |
| `mola::KeyframePointCloudMap` | `KeyframePointCloudMap.h` | Keyframe-based map: each keyframe holds a local point cloud plus a SE(3) pose. Supports map corrections without re-inserting points (just update poses). Implements `IcpPrepareCapable`, `NearestPointWithCovCapable`, `MetricMapMergeCapable`, and `KeyframeMapCapable`. |

### Experimental / work-in-progress maps

| Class | Header | Status |
|-------|--------|--------|
| `mola::SparseTreesPointCloud` | `SparseTreesPointCloud.h` | Coarse 3D grid of independent `CSimplePointsMap` sub-maps, each with its own KD-tree. Functionally complete but not benchmarked against the hash-based alternatives for typical SLAM workloads. |
| `mola::OccGrid` | `OccGrid.h` | Wraps `mrpt::maps::COccupancyGridMap2D` with a super-resolution likelihood cache. The cache infrastructure is in place but likelihood population is not yet implemented (`// TODO`). Not used in any production pipeline. |

---

## Utility / internal classes

| Class | Header | Description |
|-------|--------|-------------|
| `mola::index3d_t<T>` | `index3d_t.h` | Discrete 3D integer index. Works as key in `std::map` and `std::unordered_map` / `tsl::robin_map` via the `index3d_hash` functor, which implements the Teschner et al. (2003) spatial hash. |
| `mola::FixedDenseGrid3D<T,N,C>` | `FixedDenseGrid3D.h` | Dense NxNxN grid (N=2^SIDE_NUM_BITS) allocated with `calloc` for fast zero-init. Used as the inner block in `SparseVoxelPointCloud`. Requires trivially-copyable cell types. |

---

## Build and install

Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## License

Copyright (C) 2018-2026 Jose Luis Blanco <jlblanco@ual.es>, University of Almeria

This package is released under the GNU GPL v3 license as open source for research
and evaluation purposes only. Commercial licenses available upon request, for this
package alone or in combination with the complete SLAM system.
