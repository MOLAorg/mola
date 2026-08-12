# MOLA Repository — AI Agent Context Guide

## Project Overview

**MOLA** (Modular Optimization framework for Localization and mApping) is a modern C++ and ROS 2 framework for robot **localization and SLAM** (Simultaneous Localization and Mapping). Key capabilities:
- LiDAR Odometry (LO) and LiDAR-Inertial Odometry (LIO)
- Configurable ICP and SLAM pipelines
- Plug-and-play modules for custom localization systems
- Geo-referenced map manipulation and sensor fusion
- Standalone C++ or ROS 2 integration

**Maintainer**: Jose Luis Blanco-Claraco
**License**: GPL-3.0 (core) and BSD-3-Clause (utilities, bridge, demos)
**Current Version**: 2.6.0
**Official Docs**: https://docs.mola-slam.org/latest/

**Important**: Whenever a change is made to the repo, reflect it here if applicable, to keep it in sync with the code.

---

## Architecture

MOLA uses a **plugin-based modular architecture**:

1. All components implement virtual base interfaces defined in `mola_kernel`
2. Modules are dynamically loaded by `mola_launcher` from YAML configuration files
3. Data flows: `RawDataSource → Frontend → Filters → Map/Localization backends`
4. Visualization is decoupled (backend-agnostic `VizInterface`)
5. Runtime composition without recompilation via YAML configs

---

## Directory Structure

```
mola/                          # Metapackage (version tags, top-level docs)
mola_kernel/                   # Core virtual interfaces and data types
mola_yaml/                     # YAML parsing library with variable expansion
mola_msgs/                     # ROS 2 message/service/action definitions
mola_common/                   # External shared utilities (CMake macros)

# Core Libraries
mola_metric_maps/              # Advanced metric map classes (NDT, voxels, occupancy)
mola_relocalization/           # Global localization and loop closure
mola_pose_list/                # Searchable/spatial pose list data structure
mola_viz/                      # GUI visualization system (backend-agnostic)

# Data Input Sources (RawDataSource implementations)
mola_input_rosbag2/            # ROS 2 bag file player
mola_input_kitti_dataset/      # KITTI odometry/SLAM dataset reader
mola_input_kitti360_dataset/   # KITTI-360 panoramic dataset reader
mola_input_euroc_dataset/      # EuRoC UAV stereo dataset reader
mola_input_video/              # Live/offline video sources (OpenCV)
mola_input_rawlog/             # MRPT rawlog binary format
mola_input_lidar_bin_dataset/  # Generic binary LiDAR format
mola_input_mulran_dataset/     # MulRan urban SLAM dataset
mola_input_paris_luco_dataset/ # Paris-LUCO dataset

# Integration & Tooling
mola_launcher/                 # CLI app (`mola-cli`) for launching MOLA systems
mola_bridge_ros2/              # Bidirectional ROS 2 ↔ MOLA bridge
mola_traj_tools/               # CLI tools for trajectory file manipulation
mola_demos/                    # Example YAML launch configurations

# Evaluation
kitti_metrics_eval/            # KITTI benchmark evaluation tools
```

---

## Key Packages In Depth

### `mola_kernel` — Core Interfaces
Location: `mola_kernel/include/mola_kernel/interfaces/`

All plugin modules derive from these virtual base classes:
- `ExecutableBase` — base for all executable modules
- `RawDataSourceBase` — sensor/dataset data sources
- `FrontEndBase` — LiDAR/visual frontend algorithms
- `FilterBase` — generic filters
- `NavStateFilter` — navigation state estimators
- `LocalizationSourceBase` — localization systems
- `MapSourceBase` / `MapServer` — map sources/servers
- `VizInterface` — visualization (backend-agnostic, updated in v2.6). The
  `mola_viz_imgui` backend auto-exposes a dockable "Console" subwindow that
  aggregates mrpt-logger output from all running modules (gated by the
  `console_enabled` param, added subsequently). On a fresh profile (no saved
  `imgui_*.ini` layout yet) the Console is auto-docked at the bottom of the
  main window instead of floating; an existing saved layout is left as-is.
  The same fresh-profile layout also reserves a strip at the top of the main
  window (`PerWindowData::default_dock_top_id`) for windows whose title is
  only known at runtime; any `GuiWidgetDescription::WindowDescription` with
  `dock_top_by_default = true` (e.g. the per-module `Dataset_UI` playback
  panel) is docked there the first time it is created. No-op once a saved
  layout exists, and on the nanogui `MolaViz` backend.
  Also exposes `register_metric()`/`push_metric()` (feature macro
  `MOLA_KERNEL_VIZ_HAS_METRICS`) so any module can stream timestamped scalar
  values ("metrics") to live, autoscrolling plot windows opened from the
  built-in "View" menu (gated by the `plots_enabled` param); rendered via
  ImPlot on the `mola_viz_imgui` backend only, no-op on the nanogui `MolaViz`
  backend. The top main menu bar itself (host mode only; hosts the "MOLA"
  menu (with "Quit", which requests full application shutdown), the "View"
  menu, plus any module-installed `set_menu_bar()` menus) can be turned off
  entirely via `menu_bar_enabled` (default `true`); existing plot/console
  windows keep working when it is disabled. `output_console_message()` (the
  older fading on-screen text overlay) is a no-op on the `mola_viz_imgui`
  backend, superseded by the Console subwindow above; still implemented on
  the nanogui `MolaViz` backend.
- `Relocalization` — global localization / loop closure
- `OfflineDatasetSource` — offline dataset handling
- `SharedKeyframeMap` — central-map keyframe-insertion sink (new 2026, see
  `mola_mapper_3d`): front ends (LIO/VIO) push sparse keyframes via
  `requestInsertKeyframe()`, decoupled from their own local map/odometry
  frame. Detected via `findService<>()` the same way as `NavStateFilter`,
  but optional. Feature macro `MOLA_KERNEL_HAS_SHARED_KEYFRAME_MAP`.

Other key types:
- `GuiWidgetDescription` — descriptor for GUI widget creation (backend-agnostic)
- `MinimalModuleContainer` — module loading and lifecycle

### `mola_yaml` — YAML Parser
Features: variable substitution (`$var`), file includes (`@include`), deep-merge
`$import`, and the `$define` map directive (binds `${NAME}` variables for the subtree
it appears in, including sibling `$import`ed files; priority `env > $define > |default`),
C++17 filesystem.
Tests: `mola_yaml/tests/test-yaml-parser.cpp`

### `mola_launcher` — CLI Entry Point
- **`mola-cli`**: main executable, takes YAML config, loads and orchestrates modules
- **`mola-yaml-parser`**: standalone tool for testing YAML parsing/variable expansion
- **`mola-dir`**: directory and environment utilities

### `mola_bridge_ros2` — ROS 2 Integration
- Consumes ROS 2 sensor topics as MOLA `RawDataSource`
- Publishes MOLA outputs (maps, poses) as ROS 2 topics/TF
- Must have ROS 2 environment sourced before building

### `mola_metric_maps` — Map Types
- `OccupancyGridMap` (super-resolution)
- `SparseVoxelPointCloud`
- `NDTMap` (Normal Distribution Transform)
- `KeyframeMap`
- `IncrementalPointCloud` (new 2026): sliding-window LIO local map derived from
  `mrpt::maps::CGenericPointsMap`, backed by **one** incremental self-balancing
  nanoflann k-d tree (`KDTreeSingleIndexIncrementalAdaptor`, or the `…MT`
  variant with background rebuilds) instead of rebuilding a static tree on every
  scan. Insertion goes through the inherited `CPointsMap` entry points (the
  index picks up appended points lazily); `creationOptions.remove_points_farther_than`
  trims a cube around the robot on each `insertObservation()`, and the slots
  freed by the index are recycled, keeping storage bounded under churn.
  `remove_points_farther_than` is a **cube half-side** (Chebyshev distance, as in
  `HashedVoxelPointCloud::remove_voxels_farther_than`), NOT a radius on keyframe
  centres like KFM's `remove_frames_farther_than`: every point inside it is kept
  in a single tree and rendered, so it needs a far tighter budget than KFM's
  equivalent -- around the sensor range, not several times it.
  `async_rebuild` (the `…MT` index) moves the balancing rebuilds off the mapping
  thread and is what keeps insertion latency flat -- on Oxford Spires it takes
  local-map insertion from mean 28 ms / max 371 ms to mean 10 ms / max 38 ms with
  no spike over 100 ms -- so `lidar3d-gicp.yaml` enables it by default. Its
  worker reads the inherited coordinate buffers, so those must not be
  reallocated while it runs: `reserve()`/`resize()`/`setSize()` are overridden to
  wait for it first (which covers `insertObservation()`/`insertAnotherMap()`),
  and insertion keeps spare capacity for one batch so a raw `insertPointFast()`
  loop cannot realloc either. Don't add a growth path that bypasses those.
  Reclaimed-but-not-yet-reused slots are blanked to NaN so that generic
  `CPointsMap` walkers (notably `insertAnotherMap()`, used by
  `LidarOdometry_Publish.cpp` to copy layers out for visualization) cannot
  resurrect evicted geometry. The storage array itself never shrinks on its own:
  it settles at its high-water mark and slots are recycled; `compact()` releases
  it on demand.
  `changeCoordinatesReference()` (all 3 overloads) is shadowed: a global SE(3)
  re-map moves every coordinate, so it applies the transform and then rebuilds
  the index over the *live* slot set (`rebuildIndexInPlace()`), dropping the
  covariance cache. The base-class methods are **not virtual**, so a call
  through a `mrpt::maps::CPointsMap*` cannot be intercepted; instead
  `ensureIndexUpToDate()` compares a handful of sampled slot coordinates
  (`coordinates_watch_`, refreshed by every internal mutator) against their
  last known values and forces the same rebuild when they moved. Since the
  sample is bounded, that guard covers a **global** re-map (all points move),
  not a mutator rewriting a few points or a caller poking the inherited
  coordinate buffers directly; those stay as stale-index hazards.
  Implements `mp2p_icp::NearestPointWithCovCapable` with lazily computed,
  cached, plane-regularized per-point covariances (the "option A" of the plan;
  voxel/NDT-style and dirty-propagation covariances remain future work). Not for
  loop closure (a global SE(3) re-map would force a full rebuild) -- use
  `KeyframePointCloudMap` there. Caveats: point removal is lazy, so the
  inherited `size()` counts live + not-yet-reclaimed slots (use
  `livePointCount()`, or `compact()` to drop them); `nn_*` indices are storage
  slots; 2D `nn_*` queries throw.
  Requires nanoflann >= 1.10.0. On distributions shipping an older one the build
  still succeeds, with a CMake warning: the class is compiled and registered as
  usual, but `src/IncrementalKDTree_stub.cpp` replaces the k-d tree factory with
  one that throws an explanatory `std::runtime_error` naming the version found,
  so a YAML asking for the class fails with that instead of MRPT's generic "no
  such registered CMetricMap class". `MOLA_METRIC_MAPS_HAS_INCREMENTAL_POINT_CLOUD`
  is defined (PUBLIC) only when the feature is functional; the header is always
  usable either way.
  `TCreationOptions::serialize_kdtree` (default `false`): bakes the incremental
  k-d tree index into the `.mm` on save, so it does not have to be rebuilt (an
  O(N log N) bulk build) on load. Unlike `KeyframePointCloudMap`'s baked static
  trees, nanoflann's incremental index (`KDTreeSingleIndexIncrementalAdaptor`)
  had no save/load support at all until `saveIndex()`/`loadIndex()` were added
  upstream (nanoflann >= 1.11.0); gated via `MOLA_METRIC_MAPS_HAS_INCREMENTAL_KDTREE_BAKE`
  (PUBLIC, set by CMake when `nanoflann_VERSION >= 1.11`), a no-op on write and
  a skipped blob on read otherwise. Serialization always writes/reads the
  *compacted* (tombstone-free) point order (see `serializeTo()`), so baking
  builds a throwaway index over that exact order rather than reusing the live
  `index_` (whose slots may not match after tombstones/recycling). CLI tool
  `mm-ipc-bake-kdtree` (in `apps/`), analogous to `mm-kf-bake-kdtrees`.
  **nanoflann is included by `src/IncrementalKDTree.cpp` alone, by absolute path
  (`MOLA_NANOFLANN_HEADER`, set by CMake) and with its namespace renamed**,
  because MRPT bundles its own, usually older, copy of the same header and
  mixing both would be an ODR violation (their pooled allocators differ while
  sharing mangled names). An `-I` cannot select ours: MRPT exports its copy's
  directory as `-isystem`, and a directory listed both ways is deduplicated by
  GCC in favor of the system entry. Keep that file free of MRPT headers, and
  keep nanoflann out of the public headers.
- All support MRPT serialization
- `mola::OptionsCapable` (`include/mola_metric_maps/OptionsCapable.h`): mixin interface
  implemented by `NDT`, `HashedVoxelPointCloud`, `SparseVoxelPointCloud`, `SparseTreesPointCloud`,
  and `KeyframePointCloudMap` (i.e. all map classes defined in this library; basic `mrpt::maps`
  classes do not implement it -- see the `mm2ini`/`ini2mm` fallback below), exposing their
  `CLoadableOptions` groups generically by name (`optionsByName()`) and a safe creation-options
  setter (`trySetCreationOptions()`, which returns `false` instead of discarding map contents
  when a structural change, e.g. voxel size, is incompatible with already-inserted data).
  Temporary home for a feature that may eventually move into MRPT itself.
- CLI tools `mm2ini` / `ini2mm` (in `apps/`): export/import the `CLoadableOptions`
  (creation/insertion/likelihood/render options) of all layers in a `.mm` file to/from a
  `.ini` file, using `OptionsCapable` generically for classes in this library, plus a
  `dynamic_cast`-based fallback in `include/mola_metric_maps/OptionsIniIO.h` for basic
  `mrpt::maps` classes (`CPointsMap`-derived, `COccupancyGridMap2D`).
- CLI tool `mm-kf-regroup` (in `apps/`): offline optimization of a `KeyframePointCloudMap`
  layer for fast **localization-only** operation. It groups many small keyframes into a few
  larger, deliberately-overlapping "super-keyframes" so the ICP active set stays size 1 (no
  per-scan keyframe switching). Backed by `KeyframePointCloudMap::regroupKeyframes()`
  (see `RegroupParams`): builds a keyframe adjacency graph with voxel Jaccard-min overlap
  edge weights, then greedy overlapping set-cover clustering auto-sized from each keyframe's
  bounding box (large outdoors, small indoors). Merged clouds are voxel-decimated
  (`--decimate-voxel`, view-direction fields carried) to bound the overlap-induced point
  blow-up. On a 1000-KF outdoor map this yields ~5 super-KFs. This is "idea 1" of the
  keyframe-map persistence plan. Building each super-KF cloud (the merge +
  voxel-decimate step) is the most expensive part of `regroupKeyframes()` and is
  parallelized across clusters with TBB (`tbb::parallel_for`, gated by
  `MOLA_METRIC_MAPS_USE_TBB`, falling back to a serial loop when TBB is absent).
- `KeyframePointCloudMap::TCreationOptions::max_distance_for_cov` bounds the per-point
  covariance neighborhood, which maps onto nanoflann's radius-limited kNN (RKNN) via the
  optional max-distance argument of MRPT's `kdTreeNClosestPoint3DIdx()`. That overload
  throws at runtime on nanoflann < 1.5.1 (still the case on Ubuntu jammy / Humble), so
  `computeCovariancesAndDensity()` guards it with `MOLA_MM_HAS_RKNN_SEARCH`
  (`NANOFLANN_VERSION >= 0x151`) and otherwise runs a plain kNN truncated at the same
  radius, which yields exactly the same neighbor set since results come back sorted.
  The `NANOFLANN_VERSION` that decides is the one MRPT's own templates were built
  against, picked up transitively from the MRPT headers: do not include nanoflann there.
- `KeyframePointCloudMap::TCreationOptions::approximate_cov` (default `false`): for
  `nn_search_cov2cov()` (used by `mp2p_icp::Matcher_Cov2Cov`, i.e. GICP-style pipelines).
  When `true`, `icp_get_prepared_as_global()` skips assembling the merged, multi-keyframe
  submap for the active KF set and instead only warms each active KF's own (already cached)
  KD-tree and per-point covariances; `nn_search_cov2cov()` then does one KD-tree query per
  active KF ("N" queries instead of 1 on a merged cloud) and keeps the closest, using that
  KF's own cached covariance. Trades exactness (covariances are estimated only from
  within-KF neighbors, not the merged multi-KF cloud) for speed (no merge/KD-tree rebuild).
  Only affects `nn_search_cov2cov()`; the generic `NearestNeighborsCapable` entry points
  still require the merged submap. Exposed in `mola_lidar_odometry`'s
  `pipelines/lidar3d-gicp.yaml` as `MOLA_LOCALMAP_APPROXIMATE_COV`.
- `nn_search_cov2cov()` has two overloads: the original scalar `float` search distance
  (still supported, unchanged signature) and an additive overload taking an
  `mp2p_icp::MatchingDistanceProfile`. The scalar overload just wraps its value into a flat
  `MatchingDistanceProfile` and forwards to the same implementation. Both map classes (and the
  approximate-cov path) keep a fast path for the flat case: no per-point range is computed and
  the KD-tree query stays `k=1`. A range-adaptive profile opts into a per-point range.
  **Queries the per-KF *local*-frame cloud/KD-tree directly** (`kf.pointcloud()`, which is
  what `mm-kf-bake-kdtrees` bakes on disk): the query point is transformed into each active
  KF's local frame (`pose^{-1}`) before the lookup, and the match is composed back to global
  for the output pairing. A KD-tree's structure is invariant under the rigid KF pose, so this
  avoids ever materializing a per-KF *global*-frame cloud or rebuilding a KD-tree on it — the
  reason the baked (local) index previously did not accelerate this path, and why a fresh KF
  entering the active set used to stall the LiDAR worker for seconds. Also robust to online KF
  pose nudges (e.g. LIO gravity-tilt correction), which no longer invalidate a global cloud.
- CLI tool `mm-kf-bake-kdtrees` (in `apps/`): "idea 2" of the same plan. Caches heavy
  per-keyframe structures inside the `.mm` so they are not rebuilt on every load (faster
  localization-only startup). Two independent, self-describing caches (each gated by a flag
  byte, so `.mm` files stay interoperable across MRPT builds):
  - **KD-trees** (`serialize_kdtrees` creationOption, default false): each keyframe's
    per-cloud 3D KD-tree index (on the *local*-frame cloud). Requires the MRPT KD-tree
    save/load index API (`mrpt::math::KDTreeCapable::kdtree_save_index_3D()` /
    `kdtree_load_index_3D()`, feature-detected via the `MRPT_HAS_KDTREE_SAVE_LOAD_INDEX`
    macro); when absent the option is a no-op on write and the reader skips any stored blob.
    On the 5-super-KF outdoor map above (9.4M points), baking adds ~58 MB. This is the same
    local index the `approximate_cov` path now queries directly (see above), so baking finally
    accelerates the localization hot path, not just startup.
  - **Covariances** (`serialize_covariances` creationOption, default false): each keyframe's
    per-point *local*-frame covariances (the plane-regularized SVD result). Needs **no special
    MRPT API** (works on any build). Covariance computation (one K-NN + 3×3 SVD per point) is
    the single most expensive part of warming a keyframe, so persisting it removes the
    multi-second stall paid the first time each KF becomes active. The cheap per-pose global
    rotation is still done at runtime.

  Bumps map serialization to v3 (v2 = KD-tree byte/blob after each KF cloud; v3 adds a
  covariance byte + per-point matrices after that). The tool bakes both by default; use
  `--no-kdtrees` / `--no-covariances` to select, or `--disable` to strip both.

  The tool also sets `approximate_cov=YES` on the output map by default (`--no-approximate-cov`
  to opt out; forced off by `--disable`). This matters: the baked data is only consumed by the
  approximate cov2cov path, so without it the loaded map ignores the cache and rebuilds a merged
  submap from scratch on every KF-set change (the multi-second stall this whole feature exists to
  avoid). Note the runtime `MOLA_LOCALMAP_APPROXIMATE_COV` env var only configures the live local
  map, not a loaded `.mm`, whose `approximate_cov` is read from the file.

---

## Build System

- **Build tool**: CMake 3.5+ / Colcon (for ROS 2)
- **C++ standard**: C++17
- **Config**: `colcon_defaults.yaml` (symlink install, RelWithDebInfo, compile_commands.json)
- **CMake macros** (from `mola_common`): `mola_add_library()`, `find_mola_package()`
- **Platforms**: Linux Ubuntu 22.04/24.04, AMD64 and ARM64

Build standalone:
```bash
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
```

Build with ROS 2 (source workspace first):
```bash
colcon build --symlink-install
```

---

## Key External Dependencies

| Dependency | Purpose |
|------------|---------|
| **MRPT** (≥2.1.0) | Core robotics toolkit: poses, observations, maps, OpenGL GUI, serialization |
| **mp2p_icp** | Point cloud ICP registration (used by relocalization and metric maps) |
| **TBB** | Intel Threading Building Blocks for parallelism |
| **OpenCV** | Image/video handling (via MRPT) |
| **rclcpp** | ROS 2 C++ client library (optional, only if ROS 2 present) |
| **rosbag2_cpp** | ROS 2 bag I/O (mola_input_rosbag2) |
| **tf2** | ROS 2 transforms (mola_bridge_ros2) |

---

## Testing

- **Framework**: CMake + GTest
- Each package has `tests/` with its own `CMakeLists.txt`
- CI/CD: `.github/workflows/` — builds on ROS 2 Humble, Jazzy, Kilted, Rolling
- Style: enforced with `.clang-format` and `.clang-tidy`

Test coverage exists for: `mola_yaml`, `mola_metric_maps`, `mola_pose_list`, `mola_relocalization`

---

## Common Patterns

### Adding a New Module
1. Inherit from appropriate `mola_kernel` interface (e.g., `RawDataSourceBase`)
2. Override virtual methods (`initialize()`, `spinOnce()`, etc.)
3. Register with `mola_launcher` plugin system via CMake macros
4. Write YAML config for the module

### YAML Configuration
All MOLA systems are described in YAML. See `mola_demos/` for examples.
Variable expansion and file includes are supported by `mola_yaml`.

### Exposing a /tf tree: `mola::TransformTreeSource`

`mola_kernel/interfaces/TransformTreeSource.h` lets a module publish its tree
of coordinate frames (ROS `/tf`) to other MOLA modules without dragging ROS
types into `mola_kernel`: `transform_tree(root)` returns the subtree below
`root` with poses already resolved against it, as plain `mrpt::poses::CPose3D`.

- Implemented by `Rosbag1Dataset`, `Rosbag2Dataset` and `BridgeROS2`, all of
  which already own a `tf2::BufferCore`. **The filtering is done in the
  source**, since it is what owns the buffer: a consumer never receives, nor
  walks, the frames of unrelated subtrees.
- No locking is needed around the walk: `tf2::BufferCore` guards its own
  internals, so it may run while another thread feeds `/tf`.
- Consumers detect it with `findService<mola::TransformTreeSource>()`.
- `TransformTree`/`TransformTreeNode` must stay **aggregates**: every producer
  fills them with brace initialization, and since C++20 giving them any
  constructor — even `= default` — makes that stop compiling. Lyrical builds
  at C++20 and Humble/Jazzy at C++17, so this breaks on one distro only.
  `push_back({...})` into a `vector<TransformTreeNode>` must name the type
  explicitly (`push_back(TransformTreeNode{...})`, not a bare braced list): on
  GCC 15 (ROS 2 Rolling) the bare form fails to resolve between the
  `push_back` overloads with "no known conversion from '<brace-enclosed
  initializer list>'"; naming the type first sidesteps that overload
  resolution entirely. Older GCC (Humble/Jazzy/Kilted) accepts either form.

### GUI Widget Creation (v2.6+)
Use `GuiWidgetDescription` for backend-agnostic widget creation in `VizInterface`.
Do not use direct MRPT GUI calls in modules — use the `VizInterface` abstraction.

---

## File Navigation Tips

- Interface definitions: `mola_kernel/include/mola_kernel/interfaces/`
- GUI/widget types: `mola_kernel/include/mola_kernel/` (look for `GuiWidgetDescription`)
- Example configs: `mola_demos/mola-cli-launchs/`
- Per-package docs: each directory has a `README.md`
- Main docs source: `docs/` (Sphinx + Doxygen)

---

## Environment Variables (Debug/Tracing Flags)

All debug/tracing flags use `mrpt::get_env<T>(name, default)` (from
`<mrpt/core/get_env.h>`), never plain `::getenv`/`std::getenv`. The one
exception is `mola_yaml`'s `${VAR}` expansion (`yaml_helpers.cpp`), which
needs tri-state unset-vs-empty semantics that `mrpt::get_env` cannot express,
so it keeps `::getenv` by design.

| Variable | Type | Default | Location | Purpose |
|----------|------|---------|----------|---------|
| `MOLA_MODULES_LIB_PATH` | path list | (unset) | `mola_launcher/src/MolaLauncherApp.cpp` | Extra directories to search for module shared libraries |
| `MOLA_MODULES_SHARED_PATH` | path list | (unset) | `mola_launcher/src/MolaLauncherApp.cpp` | Extra directories to search for module shared (data) files |
| `MOLA_INCREMENTAL_MAP_DEBUG_STATS` | bool | false | `mola_metric_maps/src/IncrementalPointCloud.cpp` | Trace live/storage/free-slot counts and the map bbox on every insertion |
| `MOLA_KEYFRAME_MAP_PROFILE_COV` | bool | false | `mola_metric_maps/src/KeyframePointCloudMap.cpp` | Print profiling stats for per-KF covariance computation |
| `MOLA_KEYFRAME_MAP_DEBUG_ACTIVE_KFS` | bool | false | `mola_metric_maps/src/KeyframePointCloudMap.cpp` | Trace which keyframes are in the active ICP set |
| `MOLA_KEYFRAME_MAP_DEBUG_DUMP_KFS_ON_LOAD` | bool | false | `mola_metric_maps/src/KeyframePointCloudMap.cpp` | Dump per-keyframe debug info right after loading a `.mm` map |
| `MOLA_KEYFRAME_MAP_DEBUG_MATCH_STATS` | bool | false | `mola_metric_maps/src/KeyframePointCloudMap.cpp` | Trace nearest-keyframe selection / point-density match statistics |
| `MOLA_KEYFRAME_MAP_VIZ_SHOW_ACTIVE_SUBMAP` | bool | false | `mola_metric_maps/src/KeyframePointCloudMap.cpp` | Render the merged active-KF submap used for ICP |
| `MOLA_KEYFRAME_MAP_VIZ_OVERRIDE_AXES_LENGTH` | float | 0 | `mola_metric_maps/src/KeyframePointCloudMap.cpp` | Override the length of per-KF pose axes in the 3D view |
| `MOLA_KEYFRAME_MAP_VIZ_SHOW_COV` | bool | false | `mola_metric_maps/src/KeyframePointCloudMap.cpp` | Render per-point covariance ellipsoids |
| `MOLA_KEYFRAME_MAP_VIZ_COLOR_BY_KF` | bool | false | `mola_metric_maps/src/KeyframePointCloudMap.cpp` | Color rendered points by owning keyframe instead of by intensity/height |
| `MOLA_KEYFRAME_MAP_VIZ_SHOW_COV_DECIMATION` | uint32 | 0 | `mola_metric_maps/src/KeyframePointCloudMap.cpp` | Decimation factor when rendering covariance ellipsoids |
| `MOLA_LOCALMAP_APPROXIMATE_COV` | bool | (module default) | consumed via YAML, see `KeyframePointCloudMap::TCreationOptions::approximate_cov` | Configures the live local map's approximate-cov2cov path (does not affect a loaded `.mm`, whose flag is read from the file) |
| `TEST_GENERATE_3D_SCENES` | bool | false | `mola_metric_maps/tests/test-mola_metric_maps_ndt.cpp` | Regenerate reference 3D scene files instead of comparing against them |
| `MOLA_YAML_VERBOSE` | bool | false | `mola_yaml/src/yaml_helpers.cpp` | Print each external YAML file loaded via `@include`/`@import` |

---

## Documentation Build (Multi-Repo)

The MOLA documentation website is built from **all MOLAorg repos cloned together**
in a common parent directory. The Sphinx/Doxygen build in `docs/` pulls content from
sibling repos via relative paths (e.g. `../../../mola_academic_datasets/` in
`docs/source/Doxyfile`, and toctree stubs in `.rst` files that `.. include::` from
sibling checkouts).

**Consequence**: do not remove toctree entries from `docs/source/modules.rst` (or
other `.rst` files) just because a package moved to a different repo — the entry will
still resolve correctly at build time as long as the new repo is cloned alongside this
one. Add the new repo's root to `docs/source/Doxyfile` `INPUT` instead.
