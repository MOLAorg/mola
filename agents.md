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
- `VizInterface` — visualization (backend-agnostic, updated in v2.6)
- `Relocalization` — global localization / loop closure
- `OfflineDatasetSource` — offline dataset handling

Other key types:
- `GuiWidgetDescription` — descriptor for GUI widget creation (backend-agnostic)
- `MinimalModuleContainer` — module loading and lifecycle

### `mola_yaml` — YAML Parser
Features: variable substitution (`$var`), file includes (`@include`), C++17 filesystem.
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
- All support MRPT serialization

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
