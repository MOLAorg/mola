#!/usr/bin/env bash
# Usage: scripts/formatter.sh [--check]
#   (default) Reformat all C/C++ sources in-place with clang-format-14.
#   --check   Dry-run: exit non-zero if any file would be reformatted.

set -euo pipefail

if [ "${1:-}" = "--check" ]; then
  MODE=(--dry-run --Werror)
else
  MODE=(-i)
fi

find \
    mola \
    mola_bridge_ros2 \
    mola_demos \
    mola_input_lidar_bin_dataset \
    mola_input_rawlog \
    mola_input_rosbag2 \
    mola_input_video \
    mola_kernel \
    mola_launcher \
    mola_metric_maps \
    mola_msgs \
    mola_pose_list \
    mola_relocalization \
    mola_traj_tools \
    mola_viz \
    mola_viz_imgui \
    mola_yaml \
    \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \) \
    -not -path "*/3rdparty/*" \
  -print0 | xargs -0 -r -t clang-format-14 "${MODE[@]}"
