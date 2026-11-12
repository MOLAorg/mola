#!/bin/bash
# Build dependencies of the mola-metric-maps wheel inside the manylinux_2_28
# images (AlmaLinux 8), run once per image by cibuildwheel ("before-all").
# MRPT, Eigen, nanoflann and oneTBB come from the C++ SDK of the mrpt3 wheel,
# and mp2p_icp_map and the mola_common CMake functions from the one of the
# mp2p-icp wheel (both, build requirements in pyproject.toml).
set -euo pipefail

dnf install -y epel-release
dnf install -y ccache
