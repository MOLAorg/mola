# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_pose_list LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find CMake dependencies:
find_package(mrpt_maps REQUIRED)
find_package(mrpt_poses REQUIRED)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_maps
    mrpt::mrpt_poses
  CMAKE_DEPENDENCIES
    mola_common
    mrpt_maps
    mrpt_poses
)

# -----------------------
# define tests:
enable_testing()
add_subdirectory(tests)
