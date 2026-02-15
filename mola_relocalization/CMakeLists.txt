# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.5)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_relocalization LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find CMake dependencies:
find_package(mrpt-obs REQUIRED)
find_package(mrpt-maps REQUIRED)
find_package(mrpt-slam REQUIRED)
find_package(mp2p_icp REQUIRED)
find_package(mola_pose_list REQUIRED)

find_package(mola_test_datasets QUIET) # optional (for testing builds only)

# -----------------------
# define lib:
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES
    src/find_best_poses_se2.cpp
    src/RelocalizationICP_SE2.cpp
    src/RelocalizationLikelihood_SE2.cpp
    include/mola_relocalization/relocalization.h
  PUBLIC_LINK_LIBRARIES
    mrpt::obs
    mrpt::maps
    mrpt::slam
    mola::mp2p_icp
    mola::mola_pose_list
#  PRIVATE_LINK_LIBRARIES
#    mrpt::obs
  CMAKE_DEPENDENCIES
    mola_common
    mp2p_icp
    mola_pose_list
    mrpt-slam
)

# -----------------------
# define tests:
enable_testing()
add_subdirectory(tests)
