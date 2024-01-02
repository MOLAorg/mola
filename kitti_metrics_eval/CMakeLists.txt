# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2024, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

# Tell CMake we'll use C++ for use in its tests/flags
project(kitti_metrics_eval LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find dependencies:
find_package(mrpt-math) # for MRPT Eigen utilities
find_package(mrpt-poses)
find_package(mrpt-tclap)

# ----------------------
# define app target:
mola_add_executable(
	TARGET  kitti-metrics-eval
	SOURCES apps/kitti-metrics-eval.cpp
	LINK_LIBRARIES
		mrpt::tclap
		mrpt::poses
		mrpt::math
)

# Silent tons of warnings from kitti-eval original code:
if (CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(kitti-metrics-eval PRIVATE "-Wno-unused-result" "-Wno-shadow" "-Wno-sign-compare")
endif()
