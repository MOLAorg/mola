# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2019, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola-input-kitti-dataset LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola-common REQUIRED)

# find dependencies:
find_package(mrpt-math) # for MRPT Eigen utilities
find_package(mrpt-maps)
find_package(mrpt-poses)
find_package(mrpt-tclap)

find_mola_package(mola-kernel)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mola-kernel
	PRIVATE_LINK_LIBRARIES
		mrpt::maps
		mrpt::math
	CMAKE_DEPENDENCIES
		mola-kernel
)

# ----------------------
# define app target:
mola_add_executable(
	TARGET  mola-kitti-eval-error
	SOURCES apps/mola-kitti-eval-error.cpp
	LINK_LIBRARIES
		mrpt::tclap
		mrpt::poses
		${PROJECT_NAME}
)

# Silent tons of warnings from kitti-eval original code:
if (CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(mola-kitti-eval-error PRIVATE "-Wno-unused-result" "-Wno-shadow" "-Wno-sign-compare")
endif()
