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
project(mola_traj_tools LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find dependencies:
find_package(mrpt-poses)

# ----------------------
# define app targets:

mola_add_executable(
	TARGET  traj_ypr2tum
	SOURCES src/traj_ypr2tum.cpp
	LINK_LIBRARIES
		mrpt::poses
)

mola_add_executable(
	TARGET  traj_tf
	SOURCES src/traj_tf.cpp
	LINK_LIBRARIES
		mrpt::poses
)

mola_add_executable(
	TARGET  traj_rebase
	SOURCES src/traj_rebase.cpp
	LINK_LIBRARIES
		mrpt::poses
)
