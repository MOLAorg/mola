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
project(mola-input-ros1 LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola-common REQUIRED)
# find dependencies:
find_package(mrpt-maps REQUIRED)

find_mola_package(mola-kernel)

# Find ROS1 roscpp lib:
mola_find_package_or_return(roscpp)
mola_find_package_or_return(sensor_msgs)

# These ones should not be needed if only using cmake exported targets in ROS,
# but I'm not sure it's done for all current ROS1 packages, so...
include_directories(SYSTEM ${roscpp_INCLUDE_DIRS})
include_directories(SYSTEM ${sensor_msgs_INCLUDE_DIRS})

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PRIVATE_LINK_LIBRARIES
		mola-kernel
		mrpt::maps
		${roscpp_LIBRARIES}
		${sensor_msgs_LIBRARIES}
)
