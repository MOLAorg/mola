# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2021, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

if("$ENV{ROS_VERSION}" STREQUAL "2")
	set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola-foo LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find CMake dependencies:
#find_package(mrpt-core)

# Find MOLA packages:
#find_mola_package(mola_kernel)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mola_kernel
	PRIVATE_LINK_LIBRARIES
		mrpt::core
	CMAKE_DEPENDENCIES
		mola_kernel
)

# ----------------------
# define app target:
mola_add_executable(
	TARGET  mola-myapp
	SOURCES apps/mola-myapp.cpp
	LINK_LIBRARIES
		mrpt::tclap
		#mrpt::poses
		${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
#if (DETECTED_ROS2)
#	# find dependencies
#	find_package(ament_cmake REQUIRED)
#
#	#ament_target_dependencies(MY_TARGET
#	#  rclcpp...
#	#)
#
#	ament_package()
#endif()
