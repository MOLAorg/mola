# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2024, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_bridge_ros2 LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)
# find dependencies:
find_package(mrpt-maps REQUIRED)

find_package(mrpt-ros2bridge QUIET)
if (NOT mrpt-ros2bridge_FOUND)
	message(STATUS "*NOTE*: Module mola_input_ros2 will NOT be built due to missing dependency mrpt-ros2bridge. Set mrpt-ros2bridge_DIR to enable it.")
	return()
endif()

# Find MOLA pkgs:
find_mola_package(mola_kernel)

# Find ROS 2 pkgs:
mola_find_package_or_return(ament_cmake REQUIRED)
mola_find_package_or_return(geometry_msgs REQUIRED)
mola_find_package_or_return(nav_msgs REQUIRED)
mola_find_package_or_return(rclcpp)
mola_find_package_or_return(sensor_msgs REQUIRED)
mola_find_package_or_return(sensor_msgs)
mola_find_package_or_return(tf2 REQUIRED)
mola_find_package_or_return(tf2_geometry_msgs REQUIRED)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

# This adds non-ROS deps. The rest are handled below with ament_target_dependencies()
mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PRIVATE_LINK_LIBRARIES
		mola::mola_kernel
		mrpt::maps
		mrpt::ros2bridge
	CMAKE_DEPENDENCIES
		mola_kernel
		mrpt-maps
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
	geometry_msgs
	nav_msgs
	rclcpp
	sensor_msgs
	tf2
	tf2_geometry_msgs
)

ament_export_dependencies()
# Export modern CMake targets
#ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  #find_package(ament_cmake_gtest REQUIRED)
  #ament_add_gtest(${PROJECT_NAME}-test test/test_xxx.cpp)

  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
