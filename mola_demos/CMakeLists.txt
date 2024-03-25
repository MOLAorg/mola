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

if("$ENV{ROS_VERSION}" STREQUAL "2")
	set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_demos LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
#find_package(mola_common REQUIRED)

# Instrumental lib:
add_library(${PROJECT_NAME} INTERFACE)

# Install files:
install(DIRECTORY
  mola-cli-launchs
  ros2-launchs
  ros2-params
  rviz2
  DESTINATION
    share/${PROJECT_NAME}
)


# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if (DETECTED_ROS2)
	# find dependencies
	find_package(ament_cmake REQUIRED)

	#ament_target_dependencies(MY_TARGET
	#  rclcpp...
	#)

	ament_package()
endif()
