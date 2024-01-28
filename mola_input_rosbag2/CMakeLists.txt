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
project(mola_input_rosbag2 LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)
# find dependencies:
find_package(mrpt-obs REQUIRED)
find_package(mrpt-ros2bridge REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_mola_package(mola_kernel)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PRIVATE_LINK_LIBRARIES
		mola::mola_kernel
		mrpt::obs
		mrpt::ros2bridge
		tf2::tf2
		rosbag2_cpp::rosbag2_cpp
		#nav_msgs
		tf2_geometry_msgs::tf2_geometry_msgs
		tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
		sensor_msgs::sensor_msgs_library
		cv_bridge::cv_bridge
	CMAKE_DEPENDENCIES
		mola_kernel
		mrpt-obs
		mrpt-ros2bridge
)

# Credits: https://gist.github.com/jtanx/96ded5e050d5ee5b19804195ee5cf5f9
function(pad_string output str padchar length)
  string(LENGTH "${str}" _strlen)
  math(EXPR _strlen "${length} - ${_strlen}")

  if(_strlen GREATER 0)
    string(REPEAT ${padchar} ${_strlen} _pad)
    string(PREPEND str ${_pad})
  endif()

  set(${output} "${str}" PARENT_SCOPE)
endfunction()

# 2 hex digits for each version part:
# For example: "0.5.1"  => "0x000501"
macro(mrpt_version_to_hex VER TARGET_VAR_NAME)
	string(REGEX MATCHALL "[0-9]+" __parts "${${VER}}")

	if(__parts)
		list(GET __parts 0 __VERSION_NUMBER_MAJOR)
		list(GET __parts 1 __VERSION_NUMBER_MINOR)
		list(GET __parts 2 __VERSION_NUMBER_PATCH)
		pad_string(__VERSION_NUMBER_MAJOR ${__VERSION_NUMBER_MAJOR} "0" 2)
		pad_string(__VERSION_NUMBER_MINOR ${__VERSION_NUMBER_MINOR} "0" 2)
		pad_string(__VERSION_NUMBER_PATCH ${__VERSION_NUMBER_PATCH} "0" 2)
		set(${TARGET_VAR_NAME} "0x${__VERSION_NUMBER_MAJOR}${__VERSION_NUMBER_MINOR}${__VERSION_NUMBER_PATCH}")
	else()
		set(${TARGET_VAR_NAME} "0x000")
	endif()
endmacro()

# Convert package versions to hex so they can be used in preprocessor for wider
# versions compatibility of "one-source for all":
mrpt_version_to_hex(cv_bridge_VERSION  cv_bridge_VERSION_HEX)

target_compile_definitions(${PROJECT_NAME} PRIVATE CV_BRIDGE_VERSION=${cv_bridge_VERSION_HEX})

message(STATUS "Found: cv_bridge_VERSION: ${cv_bridge_VERSION} (${cv_bridge_VERSION_HEX})")