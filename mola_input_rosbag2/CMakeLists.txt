# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2025, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.5)

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
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
    sensor_msgs::sensor_msgs_library
    cv_bridge::cv_bridge
  CMAKE_DEPENDENCIES
    mola_kernel
    mrpt-obs
    mrpt-ros2bridge
)

# Convert package versions to hex so they can be used in preprocessor for wider
# versions compatibility of "one-source for all":
mola_version_to_hexadecimal(cv_bridge_VERSION_HEX ${cv_bridge_VERSION})
target_compile_definitions(${PROJECT_NAME} PRIVATE CV_BRIDGE_VERSION=${cv_bridge_VERSION_HEX})
message(STATUS "Found: cv_bridge_VERSION: ${cv_bridge_VERSION} (${cv_bridge_VERSION_HEX})")

# Same for mrpt_libros_bridge
mola_version_to_hexadecimal(mrpt_libros_bridge_VERSION_HEX ${mrpt_libros_bridge_VERSION})
target_compile_definitions(${PROJECT_NAME} PRIVATE MRPT_ROS2_BRIDGE_VERSION=${mrpt_libros_bridge_VERSION_HEX})
message(STATUS "Found: mrpt_libros_bridge_VERSION: ${mrpt_libros_bridge_VERSION} (${mrpt_libros_bridge_VERSION_HEX})")