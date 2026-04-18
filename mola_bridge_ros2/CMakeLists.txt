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
project(mola_bridge_ros2 LANGUAGES CXX)

# find dependencies:
find_package(mrpt-maps REQUIRED)

find_package(mrpt-ros2bridge QUIET)
if(NOT mrpt-ros2bridge_FOUND)
  message(STATUS "*NOTE*: Module mola_input_ros2 will NOT be built due to missing"
    "dependency mrpt-ros2bridge. Set mrpt-ros2bridge_DIR to enable it.")
  return()
endif()

# MOLA core dependencies:
# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)
find_mola_package(mola_kernel)

# Find ROS 2 pkgs:
mola_find_package_or_return(ament_cmake)
mola_find_package_or_return(diagnostic_msgs)
mola_find_package_or_return(geometry_msgs)
mola_find_package_or_return(nav_msgs)
mola_find_package_or_return(rclcpp)
mola_find_package_or_return(sensor_msgs)
mola_find_package_or_return(geographic_msgs)
mola_find_package_or_return(tf2)
mola_find_package_or_return(tf2_geometry_msgs)

find_package(gps_msgs QUIET) # optional

# MOLA ROS2 services:
mola_find_package_or_return(mola_msgs)
mola_find_package_or_return(mrpt_nav_interfaces)

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
  PUBLIC_LINK_LIBRARIES
    rclcpp::rclcpp
    tf2::tf2
    ${diagnostic_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${geographic_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    ${mrpt_nav_interfaces_TARGETS}
    ${mola_msgs_TARGETS}
    ${gps_msgs_TARGETS}
  CMAKE_DEPENDENCIES
    mola_kernel
    mrpt-maps
)

# Same for mrpt_libros_bridge
mola_version_to_hexadecimal(mrpt_libros_bridge_VERSION_HEX ${mrpt_libros_bridge_VERSION})
target_compile_definitions(${PROJECT_NAME} PRIVATE MRPT_ROS2_BRIDGE_VERSION=${mrpt_libros_bridge_VERSION_HEX})
message(STATUS "Found: mrpt_libros_bridge_VERSION: ${mrpt_libros_bridge_VERSION} (${mrpt_libros_bridge_VERSION_HEX})")

if (gps_msgs_FOUND)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_GPS_MSGS)
endif()


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


