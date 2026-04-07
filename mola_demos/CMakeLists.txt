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
  demos
  DESTINATION
    share/${PROJECT_NAME}
)


# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if(DETECTED_ROS2)
  # find dependencies
  find_package(ament_cmake REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
endif()

# Silent warning on unused variable:
set(ignored ${CMAKE_EXPORT_COMPILE_COMMANDS})
unset(ignored)
