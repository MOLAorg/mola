# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco.
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.5)

if("$ENV{ROS_VERSION}" STREQUAL "2")
  set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_metric_maps LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find CMake dependencies:
find_package(mrpt-maps REQUIRED)
find_package(mp2p_icp_map REQUIRED)

find_package(TBB) # optional

if(TBB_FOUND)
  option(MOLA_METRIC_MAPS_USE_TBB "If found TBB, this option controls whether to use it or not" ON)
endif()


# 3rdparty deps:
add_subdirectory(3rdparty)

# -----------------------
# define lib:
set(LIB_SRCS
  src/HashedVoxelPointCloud.cpp
  src/KeyframePointCloudMap.cpp
  src/NDT.cpp
  src/OccGrid.cpp
  src/register.cpp
  src/SparseTreesPointCloud.cpp
  src/SparseVoxelPointCloud.cpp
)
set(LIB_PUBLIC_HDRS
  include/mola_metric_maps/FixedDenseGrid3D.h
  include/mola_metric_maps/HashedVoxelPointCloud.h
  include/mola_metric_maps/index3d_t.h
  include/mola_metric_maps/KeyframePointCloudMap.h
  include/mola_metric_maps/NDT.h
  include/mola_metric_maps/OccGrid.h
  include/mola_metric_maps/SparseTreesPointCloud.h
  include/mola_metric_maps/SparseVoxelPointCloud.h
)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::maps
    tsl::robin_map
    mola::mp2p_icp_map
  CMAKE_DEPENDENCIES
    mrpt-maps
    mp2p_icp_map
    robin_map
)

if (TBB_FOUND AND MOLA_METRIC_MAPS_USE_TBB)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MOLA_METRIC_MAPS_USE_TBB)
  target_link_libraries(${PROJECT_NAME} PRIVATE TBB::tbb)
endif()


# -----------------------
# define tests:
enable_testing()
add_subdirectory(tests)

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if(DETECTED_ROS2)
  # find dependencies
  find_package(ament_cmake REQUIRED)

  ament_package()
endif()

