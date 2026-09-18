# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco.
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

if("$ENV{ROS_VERSION}" STREQUAL "2")
  set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_metric_maps LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find CMake dependencies:
find_package(mola_kernel REQUIRED)
find_package(mrpt_maps REQUIRED)
find_package(mrpt_viz REQUIRED)
find_package(mp2p_icp_map REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(TBB) # optional

if(TBB_FOUND)
  option(MOLA_METRIC_MAPS_USE_TBB "If found TBB, this option controls whether to use it or not" ON)
endif()

# mola::IncrementalPointCloud needs nanoflann's incremental (self-balancing)
# k-d tree index, added in nanoflann 1.10.0. On ROS distributions shipping an
# older nanoflann the build still succeeds: the class is compiled and registered
# as usual, but its k-d tree factory is replaced by a stub that throws an
# explanatory error if anything actually tries to instantiate the map.
find_package(nanoflann QUIET)
if(nanoflann_FOUND AND nanoflann_VERSION VERSION_GREATER_EQUAL 1.10)
  set(MOLA_HAS_INCREMENTAL_POINT_CLOUD ON)
  message(STATUS "mola_metric_maps: nanoflann ${nanoflann_VERSION} found, building mola::IncrementalPointCloud")
else()
  set(MOLA_HAS_INCREMENTAL_POINT_CLOUD OFF)
  if(nanoflann_FOUND)
    set(MOLA_NANOFLANN_FOUND_VERSION "${nanoflann_VERSION}")
  else()
    set(MOLA_NANOFLANN_FOUND_VERSION "not found")
  endif()
  message(WARNING
    "mola_metric_maps: nanoflann >=1.10 is required by mola::IncrementalPointCloud, "
    "but the available version is '${MOLA_NANOFLANN_FOUND_VERSION}'. The class will be "
    "built as a stub that throws at runtime; everything else is unaffected.")
endif()

# mola::IncrementalPointCloud's k-d tree *baking* (TCreationOptions::serialize_kdtree,
# the mm-ipc-bake-kdtree CLI tool) additionally needs nanoflann's
# KDTreeSingleIndexIncrementalAdaptor::saveIndex()/loadIndex(), added in 1.11.0.
# Without it, everything else in this library works as usual: the option is a
# no-op on write and a loaded .mm's baked blob (if any) is skipped on read.
if(MOLA_HAS_INCREMENTAL_POINT_CLOUD AND nanoflann_VERSION VERSION_GREATER_EQUAL 1.11)
  set(MOLA_HAS_INCREMENTAL_POINT_CLOUD_KDTREE_BAKE ON)
  message(STATUS "mola_metric_maps: nanoflann ${nanoflann_VERSION} >= 1.11, enabling IncrementalPointCloud k-d tree baking")
else()
  set(MOLA_HAS_INCREMENTAL_POINT_CLOUD_KDTREE_BAKE OFF)
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
  src/OptionsIniIO.cpp
  src/register.cpp
  src/SparseTreesPointCloud.cpp
  src/SparseVoxelPointCloud.cpp
  src/TSDF.cpp
)
set(LIB_PUBLIC_HDRS
  include/mola_metric_maps/FixedDenseGrid3D.h
  include/mola_metric_maps/HashedVoxelPointCloud.h
  include/mola_metric_maps/index3d_t.h
  include/mola_metric_maps/KeyframePointCloudMap.h
  include/mola_metric_maps/NDT.h
  include/mola_metric_maps/OccGrid.h
  include/mola_metric_maps/OptionsCapable.h
  include/mola_metric_maps/OptionsIniIO.h
  include/mola_metric_maps/SparseTreesPointCloud.h
  include/mola_metric_maps/SparseVoxelPointCloud.h
  include/mola_metric_maps/TSDF.h
)

# The map class is always built; only the k-d tree backend is conditional (see
# the nanoflann version check above).
list(APPEND LIB_SRCS src/IncrementalPointCloud.cpp)
list(APPEND LIB_PUBLIC_HDRS include/mola_metric_maps/IncrementalPointCloud.h)

if(MOLA_HAS_INCREMENTAL_POINT_CLOUD)
  list(APPEND LIB_SRCS src/IncrementalKDTree.cpp)
else()
  list(APPEND LIB_SRCS src/IncrementalKDTree_stub.cpp)
endif()

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_maps
    mrpt::mrpt_viz
    tsl::robin_map
    mola::mola_kernel
    mola::mp2p_icp_map
  PRIVATE_LINK_LIBRARIES
    Eigen3::Eigen
  CMAKE_DEPENDENCIES
    mola_kernel
    mrpt_maps
    mrpt_viz
    mp2p_icp_map
    robin_map
)

if (TBB_FOUND AND MOLA_METRIC_MAPS_USE_TBB)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MOLA_METRIC_MAPS_USE_TBB)
  target_link_libraries(${PROJECT_NAME} PRIVATE TBB::tbb)

  # Sorting mp2p_icp::point_with_cov_pair_t (which embeds an over-aligned
  # Eigen-backed covariance matrix) makes GCC emit a -Wpsabi note about an
  # internal ABI change. The affected comparator never crosses a shared-library
  # boundary, so the note is safe to silence.
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi)
  endif()
endif()

if(MOLA_HAS_INCREMENTAL_POINT_CLOUD)
  target_compile_definitions(${PROJECT_NAME}
    PUBLIC MOLA_METRIC_MAPS_HAS_INCREMENTAL_POINT_CLOUD)

  if(MOLA_HAS_INCREMENTAL_POINT_CLOUD_KDTREE_BAKE)
    target_compile_definitions(${PROJECT_NAME}
      PUBLIC MOLA_METRIC_MAPS_HAS_INCREMENTAL_KDTREE_BAKE)
  endif()

  # nanoflann is deliberately NOT linked as a target, and its header is NOT
  # reached through the include path at all: src/IncrementalKDTree.cpp includes
  # it by ABSOLUTE path via the macro below.
  #
  # The reason is that MRPT ships (and its binaries are built against) its own,
  # usually older, bundled copy of the very same "nanoflann.hpp", exported as an
  # -isystem directory. Adding the right directory as "-I" does not help: when a
  # directory appears both as -I and as -isystem, GCC drops the -I entry and
  # keeps the system one, so MRPT's copy would still win. An absolute path
  # removes the ambiguity entirely, and keeps the header out of every other
  # translation unit of this library.
  get_target_property(MOLA_NANOFLANN_INCLUDE_DIRS nanoflann::nanoflann INTERFACE_INCLUDE_DIRECTORIES)
  find_file(MOLA_NANOFLANN_HEADER nanoflann.hpp
    PATHS ${MOLA_NANOFLANN_INCLUDE_DIRS}
    NO_DEFAULT_PATH
  )
  if(NOT MOLA_NANOFLANN_HEADER)
    message(FATAL_ERROR "nanoflann >=1.10 was found, but its nanoflann.hpp could not be located in '${MOLA_NANOFLANN_INCLUDE_DIRS}'")
  endif()

  set_source_files_properties(src/IncrementalKDTree.cpp
    PROPERTIES COMPILE_DEFINITIONS "MOLA_NANOFLANN_HEADER=\"${MOLA_NANOFLANN_HEADER}\"")
else()
  # So the stub can name the version actually found in its error message:
  set_source_files_properties(src/IncrementalKDTree_stub.cpp
    PROPERTIES COMPILE_DEFINITIONS "MOLA_NANOFLANN_FOUND_VERSION=\"${MOLA_NANOFLANN_FOUND_VERSION}\"")
endif()

# -----------------------
# define apps:
add_subdirectory(apps)

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

