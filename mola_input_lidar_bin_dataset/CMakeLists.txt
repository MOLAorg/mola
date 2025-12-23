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
project(mola_input_bin_dataset LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find dependencies:
find_package(mrpt-maps REQUIRED)    # For CGenericPointsMap
find_package(mrpt-obs REQUIRED)     # For CObservationPointCloud
find_package(mrpt-system REQUIRED)  # For file system utilities
find_package(mrpt-core REQUIRED)    # For mrpt::Clock

find_mola_package(mola_kernel REQUIRED)
find_mola_package(mola_yaml REQUIRED) # For parameter parsing

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/BinFileDataset.cpp)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/mola_input_bin_dataset/BinFileDataset.h)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mola::mola_kernel
    mola::mola_yaml
  PRIVATE_LINK_LIBRARIES
    mrpt::maps
    mrpt::obs
    mrpt::system
    mrpt::core
  CMAKE_DEPENDENCIES
    mola_kernel
    mola_yaml
)