# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2019, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# Include std cmake scripts:
include(CheckCXXCompilerFlag) # for check_cxx_compiler_flag()
include(CMakeDetermineCXXCompiler)

# Allow finding cmake packages that we are just configuring now:
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

# Common CMake scripts:
add_subdirectory(modules/mola-common)
find_package(mola-common REQUIRED) # find here so external projects can use it

# GTSAM:
find_package(GTSAM QUIET)
if (NOT GTSAM_FOUND)
  message(FATAL_ERROR
    " *** GTSAM was not found ***\n"
    "Please, install GTSAM or build from sources and set the cmake variable GTSAM_DIR to the build directory.\n"
    "In Ubuntu, the easiest solution is to install gtsam from: https://launchpad.net/~joseluisblancoc/+archive/ubuntu/gtsam-develop"
  )
endif()

# Process subdirectories:
option(MOLA_ENABLE_UNIT_TESTS "Build unit tests" ON)
if(MOLA_ENABLE_UNIT_TESTS)
    include(GoogleTest)
    enable_testing()
endif()

# External projects:
add_subdirectory(externals)

# MOLA modules:
add_subdirectory(modules)

# Docs:
find_program(SPHINX_EXECUTABLE NAMES sphinx-build HINTS $ENV{SPHINX_DIR} PATH_SUFFIXES bin DOC "Sphinx documentation generator")
if(SPHINX_EXECUTABLE)
  add_subdirectory(docs)
endif()

message(STATUS "==== MOLA configuration ====")
message(STATUS "C++ compiler              : ${CMAKE_CXX_COMPILER}")
message(STATUS "CMake generator           : ${CMAKE_GENERATOR}")
message(STATUS "CMake build tool          : ${CMAKE_BUILD_TOOL}")
message(STATUS "Compiler                  : ${CMAKE_CXX_COMPILER} Version: ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Sphinx (for docs)         : ${SPHINX_EXECUTABLE}")
message(STATUS "============================")
