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
project(mola_kernel LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)
find_package(mola_yaml REQUIRED)

# find dependencies:
find_package(MRPT 2.15.0 REQUIRED COMPONENTS gui obs maps topography)

# define lib:
set(LIB_SRCS
  src/interfaces/ExecutableBase.cpp
  src/interfaces/FilterBase.cpp
  src/interfaces/FrontEndBase.cpp
  src/interfaces/NavStateFilter.cpp
  src/interfaces/RawDataSourceBase.cpp
  src/interfaces/VizInterface.cpp
  src/utils/Synchronizer.cpp
  src/MinimalModuleContainer.cpp
  src/pretty_print_exception.cpp
  src/register.cpp
)

set(LIB_PUBLIC_HDRS
  include/mola_kernel/FastAllocator.h
  include/mola_kernel/Georeferencing.h
  include/mola_kernel/GuiWidgetDescription.h
  include/mola_kernel/id.h
  include/mola_kernel/interfaces/Dataset_UI.h
  include/mola_kernel/interfaces/DiagnosticsProvider.h
  include/mola_kernel/interfaces/ExecutableBase.h
  include/mola_kernel/interfaces/FilterBase.h
  include/mola_kernel/interfaces/FrontEndBase.h
  include/mola_kernel/interfaces/LocalizationSourceBase.h
  include/mola_kernel/interfaces/MapServer.h
  include/mola_kernel/interfaces/MapSourceBase.h
  include/mola_kernel/interfaces/NavStateFilter.h
  include/mola_kernel/interfaces/OfflineDatasetSource.h
  include/mola_kernel/interfaces/RawDataConsumer.h
  include/mola_kernel/interfaces/RawDataSourceBase.h
  include/mola_kernel/interfaces/Relocalization.h
  include/mola_kernel/interfaces/VizInterface.h
  include/mola_kernel/MinimalModuleContainer.h
  include/mola_kernel/pretty_print_exception.h
  include/mola_kernel/utils/RegexCache.h
  include/mola_kernel/utils/Synchronizer.h
  include/mola_kernel/variant_helper.h
  include/mola_kernel/Yaml.h
)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::obs
    mrpt::topography
    mola::mola_yaml
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt-obs
    mrpt-topography
    mola_yaml
)

if("$ENV{VERBOSE}")
  message(STATUS "mola_kernel: Using MRPT version: ${MRPT_VERSION}")
endif()

#----
# Extract version from package.xml
# Example line:" <version>0.3.2</version>"
file(READ package.xml contentPackageXML)
string(REGEX MATCH "<version>([0-9\.]*)</version>" _ ${contentPackageXML})
set(MOLA_VERSION ${CMAKE_MATCH_1})
message(STATUS "MOLA detected version (from package.xml): ${MOLA_VERSION}")
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)" _ ${MOLA_VERSION})
set(MOLA_MAJOR_VERSION ${CMAKE_MATCH_1})
set(MOLA_MINOR_VERSION ${CMAKE_MATCH_2})
set(MOLA_PATCH_VERSION ${CMAKE_MATCH_3})
#----


target_compile_definitions(${PROJECT_NAME} PUBLIC MOLA_MAJOR_VERSION=${MOLA_MAJOR_VERSION})
target_compile_definitions(${PROJECT_NAME} PUBLIC MOLA_MINOR_VERSION=${MOLA_MINOR_VERSION})
target_compile_definitions(${PROJECT_NAME} PUBLIC MOLA_PATCH_VERSION=${MOLA_PATCH_VERSION})

# Tests
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
