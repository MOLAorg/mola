# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco.
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Superbuild for the mola-metric-maps Python wheel, driven by scikit-build-core
# (see pyproject.toml). It builds mola_metric_maps (the C++ library and its
# Python bindings, no apps nor tests) as an ExternalProject, against the C++
# SDKs of the mp2p-icp wheel (mp2p_icp/sdk: mp2p_icp_map and the mola_common
# CMake functions) and of the mrpt3 wheel (mrpt/sdk: MRPT, Eigen, nanoflann,
# oneTBB), and puts into the wheel:
#
#   mola_metric_maps/   Python package, .pyi stubs, _core.*.so and
#                       libmola_metric_maps.so
#
# The mp2p_icp, MRPT and oneTBB libraries are never bundled: they are loaded
# from the installed mp2p-icp and mrpt3 wheels (RUNPATH), since there must be
# only one MRPT class registry per process.

cmake_minimum_required(VERSION 3.24)
project(mola_metric_maps_wheel LANGUAGES C CXX)

include(ExternalProject)

get_filename_component(MOLA_METRIC_MAPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# The C++ SDKs of the mrpt3 and mp2p-icp wheels installed in the build
# environment:
execute_process(
  COMMAND "${Python_EXECUTABLE}" -c
    "import mrpt.sdk as m, mp2p_icp.sdk as p; print(m.get_cmake_prefix_path(), m.MRPT_VERSION, m.PYBIND11_VERSION, p.get_cmake_prefix_path(), p.MP2P_ICP_VERSION, p.PYBIND11_VERSION, sep=';')"
  OUTPUT_VARIABLE _sdk_info
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE _sdk_result
)
if (NOT _sdk_result EQUAL 0)
  message(FATAL_ERROR
    "Cannot import mrpt.sdk and mp2p_icp.sdk: the mrpt3 and mp2p-icp wheels must be installed in the build environment")
endif()
list(GET _sdk_info 0 MRPT_SDK_DIR)
list(GET _sdk_info 1 MRPT_SDK_VERSION)
list(GET _sdk_info 2 MRPT_SDK_PYBIND11_VERSION)
list(GET _sdk_info 3 MP2P_SDK_DIR)
list(GET _sdk_info 4 MP2P_SDK_VERSION)
list(GET _sdk_info 5 MP2P_SDK_PYBIND11_VERSION)
message(STATUS "MRPT SDK ${MRPT_SDK_VERSION} (pybind11 ${MRPT_SDK_PYBIND11_VERSION}): ${MRPT_SDK_DIR}")
message(STATUS "mp2p_icp SDK ${MP2P_SDK_VERSION} (pybind11 ${MP2P_SDK_PYBIND11_VERSION}): ${MP2P_SDK_DIR}")

# Python types are only shared between extensions built with the same pybind11:
foreach(_v MRPT_SDK_PYBIND11_VERSION MP2P_SDK_PYBIND11_VERSION)
  if (NOT pybind11_VERSION VERSION_EQUAL ${_v})
    message(FATAL_ERROR
      "pybind11 ${pybind11_VERSION} found, but the mrpt3 and mp2p-icp wheels use "
      "${MRPT_SDK_PYBIND11_VERSION} and ${MP2P_SDK_PYBIND11_VERSION}: update the pins in pyproject.toml")
  endif()
endforeach()

# IncrementalPointCloud needs nanoflann >= 1.10 for its k-d tree index (and
# >= 1.11 for k-d tree baking), from the MRPT SDK. Without it, the library
# would build a stub that throws at run time.
find_package(nanoflann 1.10 CONFIG REQUIRED
  PATHS "${MRPT_SDK_DIR}" NO_DEFAULT_PATH)
message(STATUS "nanoflann ${nanoflann_VERSION} (MRPT SDK)")

# The ExternalProject is installed here; only the Python package and the
# library go into the wheel (no headers nor CMake files):
set(MMM_PREFIX_DIR "${CMAKE_BINARY_DIR}/prefix")

# Both libmola_metric_maps.so and the extension end up in
# site-packages/mola_metric_maps/, next to site-packages/mp2p_icp/sdk/lib/ and
# site-packages/mrpt/sdk/lib/:
set(_rpath "\$ORIGIN;\$ORIGIN/../mp2p_icp/sdk/lib;\$ORIGIN/../mrpt/sdk/lib")

set(_cache_args
  -DCMAKE_BUILD_TYPE:STRING=Release
  -DCMAKE_INSTALL_PREFIX:PATH=${MMM_PREFIX_DIR}
  -DCMAKE_INSTALL_LIBDIR:STRING=lib
  -DCMAKE_PREFIX_PATH:STRING=${MP2P_SDK_DIR};${MRPT_SDK_DIR}
  -DCMAKE_INSTALL_RPATH:STRING=${_rpath}
  # Wheels cannot hold the libfoo.so -> libfoo.so.X.Y symlinks:
  -DMOLA_NO_SOVERSION:BOOL=ON
  # No C++20 modules are used, and the module scanning of C++20 targets
  # (with Ninja) fails with ccache:
  -DCMAKE_CXX_SCAN_FOR_MODULES:BOOL=OFF
  -DPython_EXECUTABLE:FILEPATH=${Python_EXECUTABLE}
  -DPython3_EXECUTABLE:FILEPATH=${Python_EXECUTABLE}
  -Dpybind11_DIR:PATH=${pybind11_DIR}
  # Only use the SDKs of the wheels, not other packages from the user package
  # registry (~/.cmake/packages) or from a sourced ROS environment
  # ($CMAKE_PREFIX_PATH): they would not be there at run time.
  -DCMAKE_FIND_USE_PACKAGE_REGISTRY:BOOL=OFF
  -DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH:BOOL=OFF
  -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY:BOOL=ON
)
# (CMAKE_MAKE_PROGRAM changes on each build: pip uses temporary environments)
foreach(_var
    CMAKE_MAKE_PROGRAM CMAKE_C_COMPILER CMAKE_CXX_COMPILER
    CMAKE_C_COMPILER_LAUNCHER CMAKE_CXX_COMPILER_LAUNCHER
    CMAKE_TOOLCHAIN_FILE)
  if (DEFINED ${_var} AND NOT "${${_var}}" STREQUAL "")
    list(APPEND _cache_args "-D${_var}:STRING=${${_var}}")
  endif()
endforeach()

# mola_metric_maps: the library and the Python bindings (with oneTBB, if found
# in the MRPT SDK).
ExternalProject_Add(ep_mola_metric_maps
  SOURCE_DIR "${MOLA_METRIC_MAPS_DIR}"
  BINARY_DIR "${CMAKE_BINARY_DIR}/mola_metric_maps"
  CMAKE_CACHE_ARGS ${_cache_args}
    # The Python package goes to ${MMM_PREFIX_DIR}/mola_metric_maps:
    -DMOLA_METRIC_MAPS_PYTHON_INSTALL_DIR:PATH=.
  CMAKE_ARGS
    -DMOLA_METRIC_MAPS_BUILD_APPS=OFF
    -DMOLA_METRIC_MAPS_BUILD_TESTING=OFF
    -DMOLA_METRIC_MAPS_BUILD_PYTHON=ON
  # Always re-run the build tool, so source edits are picked up:
  BUILD_ALWAYS ON
  USES_TERMINAL_CONFIGURE ON
  USES_TERMINAL_BUILD ON
  USES_TERMINAL_INSTALL ON
)

# Into the wheel. The bindings are skipped without error if pybind11 or Python
# are not found: make sure they are there.
set(_pkg_dir "${MMM_PREFIX_DIR}/mola_metric_maps")
set(_lib "${MMM_PREFIX_DIR}/lib/libmola_metric_maps.so")
install(CODE "
  file(GLOB _ext \"${_pkg_dir}/_core*.so\")
  if (NOT _ext OR NOT EXISTS \"${_lib}\")
    message(FATAL_ERROR \"The mola_metric_maps._core extension or library was not built\")
  endif()
")
install(DIRECTORY "${_pkg_dir}" DESTINATION .
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
)
install(PROGRAMS "${_lib}" DESTINATION mola_metric_maps)
