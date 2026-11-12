# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco.
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------
#
# Python bindings: the extension module `mola_metric_maps._core` plus the pure
# Python package `mola_metric_maps`. Silently skipped if pybind11 is not found.
#
# Options:
#  - MOLA_METRIC_MAPS_PYTHON_INSTALL_DIR: install dir of the `mola_metric_maps`
#    package, relative to CMAKE_INSTALL_PREFIX. Default:
#    lib/pythonX.Y/site-packages, as colcon expects.
# ------------------------------------------------------------------------------

set(PYBIND11_FINDPYTHON ON)
# Development.Module: extension modules do not need libpython, which some
# Python builds (e.g. the manylinux images) do not ship.
find_package(Python3 QUIET COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG QUIET)
if (NOT pybind11_FOUND OR NOT Python3_FOUND)
  return()
endif()

if (NOT MOLA_METRIC_MAPS_PYTHON_INSTALL_DIR)
  set(MOLA_METRIC_MAPS_PYTHON_INSTALL_DIR
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif()

# Package version, from package.xml:
file(READ "${PROJECT_SOURCE_DIR}/package.xml" _package_xml)
string(REGEX MATCH "<version>([0-9.]*)</version>" _ "${_package_xml}")
set(MOLA_METRIC_MAPS_VERSION "${CMAKE_MATCH_1}")

message(STATUS "mola_metric_maps: Python bindings enabled (pybind11 ${pybind11_VERSION}), install dir: ${MOLA_METRIC_MAPS_PYTHON_INSTALL_DIR}")

pybind11_add_module(_core MODULE
  src/mola_metric_maps_py.cpp
  src/py_common.cpp
  src/py_common.h
  src/py_incremental_point_cloud.cpp
  src/py_keyframe_map.cpp
  src/py_occgrid.cpp
  src/py_voxel_maps.cpp
)
mola_set_target_build_options(_core)
target_compile_definitions(_core PRIVATE
  MOLA_METRIC_MAPS_VERSION_STRING="${MOLA_METRIC_MAPS_VERSION}")
# Linking the library also loads it on import, which registers its map classes
# in the MRPT class factory (needed to deserialize them):
target_link_libraries(_core PRIVATE mola_metric_maps)

# Stage the package (Python sources plus the extension) in the build tree, so
# it can be imported and tested before installing:
set(_staging_dir "${CMAKE_CURRENT_BINARY_DIR}/python_staging")
set_target_properties(_core PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${_staging_dir}/mola_metric_maps")
add_custom_target(mola_metric_maps_python_sources ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/mola_metric_maps" "${_staging_dir}/mola_metric_maps"
  COMMENT "Staging the mola_metric_maps Python package"
)

install(TARGETS _core LIBRARY DESTINATION "${MOLA_METRIC_MAPS_PYTHON_INSTALL_DIR}/mola_metric_maps")
install(DIRECTORY mola_metric_maps DESTINATION "${MOLA_METRIC_MAPS_PYTHON_INSTALL_DIR}"
  PATTERN "__pycache__" EXCLUDE
)

# Tests (pytest), run against the staged package, if the test dependencies and
# the MRPT and mp2p_icp Python modules are available:
if (MOLA_METRIC_MAPS_BUILD_TESTING)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import pytest, numpy, mrpt.maps, mp2p_icp"
    RESULT_VARIABLE _pytest_result OUTPUT_QUIET ERROR_QUIET
  )
  if (_pytest_result EQUAL 0)
    add_test(
      NAME python_mola_metric_maps
      COMMAND "${Python3_EXECUTABLE}" -m pytest -q -p no:cacheprovider "${CMAKE_CURRENT_SOURCE_DIR}/tests"
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    )
    set(_test_env
      "PYTHONPATH=path_list_prepend:${_staging_dir}"
      "PYTHONDONTWRITEBYTECODE=set:1"
    )
    if (mola_test_datasets_FOUND)
      list(APPEND _test_env
        "MOLA_TEST_DATASETS_DIR=set:${mola_test_datasets_DIR}/../datasets")
    endif()
    set_tests_properties(python_mola_metric_maps PROPERTIES
      LABELS "python_bindings"
      ENVIRONMENT_MODIFICATION "${_test_env}"
    )
  else()
    message(STATUS "mola_metric_maps: pytest, numpy, or the mrpt or mp2p_icp Python modules not found, Python tests disabled")
  endif()
endif()
