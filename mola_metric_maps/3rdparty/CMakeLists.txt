# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2024, Jose Luis Blanco-Claraco.
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# This defines the target "tsl::robin_map"
add_subdirectory(robin-map)

# Install lib:
install(TARGETS robin_map EXPORT robin_map-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
# Install hdrs:
install(
  DIRECTORY robin-map/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

# make project importable from build_dir:
export(
    TARGETS robin_map
    # export to ROOT cmake directory (when building MOLA as a superproject)
    FILE ${CMAKE_BINARY_DIR}/robin_map-targets.cmake
    NAMESPACE tsl::
)

# And generate the -config.cmake file:
set(ALL_DEPS_LIST "") # used in xxx-config.cmake.in
set(MOLA_MODULE_NAME robin_map)
configure_file(
"${_MOLACOMMON_MODULE_BASE_DIR}/mola-xxx-config.cmake.in"
"${CMAKE_BINARY_DIR}/robin_map-config.cmake" IMMEDIATE @ONLY
)
# Install cmake config module
install(
	EXPORT
		robin_map-targets
	DESTINATION
		${CMAKE_INSTALL_LIBDIR}/robin_map/cmake
)
install(
	FILES
		${CMAKE_BINARY_DIR}/robin_map-config.cmake
		#${CMAKE_BINARY_DIR}/robin_map-config-version.cmake
	DESTINATION
		${CMAKE_INSTALL_LIBDIR}/robin_map/cmake
)
