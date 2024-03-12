# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2024, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

if("$ENV{ROS_VERSION}" STREQUAL "2")
	set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_launcher LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find dependencies:
find_package(mrpt-core)

find_mola_package(mola_kernel)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

# Use C++17 filesystem lib:
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Filesystem REQUIRED)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mrpt::core
		mola::mola_kernel
	PRIVATE_LINK_LIBRARIES
		${CMAKE_DL_LIBS} # To be able to load DLLs/.so's
		CXX::Filesystem  # the C++>=17 std lib
	CMAKE_DEPENDENCIES
		mola_kernel
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    BUILDTIME_MOLA_MODULES_LIB_PATH="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
    # Use top-level source directory:
    BUILDTIME_MOLA_MODULES_SHARED_PATH="${CMAKE_SOURCE_DIR}/modules"
    )

# apps:
add_subdirectory(apps)


# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if (DETECTED_ROS2)


	# Mark executables and/or libraries for installation
	set(CATKIN_PACKAGE_LIB_DESTINATION lib)
	set(CATKIN_PACKAGE_BIN_DESTINATION lib/${PROJECT_NAME})
	set(CATKIN_PACKAGE_INCLUDE_DESTINATION include)
	set(CATKIN_PACKAGE_SHARE_DESTINATION share/${PROJECT_NAME})

	# Mark executables and/or libraries for installation
	install(TARGETS mola-cli
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

	# find dependencies
	find_package(ament_cmake REQUIRED)

	#ament_target_dependencies(MY_TARGET
	#  rclcpp...
	#)

	ament_package()
endif()
