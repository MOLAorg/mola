# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2019, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

find_package(mrpt-core)
find_package(mrpt-tclap) # tclap wrapper, useful for Windows, etc.

mola_add_executable(
	TARGET  mola-cli
	SOURCES mola-cli.cpp
	LINK_LIBRARIES
		mola-launcher
		mrpt::core
		mrpt::tclap
		yaml-cpp
)

mola_add_executable(
	TARGET  mola-dir
	SOURCES mola-dir.cpp
	LINK_LIBRARIES
		mola-launcher
)
