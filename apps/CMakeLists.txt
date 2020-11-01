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
find_package(mrpt-containers)

mola_add_executable(
	TARGET  mola-cli
	SOURCES mola-cli.cpp
	LINK_LIBRARIES
		mola-launcher
		mrpt::core
		mrpt::tclap
)

mola_add_executable(
	TARGET  mola-dir
	SOURCES mola-dir.cpp
	LINK_LIBRARIES
		mola-launcher
)

mola_add_executable(
	TARGET  mola-yaml-parser
	SOURCES mola-yaml-parser.cpp
	LINK_LIBRARIES
	mola-kernel
	mrpt::tclap
	mrpt::containers
)
