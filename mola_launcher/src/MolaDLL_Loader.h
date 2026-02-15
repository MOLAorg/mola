/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   MolaDLL_Loader.h
 * @brief  Manager of dynamically loaded modules (.dll/.so)
 * @author Jose Luis Blanco Claraco
 * @date   Nov 29, 2018
 */

#pragma once

#include <mrpt/system/COutputLogger.h>

#include <string>
#include <vector>

/** Used in internal_load_lib_modules() */
struct LoadedModules
{
  std::string lib_path;
  void*       handle{nullptr};
};

/** Loads all libs under lib_search_paths_. \sa setup() */
void internal_load_lib_modules(
    mrpt::system::COutputLogger& app, const std::vector<std::string>& lib_search_paths);

/** Returns the current list of loaded module dyanmic libraries. */
const std::map<std::string, LoadedModules>& get_loaded_modules();
