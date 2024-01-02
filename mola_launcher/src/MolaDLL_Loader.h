/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
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
    mrpt::system::COutputLogger&    app,
    const std::vector<std::string>& lib_search_paths);

/** Returns the current list of loaded module dyanmic libraries. */
const std::map<std::string, LoadedModules>& get_loaded_modules();
