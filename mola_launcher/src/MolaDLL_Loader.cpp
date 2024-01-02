/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MolaDLL_Loader.cpp
 * @brief  Manager of dynamically loaded modules (.dll/.so)
 * @author Jose Luis Blanco Claraco
 * @date   Nov 29, 2018
 */

#include "MolaDLL_Loader.h"
#include <mrpt/core/exceptions.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <map>

#if defined(__unix__)
#include <dlfcn.h>
#endif

MRPT_TODO(
    "Improvement: automatic scanning libraries and extract their classes");

/** From internal_load_lib_modules() */
static std::map<std::string, LoadedModules> loaded_lib_handled;

const std::map<std::string, LoadedModules>& get_loaded_modules()
{
    return loaded_lib_handled;
}

/** Loads all libs under lib_search_paths_. \sa setup() */
void internal_load_lib_modules(
    mrpt::system::COutputLogger&    app,
    const std::vector<std::string>& lib_search_paths)
{
    MRPT_TRY_START

#if defined(__unix__)
#define DLL_EXT "so"
#else
#define DLL_EXT "dll"
#endif

    using direxpl = mrpt::system::CDirectoryExplorer;
    using mrpt::system::LVL_DEBUG;

    for (const auto& path : lib_search_paths)
    {
        app.logStr(
            LVL_DEBUG, mrpt::format(
                           "[load modules]: Searching for modules under: `%s`",
                           path.c_str()));

        direxpl::TFileInfoList lst;
        direxpl::explore(path, FILE_ATTRIB_ARCHIVE, lst);
        direxpl::filterByExtension(lst, DLL_EXT);
        for (const auto& lib : lst)
        {
            if (lib.name.find("mola") == std::string::npos) continue;  // skip

            // Already loaded?
            if (loaded_lib_handled.count(lib.name) != 0) continue;  // skip

#if defined(__unix__)
            // Check if already loaded?
            {
                void* handle = dlopen(lib.wholePath.c_str(), RTLD_NOLOAD);
                if (handle != nullptr)
                {
                    app.logStr(
                        LVL_DEBUG, mrpt::format(
                                       "Skipping already loaded lib: %s",
                                       lib.name.c_str()));
                    continue;  // skip
                }
            }

            void* handle = dlopen(lib.wholePath.c_str(), RTLD_LAZY);
#else
            HMODULE handle = LoadLibrary(lib.wholePath.c_str());
#endif
            if (handle == nullptr)
            {
                const char* err = dlerror();
                if (!err) err = "(error calling dlerror())";
                THROW_EXCEPTION(mrpt::format(
                    "Error loading module: `%s`\ndlerror(): `%s`",
                    lib.wholePath.c_str(), err));
            }

            app.logStr(
                LVL_DEBUG, mrpt::format(
                               "[load modules]: Successfully loaded: `%s`",
                               lib.name.c_str()));

            loaded_lib_handled[lib.name] = LoadedModules{lib.wholePath, handle};
        }
    }

#undef DLL_EXT
    MRPT_TRY_END
}

#if defined(_MSC_VER)
static void internal_unload_lib_modules();
#else
static void internal_unload_lib_modules() __attribute__((destructor));
#endif

void internal_unload_lib_modules()
{
    for (const auto& lib : loaded_lib_handled)
    {
        // printf("[mola-launcher] Unloading: %s\n",
        // lib.second.lib_path.c_str());
#if defined(__unix__)
        dlclose(lib.second.handle);
#else
        FreeLibrary(lib.second.handle);
#endif
    }
    loaded_lib_handled.clear();
}
