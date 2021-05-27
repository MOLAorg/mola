/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mola-dir.cpp
 * @brief  main() for mola-dir: finds modules shared paths
 * @author Jose Luis Blanco Claraco
 * @date   Jun 24, 2019
 */

#include <mola-launcher/MolaLauncherApp.h>
#include <mrpt/core/exceptions.h>
#include <iostream>

int main(int argc, char** argv)
{
    try
    {
        if (argc != 2)
            throw std::runtime_error(
                "Usage: mola-dir <MODULE_NAME>\n"
                "You can also use `mola-cli --list-module-shared-dirs` to list "
                "all known module shared-files directories.");

        mola::MolaLauncherApp app;

        const auto modName   = std::string(argv[1]);
        const auto foundPath = app.findModuleSharedDir(modName);

        if (foundPath.empty())
        {
            std::cerr << "Module `" << modName << "` was not found.";
            return 1;
        }

        std::cout << foundPath << "\n";

        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "[mola-dir] Exit due to exception:\n"
                  << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
