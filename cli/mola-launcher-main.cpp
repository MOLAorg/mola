/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mola-launcher-main.cpp
 * @brief  main() for mola-launcher app
 * @author Jose Luis Blanco Claraco
 * @date   Nov 28, 2018
 */

#include <mrpt/core/exceptions.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>
#include <mola-launcher/MolaLauncherApp.h>
#include <string>
#include <iostream>

// Declare supported cli switches ===========
TCLAP::CmdLine               cmd("mola-launcher");
TCLAP::ValueArg<std::string> arg_yaml_cfg(
    "c", "config", "Input YAML config file (required) (*.yml)", true, "",
    "demo.yml", cmd);

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        // Load YAML config file:
        const auto file_yml = arg_yaml_cfg.getValue();

        YAML::Node cfg = YAML::LoadFile(file_yml);

        MolaLauncherApp app;
        app.setup(cfg);
        app.spin();

        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "[mola-launcher] Exit due to exception:\n"
                  << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
