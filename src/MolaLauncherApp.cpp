/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MolaLauncherApp.h
 * @brief  Main launcher for MOLA systems
 * @author Jose Luis Blanco Claraco
 * @date   Nov 28, 2018
 */

/** \defgroup mola_launcher_grp mola-launcher: Library to load and launch MOLA
 * systems
 */

#include <mola-launcher/MolaLauncherApp.h>
#include <mrpt/core/exceptions.h>
#include <iostream>

using namespace mola;

#define ENSURE_CFG_SECTION_EXIST(_c, _name) \
    ASSERTMSG_(_c[_name], "YAML file: Missing required entry: `" _name "`");

void MolaLauncherApp::setup(const YAML::Node& cfg)
{
    MRPT_TRY_START

    std::cout << "[mola-launcher] Using the following configuration:\n"
                 "==================================================\n"
              << cfg << std::endl
              << "==================================================\n";

    ENSURE_CFG_SECTION_EXIST(cfg, "slam_backend");
    const auto& cfg_sb = cfg["slam_backend"];

    ENSURE_CFG_SECTION_EXIST(cfg, "front_ends");
    const auto& cfg_fe = cfg["front_ends"];

    ENSURE_CFG_SECTION_EXIST(cfg, "raw_data_sources");
    const auto& cfg_rds = cfg["raw_data_sources"];

    MRPT_TRY_END
}

void MolaLauncherApp::spin()
{
    MRPT_TRY_START
    // ...
    MRPT_TRY_END
}
