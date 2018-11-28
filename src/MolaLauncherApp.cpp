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
#include <map>
#include <sstream>

using namespace mola;

MolaLauncherApp::MolaLauncherApp()
    : mrpt::system::COutputLogger("MolaLauncherApp")
{
}

void MolaLauncherApp::setup(const YAML::Node& cfg)
{
    MRPT_TRY_START

    MRPT_LOG_INFO(
        "Setting up system from YAML config... (set DEBUG verbosity level to "
        "see full config)");

    MRPT_LOG_DEBUG_STREAM(
        "Using the following configuration:\n"
        "==================================================\n"
        << cfg << std::endl
        << "==================================================\n");

    ENSURE_YAML_ENTRY_EXISTS(cfg, "slam_backend");
    const auto& cfg_sb = cfg["slam_backend"];

    ENSURE_YAML_ENTRY_EXISTS(cfg, "front_ends");
    const auto& cfg_fe = cfg["front_ends"];

    ENSURE_YAML_ENTRY_EXISTS(cfg, "raw_data_sources");
    const auto& cfg_rds = cfg["raw_data_sources"];

    // Create raw data source objects:
    for (const auto& ds : cfg_rds)
    {
        ENSURE_YAML_ENTRY_EXISTS(ds, "type");
        ENSURE_YAML_ENTRY_EXISTS(ds, "name");
        ENSURE_YAML_ENTRY_EXISTS(ds, "params");

        const auto ds_label = ds["name"].as<std::string>();
        ASSERTMSG_(!ds_label.empty(), "`name` cannot be empty!");
        if (data_sources_.count(ds_label) != 0)
            THROW_EXCEPTION_FMT("Duplicated `name`: %s", ds_label.c_str());

        const auto ds_classname = ds["type"].as<std::string>();
        ASSERTMSG_(!ds_classname.empty(), "`type` cannot be empty!");

        InfoPerRawDataSource info;
        {
            // Make a copy of the YAML config block:
            std::stringstream ss;
            ss << ds;
            info.yaml_cfg_block = ss.str();
        }
        info.impl = mola::RawDataSourceBase::Factory(ds_classname);

        data_sources_[ds_label] = std::move(info);
    }

    MRPT_TRY_END
}

void MolaLauncherApp::spin()
{
    MRPT_TRY_START
    // ...
    MRPT_TRY_END
}
