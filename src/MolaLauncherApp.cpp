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

#include <mola-kernel/env_vars.h>
#include <mola-launcher/MolaLauncherApp.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/CRateTimer.h>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include "MolaDLL_Loader.h"

using namespace mola;

MolaLauncherApp::MolaLauncherApp()
    : mrpt::system::COutputLogger("MolaLauncherApp")
{
    lib_search_paths_.emplace_back(MOLA_MODULES_DIR);
}

MolaLauncherApp::~MolaLauncherApp() { this->shutdown(); }

void MolaLauncherApp::shutdown()
{
    // End all threads:
    threads_must_end_ = true;
    if (!data_sources_.empty())
    {
        MRPT_LOG_INFO_STREAM(
            "Shutting down " << data_sources_.size()
                             << " raw-data-source worker threads...");
        for (auto& ds : data_sources_)
            if (ds.second.executor.joinable()) ds.second.executor.join();
        MRPT_LOG_INFO("Done.");
        data_sources_.clear();
    }

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms);
}

void MolaLauncherApp::addModulesDirectory(const std::string& path)
{
    lib_search_paths_.push_back(path);
}

void MolaLauncherApp::setup(const YAML::Node& cfg_in)
{
    MRPT_TRY_START

    // make sure all available modules are loaded and classes are registered.
    internal_load_lib_modules(*this, lib_search_paths_);

    MRPT_LOG_INFO(
        "Setting up system from YAML config... (set DEBUG verbosity level to "
        "see full config)");

    // Parse YAML env variables:
    YAML::Node cfg;
    {
        std::stringstream ss;
        ss << cfg_in;
        cfg = YAML::Load(mola::parseEnvVars(ss.str()));
    }

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

        InfoPerRawDataSource& info = data_sources_[ds_label];
        {
            // Make a copy of the YAML config block:
            std::stringstream ss;
            ss << ds;
            info.yaml_cfg_block = ss.str();
        }
        info.name = ds_label;
        MRPT_LOG_INFO_STREAM(
            "Instantiating module `" << ds_label << "` of type `"
                                     << ds_classname << "`");
        // Create object (needs to be registered):
        info.impl = mola::RawDataSourceBase::Factory(ds_classname);
        // Inherit verbosity level:
        info.impl->setMinLoggingLevel(this->getMinLoggingLevel());
        info.execution_rate = ds["execution_rate"].as<double>();
    }

    MRPT_TRY_END
}

void MolaLauncherApp::spin()
{
    MRPT_TRY_START

    // Launch working threads:
    // ---------------------------------
    for (auto& ds : data_sources_)
    {
        ds.second.executor = std::thread(
            &MolaLauncherApp::executor_datasource, this, std::ref(ds.second));
    }

    // Main SLAM/Localization infinite loop
    // -------------------------------------------
    MRPT_LOG_INFO(
        "Entering main SLAM/localization loop..."
        "(CTRL+C from mola-cli to stop)");
    while (!threads_must_end_)
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(200ms);
    }
    MRPT_LOG_INFO("Main SLAM/localization loop ended.");

    MRPT_TRY_END
}

void MolaLauncherApp::executor_datasource(InfoPerRawDataSource& rds)
{
    try
    {
        if (threads_must_end_) return;

        // Initilize:
        rds.impl->initialize_common(rds.yaml_cfg_block);
        rds.impl->initialize(rds.yaml_cfg_block);

        mrpt::system::CRateTimer timer(rds.execution_rate);

        while (!threads_must_end_)
        {
            rds.impl->spinOnce();

            // Done, cycle:
            const bool ontime = timer.sleep();
            if (!ontime)
                MRPT_LOG_THROTTLE_WARN_STREAM(
                    1.0,
                    "Could not achieve desired real-time execution rate ("
                        << rds.execution_rate
                        << " Hz) on thread for sensor named: " << rds.name);
        };
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM(
            "Error: Will shutdown since thread for sensor named `"
            << rds.name << "` ended due to an exception:\n"
            << mrpt::exception_to_str(e));
        threads_must_end_ = true;
    }
}
