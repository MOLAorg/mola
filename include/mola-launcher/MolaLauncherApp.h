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
#pragma once

#include <mola-kernel/RawDataSourceBase.h>
#include <mrpt/system/COutputLogger.h>
#include <yaml-cpp/yaml.h>
#include <map>
#include <thread>

namespace mola
{
/** Main launcher for a MOLA system. See mola-cli for a ready-to-use program.
 * \ingroup mola_launcher_grp */
class MolaLauncherApp : public mrpt::system::COutputLogger
{
   public:
    MolaLauncherApp();
    virtual ~MolaLauncherApp() = default;

    /** @name SLAM system setup
     * @{ */

    /** Prepares the SLAM system based on a YAML configuration file.
     * See [mola]/demos/ for example YAML files. */
    void setup(const YAML::Node& cfg);

    /** Enters into an infinite loop executing the SLAM system. */
    void spin();
    /** @} */

    /** @name SLAM system control & monitoring
     * @{ */

    // TODO: set SLAM / localization mode

    /** @} */

   private:
    struct InfoPerRawDataSource
    {
        std::string            yaml_cfg_block;
        RawDataSourceBase::Ptr impl;
        std::thread            executor;
    };
    std::map<std::string, InfoPerRawDataSource> data_sources_;
};

#define ENSURE_YAML_ENTRY_EXISTS(_c, _name) \
    ASSERTMSG_(_c[_name], "Missing YAML required entry: `" _name "`");

}  // namespace mola
