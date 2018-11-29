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
#include <atomic>
#include <map>
#include <thread>
#include <vector>

namespace mola
{
/** Main launcher for a MOLA system. See mola-cli for a ready-to-use program.
 * \ingroup mola_launcher_grp */
class MolaLauncherApp : public mrpt::system::COutputLogger
{
   public:
    MolaLauncherApp();
    virtual ~MolaLauncherApp();

    /** @name SLAM system setup
     * @{ */

    /** Adds a directory to the list of paths to search for MOLA modules
     * (.so/.dll) when setup() is called
     * (Default=CMAKE_LIBRARY_OUTPUT_DIRECTORY).
     */
    void addModulesDirectory(const std::string& path);

    /** Prepares the SLAM system based on a YAML configuration file.
     * See [mola]/demos/ for example YAML files.
     * At this point, MOLA module libraries are searched in a list of paths
     * and loaded for their classes to be available in name-based class
     * factories. Modules must be named "libmola*" to be loaded.
     * \sa addModulesDirectory
     */
    void setup(const YAML::Node& cfg);

    /** Launches sensor and worker threads and enters into an infinite
     * loop executing the SLAM system. */
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
        std::string            name;
    };
    /** Indexed by `name` */
    std::map<std::string, InfoPerRawDataSource> data_sources_;

    void executor_datasource(InfoPerRawDataSource& rds);  //!< Thread func.

    /** Set to true to command all running threads to exit */
    std::atomic_bool threads_must_end_{false};

    /** Used in setup(), can be added to via addModulesDirectory() */
    std::vector<std::string> lib_search_paths_{};
};

#define ENSURE_YAML_ENTRY_EXISTS(_c, _name) \
    ASSERTMSG_(_c[_name], "Missing YAML required entry: `" _name "`");

}  // namespace mola
