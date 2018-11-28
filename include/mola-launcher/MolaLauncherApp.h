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

#include <yaml-cpp/yaml.h>

namespace mola
{
/** Main launcher for a MOLA system. See mola-cli for a ready-to-use program.
 * \ingroup mola_launcher_grp */
class MolaLauncherApp
{
   public:
    MolaLauncherApp()  = default;
    ~MolaLauncherApp() = default;

    void setup(const YAML::Node& cfg);

    void spin();
};

}  // namespace mola
