/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MolaViz.cpp
 * @brief  Main C++ class for MOLA GUI
 * @author Jose Luis Blanco Claraco
 * @date   May  11, 2019
 */

/** \defgroup mola_viz_grp mola-viz
 * C++ library for main MOLA GUI
 */

#include <mola-kernel/yaml_helpers.h>
#include <mola-viz/MolaViz.h>
#include <mrpt/core/initializer.h>
#include <yaml-cpp/yaml.h>

using namespace mola;

MRPT_INITIALIZER(do_register){MOLA_REGISTER_MODULE(MolaViz)}

MolaViz::MolaViz() = default;

void MolaViz::initialize(const std::string& cfg_block)
{
    MRPT_START

    // Load:
    auto c   = YAML::Load(cfg_block);
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

    MRPT_END
}
