/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawlogDataset.cpp
 * @brief  RawDataSource for datasets in MRPT rawlog format
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2019
 */

/** \defgroup mola_sensor_rawlog_grp mola_sensor_rawlog_grp.
 * RawDataSource for datasets in MRPT rawlog format
 *
 */

#include <mola-kernel/yaml_helpers.h>
#include <mola-sensor-rawlog/RawlogDataset.h>
#include <mrpt/core/initializer.h>
#include <yaml-cpp/yaml.h>

using namespace mola;
using namespace mola::sensor_rawlog_dataset;

MRPT_INITIALIZER(do_register){MOLA_REGISTER_MODULE(RawlogDataset)}

RawlogDataset::RawlogDataset() = default;

void RawlogDataset::initialize(const std::string& cfg_block)
{
    using namespace std::string_literals;

    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    // Mandatory parameters:
    auto c = YAML::Load(cfg_block);

    ENSURE_YAML_ENTRY_EXISTS(c, "params");
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg);

    // YAML_LOAD_MEMBER_REQ(base_dir, std::string);

    MRPT_END
}  // end initialize()

void RawlogDataset::spinOnce()
{
    MRPT_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    MRPT_END
}
