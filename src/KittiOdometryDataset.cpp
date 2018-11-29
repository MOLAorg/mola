/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   KittiOdometryDataset.h
 * @brief  RawDataSource from Kitti odometry/SLAM datasets
 * @author Jose Luis Blanco Claraco
 * @date   Nov 29, 2018
 */

/** \defgroup mola_sensor_kitti_dataset_grp RawDataSource from Kitti
 * odometry/SLAM datasets
 */

#include <mola-sensor-kitti-dataset/KittiOdometryDataset.h>
#include <yaml-cpp/yaml.h>

using namespace mola;

MOLA_REGISTER_RAWDATASOURCE(KittiOdometryDataset)

KittiOdometryDataset::KittiOdometryDataset()
{
    this->setLoggerName("KittiOdometryDataset");
}

void KittiOdometryDataset::initialize(const std::string& cfg_block)
{
    MRPT_START
    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg_block);

    auto cfg = YAML::Load(cfg_block);

    MRPT_END
}
void KittiOdometryDataset::spin() {}
