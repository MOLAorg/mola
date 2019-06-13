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
#pragma once

#include <mola-kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/TPose3D.h>
#include <array>

namespace mola
{
/** RawDataSource from Kitti odometry/SLAM datasets.
 * Each "sequence" directory contains these sensor streams:
 * - `image_0` & `image_1`: A grayscale stereo camera pair
 * - `image_2` & `image_3`: An RGB stereo camera pair
 * - `lidar`: Velodyne 3D LIDAR
 * - Ground truth poses
 *
 * \ingroup mola_sensor_kitti_dataset_grp */
class KittiOdometryDataset : public RawDataSourceBase
{
   public:
    KittiOdometryDataset();
    ~KittiOdometryDataset() override = default;

    // See docs in base class
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;

   private:
    std::string             base_dir_;  //!< base dir for "sequences/*".
    std::string             sequence_;  //!< "00", "01", ...
    mrpt::Clock::time_point replay_begin_time_{};
    std::size_t             replay_next_tim_index_{0};
    bool                    replay_started_{false};
    bool                    publish_lidar_{true};
    std::array<bool, 4>     publish_image_{{true, true, true, true}};
    double                  time_warp_scale_{1.0};
    std::array<mrpt::img::TCamera, 4>  cam_intrinsics_;
    std::array<mrpt::math::TPose3D, 4> cam_poses_;  //!< wrt vehicle origin

    std::array<std::vector<std::string>, 4>             lst_image_;
    std::vector<std::string>                            lst_velodyne_;
    std::map<std::size_t, mrpt::obs::CObservation::Ptr> read_ahead_lidar_obs_;
    std::map<std::size_t, std::array<mrpt::obs::CObservation::Ptr, 4>>
        read_ahead_image_obs_;

    std::vector<double> lst_timestamps_;
    double              replay_time_{.0};
    std::string         seq_dir_;

    void load_img(const unsigned int cam_idx, const std::size_t step);
    void load_lidar(std::size_t step);
};

}  // namespace mola
