/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   EurocDataset.h
 * @brief  RawDataSource from Euroc odometry/SLAM datasets
 * @author Jose Luis Blanco Claraco
 * @date   Jan 11, 2019
 */
#pragma once

#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/TPose3D.h>

#include <array>
#include <map>
#include <string>
#include <variant>

namespace mola
{
struct SensorCamera
{
    std::string                  sensor_name;
    std::string                  img_file_name;
    uint8_t                      cam_idx;
    mrpt::obs::CObservation::Ptr obs;
};
struct SensorIMU
{
    std::string                  sensor_name;
    double                       wx, wy, wz, accx, accy, accz;
    mrpt::obs::CObservation::Ptr obs;
};
using SensorEntry       = std::variant<std::monostate, SensorCamera, SensorIMU>;
using euroc_timestamp_t = uint64_t;
using euroc_dataset_t   = std::multimap<euroc_timestamp_t, SensorEntry>;

/** RawDataSource from EUROC odometry/SLAM datasets.
 * Each "sequence" directory contains these sensor streams:
 * - `cam0` & `cam1`: A grayscale stereo camera pair
 * - `imu0`: An ADIS16448 IMU sensor
 * - Ground truth poses
 *
 * \ingroup mola_input_euroc_dataset_grp */
class EurocDataset : public RawDataSourceBase
{
    DEFINE_MRPT_OBJECT(EurocDataset, mola)

   public:
    EurocDataset();
    ~EurocDataset() override = default;

    // See docs in base class
    void initialize(const Yaml& cfg) override;
    void spinOnce() override;

   private:
    std::string             base_dir_;  //!< base dir for `xxx/xx/mav0/...`
    std::string             sequence_;  //!< e.g. `machine_hall/MH_01_easy`
    mrpt::Clock::time_point replay_begin_time_{};
    bool                    replay_started_{false};
    double                  time_warp_scale_{1.0};
    std::array<mrpt::img::TCamera, 2>  cam_intrinsics_;
    std::array<mrpt::math::TPose3D, 2> cam_poses_;  //!< wrt vehicle origin
    euroc_dataset_t                    dataset_;  //!< dataset itself
    euroc_dataset_t::iterator          dataset_next_;  //!< next item to publish
    size_t                             dataset_cur_idx_ = 0;

    // double              replay_time_{.0};
    std::string seq_dir_;

    void build_dataset_entry_obs(SensorCamera& s);
    void build_dataset_entry_obs(SensorIMU& s);

    // void load_img(const unsigned int cam_idx, const std::size_t step);
    // void load_lidar(std::size_t step);
};

}  // namespace mola
