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

#include <mola_kernel/interfaces/Dataset_UI.h>
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
class EurocDataset : public RawDataSourceBase, public Dataset_UI

{
    DEFINE_MRPT_OBJECT(EurocDataset, mola)

   public:
    EurocDataset();
    ~EurocDataset() override = default;

    // See docs in base class
    void spinOnce() override;

    // Virtual interface of Dataset_UI (see docs in derived class)
    size_t datasetUI_size() const override { return dataset_.size(); }
    size_t datasetUI_lastQueriedTimestep() const override
    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        return last_used_tim_index_;
    }
    double datasetUI_playback_speed() const override
    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        return time_warp_scale_;
    }
    void datasetUI_playback_speed(double speed) override
    {
        auto lck         = mrpt::lockHelper(dataset_ui_mtx_);
        time_warp_scale_ = speed;
    }
    bool datasetUI_paused() const override
    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        return paused_;
    }
    void datasetUI_paused(bool paused) override
    {
        auto lck = mrpt::lockHelper(dataset_ui_mtx_);
        paused_  = paused;
    }
    void datasetUI_teleport(size_t timestep) override
    {
        auto lck       = mrpt::lockHelper(dataset_ui_mtx_);
        teleport_here_ = timestep;
    }

   protected:
    // See docs in base class
    void initialize_rds(const Yaml& cfg) override;

   private:
    std::string base_dir_;  //!< base dir for `xxx/xx/mav0/...`
    std::string sequence_;  //!< e.g. `machine_hall/MH_01_easy`
    std::array<mrpt::img::TCamera, 2>  cam_intrinsics_;
    std::array<mrpt::math::TPose3D, 2> cam_poses_;  //!< wrt vehicle origin
    euroc_dataset_t                    dataset_;  //!< dataset itself
    euroc_dataset_t::iterator          dataset_next_;  //!< next item to publish
    size_t                             dataset_cur_idx_ = 0;

    std::optional<mrpt::Clock::time_point> last_play_wallclock_time_;
    double                                 last_dataset_time_ = 0;

    // double              replay_time_{.0};
    std::string seq_dir_;

    void build_dataset_entry_obs(SensorCamera& s);
    void build_dataset_entry_obs(SensorIMU& s);

    mutable timestep_t    last_used_tim_index_ = 0;
    bool                  paused_              = false;
    double                time_warp_scale_     = 1.0;
    std::optional<size_t> teleport_here_;
    mutable std::mutex    dataset_ui_mtx_;
};

}  // namespace mola
