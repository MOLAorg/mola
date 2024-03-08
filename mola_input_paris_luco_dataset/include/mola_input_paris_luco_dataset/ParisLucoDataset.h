/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ParisLucoDataset.h
 * @brief  RawDataSource from Paris Luco Dataset
 * @author Jose Luis Blanco Claraco
 * @date   Feb 1, 2024
 */
#pragma once

#include <mola_kernel/interfaces/Dataset_UI.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/obs_frwds.h>

#include <array>

// fwrd decls:
namespace mrpt::obs
{
class CObservationPointCloud;
}

namespace mola
{
/** RawDataSource from Paris Luco Dataset.
 *
 * Dataset available from: https://github.com/jedeschaud/ct_icp
 *
 * The LiDAR sensor is a HDL-32, recording data at the Luxembourg Garden
 * (Paris).
 *
 * Point clouds are published as mrpt::obs::CObservationPointCloud
 * with clouds of types mrpt::maps::CPointsMapXYZIRT, with these populated
 * fields:
 * - `XYZ`
 * - `I`: Intensity, range [0.0 - 1.0?]
 * - `T`: Time of each point, in range [-0.05, 0.05] seconds (scan rate=10 Hz),
 *   such that "t=0" (the observation/scan timestamp) corresponds to the moment
 *   the scanner is facing forward.
 * - `R`: ring_id (0-31). It was not provided by the original dataset, but it is
 *   reconstructed from point elevation data in this package.
 *
 * Expected contents under `base_dir` directory:
 *
 * Example `base_dir`: `/mnt/storage/ParisLuco/`
 * Sequence: `00` (this dataset only has one sequence?)
 * \code
 * ParisLuco
 * └── 00
 *     ├── frames
 *     │   ├── frame_00000.ply
 *     │   ├── frame_00001.ply
 *     │   ...
 *     └── gt_traj_lidar.txt
 * ...
 * \endcode
 *
 * \ingroup mola_input_paris_luco_dataset_grp
 */
class ParisLucoDataset : public RawDataSourceBase,
                         public OfflineDatasetSource,
                         public Dataset_UI
{
    DEFINE_MRPT_OBJECT(ParisLucoDataset, mola)

   public:
    ParisLucoDataset();
    ~ParisLucoDataset() override = default;

    // See docs in base class
    void spinOnce() override;
    bool hasGroundTruthTrajectory() const override
    {
        return !groundTruthTrajectory_.empty();
    }
    trajectory_t getGroundTruthTrajectory() const override
    {
        return groundTruthTrajectory_;
    }

    // See docs in base class:
    size_t datasetSize() const override;

    mrpt::obs::CSensoryFrame::Ptr datasetGetObservations(
        size_t timestep) const override;

    // Virtual interface of Dataset_UI (see docs in derived class)
    size_t datasetUI_size() const override { return datasetSize(); }
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
    bool        initialized_ = false;
    std::string base_dir_;  //!< base dir for "00/*".
    std::string sequence_ = "00";  //!< only "00" in this dataset

    const double lidarPeriod_ = 1.0 / 10.0;  // [s]

    timestep_t                             replay_next_tim_index_{0};
    std::optional<mrpt::Clock::time_point> last_play_wallclock_time_;
    double                                 last_dataset_time_ = 0;

    std::vector<std::string>  lstLidarFiles_;
    mrpt::math::CMatrixDouble groundTruthTranslations_;
    trajectory_t              groundTruthTrajectory_;
    mutable std::map<timestep_t, mrpt::obs::CObservation::Ptr>
        read_ahead_lidar_obs_;

    std::vector<double> lst_timestamps_;
    double              replay_time_{.0};
    std::string         seq_dir_;

    void load_lidar(timestep_t step) const;
    void readAheadSome();
    void autoUnloadOldEntries() const;

    mutable timestep_t    last_used_tim_index_ = 0;
    bool                  paused_              = false;
    double                time_warp_scale_     = 1.0;
    std::optional<size_t> teleport_here_;
    mutable std::mutex    dataset_ui_mtx_;
};

}  // namespace mola
