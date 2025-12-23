/* _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\\___/|_|\\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
*/

/**
 * @file   BinFileDataset.h
 * @brief  RawDataSource from a directory of .bin LiDAR scan files
 * @author Gemini
 */
#pragma once

#include <mola_kernel/interfaces/Dataset_UI.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/obs/obs_frwds.h>

#include <mutex>
#include <string>

// forward declarations:
namespace mrpt::obs
{
class CObservationPointCloud;
}

namespace mola
{
/** RawDataSource from a directory of time-ordered .bin LiDAR scan files.
 * Each .bin file is expected to contain a sequence of 4 floats (x, y, z, intensity)
 * for each point, in the standard KITTI format.
 */
class BinFileDataset : public OfflineDatasetSource, public RawDataSourceBase, public Dataset_UI
{
  DEFINE_MRPT_OBJECT(BinFileDataset, mola)

 public:
  BinFileDataset();
  ~BinFileDataset() override;

  // Prevent copying and moving
  BinFileDataset(const BinFileDataset&)            = delete;
  BinFileDataset& operator=(const BinFileDataset&) = delete;
  BinFileDataset(BinFileDataset&&)                 = delete;
  BinFileDataset& operator=(BinFileDataset&&)      = delete;

  // From OfflineDatasetSource:
  void                          initialize_rds(const mrpt::containers::yaml& params) override;
  size_t                        datasetSize() const override;
  mrpt::obs::CSensoryFrame::Ptr datasetGetObservations(size_t timestep) const override;

  void spinOnce() override;
  bool hasGroundTruthTrajectory() const override { return false; }

  // From Dataset_UI (minimal implementations):

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
  // Internal data access
  std::shared_ptr<mrpt::obs::CObservationPointCloud> getPointCloud(timestep_t step) const;

  /** Loads and caches a LiDAR scan from disk. */
  void load_lidar(timestep_t step) const;

  // --- Parameters ---
  std::string bin_dir_;  //!< **path:** Path to the directory with .bin files (Required)
  double      rate_hz_{10.0};  //!< **rate_hz:** LiDAR sensor rate in Hz (Default: 10.0)

  // Dataset structure
  std::vector<std::string> lst_bin_files_;
  std::vector<double>      lst_timestamps_;

  // Runtime state (inherited from pattern)
  mutable timestep_t    last_used_tim_index_ = 0;
  bool                  paused_              = false;
  double                time_warp_scale_     = 1.0;
  std::optional<size_t> teleport_here_;
  mutable std::mutex    dataset_ui_mtx_;
  timestep_t            replay_next_tim_index_ = 0;

  std::optional<mrpt::Clock::time_point> last_play_wallclock_time_;
  double                                 last_dataset_time_ = 0;

  // Data cache
  mutable std::map<timestep_t, mrpt::obs::CObservation::Ptr> read_ahead_lidar_obs_;

  bool initialized_ = false;
};
}  // namespace mola
