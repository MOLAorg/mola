/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   MulranDataset.h
 * @brief  RawDataSource from Mulran datasets
 * @author Jose Luis Blanco Claraco
 * @date   Dec 12, 2023
 */
#pragma once

#include <mola_kernel/interfaces/Dataset_UI.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/obs_frwds.h>

// forward declarations:
namespace mrpt::obs
{
class CObservationPointCloud;
}

namespace mola
{
/** RawDataSource from Mulran datasets.
 * Each "sequence" directory contains these sensor streams:
 * - `lidar`: Ouster OS1-64 LIDAR. As mrpt::obs::CObservationPointCloud
 *            with XYZIRT channels.
 * - `gps`  : Consumer-grade GNSS receiver, as mrpt::obs::CObservationGPS with
 *            a message type NMEA GGA (mrpt::obs::gnss::Message_NMEA_GGA).
 *            HDOP in that struct is computed as reported standard deviation
 *            from the dataset, divided by HDOP_REFERENCE_METERS.
 * - `radar`: Navtech CIR204-H (Not implemented yet).
 * - `imu`  : As mrpt::obs::CObservationIMU.
 * - Ground truth poses
 *
 * Point clouds are published as mrpt::obs::CObservationPointCloud
 * with clouds with these populated fields:
 * - `XYZ`
 * - `I`: Intensity, range [0.0 - 1.0]
 * - `T`: Time of each point, in range [-0.05, 0.05] seconds (scan rate=10 Hz),
 *   such that "t=0" (the observation/scan timestamp) corresponds to the moment
 *   the scanner is facing forward.
 *
 * Expected contents under `base_dir` directory:
 *
 * Example `base_dir`: `/mnt/storage/MulRan/`
 * Sequences: `KAIST01`, `KAIST02`, etc.
 * \code
 * MulRan/
 * ├── KAIST01
 * │   ├── data_stamp.csv
 * │   ├── global_pose.csv
 * │   ├── gps.csv
 * │   ├── navtech_top_stamp.csv
 * │   ├── Ouster
 * │   │   ├── 1561000444390857630.bin
 * │   │   ├── 1561000444489636410.bin
 * │   │   ...
 * │   ├── ouster_front_stamp.csv
 * │   └── xsens_imu.csv
 * ...
 * \endcode
 *
 * \ingroup mola_input_mulran_dataset_grp
 */
class MulranDataset : public RawDataSourceBase, public OfflineDatasetSource, public Dataset_UI
{
  DEFINE_MRPT_OBJECT(MulranDataset, mola)

 public:
  MulranDataset();

  static constexpr double HDOP_REFERENCE_METERS = 4.5;

  // See docs in base class
  void         spinOnce() override;
  bool         hasGroundTruthTrajectory() const override { return !groundTruthTrajectory_.empty(); }
  trajectory_t getGroundTruthTrajectory() const override { return groundTruthTrajectory_; }

  /** Direct programmatic access to dataset observations. The return may be
   * nullptr if the given index is not of the requested type.
   *
   * `step` is in the range `0` to `getGroundTruthTrajectory()-1`
   */
  mrpt::obs::CObservationPointCloud::Ptr getPointCloud(timestep_t step) const;
  mrpt::obs::CObservationGPS::Ptr        getGPS(timestep_t step) const;
  mrpt::obs::CObservationIMU::Ptr        getIMU(timestep_t step) const;

  // See docs in base class:
  size_t datasetSize() const override;

  mrpt::obs::CSensoryFrame::Ptr datasetGetObservations(size_t timestep) const override;

  bool hasGPS() const { return !gpsCsvData_.empty(); }

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
  std::string base_dir_;  //!< base dir for "sequences/*".
  std::string sequence_;  //!< "00", "01", ...
  bool        lidar_to_ground_truth_1to1_ = true;

  bool  publish_lidar_                       = true;
  bool  publish_gps_                         = true;
  bool  publish_imu_                         = true;
  bool  publish_ground_truth_                = true;
  float normalize_intensity_channel_maximum_ = 2500.0f;

  std::optional<mrpt::Clock::time_point> last_play_wallclock_time_;
  double                                 last_dataset_time_ = 0;

  enum class EntryType : uint8_t
  {
    Invalid = 0,
    Lidar,
    GNSS,
    GroundTruth,
    IMU,
  };

  struct Entry
  {
    EntryType type = EntryType::Invalid;

    /// In lstPointCloudFiles_ and read_ahead_lidar_obs_
    timestep_t lidarIdx = 0;  // idx in lstPointCloudFiles_
    timestep_t gpsIdx   = 0;  // row indices in gpsCsvData_
    timestep_t gtIdx    = 0;  // idx in groundTruthTrajectory_
    timestep_t imuIdx   = 0;  // idx in imuCsvData_
  };

  std::multimap<double, Entry>           datasetEntries_;
  std::multimap<double, Entry>::iterator replay_next_it_;

  std::vector<std::string> lstPointCloudFiles_;

  trajectory_t                                                         groundTruthTrajectory_;
  mutable std::map<timestep_t, mrpt::obs::CObservationPointCloud::Ptr> read_ahead_lidar_obs_;

  mrpt::math::CMatrixDouble gpsCsvData_;
  /** I found no extrinsics for the GPS sensor in this dataset web, but it
   * *seems* it's the same system than in "Complex Urban Dataset":
   * https://sites.google.com/view/complex-urban-dataset/system?authuser=0
   */
  mrpt::poses::CPose3D gpsPoseOnVehicle_ = mrpt::poses::CPose3D::Identity();
  mrpt::poses::CPose3D ousterPoseOnVehicle_;

  mrpt::math::CMatrixDouble imuCsvData_;

  /** It *seems* authors used the same system than in "Complex Urban Dataset":
   * https://sites.google.com/view/complex-urban-dataset/system?authuser=0
   *
   * So IMU should be aligned XYZ with the vehicle, and 7cm backwards.
   */
  mrpt::poses::CPose3D imuPoseOnVehicle_ = mrpt::poses::CPose3D::FromTranslation(-0.07, .0, .0);

  double      replay_time_ = .0;
  std::string seq_dir_;

  void                            load_lidar(timestep_t step) const;
  mrpt::obs::CObservationGPS::Ptr get_gps_by_row_index(size_t row) const;
  mrpt::obs::CObservationIMU::Ptr get_imu_by_row_index(size_t row) const;

  void autoUnloadOldEntries() const;

  static double LidarFileNameToTimestamp(const std::string& filename);

  mutable timestep_t    last_used_tim_index_ = 0;
  bool                  paused_              = false;
  double                time_warp_scale_     = 1.0;
  std::optional<size_t> teleport_here_;
  mutable std::mutex    dataset_ui_mtx_;
};

}  // namespace mola
