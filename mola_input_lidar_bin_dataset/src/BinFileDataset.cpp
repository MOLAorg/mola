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
 * @file   BinFileDataset.cpp
 * @brief  RawDataSource from a directory of .bin LiDAR scan files
 * @author Gemini
 */

#include <mola_input_lidar_bin_dataset/BinFileDataset.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

using namespace mola;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::system;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(BinFileDataset, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_BinFileDataset)  // NOLINT(misc-use-anonymous-namespace)
{
  MOLA_REGISTER_MODULE(BinFileDataset);
}

namespace
{

std::vector<std::string> build_list_files(const std::string& dir, const std::string& file_extension)
{
  std::vector<std::string> out_lst;
  if (!mrpt::system::directoryExists(dir))
  {
    return {};
  }

  using direxpl = mrpt::system::CDirectoryExplorer;
  direxpl::TFileInfoList lstFiles;
  direxpl::explore(dir, FILE_ATTRIB_ARCHIVE, lstFiles);
  direxpl::sortByName(lstFiles);
  direxpl::filterByExtension(lstFiles, file_extension);
  out_lst.resize(lstFiles.size());
  std::transform(
      lstFiles.begin(), lstFiles.end(), out_lst.begin(), [](auto& fil) { return fil.wholePath; });
  return out_lst;
}
}  // namespace

BinFileDataset::BinFileDataset()  = default;
BinFileDataset::~BinFileDataset() = default;

/**
 * @brief Initializes the module by reading parameters, finding files, and generating timestamps.
 */
void BinFileDataset::initialize_rds(const mrpt::containers::yaml& params)
{
  using namespace std::string_literals;

  MRPT_START
  ProfilerEntry tle(profiler_, "initialize");

  MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << params);

  // Mandatory parameters:
  ENSURE_YAML_ENTRY_EXISTS(params, "params");
  auto cfg = params["params"];

  // 1. Get parameters: 'path' (required) and 'rate_hz' (optional)
  YAML_LOAD_MEMBER_REQ(bin_dir, std::string);
  YAML_LOAD_MEMBER_OPT(rate_hz, double);

  // 2. Find and sort all .bin files in the directory
  // Files are sorted alphabetically, which is assumed to be by time.
  lst_bin_files_ = build_list_files(bin_dir_, "bin");

  if (lst_bin_files_.empty())
  {
    THROW_EXCEPTION_FMT("Could not find any .bin files in directory: %s", bin_dir_.c_str());
  }

  // Check for a valid rate
  if (rate_hz_ <= 0.0)
  {
    THROW_EXCEPTION_FMT("'rate_hz' must be a positive value, got %.02f", rate_hz_);
  }

  // 3. Generate timestamps
  // Timestamps are uniformly spaced based on the provided rate.
  double dt = 1.0 / rate_hz_;
  lst_timestamps_.resize(lst_bin_files_.size());

  for (size_t i = 0; i < lst_timestamps_.size(); ++i)
  {
    lst_timestamps_[i] = static_cast<double>(i) * dt;  // Start from time t=0.0
  }

  MRPT_LOG_INFO_FMT(
      "[BinFileDataset] Found %zu scans. Sensor rate: %.02f Hz.", lst_bin_files_.size(), rate_hz_);

  initialized_ = true;

  MRPT_END
}

size_t BinFileDataset::datasetSize() const { return lst_bin_files_.size(); }

/**
 * @brief Reads the raw .bin file content and creates an MRPT observation object.
 *
 * It assumes the standard KITTI Velodyne format: 4 floats (x, y, z, intensity) per point.
 */
void BinFileDataset::load_lidar(timestep_t step) const
{
  if (read_ahead_lidar_obs_.count(step) != 0)
  {
    return;  // Already loaded
  }

  ASSERT_LT_(step, lst_bin_files_.size());
  const std::string full_path = lst_bin_files_[step];

  // Create the MRPT point cloud object for XYZ+Intensity data
  auto obs         = mrpt::obs::CObservationPointCloud::Create();
  obs->sensorLabel = "lidar";
  obs->setAsExternalStorage(
      full_path, mrpt::obs::CObservationPointCloud::ExternalStorageFormat::KittiBinFile);
  obs->load();  // force loading now from disk
  ASSERTMSG_(
      obs->pointcloud, mrpt::format("Error loading kitti scan file: '%s'", full_path.c_str()));

  // Pose: velodyne is at the origin of the vehicle coordinates in
  // Kitti datasets.
  obs->sensorPose = mrpt::poses::CPose3D();
  obs->timestamp  = mrpt::Clock::fromDouble(lst_timestamps_.at(step));

  // Cache the result
  read_ahead_lidar_obs_[step] = obs;
}

std::shared_ptr<CObservationPointCloud> BinFileDataset::getPointCloud(timestep_t step) const
{
  ASSERT_LT_(step, lst_timestamps_.size());

  load_lidar(step);
  auto o = std::dynamic_pointer_cast<CObservationPointCloud>(read_ahead_lidar_obs_.at(step));
  ASSERT_(o);
  return o;
}

mrpt::obs::CSensoryFrame::Ptr BinFileDataset::datasetGetObservations(size_t timestep) const
{
  auto lck             = mrpt::lockHelper(dataset_ui_mtx_);
  last_used_tim_index_ = timestep;

  auto sf = CSensoryFrame::Create();
  // Insert the single observation (the point cloud) into the SensoryFrame
  sf->insert(getPointCloud(timestep));

  return sf;
}

void BinFileDataset::spinOnce()
{
  MRPT_START

  ASSERT_(initialized_);

  ProfilerEntry tleg(profiler_, "spinOnce");

  const auto tNow = mrpt::Clock::now();

  // Starting time:
  if (!last_play_wallclock_time_)
  {
    last_play_wallclock_time_ = tNow;
  }

  // get current replay time:
  auto         lckUIVars       = mrpt::lockHelper(dataset_ui_mtx_);
  const double time_warp_scale = time_warp_scale_;
  const bool   paused          = paused_;
  const auto   teleport_here   = teleport_here_;
  teleport_here_.reset();
  lckUIVars.unlock();

  double dt = mrpt::system::timeDifference(*last_play_wallclock_time_, tNow) * time_warp_scale;
  last_play_wallclock_time_ = tNow;

  // override by an special teleport order?
  if (teleport_here.has_value() && *teleport_here < lst_timestamps_.size())
  {
    replay_next_tim_index_ = *teleport_here;
    last_dataset_time_     = lst_timestamps_[replay_next_tim_index_];
  }
  else
  {
    if (paused)
    {
      return;
    }
    // move forward replayed dataset time:
    last_dataset_time_ += dt;
  }

  if (replay_next_tim_index_ >= lst_timestamps_.size())
  {
    onDatasetPlaybackEnds();  // notify base class

    MRPT_LOG_THROTTLE_INFO(
        10.0, "End of dataset reached! Nothing else to publish (CTRL+C to quit)");
    return;
  }

  if (!lst_timestamps_.empty())
  {
    MRPT_LOG_THROTTLE_INFO_FMT(
        5.0, "Dataset replay progress: %lu / %lu  (%4.02f%%)",
        static_cast<unsigned long>(replay_next_tim_index_),
        static_cast<unsigned long>(lst_timestamps_.size()),
        (100.0 * replay_next_tim_index_) / (lst_timestamps_.size()));
  }

  // We have to publish all observations until "t":
  while (replay_next_tim_index_ < lst_timestamps_.size() &&
         last_dataset_time_ >= lst_timestamps_[replay_next_tim_index_])
  {
    MRPT_LOG_DEBUG_STREAM(
        "Sending observations for replay time: "
        << mrpt::system::formatTimeInterval(last_dataset_time_));

    // Save one single timestamp for all observations, since they are in
    // theory synchronized in the Kitti datasets:
    // const auto obs_tim = mrpt::Clock::fromDouble(lst_timestamps_[replay_next_tim_index_]);

    {
      ProfilerEntry tle(profiler_, "spinOnce.publishLidar");
      load_lidar(replay_next_tim_index_);
      auto o = read_ahead_lidar_obs_[replay_next_tim_index_];
      // o->timestamp = obs_tim; // already done in load_lidar()
      this->sendObservationsToFrontEnds(o);
    }

    // Free memory in read-ahead buffers:
    read_ahead_lidar_obs_.erase(replay_next_tim_index_);

    replay_next_tim_index_++;
  }

  {
    auto lck             = mrpt::lockHelper(dataset_ui_mtx_);
    last_used_tim_index_ = replay_next_tim_index_ > 0 ? replay_next_tim_index_ - 1 : 0;
  }

  // Read ahead to save delays in the next iteration:
  if (replay_next_tim_index_ < lst_timestamps_.size())
  {
    ProfilerEntry tle(profiler_, "spinOnce.read_ahead");
    if (0 == read_ahead_lidar_obs_.count(replay_next_tim_index_))
    {
      load_lidar(replay_next_tim_index_);
    }
  }

  MRPT_END
}
