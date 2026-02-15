/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   RawlogDataset.cpp
 * @brief  RawDataSource for datasets in MRPT rawlog format
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2019
 */

/** \defgroup mola_input_video_grp mola_input_video_grp.
 * RawDataSource for live or offline video/image datasets
 *
 */

#include <mola_input_video/VideoDataset.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>

IMPLEMENTS_MRPT_OBJECT(VideoDataset, RawDataSourceBase, mola)

MRPT_INITIALIZER(register_VideoDataset) { MOLA_REGISTER_MODULE(mola::VideoDataset); }

namespace
{
std::vector<std::string> build_list_files(
    const std::string& dir, const std::string& file_extension)  // NOLINT
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
      lstFiles.begin(), lstFiles.end(), out_lst.begin(), [](auto& fil) { return fil.name; });
  return out_lst;
}

}  // namespace

namespace mola
{

VideoDataset::VideoDataset() = default;

void VideoDataset::initialize_rds(const Yaml& params)
{
  using std::string_literals::operator""s;

  const auto cfg = params["params"];

  const std::string mode_str = cfg["input_mode"].as<std::string>();
  YAML_LOAD_MEMBER_OPT(image_publish_rate, double);
  ASSERT_GT_(image_publish_rate_, 0.0);

  YAML_LOAD_MEMBER_OPT(sensor_label, std::string);

  if (cfg.has("camera_model"))
  {
    camera_model_ = mrpt::img::TCamera::FromYAML(cfg["camera_model"]);
    MRPT_LOG_INFO_STREAM("Camera model loaded from config file:\n" << camera_model_.dumpAsText());
  }
  else
  {
    MRPT_LOG_WARN_STREAM(
        "No camera model provided. Using default camera model with no distortion.");
  }

  if (cfg.has("camera_pose"))
  {
    camera_pose_ =
        mrpt::poses::CPose3D::FromString("["s + cfg["camera_pose"].as<std::string>() + "]"s);
    MRPT_LOG_INFO_STREAM("Camera pose on the vehicle: " << camera_pose_.asString());
  }

  input_mode_ = mrpt::typemeta::TEnumType<InputMode>::name2value(mode_str);

  if (input_mode_ == InputMode::ImageDirectory)
  {
    image_base_dir_ = cfg["image_dir"].as<std::string>();
    const auto ext  = cfg["image_ext"].as<std::string>();
    image_files_    = build_list_files(image_base_dir_, ext);
    MRPT_LOG_INFO_STREAM(
        "Publishing a directory of images. Found " << image_files_.size() << " files under '"
                                                   << image_base_dir_ << "'");
    ASSERT_(!image_files_.empty());
  }
  else if (input_mode_ == InputMode::VideoFile)
  {
    cam_ = std::make_shared<mrpt::hwdrivers::CCameraSensor>();

    const auto video_uri = cfg["video_uri"].as<std::string>();
    if (video_uri.empty())
    {
      THROW_EXCEPTION("Missing 'video_uri' parameter");
    }
    mrpt::config::CConfigFileMemory cfgMem;
    cfgMem.write("camera", "grabber_type", "ffmpeg");
    cfgMem.write("camera", "ffmpeg_url", video_uri);

    cam_->loadConfig(cfgMem, "camera");

    MRPT_LOG_INFO_STREAM("Trying to open video stream from '" << video_uri << "'...");
    cam_->initialize();
    MRPT_LOG_INFO_STREAM("Successful.");
  }
  else if (input_mode_ == InputMode::LiveCamera)
  {
    cam_ = std::make_shared<mrpt::hwdrivers::CCameraSensor>();

    mrpt::config::CConfigFileMemory cfgMem;
    cfgMem.write("camera", "grabber_type", "opencv");

    if (cfg.has("camera_options") && cfg["camera_options"].isMap())
    {
      for (const auto& [key, value] : cfg["camera_options"].asMapRange())
      {
        cfgMem.write("camera", key.as<std::string>(), value.as<std::string>());
      }
    }

    cam_->loadConfig(cfgMem, "camera");
    cam_->initialize();
  }
  else
  {
    THROW_EXCEPTION_FMT("Unknown input mode: %s", mode_str.c_str());
  }

  last_play_wallclock_time_ = mrpt::Clock::now();
}

void VideoDataset::spinOnce()
{
  auto lambdaSetObsFields = [this](mrpt::obs::CObservationImage& obs)
  {
    // TODO(jlbc): More accurate timestamp:
    obs.timestamp    = mrpt::Clock::now();
    obs.sensorLabel  = sensor_label_;
    obs.cameraParams = camera_model_;
  };

  if (paused_)
  {
    return;
  }

  if (input_mode_ == InputMode::LiveCamera)
  {
    cam_->doProcess();

    auto obs = cam_->getNextFrame();
    if (auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(obs))
    {
      lambdaSetObsFields(*o);
      this->sendObservationsToFrontEnds(o);
    }
    return;
  }

  if (input_mode_ == InputMode::VideoFile)
  {
    // Read next video frame:
    auto obs = cam_->getNextFrame();
    if (auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(obs))
    {
      lambdaSetObsFields(*o);
      this->sendObservationsToFrontEnds(o);
    }
    else
    {
      MRPT_LOG_THROTTLE_INFO(5.0, "End of video file reached.");
    }
    return;
  }

  // Image directory mode:
  auto tNow = mrpt::Clock::now();

  std::scoped_lock lck(dataset_ui_mtx_);

  double dt = mrpt::system::timeDifference(last_play_wallclock_time_, tNow) * time_warp_scale_;
  last_play_wallclock_time_ = tNow;

  if (teleport_here_)
  {
    image_index_ = *teleport_here_;
  }
  teleport_here_.reset();

  last_dataset_time_ += dt;

  const double frame_period = 1.0 / image_publish_rate_;

  while (image_index_ < image_files_.size() &&
         (static_cast<double>(image_index_) * frame_period) <= last_dataset_time_)
  {
    MRPT_LOG_DEBUG_STREAM("Publishing image: " << image_index_ << " / " << image_files_.size());

    auto obs = std::make_shared<mrpt::obs::CObservationImage>();
    obs->image.setExternalStorage(
        mrpt::system::pathJoin({image_base_dir_, image_files_[image_index_]}));
    sendObservationsToFrontEnds(obs);

    image_index_++;
  }
}

size_t VideoDataset::datasetUI_size() const
{
  if (input_mode_ == InputMode::ImageDirectory)
  {
    return image_files_.size();
  }
  return 10000000;
}
size_t VideoDataset::datasetUI_lastQueriedTimestep() const { return image_index_; }
double VideoDataset::datasetUI_playback_speed() const { return time_warp_scale_; }
void   VideoDataset::datasetUI_playback_speed(double speed) { time_warp_scale_ = speed; }
bool   VideoDataset::datasetUI_paused() const { return paused_; }
void   VideoDataset::datasetUI_paused(bool paused) { paused_ = paused; }
void   VideoDataset::datasetUI_teleport(size_t timestep) { teleport_here_ = timestep; }

}  // namespace mola