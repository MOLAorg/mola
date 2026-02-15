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
 * @file   VideoDataset.h
 * @brief  RawDataSource for live or offline video/image datasets
 * @author Jose Luis Blanco Claraco
 * @date   Apr 25, 2025
 */
#pragma once

#include <mola_kernel/interfaces/Dataset_UI.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mola
{
enum class InputMode
{
  ImageDirectory,
  VideoFile,
  LiveCamera
};

class VideoDataset : public RawDataSourceBase, public Dataset_UI
{
  DEFINE_MRPT_OBJECT(VideoDataset, mola)

 public:
  VideoDataset();

  void spinOnce() override;

  // Dataset_UI interface:
  size_t datasetUI_size() const override;
  size_t datasetUI_lastQueriedTimestep() const override;
  double datasetUI_playback_speed() const override;
  void   datasetUI_playback_speed(double speed) override;
  bool   datasetUI_paused() const override;
  void   datasetUI_paused(bool paused) override;
  void   datasetUI_teleport(size_t timestep) override;

 protected:
  void initialize_rds(const Yaml& cfg) override;

 private:
  InputMode                           input_mode_ = InputMode::ImageDirectory;
  mrpt::hwdrivers::CCameraSensor::Ptr cam_;

  double                time_warp_scale_ = 1.0;
  bool                  paused_          = false;
  size_t                image_index_     = 0;
  std::optional<size_t> teleport_here_;
  mutable std::mutex    dataset_ui_mtx_;

  std::vector<std::string> image_files_;
  std::string              image_base_dir_;
  mrpt::Clock::time_point  last_play_wallclock_time_;
  double                   last_dataset_time_  = 0;
  double                   image_publish_rate_ = 10.0;  // Hz
  std::string              sensor_label_       = "camera";
  mrpt::img::TCamera       camera_model_;
  mrpt::poses::CPose3D     camera_pose_;
};
}  // namespace mola

MRPT_ENUM_TYPE_BEGIN(mola::InputMode)
MRPT_FILL_ENUM(mola::InputMode::ImageDirectory);
MRPT_FILL_ENUM(mola::InputMode::VideoFile);
MRPT_FILL_ENUM(mola::InputMode::LiveCamera);
MRPT_ENUM_TYPE_END()
