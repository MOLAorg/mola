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
 * @file   simplemap_attach_observations.h
 * @brief  Adds observations from a dataset (e.g. camera images from a rosbag)
 *         to the keyframes of an existing simplemap.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mola_kernel/interfaces/OfflineDatasetSource.h>

#include <cstddef>
#include <functional>
#include <map>
#include <set>
#include <string>

namespace mola
{
struct SimplemapAttachOptions
{
  /// Sensor labels of the dataset observations to attach.
  std::set<std::string> sensor_labels;

  /// Maximum time offset [s] between an observation and its keyframe.
  double max_age = 0.5;

  /** Images (CObservationImage) are written as "jpg" or "png" files to the
   *  `<output>_Images/` directory, or kept "inline" in the simplemap. */
  std::string image_format = "jpg";
  int         jpeg_quality = 95;  ///< [1-100]
};

struct SimplemapAttachResult
{
  std::size_t keyframes         = 0;
  std::size_t observations_read = 0;
  /// Observations attached, per sensor label.
  std::map<std::string, std::size_t> attached;
};

/** Adds observations of `dataset` to the keyframes of the simplemap in
 *  `inputFile`, and saves it to `outputFile` (which may be the same file).
 *
 *  Each observation goes to the keyframe closest in time, within `max_age`,
 *  and each keyframe keeps the closest one per sensor label, so no
 *  observation is stored twice. Observations already in the map (same label
 *  and time) are not added again. Frames without sensor data (only the
 *  "metadata" comment of non-keyframes) get nothing. The keyframe time is that
 *  of its "metadata" comment, or else of its first observation.
 *
 *  External files of the input map (`<input>_Images/`) are hard-linked (or
 *  copied) into `<output>_Images/` if that is another directory, so the
 *  output map is self-contained.
 *
 *  \param progress Optional, called with the fraction of the dataset read.
 *  \exception std::exception On errors reading or writing files.
 */
SimplemapAttachResult simplemap_attach_observations(
    const std::string& inputFile, const std::string& outputFile,
    const OfflineDatasetSource& dataset, const SimplemapAttachOptions& options,
    const std::function<void(double)>& progress = {});

}  // namespace mola
