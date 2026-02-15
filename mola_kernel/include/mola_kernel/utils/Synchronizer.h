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
 * @file   Synchronizer.h
 * @brief  A sliding window buffer to collect synchronized observations
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2025
 */
#pragma once

#include <mrpt/obs/obs_frwds.h>

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>

namespace mola
{
/** A sliding window buffer to collect synchronized observations.
 * Observations are of types derived from mrpt::obs::CObservation, and must have correctly defined
 * `sensorLabel` and `timestamp`. Different sensors must have different `sensorLabel`.
 *
 * Observations are added to the synchronizer with `add()`, which inserts the observation
 * into the sliding window buffer. Entries older than `parameters.window_length` are deleted.
 *
 * Whenever the user wants to check if a group of synchronized observations is ready to be
 * retrieved, the method `getObservationGroup()` must be called. This method takes the oldest
 * synchronized group, returns it, and removes it from the buffer.
 *
 * The user can select two policies to determine when a group of observations is complete:
 * - Minimum number of observations. This is the default behavior.
 *   See `parameters.minimum_observation_count`.
 * - A specific list of sensor labels, that is, named observations. To enable this behavior,
 *   fill in `parameters.expected_observation_labels`.
 *
 * \ingroup mola_kernel_grp
 */
class Synchronizer
{
 public:
  Synchronizer() = default;

  struct Parameters
  {
    Parameters() = default;

    double window_length             = 2.0;  //!< Oldest age of observations to keep [seconds]
    double synchronization_tolerance = 0.05;  //!< Max time between obs to be considered a group [s]
    std::size_t           minimum_observation_count = 2;  //!< read Synchronizer
    std::set<std::string> expected_observation_labels;  //!< read Synchronizer
  };

  Parameters parameters;

  using NamedObservationSet = std::map<std::string, std::shared_ptr<mrpt::obs::CObservation>>;

  /** Inserts the observation into the sliding window buffer. Read Synchronizer  */
  void add(const std::shared_ptr<mrpt::obs::CObservation>& obs);

  /** Retrieves (and removes from the buffer) the oldest set of synchronized observations. If none
   * is ready, std::nullopt is returned.  */
  std::optional<NamedObservationSet> getObservationGroup();

  /** Reset the buffer to an empty state */
  void clear();

 private:
  std::map<double, NamedObservationSet> buffer_;
};

}  // namespace mola
