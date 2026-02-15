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
 * @file   Synchronizer.cpp
 * @brief  A sliding window buffer to collect synchronized observations
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2025
 */

#include <mola_kernel/utils/Synchronizer.h>
#include <mrpt/obs/CObservation.h>

namespace mola
{

void Synchronizer::add(const std::shared_ptr<mrpt::obs::CObservation>& obs)
{
  if (!obs)
  {
    return;
  }
  buffer_[mrpt::Clock::toDouble(obs->timestamp)][obs->sensorLabel] = obs;
}

std::optional<Synchronizer::NamedObservationSet> Synchronizer::getObservationGroup()
{
  if (buffer_.empty())
  {
    return {};
  }

  for (const auto& [stamp, obs_set] : buffer_)
  {
    NamedObservationSet found_entries;
    std::set<double>    found_stamps;

    for (const auto& [this_stamp, this_obs_set] : buffer_)
    {
      if (std::abs(stamp - this_stamp) > parameters.synchronization_tolerance)
      {
        continue;
      }
      for (const auto& [name, obs] : this_obs_set)
      {
        found_entries[name] = obs;
        found_stamps.insert(this_stamp);
      }
    }

    // Is this set complete?
    if (parameters.expected_observation_labels.empty())
    {
      // Criteria: count
      if (found_entries.size() < parameters.minimum_observation_count)
      {
        continue;  // not full yet
      }
    }
    else
    {
      // Criteria: labels
      const bool have_them_all = std::all_of(
          parameters.expected_observation_labels.begin(),
          parameters.expected_observation_labels.end(),
          [&](const auto& expected_label) { return found_entries.count(expected_label) == 1; });
      if (!have_them_all)
      {
        continue;  // not full yet
      }
    }
    // Yes, we have a full set. Remove them from the set and return it:
    for (const auto s : found_stamps)
    {
      buffer_.erase(s);
    }
    return found_entries;
  }

  // None found:
  return {};
}

void Synchronizer::clear()
{
  // Reset the buffer
  buffer_.clear();
}

}  // namespace mola
