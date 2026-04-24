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
 * @file   MapSourceBase.h
 * @brief  Virtual interface for SLAM/odometry methods publishing a map
 * @author Jose Luis Blanco Claraco
 * @date   Mar 13, 2024
 */
#pragma once

#include <mola_kernel/Georeferencing.h>
#include <mrpt/maps/CMetricMap.h>

#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace mola
{
/** Virtual interface for SLAM/odometry methods publishing a map and/or
 * georeferencing information.
 *
 * \ingroup mola_kernel_interfaces_grp */
class MapSourceBase
{
 public:
  MapSourceBase() = default;

  struct MapUpdate
  {
    MapUpdate() = default;

    /** The timestamp associated to the new map information. */
    mrpt::Clock::time_point timestamp;

    /** Frame of reference for this map. */
    std::string reference_frame = "map";

    /** The source of the localization (e.g. "slam", "lidar_odometry",
     * "wheel_odometry", etc.) */
    std::string method = "slam";

    /** Map layer/submap name */
    std::string map_name = "local_map";

    /** Might be null if georeferencing is provided, but both can be also
     * populated. */
    mrpt::maps::CMetricMap::ConstPtr map;

    /** If the map is georeferenced, its metadata
     */
    std::optional<Georeferencing> georeferencing;

    /** Optional metadata associated to the map, e.g. a JSON or YAML string.
     * It will contain the information in the "metadata" field of mp2p_icp::metric_map_t
     * when loading a ``*.mm`` metric map file and publishing it.
     *  \note Added in MOLA 1.9.0
     */
    std::optional<std::string> map_metadata;

    /** If true, the map update invalidates all previous maps from the same
     * source/method in queue. Set to false for very fast maps that need to be aggregated over short
     * periods of time.
     *  \note Added in MOLA 2.1.0
     */
    bool keep_last_one_only = true;
  };

  using map_updates_callback_t = std::function<void(const MapUpdate&)>;

  /** Subscribe to map updates.
   *
   * Behaves analogously to ROS' `transient_local` durability: any prior
   * `advertiseUpdatedMap()` call made with `keep_last_one_only=true` is
   * cached (one entry per `map_name`) and replayed once into the new
   * callback at subscription time. This avoids losing the initial state
   * when the producer advertises before the consumer has had a chance
   * to subscribe (e.g. on startup, when modules come up in arbitrary
   * order).
   */
  /** Subscribe to map updates.
   *
   * Behaves analogously to ROS' `transient_local` durability: any prior
   * `advertiseUpdatedMap()` call made with `keep_last_one_only=true` is
   * cached (one entry per `map_name`) and replayed once into the new
   * callback at subscription time. This avoids losing the initial state
   * when the producer advertises before the consumer has had a chance
   * to subscribe (e.g. on startup, when modules come up in arbitrary
   * order).
   */
  void subscribeToMapUpdates(const map_updates_callback_t& callback)
  {
    std::map<std::string, MapUpdate> cachedUpdates;
    {
      auto lck = mrpt::lockHelper(mapUpdSubsMtx_);
      mapUpdSubs_.push_back(callback);
      cachedUpdates = lastUpdates_;
    }
    for (const auto& [name, mu] : cachedUpdates)
    {
      try
      {
        callback(mu);
      }
      catch (const std::exception& e)
      {
        std::cerr << "[MapSourceBase] Exception in callback: " << e.what();
      }
    }
  }

 protected:
  bool anyUpdateMapSubscriber()
  {
    auto lck = mrpt::lockHelper(mapUpdSubsMtx_);
    return !mapUpdSubs_.empty();
  }

  void advertiseUpdatedMap(const MapUpdate& l)
  {
    auto lck = mrpt::lockHelper(mapUpdSubsMtx_);

    if (l.keep_last_one_only)
    {
      lastUpdates_[l.map_name] = l;
    }

    for (const auto& callback : mapUpdSubs_)
    {
      try
      {
        callback(l);
      }
      catch (const std::exception& e)
      {
        std::cerr << "[MapSourceBase] Exception in callback: " << e.what();
      }
    }
  }

 private:
  std::vector<map_updates_callback_t> mapUpdSubs_;
  /// Cache of last latched update per `map_name`; replayed to late
  /// subscribers. Only populated when `keep_last_one_only=true`.
  std::map<std::string, MapUpdate> lastUpdates_;
  std::mutex                       mapUpdSubsMtx_;
};

}  // namespace mola
