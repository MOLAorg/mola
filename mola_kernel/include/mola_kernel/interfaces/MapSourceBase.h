/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MapSourceBase.h
 * @brief  Virtual interface for SLAM/odometry methods publishing a map
 * @author Jose Luis Blanco Claraco
 * @date   Mar 13, 2024
 */
#pragma once

#include <mrpt/maps/CMetricMap.h>

#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

namespace mola
{
/** Virtual interface for SLAM/odometry methods publishing a map
 *
 * \ingroup mola_kernel_grp */
class MapSourceBase
{
   public:
    MapSourceBase()          = default;
    virtual ~MapSourceBase() = default;

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

        mrpt::maps::CMetricMap::Ptr map;
    };

    using map_updates_callback_t = std::function<void(const MapUpdate&)>;

    void subscribeToMapUpdates(const map_updates_callback_t& callback)
    {
        auto lck = mrpt::lockHelper(mapUpdSubsMtx_);
        mapUpdSubs_.push_back(callback);
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
        for (const auto& callback : mapUpdSubs_)
        {
            try
            {
                callback(l);
            }
            catch (const std::exception& e)
            {
                std::cerr << "[MapSourceBase] Exception in callback: "
                          << e.what();
            }
        }
    }

   private:
    std::vector<map_updates_callback_t> mapUpdSubs_;
    std::mutex                          mapUpdSubsMtx_;
};

}  // namespace mola
