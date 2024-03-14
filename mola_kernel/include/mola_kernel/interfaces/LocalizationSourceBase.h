/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LocalizationSourceBase.h
 * @brief  Virtual interface for SLAM/odometry methods publishing poses
 * @author Jose Luis Blanco Claraco
 * @date   Mar 13, 2024
 */
#pragma once

#include <mrpt/core/Clock.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose3D.h>

#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mola
{
/** Virtual interface for SLAM/odometry methods publishing poses.
 *
 * Publishers must call advertiseUpdatedLocalization(), subscribers
 * must call subscribeToLocalizationUpdates() providing a callback that returns
 * as fast as possible.
 *
 * \ingroup mola_kernel_grp */
class LocalizationSourceBase
{
   public:
    LocalizationSourceBase()          = default;
    virtual ~LocalizationSourceBase() = default;

    struct LocalizationUpdate
    {
        LocalizationUpdate() = default;

        /** The timestamp associated to the new KeyFrame localization. */
        mrpt::Clock::time_point timestamp;

        /** Vehicle/robot pose is given wrt this frame of reference */
        std::string reference_frame = "map";

        /** The source of the localization (e.g. "slam", "lidar_odometry",
         * "wheel_odometry", etc.) */
        std::string method = "slam";

        mrpt::math::TPose3D                        pose;
        std::optional<mrpt::math::CMatrixDouble66> cov;
    };

    using localization_updates_callback_t =
        std::function<void(const LocalizationUpdate&)>;

    void subscribeToLocalizationUpdates(
        const localization_updates_callback_t& callback)
    {
        auto lck = mrpt::lockHelper(locUpdSubsMtx_);
        locUpdSubs_.push_back(callback);
    }

   protected:
    bool anyUpdateLocalizationSubscriber()
    {
        auto lck = mrpt::lockHelper(locUpdSubsMtx_);
        return !locUpdSubs_.empty();
    }

    void advertiseUpdatedLocalization(const LocalizationUpdate& l)
    {
        auto lck = mrpt::lockHelper(locUpdSubsMtx_);
        for (const auto& callback : locUpdSubs_)
        {
            try
            {
                callback(l);
            }
            catch (const std::exception& e)
            {
                std::cerr << "[LocalizationSourceBase] Exception in callback: "
                          << e.what();
            }
        }
    }

   private:
    std::vector<localization_updates_callback_t> locUpdSubs_;
    std::mutex                                   locUpdSubsMtx_;
};

}  // namespace mola
