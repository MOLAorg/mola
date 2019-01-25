/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   BackEndBase.h
 * @brief  Virtual interface for SLAM back-ends
 * @author Jose Luis Blanco Claraco
 * @date   Dec 21, 2018
 */
#pragma once

#include <mola-kernel/ExecutableBase.h>
#include <mola-kernel/WorkerThreadsPool.h>
#include <mola-kernel/WorldModel.h>
#include <mrpt/core/Clock.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <future>
#include <optional>

namespace mola
{
/** Virtual interface for SLAM back-ends.
 * All calls to `onXXX()` methods are enqueued and executed in a separate
 * thread.
 *
 * \ingroup mola_kernel_grp */
class BackEndBase : public ExecutableBase
{
   public:
    BackEndBase();
    virtual ~BackEndBase() = default;

    using Ptr = std::shared_ptr<BackEndBase>;

    /** Loads common parameters for all back-ends. Called by launcher just
     * before initialize(). */
    void initialize_common(const std::string& cfg_block);

    /** @name User interface for a SLAM back-end
     *{ */

    struct ProposeKF_Input
    {
        /** The timestamp associated to the new Key-Frame. Must be valid. */
        mrpt::Clock::time_point timestamp{};

        /** Optional set of raw observations seen from this KF. */
        std::optional<mrpt::obs::CSensoryFrame> observations{std::nullopt};
    };

    struct ProposeKF_Output
    {
        bool                       success{false};
        std::optional<mola::id_t>  new_kf_id{std::nullopt};
        std::optional<std::string> error_msg{std::nullopt};
    };

    /** Creates a new KeyFrame in the world model. */
    std::future<ProposeKF_Output> addKeyFrame(const ProposeKF_Input& i)
    {
        return slam_be_threadpool_.enqueue(
            &BackEndBase::doAddKeyFrame, this, i);
    }

    struct AddFactor_Output
    {
        bool                       success{false};
        std::optional<mola::fid_t> new_factor_id{std::nullopt};
        std::optional<std::string> error_msg{std::nullopt};
    };

    /** Adds a new constraint factor to the world model.
     * Note that the object is **moved**, so it will not hold a valid value upon
     * return.
     */
    std::future<AddFactor_Output> addFactor(Factor& f)
    {
        return slam_be_threadpool_.enqueue(&BackEndBase::doAddFactor, this, f);
    }

    struct AdvertiseUpdatedLocalization_Input
    {
        /** The timestamp associated to the new Key-Frame. Must be valid. */
        mrpt::Clock::time_point timestamp{};

        /** Coordinates are given wrt this frame of reference */
        mola::id_t reference_kf;

        mrpt::math::TPose3D                        pose;
        std::optional<Eigen::Matrix<double, 6, 6>> cov{std::nullopt};
    };
    std::future<void> advertiseUpdatedLocalization(
        const AdvertiseUpdatedLocalization_Input& l)
    {
        return slam_be_threadpool_.enqueue(
            &BackEndBase::doAdvertiseUpdatedLocalization, this, l);
    }

    /** @} */

   protected:
    WorldModel::Ptr worldmodel_;

    WorkerThreadsPool slam_be_threadpool_{2};

    /** @name Virtual methods to be implemented by SLAM back-end
     *{ */

    virtual ProposeKF_Output doAddKeyFrame(const ProposeKF_Input& i) = 0;
    virtual AddFactor_Output doAddFactor(Factor& f)                  = 0;
    virtual void             doAdvertiseUpdatedLocalization(
                    AdvertiseUpdatedLocalization_Input l) = 0;
    /** @} */
};

}  // namespace mola
