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
        std::optional<mrpt::obs::CSensoryFrame> observations{};
    };

    struct ProposeKF_Output
    {
        bool                       success{false};
        std::optional<mola::id_t>  new_kf_id{};
        std::optional<std::string> error_msg{};
    };

    /** Propose a new KeyFrame to be inserted into the world model.
     */
    std::future<ProposeKF_Output> onProposeNewKeyFrame(const ProposeKF_Input& i)
    {
        return slam_be_threadpool_.enqueue(
            &BackEndBase::doProposeNewKeyFrame, this, i);
    }

    /** @} */

   protected:
    WorldModel::Ptr worldmodel_;

    WorkerThreadsPool slam_be_threadpool_{1};

    /** @name Virtual methods to be implemented by SLAM back-end
     *{ */

    virtual ProposeKF_Output doProposeNewKeyFrame(const ProposeKF_Input& i) = 0;

    /** @} */
};

}  // namespace mola
