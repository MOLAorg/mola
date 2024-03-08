/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   BackEndBase.h
 * @brief  Virtual interface for SLAM back-ends
 * @author Jose Luis Blanco Claraco
 * @date   Dec 21, 2018
 */
#pragma once

#include <mola_kernel/WorldModel.h>
#include <mola_kernel/Yaml.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/img/TCamera.h>  // TODO: Remove after unused below
#include <mrpt/math/CMatrixFixed.h>
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
    DEFINE_VIRTUAL_MRPT_OBJECT(BackEndBase)

   public:
    BackEndBase();
    virtual ~BackEndBase() = default;

    /** Loads common parameters for all back-ends. */
    void initialize(const Yaml& cfg) override final;

   protected:
    /** Loads children specific parameters */
    virtual void initialize_backend(const Yaml& cfg) = 0;

   public:
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
        mola::id_t reference_kf{mola::INVALID_ID};

        mrpt::math::TPose3D                        pose;
        std::optional<mrpt::math::CMatrixDouble66> cov{std::nullopt};
    };
    std::future<void> advertiseUpdatedLocalization(
        const AdvertiseUpdatedLocalization_Input& l)
    {
        const auto copyOfL = l;
        return slam_be_threadpool_.enqueue(
            [this, copyOfL]() { doAdvertiseUpdatedLocalization(copyOfL); });
    }

    /** @} */

    virtual void onSmartFactorChanged(
        [[maybe_unused]] mola::fid_t             id,
        [[maybe_unused]] const mola::FactorBase* f)
    {
    }

    /** TODO: Refactor this!! */
    virtual mola::id_t temp_createStereoCamera(
        [[maybe_unused]] const mrpt::img::TCamera& left,
        [[maybe_unused]] const mrpt::img::TCamera& right,
        [[maybe_unused]] const double              baseline)
    {
        THROW_EXCEPTION("Not implemented in selected back-end!");
    }

    virtual mola::id_t temp_createLandmark(
        [[maybe_unused]]  //
        const mrpt::math::TPoint3D& init_value)
    {
        THROW_EXCEPTION("Not implemented in selected back-end!");
    }

    virtual void lock_slam() {}
    virtual void unlock_slam() {}

    /** @name Virtual methods to be implemented by SLAM back-end.
     * `doXXX()` run synchronously (blocking) in the same thread than the
     *caller. `onXXX()` run asynch in another thread.
     *{ */

    virtual ProposeKF_Output doAddKeyFrame(const ProposeKF_Input& i) = 0;
    virtual AddFactor_Output doAddFactor(Factor& f)                  = 0;
    virtual void             doAdvertiseUpdatedLocalization(
                    const AdvertiseUpdatedLocalization_Input& l) = 0;
    /** @} */

   protected:
    WorldModel::Ptr worldmodel_;

    mrpt::WorkerThreadsPool slam_be_threadpool_{
        2, mrpt::WorkerThreadsPool::POLICY_FIFO, "slam_backend"};
};

}  // namespace mola
