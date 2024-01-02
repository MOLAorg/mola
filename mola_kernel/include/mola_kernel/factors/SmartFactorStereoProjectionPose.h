/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   SmartFactorStereoProjectionPose.h
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 28, 2019
 */
#pragma once

#include <mola_kernel/factors/FactorBase.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/math/TPose3D.h>

#include <deque>

namespace mola
{
class BackEndBase;

/** Smart factor for a set of stereo-camera observations of one point landmark.
 * The ID of the factor is the ID of the landmark.
 * \ingroup mola_kernel_grp
 */
class SmartFactorStereoProjectionPose : public FactorBase
{
    DEFINE_SERIALIZABLE(SmartFactorStereoProjectionPose, mola)

   public:
    struct StereoObservation
    {
        double x_left{0}, x_right{0}, y{0};
    };

    SmartFactorStereoProjectionPose() = default;

    SmartFactorStereoProjectionPose(
        double sigma_xleft, double sigma_xright, double sigma_y,
        BackEndBase*               slam_backend,
        const mrpt::math::TPose3D& cameraPoseOnRobot = mrpt::math::TPose3D());

    /** Noise model */
    double sigma_xleft_, sigma_xright_, sigma_y_;

    std::size_t         edge_count() const override;
    mola::id_t          edge_indices(const std::size_t i) const override;
    mrpt::math::TPose3D cameraPoseOnRobot_;

    void addObservation(
        const StereoObservation& st, const id_t observing_kf,
        const id_t camera_params);

    struct obs_tuple_t
    {
        obs_tuple_t(const StereoObservation& pt, id_t obs_kf, id_t cam_params)
            : pixel_coords(pt), observing_kf(obs_kf), camera_params(cam_params)
        {
        }

        StereoObservation pixel_coords;
        id_t              observing_kf;
        id_t              camera_params;
    };

    using obs_tuple_list_t = std::deque<obs_tuple_t>;

    const obs_tuple_list_t& allObservations() const
    {
        return all_observations_;
    }

   private:
    BackEndBase*     slam_backend_{nullptr};
    obs_tuple_list_t all_observations_;
};

}  // namespace mola
