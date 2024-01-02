/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FactorStereoProjectionPose.h
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
struct StereoObservation
{
    double x_left{0}, x_right{0}, y{0};
};

/** Smart factor for a set of stereo-camera observations of one point landmark.
 * The ID of the factor is the ID of the landmark.
 * \ingroup mola_kernel_grp
 */
class FactorStereoProjectionPose : public FactorBase
{
    DEFINE_SERIALIZABLE(FactorStereoProjectionPose, mola)

   public:
    FactorStereoProjectionPose() = default;

    FactorStereoProjectionPose(
        double sigma_xleft, double sigma_xright, double sigma_y,
        const StereoObservation& st, const id_t observing_kf,
        const id_t observed_landmark, const id_t camera_params,
        const mrpt::math::TPose3D& cameraPoseOnRobot = mrpt::math::TPose3D())
        : sigma_xleft_(sigma_xleft),
          sigma_xright_(sigma_xright),
          sigma_y_(sigma_y),
          observation_(st),
          observing_kf_(observing_kf),
          observed_landmark_(observed_landmark),
          camera_params_id_(camera_params),
          cameraPoseOnRobot_(cameraPoseOnRobot)
    {
    }

    /** Noise model */
    double sigma_xleft_, sigma_xright_, sigma_y_;

    StereoObservation   observation_;
    mola::id_t          observing_kf_;
    mola::id_t          observed_landmark_;
    mola::id_t          camera_params_id_;
    mrpt::math::TPose3D cameraPoseOnRobot_;

    std::size_t edge_count() const override;
    mola::id_t  edge_indices(const std::size_t i) const override;
};

}  // namespace mola
