/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
/**
 * @file   NavStateFuse.cpp
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#include <mola_navstate_fuse/NavStateFuse.h>
#include <mrpt/poses/Lie/SO.h>

using namespace mola;

void NavStateFuse::initialize(const mrpt::containers::yaml& cfg)
{
    reset();

    // Load params:
    params_.loadFrom(cfg);
}

void NavStateFuse::reset()
{
    // reset:
    state_ = State();
}

void NavStateFuse::fuse_odometry(const mrpt::obs::CObservationOdometry& odom)
{
    // TODO(jlbc): proper time-based data fusion.

    // temporarily, this will work well only for simple datasets:
    if (state_.last_odom_obs && state_.last_pose)
    {
        const auto poseIncr = odom.odometry - state_.last_odom_obs->odometry;

        state_.last_pose->mean =
            state_.last_pose->mean + mrpt::poses::CPose3D(poseIncr);

        // and discard velocity-based model:
        state_.last_twist = mrpt::math::TTwist3D(0, 0, 0, 0, 0, 0);
    }
    // copy:
    state_.last_odom_obs = odom;
}

void NavStateFuse::fuse_imu(const mrpt::obs::CObservationIMU& imu)
{
    // TODO(jlbc)
    (void)imu;
}

void NavStateFuse::fuse_pose(
    const mrpt::Clock::time_point&         timestamp,
    const mrpt::poses::CPose3DPDFGaussian& pose)
{
    mrpt::poses::CPose3D incrPose;

    // numerical sanity:
    for (int i = 0; i < 6; i++) ASSERT_GT_(pose.cov(i, i), .0);

    double dt = 0;
    if (state_.last_pose_obs_tim)
        dt = mrpt::system::timeDifference(*state_.last_pose_obs_tim, timestamp);

    if (dt < params_.max_time_to_use_velocity_model && state_.last_pose)
    {
        ASSERT_GT_(dt, .0);

        auto& tw = state_.last_twist.emplace();

        incrPose = pose.mean - (state_.last_pose)->mean;

        tw.vx = incrPose.x() / dt;
        tw.vy = incrPose.y() / dt;
        tw.vz = incrPose.z() / dt;

        const auto logRot =
            mrpt::poses::Lie::SO<3>::log(incrPose.getRotationMatrix());

        tw.wx = logRot[0] / dt;
        tw.wy = logRot[1] / dt;
        tw.wz = logRot[2] / dt;
    }
    else
    {
        state_.last_twist.reset();
    }

    // save for next iter:
    state_.last_pose         = pose;
    state_.last_pose_obs_tim = timestamp;
}

void NavStateFuse::fuse_twist(
    const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist)
{
    (void)timestamp;

    state_.last_twist = twist;
}

void NavStateFuse::force_last_twist(const mrpt::math::TTwist3D& twist)
{
    state_.last_twist = twist;
}

std::optional<NavState> NavStateFuse::estimated_navstate(
    const mrpt::Clock::time_point& timestamp) const
{
    if (!state_.last_pose_obs_tim) return {};  // None

    const double dt =
        mrpt::system::timeDifference(*state_.last_pose_obs_tim, timestamp);

    if (!state_.last_twist || !state_.last_pose ||
        std::abs(dt) > params_.max_time_to_use_velocity_model)
        return {};  // None

    NavState ret;

    const auto& tw = state_.last_twist.value();

    // For the velocity model, we don't have any known "bias":
    const mola::RotationIntegrationParams rotParams = {};

    const auto rot33 =
        mola::incremental_rotation({tw.wx, tw.wy, tw.wz}, rotParams, dt);

    // pose mean:
    ret.pose.mean =
        (state_.last_pose->mean +
         mrpt::poses::CPose3D::FromRotationAndTranslation(
             rot33, mrpt::math::TVector3D(tw.vx, tw.vy, tw.vz) * dt));

    // pose cov:
    auto cov = state_.last_pose->cov;

    double varXYZ =
        mrpt::square(dt * params_.sigma_random_walk_acceleration_linear);
    double varRot =
        mrpt::square(dt * params_.sigma_random_walk_acceleration_angular);

    for (int i = 0; i < 3; i++) cov(i, i) += varXYZ;
    for (int i = 3; i < 6; i++) cov(i, i) += varRot;

    ret.pose.cov_inv = cov.inverse_LLt();

    // twist:
    ret.twist = state_.last_twist.value();

    // TODO(jlbc): twist covariance

    return ret;
}
