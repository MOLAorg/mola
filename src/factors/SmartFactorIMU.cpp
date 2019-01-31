/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   SmartFactorIMU.cpp
 * @brief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 28, 2019
 */

#include <mola-kernel/BackEndBase.h>
#include <mola-kernel/factors/SmartFactorIMU.h>

// TODO: make serializable

using namespace mola;

SmartFactorIMU::SmartFactorIMU(BackEndBase* slam_backend)
    : slam_backend_(slam_backend)
{
}

std::size_t SmartFactorIMU::edge_count() const { return 0; }
mola::id_t  SmartFactorIMU::edge_indices(const std::size_t i) const
{
    throw std::runtime_error("invalid method for this factor");
}

void SmartFactorIMU::integrateMeasurement(
    double accx, double accy, double accz, double wx, double wy, double wz,
    double dt)
{
    ax_        = accx;
    ay_        = accy;
    az_        = accz;
    wx_        = wx;
    wy_        = wy;
    wz_        = wz;
    dt_        = dt;
    new_state_ = NewState::MEASURE;
    slam_backend_->onSmartFactorChanged(my_id_, this);
    new_state_ = NewState::NONE;
}

void SmartFactorIMU::createIMUFactor(
    mola::id_t prev_pose_kf, mola::id_t new_pose_kf)
{
    prev_pose_kf_ = prev_pose_kf;
    new_pose_kf_  = new_pose_kf;
    new_state_    = NewState::FACTOR;
    slam_backend_->onSmartFactorChanged(my_id_, this);
    new_state_ = NewState::NONE;
}
