/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   SmartFactorStereoProjectionPose.cpp
 * @brief  Base class for all "factors" in the world model
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */

#include <mola_kernel/factors/FactorBase.h>
#include <mola_kernel/factors/SmartFactorStereoProjectionPose.h>
#include <mola_kernel/interfaces/BackEndBase.h>
#include <mrpt/serialization/CArchive.h>

using namespace mola;

// arguments: classname, parent class, namespace
IMPLEMENTS_SERIALIZABLE(SmartFactorStereoProjectionPose, FactorBase, mola);

SmartFactorStereoProjectionPose::SmartFactorStereoProjectionPose(
    double sigma_xleft, double sigma_xright, double sigma_y,
    BackEndBase* slam_backend, const mrpt::math::TPose3D& cameraPoseOnRobot)
    : sigma_xleft_(sigma_xleft),
      sigma_xright_(sigma_xright),
      sigma_y_(sigma_y),
      cameraPoseOnRobot_(cameraPoseOnRobot),
      slam_backend_(slam_backend)
{
}

std::size_t SmartFactorStereoProjectionPose::edge_count() const
{
    MRPT_TODO("Properly handle camera_id: count how many camera params, etc.");
    return all_observations_.size();
}
mola::id_t SmartFactorStereoProjectionPose::edge_indices(
    const std::size_t i) const
{
    return all_observations_.at(i).observing_kf;
}

void SmartFactorStereoProjectionPose::addObservation(
    const SmartFactorStereoProjectionPose::StereoObservation& st,
    const id_t observing_kf, const id_t camera_params)
{
    ASSERT_(slam_backend_);

    all_observations_.emplace_back(st, observing_kf, camera_params);

    slam_backend_->onSmartFactorChanged(my_id_, this);
}

// Implementation of the CSerializable virtual interface:
uint8_t SmartFactorStereoProjectionPose::serializeGetVersion() const
{
    return 0;
}
void SmartFactorStereoProjectionPose::serializeTo(
    mrpt::serialization::CArchive& out) const
{
    baseSerializeTo(out);
    THROW_EXCEPTION("TO DO");
}
void SmartFactorStereoProjectionPose::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    baseSerializeFrom(in);

    switch (version)
    {
        case 0:
        {
            THROW_EXCEPTION("TO DO");
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}
