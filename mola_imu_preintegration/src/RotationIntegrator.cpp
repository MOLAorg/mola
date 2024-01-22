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
 * @file   RotationIntegrator.cpp
 * @brief  Integrator of IMU accelerations and angular velocity readings.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2021
 */

#include <mola_imu_preintegration/RotationIntegrator.h>
#include <mrpt/poses/Lie/SO.h>

using namespace mola;

void RotationIntegrator::initialize(const mrpt::containers::yaml& cfg)
{
    reset_integration();

    // Load params:
    params_.load_from(cfg);
}

void RotationIntegrator::reset_integration()
{
    // reset:
    state_ = IntegrationState();
}

void RotationIntegrator::integrate_measurement(
    const mrpt::math::TVector3D& w, double dt)
{
    const auto incrR = mola::incremental_rotation(w, params_, dt);

    // Update integration state:
    state_.deltaTij_ += dt;
    state_.deltaRij_ = state_.deltaRij_ * incrR;

    // TODO: Update Jacobian
}

mrpt::math::CMatrixDouble33 mola::incremental_rotation(
    const mrpt::math::TVector3D& w, const RotationIntegrationParams& params,
    double dt,
    const mrpt::optional_ref<mrpt::math::CMatrixDouble33>&
        D_incrR_integratedOmega)
{
    using mrpt::math::TVector3D;

    // Bias:
    TVector3D correctedW = w - params.gyroBias;

    // Translate to vehicle frame:
    if (params.sensorPose.has_value())
        correctedW = params.sensorPose->rotateVector(correctedW);

    // Integrate:
    const TVector3D w_dt = correctedW * dt;

    if (D_incrR_integratedOmega.has_value())
    {
        // TODO: Jacobian: mrpt::poses::Lie::SO<3>::jacob_dexpe_de()
        THROW_EXCEPTION("Jacobian not implemented yet");
    }

    return mrpt::poses::Lie::SO<3>::exp(
        mrpt::math::CVectorFixedDouble<3>(w_dt));
}
