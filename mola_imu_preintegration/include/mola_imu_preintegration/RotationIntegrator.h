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
 * @file   RotationIntegrator.h
 * @brief  Integrator of IMU accelerations and angular velocity readings.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2021
 */
#pragma once

#include <mola_imu_preintegration/RotationIntegrationParams.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/optional_ref.h>

namespace mola
{
/** Integrates gyroscope angular velocity readings.
 *
 *  Bias is assumed to be constant, although the user is free of updating it at
 * any moment by accessing params_.
 *
 * See IMUIntegrationParams for a list of related papers explaining the methods
 * and parameters.
 *
 * Usage:
 * - (1) Call initialize() or set the required parameters directly in params_.
 * - (2) Integrate measurements with integrate_measurement()
 * - (3) Repeat (2) N times as needed.
 * - (4) Take the estimation up to this point with current_integration_state()
 *       and reset with reset_integration() if you create a new key-frame.
 * - (5) Go to (2).
 *
 * \note Initially based in part on GTSAM sources gtsam::PreintegratedRotation.
 *
 * \sa IMUIntegrator
 * \ingroup mola_imu_preintegration_grp
 */
class RotationIntegrator
{
   public:
    RotationIntegrator()  = default;
    ~RotationIntegrator() = default;

    struct IntegrationState
    {
        IntegrationState()  = default;
        ~IntegrationState() = default;

        /// Time interval from i to j
        double deltaTij_ = 0;

        /// Preintegrated relative orientation (in frame i)
        mrpt::math::CMatrixDouble33 deltaRij_ =
            mrpt::math::CMatrixDouble33::Identity();

        /// Jacobian of preintegrated rotation w.r.t. angular rate bias
        // (TODO)
        // mrpt::math::CMatrixDouble33 delRdelBiasOmega_ =
        // mrpt::math::CMatrixDouble33::Zero();
    };

    /** \name Main API
     *  @{ */

    /**
     * @brief Initializes the object and reads all parameters from a YAML node.
     * @param cfg a YAML node with a dictionary of parameters to load from, as
     * expected by RotationIntegrationParams (see its docs).
     */
    void initialize(const mrpt::containers::yaml& cfg);

    /** Resets the integrator state to an initial state.
     *  \sa currentIntegrationState
     */
    void reset_integration();

    const IntegrationState& current_integration_state() const { return state_; }

    /** Accumulates a new gyroscope measurement of the angular velocity (ω) into
     * the current preintegration state, integrating it forward in
     * time for a period dt [s].
     *
     * \sa current_integration_state(), reset_integration()
     */
    void integrate_measurement(const mrpt::math::TVector3D& w, double dt);

    RotationIntegrationParams params_;

    /** @} */

   private:
    IntegrationState state_;
};

/**
 * This takes a gyroscope measurement of the angular
 * velocity (ω) and integrates it forward in time for a period dt [s] as:
 *
 *  Rot = Exp((ω-ω_{bias})·dt)
 *
 * \ingroup mola_imu_preintegration_grp
 */
mrpt::math::CMatrixDouble33 incremental_rotation(
    const mrpt::math::TVector3D& w, const RotationIntegrationParams& params,
    double dt,
    const mrpt::optional_ref<mrpt::math::CMatrixDouble33>&
        D_incrR_integratedOmega = std::nullopt);

}  // namespace mola
