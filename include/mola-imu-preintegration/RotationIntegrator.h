/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RotationIntegrator.h
 * @brief  Integrator of IMU accelerations and angular velocity readings.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2021
 */
#pragma once

#include <mola-imu-preintegration/RotationIntegrationParams.h>
#include <mola-kernel/Yaml.h>
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
     * @param cfg a YAML node with a dictionary of parameters to load from.
     */
    void initialize(const Yaml& cfg);

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

    /**
     * Static auxiliary method: it takes a gyroscope measurement of the angular
     * velocity (ω) and integrates it forward in time for a period dt [s] as:
     *
     *  Rot = Exp((ω-ω_{bias})·dt)
     */
    static mrpt::math::CMatrixDouble33 IncrementalRotation(
        const mrpt::math::TVector3D& w, const RotationIntegrationParams& params,
        double dt,
        mrpt::optional_ref<mrpt::math::CMatrixDouble33>&
            D_incrR_integratedOmega = std::nullopt);

   private:
    IntegrationState state_;
};

}  // namespace mola
