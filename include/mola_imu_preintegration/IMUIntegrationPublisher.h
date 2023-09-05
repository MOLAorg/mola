/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   IMUIntegrationPublisher.h
 * @brief  A mola node that takes IMU observations, integrate them since the
 * last KF, and publishes the preintegrated rotation.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 18, 2021
 */
#pragma once

#include <mola_imu_preintegration/IMUIntegrator.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/RawDataConsumer.h>

namespace mola
{
/** A MOLA node running an IMUIntegrator inside.
 *  Must be configured to subscribe to an IMU source.
 *
 * See IMUIntegrationParams for a list of related papers explaining the methods
 * and parameters.
 *
 *
 * \ingroup mola_imu_preintegration_grp
 */
class IMUIntegrationPublisher : public RawDataConsumer, public ExecutableBase
{
    DEFINE_MRPT_OBJECT(IMUIntegrationPublisher, mola)

   public:
    IMUIntegrationPublisher()           = default;
    ~IMUIntegrationPublisher() override = default;

    // See docs in base class
    void initialize_common([[maybe_unused]] const Yaml& cfg) override {}
    void initialize(const Yaml& cfg) override;
    void spinOnce() override;

    /** @name Virtual interface of any RawDataConsumer
     *{ */

    void onNewObservation(CObservation::Ptr& o) override;

    /** @} */

   private:
    IMUIntegrator integrator_;
};

}  // namespace mola
