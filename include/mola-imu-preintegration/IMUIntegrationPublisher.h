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

#include <mola-kernel/interfaces/ExecutableBase.h>
#include <mola-kernel/interfaces/RawDataConsumer.h>

namespace mola
{
/** Write me!
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
    void initialize(const std::string& cfg_block) override;
    void onNewObservation(CObservation::Ptr& o) override;
};

}  // namespace mola
