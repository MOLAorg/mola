/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FrontEndBase.h
 * @brief  Virtual interface for SLAM front-ends
 * @author Jose Luis Blanco Claraco
 * @date   Dec 11, 2018
 */
#pragma once

#include <mola-kernel/interfaces/BackEndBase.h>
#include <mola-kernel/interfaces/ExecutableBase.h>
#include <mola-kernel/interfaces/RawDataConsumer.h>

namespace mola
{
/** Virtual interface for SLAM front-ends.
 *
 * Instructions for implementing new front-ends:
 * Raw observations arrive via calls to the virtual method `onNewObservation()`,
 * which must be implemented. Minimum time should be spent there, just copy the
 * incomming data (smart pointer). Actual processing can be normally done by any
 * of these two ways:
 * - Wait until `spinOnce()` is called, at the rate especified in the yaml file
 * (default=1 Hz), or
 * - Use your own logic to enque a task into a worker thread pool (preferred).
 *
 * \ingroup mola_kernel_grp */
class FrontEndBase : public ExecutableBase, RawDataConsumer
{
    DEFINE_VIRTUAL_MRPT_OBJECT(FrontEndBase)

   public:
    FrontEndBase();
    virtual ~FrontEndBase() = default;

    /** Loads common parameters for all front-ends. Called by launcher just
     * before initialize(). */
    void initialize_common(const std::string& cfg_block);

   protected:
    /** The name of the sensor to subscribe to, from the YAML config file
     * parameter `raw_sensor_label` */
    std::string raw_sensor_label_{"uninitialized"};

    /** A reference to my associated SLAM backend.
     * Populated by initialize_common() */
    BackEndBase::Ptr slam_backend_;
    WorldModel::Ptr  worldmodel_;
};

}  // namespace mola
