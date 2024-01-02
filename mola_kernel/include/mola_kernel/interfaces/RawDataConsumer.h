/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawDataConsumer.h
 * @brief  Virtual interface for raw data consumers, typically, SLAM front-ends
 * @author Jose Luis Blanco Claraco
 * @date   Nov 21, 2018
 */
#pragma once

#include <mrpt/obs/CObservation.h>

#include <memory>

namespace mola
{
using CObservation = mrpt::obs::CObservation;

/** Virtual base for raw-observation consumers
 * \ingroup mola_kernel_grp */
class RawDataConsumer
{
   public:
    RawDataConsumer()          = default;
    virtual ~RawDataConsumer() = default;

    /** @name Virtual interface of any RawDataConsumer
     *{ */

    /** To be called whenever a new observation arrives. It should return as
     * fast as possible, enqueuing the data for processing in another thread.
     */
    virtual void onNewObservation(const CObservation::Ptr& o) = 0;
    /** @} */
};

}  // namespace mola
