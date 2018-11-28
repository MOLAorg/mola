/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawDataSourceBase.h
 * @brief  Virtual interface for data sources, either real sensors or datasets
 * @author Jose Luis Blanco Claraco
 * @date   Nov 21, 2018
 */
#pragma once

#include <mola-kernel/RawDataConsumer.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/system/COutputLogger.h>
#include <yaml-cpp/node/node.h>

namespace mola
{
/** Virtual base for raw-observation data sources
 * \ingroup mola_kernel_grp */
class RawDataSourceBase : public mrpt::system::COutputLogger
{
   public:
    RawDataSourceBase(RawDataConsumer& rdc);
    virtual ~RawDataSourceBase() = default;

    /** @name Virtual interface of any RawDataSource
     *{ */

    /** This should be reimplemented to read all the required parameters */
    virtual void initialize(const YAML::Node& cfg);

    /** This will be called in an infinite loop at the sensor execution
     * rate. New observations should be sent to the associated MOLA
     * front-end by calling \a sendObservationsToFrontEnds().
     */
    virtual void spin() = 0;
    /** @} */

   protected:
    /** Send an observation to the associated target front-ends */
    void sendObservationsToFrontEnds(mrpt::obs::CObservation::ConstPtr& obs);

   private:
    /** Target of captured data */
    RawDataConsumer& rdc_;
};

}  // namespace mola
