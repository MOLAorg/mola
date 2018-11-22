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

#include <mola-kernel/RawDataSourceBase.h>
#include <iostream>

using namespace mola;

RawDataSourceBase::RawDataSourceBase(RawDataConsumer& rdc)
    : mrpt::system::COutputLogger("RawDataSourceBase"), rdc_(rdc)
{
}

/** This should be reimplemented to read all the required parameters */
void RawDataSourceBase::initialize(const YAML::Node& cfg)
{
    MRPT_LOG_WARN_STREAM(
        "`initialize()` not reimplemented by derived class. "
        "Ignoring YAML config block: "
        << cfg);
}

void RawDataSourceBase::sendObservationsToFrontEnds(
    mrpt::obs::CObservation::ConstPtr& obs)
{
    // Just forward the data to my associated consumer:
    rdc_.onNewObservation(obs);
}
