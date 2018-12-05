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
#include <yaml-cpp/node/node.h>
#include <iostream>

using namespace mola;

// Class factory:
static std::map<std::string, std::function<RawDataSourceBase*(void)>> registry;

RawDataSourceBase::RawDataSourceBase()
    : mrpt::system::COutputLogger("RawDataSourceBase")
{
}

RawDataSourceBase::Ptr RawDataSourceBase::Factory(const std::string& name)
{
    const auto f = registry.find(name);
    if (f == registry.end())
        THROW_EXCEPTION_FMT(
            "[RawDataSourceBase::Factory] Request for unregistered class: `%s`",
            name.c_str());
    return Ptr((f->second)());
}

void RawDataSourceBase::registerClass(
    const std::string_view&                 classname,
    std::function<RawDataSourceBase*(void)> func)
{
    registry.emplace(classname, func);
}

/** This should be reimplemented to read all the required parameters */
void RawDataSourceBase::initialize(const std::string& cfg_block)
{
    MRPT_LOG_WARN_STREAM(
        "`initialize()` not reimplemented by derived class. "
        "Ignoring YAML config block:\n"
        << cfg_block);
}

void RawDataSourceBase::sendObservationsToFrontEnds(
    mrpt::obs::CObservation::Ptr& obs)
{
    // Just forward the data to my associated consumer:
    if (rdc_) { rdc_->onNewObservation(obs); }
    else
    {
        MRPT_LOG_WARN(
            "[sendObservationsToFrontEnds] Dropping observation: no consumer "
            "is attached.");
    }
}

void RawDataSourceBase::attachToDataConsumer(RawDataConsumer& rdc)
{
    rdc_ = rdc.getAsPtr();
}
