/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterBase.h
 * @brief  Virtual base class for sensor pipeline filters
 * @author Jose Luis Blanco Claraco
 * @date   Dec 11, 2018
 */

#include <mola-kernel/FilterBase.h>
#include <iostream>

using namespace mola;

// Class factory:
static std::map<std::string, std::function<FilterBase*(void)>> registry;

FilterBase::FilterBase() = default;

FilterBase::Ptr FilterBase::Factory(const std::string& name)
{
    const auto f = registry.find(name);
    if (f == registry.end())
        THROW_EXCEPTION_FMT(
            "[FilterBase::Factory] Request for unregistered class: `%s`",
            name.c_str());
    return Ptr((f->second)());
}

void FilterBase::registerClass(
    const std::string_view& classname, std::function<FilterBase*(void)> func)
{
    registry.emplace(classname, func);
}

// Virtual interface of any RawDataSource
void FilterBase::initialize(const std::string& cfg_block)
{
    MRPT_LOG_WARN_STREAM(
        "`initialize()` not reimplemented by derived class. "
        "Ignoring YAML config block:\n"
        << cfg_block);
}

void FilterBase::spinOnce()
{
    // Nothing to do by default. We work data-driven via the worker thread.
    // See onNewObservation().
}

// Virtual interface of any RawDataConsumer
void FilterBase::onNewObservation(CObservation::Ptr& o)
{
    thread_pool_.enqueue(
        [this](CObservation::Ptr& in) {
            try
            {
                // Process the observation:
                CObservation::Ptr out = this->doFilter(in);
                // Forward it:
                if (out) this->sendObservationsToFrontEnds(out);
            }
            catch (const std::exception& e)
            {
                MRPT_LOG_ERROR_STREAM(
                    "[FilterBase::onNewObservation] Error: "
                    << std::endl
                    << mrpt::exception_to_str(e));
            }
        },
        o);
}
