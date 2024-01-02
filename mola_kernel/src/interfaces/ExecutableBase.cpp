/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ExecutableBase.cpp
 * @brief  Virtual interface for objects that can be run into MOLA
 * @author Jose Luis Blanco Claraco
 * @date   Dec 14, 2018
 */

#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mrpt/core/lock_helper.h>

#include <mutex>

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_VIRTUAL_MRPT_OBJECT(ExecutableBase, mrpt::rtti::CObject, mola)

using namespace mola;

ExecutableBase::Ptr ExecutableBase::Factory(const std::string& name)
{
    auto o = mrpt::rtti::classFactory(name);

    if (!o)
        THROW_EXCEPTION_FMT(
            "[ExecutableBase::Factory] Request for unregistered class: `%s`",
            name.c_str());
    return mrpt::ptr_cast<ExecutableBase>::from(o);
}

ExecutableBase::ExecutableBase() = default;

ExecutableBase::~ExecutableBase()
{
    // Ensure profiler stats are saved now, if enabled, before
    // members dtor's are called.
    profiler_dtor_save_stats_.reset();
    MRPT_LOG_DEBUG_STREAM(
        "ExecutableBase dtor called for module: `" << module_instance_name
                                                   << "`");
}

/** This should be reimplemented to read all the required parameters */
void ExecutableBase::initialize(const Yaml& cfg)
{
    if (!cfg.empty())
    {
        MRPT_LOG_WARN_STREAM(
            "`initialize()` not reimplemented by derived class. "
            "Ignoring YAML config block:\n"
            << cfg);
    }
}

void ExecutableBase::setModuleInstanceName(const std::string& s)
{
    module_instance_name = s;
}
std::string ExecutableBase::getModuleInstanceName() const
{
    return module_instance_name;
}
