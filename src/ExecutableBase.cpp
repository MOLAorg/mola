/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ExecutableBase.cpp
 * @brief  Virtual interface for objects that can be run into MOLA
 * @author Jose Luis Blanco Claraco
 * @date   Dec 14, 2018
 */

#include <mola-kernel/ExecutableBase.h>

using namespace mola;

// Class factory:
static std::map<std::string, std::function<ExecutableBase::Ptr(void)>> registry;
ExecutableBase::Ptr ExecutableBase::Factory(const std::string& name)
{
    const auto f = registry.find(name);
    if (f == registry.end())
        THROW_EXCEPTION_FMT(
            "[ExecutableBase::Factory] Request for unregistered class: `%s`",
            name.c_str());
    return (f->second)();
}

void ExecutableBase::registerClass(
    const std::string_view& classname, std::function<Ptr(void)> func)
{
    registry.emplace(classname, func);
}

ExecutableBase::ExecutableBase() = default;

ExecutableBase::~ExecutableBase() = default;

/** This should be reimplemented to read all the required parameters */
void ExecutableBase::initialize(const std::string& cfg_block)
{
    if (!cfg_block.empty())
    {
        MRPT_LOG_WARN_STREAM(
            "`initialize()` not reimplemented by derived class. "
            "Ignoring YAML config block:\n"
            << cfg_block);
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
