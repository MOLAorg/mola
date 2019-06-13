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

#include <mola-kernel/interfaces/ExecutableBase.h>
#include <mola-kernel/lock_helper.h>
#include <mutex>

using namespace mola;

// Class factory:
struct Registry
{
    static Registry& Instance()
    {
        static Registry r;
        return r;
    }

    std::mutex registry_mtx_;
    std::map<std::string, std::function<ExecutableBase::Ptr(void)>> registry_;

   private:
    Registry() = default;
};

ExecutableBase::Ptr ExecutableBase::Factory(const std::string& name)
{
    Registry& r = Registry::Instance();
    auto      l = lockHelper(r.registry_mtx_);

    const auto f = r.registry_.find(name);
    if (f == r.registry_.end())
        THROW_EXCEPTION_FMT(
            "[ExecutableBase::Factory] Request for unregistered class: `%s`",
            name.c_str());
    return (f->second)();
}

void ExecutableBase::registerClass(
    const std::string_view& classname, std::function<Ptr(void)> func)
{
    Registry& r = Registry::Instance();
    auto      l = lockHelper(r.registry_mtx_);

    r.registry_.emplace(classname, func);
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
