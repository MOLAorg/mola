/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FrontEndBase.h
 * @brief  Virtual interface for SLAM front-ends
 * @author Jose Luis Blanco Claraco
 * @date   Dec 11, 2018
 */

#include <mola-kernel/FrontEndBase.h>
//#include <mola-kernel/WorkerThreadsPool.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

using namespace mola;

// Class factory:
static std::map<std::string, std::function<FrontEndBase*(void)>> registry;

FrontEndBase::FrontEndBase() : mrpt::system::COutputLogger("FrontEndBase") {}

FrontEndBase::Ptr FrontEndBase::Factory(const std::string& name)
{
    const auto f = registry.find(name);
    if (f == registry.end())
        THROW_EXCEPTION_FMT(
            "[FrontEndBase::Factory] Request for unregistered class: `%s`",
            name.c_str());
    return Ptr((f->second)());
}

void FrontEndBase::registerClass(
    const std::string_view& classname, std::function<FrontEndBase*(void)> func)
{
    registry.emplace(classname, func);
}

void FrontEndBase::initialize(const std::string& cfg_block)
{
    MRPT_LOG_WARN_STREAM(
        "`initialize()` not reimplemented by derived class. "
        "Ignoring YAML config block:\n"
        << cfg_block);
}

void FrontEndBase::initialize_common(const std::string& cfg_block)
{
    MRPT_TRY_START
    auto cfg = YAML::Load(cfg_block);

    MRPT_TRY_END
}
