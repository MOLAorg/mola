/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   ExecutableBase.cpp
 * @brief  Virtual interface for objects that can be run into MOLA
 * @author Jose Luis Blanco Claraco
 * @date   Dec 14, 2018
 */

#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mrpt/core/lock_helper.h>

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_VIRTUAL_MRPT_OBJECT(ExecutableBase, mrpt::rtti::CObject, mola)

using namespace mola;

ExecutableBase::Ptr ExecutableBase::Factory(const std::string& name)
{
  auto o = mrpt::rtti::classFactory(name);

  if (!o)
  {
    THROW_EXCEPTION_FMT(
        "[ExecutableBase::Factory] Request for unregistered class: `%s`", name.c_str());
  }
  return mrpt::ptr_cast<ExecutableBase>::from(o);
}

ExecutableBase::ExecutableBase() = default;

ExecutableBase::~ExecutableBase()
{
  // Ensure profiler stats are saved now, if enabled, before
  // members dtor's are called.
  profiler_dtor_save_stats_.reset();
  MRPT_LOG_DEBUG_STREAM("ExecutableBase dtor called for module: `" << module_instance_name << "`");
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

void ExecutableBase::setModuleInstanceName(const std::string& s) { module_instance_name = s; }
std::string ExecutableBase::getModuleInstanceName() const { return module_instance_name; }

void ExecutableBase::exposeParameters(const mrpt::containers::yaml& names_values)
{
  auto lck = mrpt::lockHelper(module_params_mtx_);

  if (names_values.isNullNode() || names_values.empty())
  {
    return;
  }

  ASSERT_(names_values.isMap());

  for (const auto& [k, v] : names_values.asMapRange())
  {
    const auto name      = k.as<std::string>();
    module_params_[name] = v;

    MRPT_LOG_DEBUG_STREAM("Exposing parameter: '" << k << "'='" << v.as<std::string>() << "'");
  }
}

mrpt::containers::yaml ExecutableBase::getModuleParameters() const
{
  auto lck = mrpt::lockHelper(module_params_mtx_);
  return module_params_;
}

void ExecutableBase::changeParameters(const mrpt::containers::yaml& names_values)
{
  auto lck = mrpt::lockHelper(module_params_mtx_);

  if (names_values.isNullNode() || names_values.empty())
  {
    return;
  }
  ASSERT_(names_values.isMap());

  for (const auto& [k, v] : names_values.asMapRange())
  {
    const auto name      = k.as<std::string>();
    module_params_[name] = v;

    MRPT_LOG_DEBUG_STREAM("Changing parameter: '" << k << "'='" << v.as<std::string>() << "'");
  }

  // Notify module:
  this->onParameterUpdate(names_values);
}

void ExecutableBase::module_publish_diagnostics(const DiagnosticsOutput& msg)
{
  auto lck = mrpt::lockHelper(module_diagnostics_out_queue_mtx_);
  module_diagnostics_out_queue_.push_back(msg);
  module_diagnostics_last_clockwall_stamp_ = mrpt::Clock::nowDouble();
}

bool ExecutableBase::module_is_time_to_publish_diagnostics() const
{
  if (module_diagnostics_period_sec_ <= 0) return false;

  const double dt = mrpt::Clock::nowDouble() - module_diagnostics_last_clockwall_stamp_;
  return dt >= module_diagnostics_period_sec_;
}

std::vector<ExecutableBase::DiagnosticsOutput>
    ExecutableBase::module_move_out_diagnostics_messages()
{
  auto lck    = mrpt::lockHelper(requested_system_shutdown_mtx_);
  auto myCopy = std::move(module_diagnostics_out_queue_);
  module_diagnostics_out_queue_.clear();
  return myCopy;
}
