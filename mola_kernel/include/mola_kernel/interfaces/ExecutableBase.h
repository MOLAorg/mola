/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   ExecutableBase.h
 * @brief  Virtual interface for objects that can be run into MOLA
 * @author Jose Luis Blanco Claraco
 * @date   Dec 14, 2018
 */
#pragma once

#include <mola_kernel/Yaml.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/version.h>

#include <any>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mola
{
using Profiler            = mrpt::system::CTimeLogger;
using ProfilerEntry       = mrpt::system::CTimeLoggerEntry;
using ProfilerSaverAtDtor = mrpt::system::CTimeLoggerSaveAtDtor;

/** Base virtual class for all executable (nodelet-like) units inside a
 * SLAM system. \ingroup mola_kernel_interfaces_grp */
class ExecutableBase : public mrpt::system::COutputLogger,  // for logging
                       public mrpt::rtti::CObject,  // RTTI helpers
                       std::enable_shared_from_this<ExecutableBase>
{
  // This macro defines `Ptr=shared_ptr<T>`, among other types and methods.
  DEFINE_VIRTUAL_MRPT_OBJECT(ExecutableBase, mola)

 public:
  ExecutableBase();
  virtual ~ExecutableBase();

  // Delete copy constructor and copy assignment operator
  ExecutableBase(const ExecutableBase&)            = delete;
  ExecutableBase& operator=(const ExecutableBase&) = delete;

  // Define move constructor and move assignment operator
  ExecutableBase(ExecutableBase&&) noexcept            = delete;
  ExecutableBase& operator=(ExecutableBase&&) noexcept = delete;

  /** Class factory. Register using MOLA_REGISTER_MODULE() */
  static Ptr Factory(const std::string& classname);

  /** Get as shared_ptr via enable_shared_from_this<> */
  Ptr getAsPtr() { return shared_from_this(); }

  /** @name Virtual interface of any ExecutableBase
   *{ */

  /** This must be implemented to read all the required parameters */
  virtual void initialize(const Yaml& cfg) = 0;

  /** Runs any required action on a timely manner */
  virtual void spinOnce() = 0;

  /** Modules will be initialized in the order determined by:
   * - First: the "order priority", which is the number returned here.
   * - Second: modules with the same "priority", will be sorted by ascending
   * lexicographical order or their "instance names".
   */
  virtual int launchOrderPriority() const { return 50; }

  /** Called while destroying the SLAM system. A perfect placeholder for
   * saving data to filesystem, clean up, etc. before any module destructor
   * has been actually beeing invoked.
   */
  virtual void onQuit() {}

  /** Called whenever parameter(s) changed and the module must know about it.
   *  \name names_values Is a YAML map from `names` to `values`.
   *  \sa exposeParameters()
   */
  virtual void onParameterUpdate([[maybe_unused]] const mrpt::containers::yaml& names_values) {}

  /** @} */

  /** @name Directory services
   *{ */

  /** A name server function to search for other ExecutableBase objects in
   * my running system. Empty during ctor, should be usable from
   * initialize_common() and initialize().
   * \note In a standard system, this is implemented by
   * MolaLauncherApp::nameServerImpl()
   * \sa findService() */
  std::function<Ptr(const std::string&)> nameServer_;

  /** Finds (an)other ExecutableBase(s) by its expected Interface, that is,
   * a virtual base class.
   * \sa nameServer_ */
  template <class Interface>
  std::vector<Ptr> findService() const;

  void        setModuleInstanceName(const std::string& s);
  std::string getModuleInstanceName() const;
  /** @} */

  /** @name Module parameter handling
   *
   *{ */

 protected:
  /** Must be called by modules to declared the existence of runtime
   * configurable parameters, as well as their present values.
   * \name names_values Is a YAML map from `names` to `values`.
   * \sa onParameterUpdate()
   * \note New in MOLA v1.4.0
   */
  void exposeParameters(const mrpt::containers::yaml& names_values);

 public:
  /** Returns the current list of all known parameters and their values, in
   * the same format than used in exposeParameters(). An empty YAML map will
   * be returned if no parameter exists.
   * \sa changeParameters()
   * \note New in MOLA v1.4.0
   */
  mrpt::containers::yaml getModuleParameters() const;

  /** Called by an external entity to change parameters of this module.
   * \name names_values Is a YAML map from `names` to `values`.
   * \sa getModuleParameters()
   * \note New in MOLA v1.4.0
   */
  void changeParameters(const mrpt::containers::yaml& names_values);

  /** @} */

  /** Enabled from mola-cli with `--profiler-whole` to save full profile stats
   * to .m files at program end.
   */
  std::optional<ProfilerSaverAtDtor> profiler_dtor_save_stats_;

  /** Time profiler (disabled by default). All profilers can be globally
   * enabled from MolaLauncherApp. */
  Profiler profiler_{false};

  [[nodiscard]] bool requestedShutdown() const
  {
    auto lck = mrpt::lockHelper(requested_system_shutdown_mtx_);
    return requested_system_shutdown_;
  }

  /** Diagnostics message: a human-readabale label, and "any" content. */
  struct DiagnosticsOutput
  {
    DiagnosticsOutput() = default;

    mrpt::Clock::time_point timestamp;
    std::string             label;
    std::any                value;
  };

  [[nodiscard]] auto module_move_out_diagnostics_messages() -> std::vector<DiagnosticsOutput>;

 protected:
  void requestShutdown()
  {
    auto lck                   = mrpt::lockHelper(requested_system_shutdown_mtx_);
    requested_system_shutdown_ = true;
  }

  void module_publish_diagnostics(const DiagnosticsOutput& msg);

  [[nodiscard]] bool module_is_time_to_publish_diagnostics() const;

  /** Period (seconds) to publish loop diagnostics. Zero means disabled. */
  double module_diagnostics_period_sec_ = 1.0;

 private:
  std::string module_instance_name{"unnamed"};
  bool        requested_system_shutdown_ = false;
  std::mutex  requested_system_shutdown_mtx_;

  std::mutex             module_params_mtx_;
  mrpt::containers::yaml module_params_;

  std::mutex                     module_diagnostics_out_queue_mtx_;
  std::vector<DiagnosticsOutput> module_diagnostics_out_queue_;
  double                         module_diagnostics_last_clockwall_stamp_ = 0;
};

// Impl:
template <class Interface>
std::vector<ExecutableBase::Ptr> ExecutableBase::findService() const
{
  std::vector<ExecutableBase::Ptr> ret;
  if (!nameServer_) return ret;
  for (size_t idx = 0;; ++idx)
  {
    using namespace std::string_literals;
    const auto req = "["s + std::to_string(idx);
    auto       mod = nameServer_(req);
    if (!mod) break;  // end of list of modules
    if (std::dynamic_pointer_cast<Interface>(mod)) ret.emplace_back(std::move(mod));
  }
  return ret;
}

#define MOLA_REGISTER_MODULE(_classname) mrpt::rtti::registerClass(CLASS_ID(_classname))

}  // namespace mola
