/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ExecutableBase.h
 * @brief  Virtual interface for objects that can be run into MOLA
 * @author Jose Luis Blanco Claraco
 * @date   Dec 14, 2018
 */
#pragma once

#include <mola_kernel/Yaml.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>

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
 * SLAM system. \ingroup mola_kernel_grp */
class ExecutableBase : public mrpt::system::COutputLogger,  // for logging
                       public mrpt::rtti::CObject,  // RTTI helpers
                       std::enable_shared_from_this<ExecutableBase>
{
    // This macro defines `Ptr=shared_ptr<T>`, among other types and methods.
    DEFINE_VIRTUAL_MRPT_OBJECT(ExecutableBase)

   public:
    ExecutableBase();
    virtual ~ExecutableBase();

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
    /** @} */

    /** @name Directory services
     *{ */

    /** A name server function to search for other ExecutableBase objects in my
     * running system. Empty during ctor, should be usable from
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

   protected:
    void requestShutdown()
    {
        auto lck = mrpt::lockHelper(requested_system_shutdown_mtx_);
        requested_system_shutdown_ = true;
    }

   private:
    std::string module_instance_name{"unnamed"};
    bool        requested_system_shutdown_ = false;
    std::mutex  requested_system_shutdown_mtx_;
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
        if (std::dynamic_pointer_cast<Interface>(mod))
            ret.emplace_back(std::move(mod));
    }
    return ret;
}

#define MOLA_REGISTER_MODULE(_classname) \
    mrpt::rtti::registerClass(CLASS_ID(_classname))

}  // namespace mola
