/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ExecutableBase.h
 * @brief  Virtual interface for objects that can be run into MOLA
 * @author Jose Luis Blanco Claraco
 * @date   Dec 14, 2018
 */
#pragma once

#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mola
{
using Profiler            = mrpt::system::CTimeLogger;
using ProfilerEntry       = mrpt::system::CTimeLoggerEntry;
using ProfilerSaverAtDtor = mrpt::system::CTimeLoggerSaveAtDtor;

/** Base virtual class for all executable (nodelets-like) units inside a
 * SLAM system. \ingroup mola_kernel_grp */
class ExecutableBase : public mrpt::system::COutputLogger,
                       std::enable_shared_from_this<ExecutableBase>
{
   public:
    ExecutableBase();
    virtual ~ExecutableBase();

    using Ptr = std::shared_ptr<ExecutableBase>;

    /** Class factory. Register using MOLA_REGISTER_MODULE() */
    static Ptr Factory(const std::string& classname);

    static void registerClass(
        const std::string_view& classname, std::function<Ptr(void)> func);

    /** Get as shared_ptr via enable_shared_from_this<> */
    Ptr getAsPtr() { return shared_from_this(); }

    /** @name Virtual interface of any ExecutableBase
     *{ */
    virtual void initialize_common(const std::string& cfg_block) = 0;
    virtual void initialize(const std::string& cfg_block);
    virtual void spinOnce() = 0;

    /** Modules will be initialized in the order determined by:
     * - First: the "oder priority", which is the number returned here.
     * - Second: modules with the same "priority", will be sorted by ascending
     * lexicographical order or their "instance names".
     */
    virtual int launchOrderPriority() const { return 50; }
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
     * to CSV files at program end.
     */
    std::optional<ProfilerSaverAtDtor> profiler_dtor_save_stats_;

    /** Time profiler (disabled by default). All profilers can be globally
     * enabled from MolaLauncherApp. */
    Profiler profiler_{false};

   private:
    std::string module_instance_name{"unnamed"};
};

// Impl:
template <class Interface>
std::vector<ExecutableBase::Ptr> ExecutableBase::findService() const
{
    ASSERT_(nameServer_);
    std::vector<ExecutableBase::Ptr> ret;
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
    mola::ExecutableBase::registerClass( \
        #_classname, []() { return std::make_shared<_classname>(); });

}  // namespace mola
