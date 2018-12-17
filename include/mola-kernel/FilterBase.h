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
#pragma once

#include <mola-kernel/RawDataConsumer.h>
#include <mola-kernel/RawDataSourceBase.h>
#include <mola-kernel/WorkerThreadsPool.h>
#include <mrpt/core/initializer.h>  // MRPT_INITIALIZER()
#include <mrpt/system/COutputLogger.h>

namespace mola
{
/** Base class for filters: get observations via `onNewObservation()`, and
 * immediately send them to a worker thread that works on it, possibly
 * publishing the output via `sendObservationsToFrontEnds()`.
 * \ingroup mola_kernel_grp */
class FilterBase : public RawDataSourceBase, RawDataConsumer
{
   public:
    FilterBase();

    using Ptr = std::shared_ptr<FilterBase>;

    /** Class factory. Register using MOLA_REGISTER_FILTER() */
    static Ptr Factory(const std::string& classname);

    /** @name Virtual interface of any Filter
     *{ */

    /** To be called for each incoming observation. Process it and return
     * the modified observation.
     */
    virtual CObservation::Ptr doFilter(CObservation::Ptr& o) = 0;
    /** @} */

    // Virtual interface of any RawDataSource
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;

    // Virtual interface of any RawDataConsumer
    void onNewObservation(CObservation::Ptr& o) override;

    static void registerClass(
        const std::string_view& classname, std::function<Ptr(void)> func);

   private:
    WorkerThreadsPool thread_pool_;
};

#define MOLA_REGISTER_FILTER(_classname)                                   \
    MRPT_INITIALIZER(do_register_class)                                    \
    {                                                                      \
        mola::FilterBase::registerClass(                                   \
            #_classname, []() { return std::make_shared<_classname>(); }); \
    }

}  // namespace mola
