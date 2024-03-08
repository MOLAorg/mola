/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterBase.h
 * @brief  Virtual base class for sensor pipeline filters
 * @author Jose Luis Blanco Claraco
 * @date   Dec 11, 2018
 */
#pragma once

#include <mola_kernel/interfaces/RawDataConsumer.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/system/COutputLogger.h>

namespace mola
{
/** Base class for filters: get observations via `onNewObservation()`, and
 * immediately send them to a worker thread that works on it, possibly
 * publishing the output via `sendObservationsToFrontEnds()`.
 * \ingroup mola_kernel_grp */
class FilterBase : public RawDataSourceBase, RawDataConsumer
{
    DEFINE_VIRTUAL_MRPT_OBJECT(FilterBase)

   public:
    FilterBase();

    /** @name Virtual interface of any Filter
     *{ */

    /** To be called for each incoming observation. Process it and return
     * the modified observation.
     */
    virtual CObservation::Ptr doFilter(const CObservation::Ptr& o) = 0;
    /** @} */

    void spinOnce() override;

    // Virtual interface of any RawDataConsumer
    void onNewObservation(const CObservation::Ptr& o) override;

   protected:
    // Virtual interface of any RawDataSource
    void initialize_rds(const Yaml& cfg) override;

   private:
    mrpt::WorkerThreadsPool thread_pool_;
};

}  // namespace mola
