/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawDataSourceBase.h
 * @brief  Virtual interface for data sources, either real sensors or datasets
 * @author Jose Luis Blanco Claraco
 * @date   Nov 21, 2018
 */
#pragma once

#include <mola-kernel/interfaces/ExecutableBase.h>
#include <mola-kernel/interfaces/RawDataConsumer.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/core/initializer.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/obs/CObservation.h>

#include <functional>
#include <memory>

namespace mola
{
/** Virtual base for raw-observation data sources (sensors, dataset parsers,...)
 * \ingroup mola_kernel_grp */
class RawDataSourceBase : public mola::ExecutableBase
{
    DEFINE_VIRTUAL_MRPT_OBJECT(RawDataSourceBase)

   public:
    RawDataSourceBase();
    virtual ~RawDataSourceBase();

    /** Attach this object to a consumer. A shared_ptr is created to keep a
     * reference to the object. */
    void attachToDataConsumer(RawDataConsumer& rdc);

    /** Loads common parameters for all RDS. Called by launcher just before
     * initialize(). */
    void initialize_common(const std::string& cfg_block);

   protected:
    /** Send an observation to the associated target front-ends */
    void sendObservationsToFrontEnds(CObservation::Ptr& obs);

    /** Make sure the observation is loaded in memory (for exernally-stored
     * classes), etc. */
    virtual void prepareObservationBeforeFrontEnds(
        CObservation::Ptr& obs) const;

   private:
    /** Target of captured data */
    std::vector<RawDataConsumer*> rdc_;

    /** used to optionally export captured observations to an MRPT rawlog */
    mrpt::io::CFileGZOutputStream export_to_rawlog_out_;
    mrpt::WorkerThreadsPool       worker_pool_export_rawlog_{1};

    mrpt::WorkerThreadsPool gui_updater_threadpool_{1 /* 1 thread */};

    struct SensorViewerImpl;
    /** Optional real-time GUI view of sensor data. Viewers indexed by
     * sensor_label */
    std::map<std::string, mrpt::pimpl<SensorViewerImpl>> sensor_preview_gui_;
};

}  // namespace mola
