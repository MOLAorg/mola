/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RawDataSourceBase.h
 * @brief  Virtual interface for data sources, either real sensors or datasets
 * @author Jose Luis Blanco Claraco
 * @date   Nov 21, 2018
 */
#pragma once

#include <mola_kernel/Yaml.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/RawDataConsumer.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/core/initializer.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/obs/CObservation.h>

#include <functional>
#include <memory>

namespace mola
{
/** 0-based indices of observations in a dataset */
using timestep_t = std::size_t;

/** Virtual base for raw-observation data sources (sensors, dataset
 * parsers,...) \ingroup mola_kernel_grp */
class RawDataSourceBase : public mola::ExecutableBase
{
    DEFINE_VIRTUAL_MRPT_OBJECT(RawDataSourceBase)

   public:
    RawDataSourceBase();
    virtual ~RawDataSourceBase();

    /** Attach this object to a consumer. A shared_ptr is created to keep a
     * reference to the object. */
    void attachToDataConsumer(RawDataConsumer& rdc);

    /** Loads common parameters for all RDS.
     * This handles:
     * - `gui_preview_sensors`: Enable displaying sensor data in a subwindow.
     * - `export_to_rawlog`: If defined, save observations to the given rawlog
     * file.
     * - `force_load_lazy_load`: (Default=false) Force load() on all incoming
     * observations.
     * - `quit_mola_app_on_dataset_end`: (Default=false) Quits the MOLA app when
     * end of dataset is reached.
     */
    void initialize(const Yaml& cfg) override final;

   protected:
    /** Loads children specific parameters */
    virtual void initialize_rds(const Yaml& cfg) = 0;

   public:
   protected:
    /** Send an observation to the associated target front-ends */
    void sendObservationsToFrontEnds(const CObservation::Ptr& obs);

    /** Make sure the observation is loaded in memory (for externally-stored
     * classes), etc. Only has effect if the option `force_load_lazy_load` was
     * set to `true` */
    virtual void prepareObservationBeforeFrontEnds(
        const CObservation::Ptr& obs) const;

    /** Should be called by derived classes if the end of a dataset was reached
     * during spin()  */
    void onDatasetPlaybackEnds();

   private:
    /** Target of captured data */
    std::vector<RawDataConsumer*> rdc_;

    /** used to optionally export captured observations to an MRPT rawlog */
    mrpt::io::CFileGZOutputStream export_to_rawlog_out_;
    mrpt::WorkerThreadsPool       worker_pool_export_rawlog_{
        1, mrpt::WorkerThreadsPool::POLICY_FIFO, "worker_pool_export_rawlog"};

    mrpt::WorkerThreadsPool gui_updater_threadpool_{
        1 /* 1 thread */, mrpt::WorkerThreadsPool::POLICY_FIFO,
        "gui_updater_threadpool"};

    struct SensorViewerImpl;
    /** Optional real-time GUI view of sensor data. Viewers indexed by
     * sensor_label */
    std::map<std::string, mrpt::pimpl<SensorViewerImpl>> sensor_preview_gui_;

    bool force_load_lazy_load_         = false;
    bool quit_mola_app_on_dataset_end_ = false;
};

}  // namespace mola
