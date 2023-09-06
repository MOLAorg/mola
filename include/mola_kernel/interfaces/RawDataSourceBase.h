/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
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
#include <mrpt/poses/CPose3DInterpolator.h>

#include <functional>
#include <memory>

namespace mola
{
/** 0-based indices of observations in a dataset */
using timestep_t = std::size_t;

/** We reuse mrpt::poses::CPose3DInterpolator as a time-indexed map of SE(3)
 * poses */
using trajectory_t = mrpt::poses::CPose3DInterpolator;

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

    /** Loads common parameters for all RDS. Called by launcher just before
     * initialize().
     * This handles:
     * - `gui_preview_sensors`: Enable displaying sensor data in a subwindow.
     * - `export_to_rawlog`: If defined, save observations to the given rawlog
     * file.
     */
    void initialize_common(const Yaml& cfg) override;

    /** For real sensors, this will always return false.
     *  For datasets, this returns true if a groundtruth is available
     *  for the vehicle trajectory.
     *  \sa getGroundTruthTrajectory()
     */
    virtual bool hasGroundTruthTrajectory() const { return false; }

    /** If hasGroundTruthTrajectory() returns true, this returns the dataset
     *  groundtruth for the vehicle trajectory.
     *
     *  Note that timestamps for datasets are not wall-clock time ("now"), but
     *  old timestamps of when original observations were grabbed.
     *
     *  \sa hasGroundTruthTrajectory()
     */
    virtual trajectory_t getGroundTruthTrajectory() const { return {}; }

   protected:
    /** Send an observation to the associated target front-ends */
    void sendObservationsToFrontEnds(const CObservation::Ptr& obs);

    /** Make sure the observation is loaded in memory (for exernally-stored
     * classes), etc. */
    virtual void prepareObservationBeforeFrontEnds(
        const CObservation::Ptr& obs) const;

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
};

}  // namespace mola
