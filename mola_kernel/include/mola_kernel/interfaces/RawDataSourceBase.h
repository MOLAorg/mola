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
#include <mrpt/obs/CObservation.h>
#include <mrpt/version.h>  // MRPT_VERSION

#if MRPT_VERSION >= 0x020f07
#include <mrpt/io/CCompressedOutputStream.h>
#else
#include <mrpt/io/CFileGZOutputStream.h>
#endif

namespace mola
{
/** 0-based indices of observations in a dataset */
using timestep_t = std::size_t;

/** Virtual base for raw-observation data sources (sensors, dataset
 * parsers,...) \ingroup mola_kernel_interfaces_grp */
class RawDataSourceBase : public mola::ExecutableBase
{
  DEFINE_VIRTUAL_MRPT_OBJECT(RawDataSourceBase, mola)

 public:
  RawDataSourceBase();
  virtual ~RawDataSourceBase();

  // Delete copy constructor and copy assignment operator
  RawDataSourceBase(const RawDataSourceBase&)            = delete;
  RawDataSourceBase& operator=(const RawDataSourceBase&) = delete;

  // Delete move constructor and move assignment operator
  RawDataSourceBase(RawDataSourceBase&&)            = delete;
  RawDataSourceBase& operator=(RawDataSourceBase&&) = delete;

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
  virtual void prepareObservationBeforeFrontEnds(const CObservation::Ptr& obs) const;

  /** Should be called by derived classes if the end of a dataset was reached
   * during spin()  */
  void onDatasetPlaybackEnds();

 private:
  /** Target of captured data */
  std::vector<RawDataConsumer*> rdc_;

  /** used to optionally export captured observations to an MRPT rawlog */
#if MRPT_VERSION >= 0x020f07
  mrpt::io::CCompressedOutputStream export_to_rawlog_out_;
#else
  mrpt::io::CFileGZOutputStream export_to_rawlog_out_;
#endif

  mrpt::WorkerThreadsPool worker_pool_export_rawlog_{
      1, mrpt::WorkerThreadsPool::POLICY_FIFO, "worker_pool_export_rawlog"};

  mrpt::WorkerThreadsPool gui_updater_threadpool_{
      1 /* 1 thread */, mrpt::WorkerThreadsPool::POLICY_FIFO, "gui_updater_threadpool"};

  struct SensorViewerImpl;
  /** Optional real-time GUI view of sensor data. Viewers indexed by
   * sensor_label */
  std::map<std::string, mrpt::pimpl<SensorViewerImpl>> sensor_preview_gui_;

  bool force_load_lazy_load_         = false;
  bool quit_mola_app_on_dataset_end_ = false;
};

}  // namespace mola
