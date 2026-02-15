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
 * @file   FrontEndBase.h
 * @brief  Virtual interface for SLAM front-ends
 * @author Jose Luis Blanco Claraco
 * @date   Dec 11, 2018
 */
#pragma once

#include <mola_kernel/Yaml.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/RawDataConsumer.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mrpt/version.h>

#include <set>
#include <string>

namespace mola
{
/** Virtual interface for SLAM front-ends.
 *
 * Instructions for implementing new front-ends:
 * Raw observations arrive via calls to the virtual method `onNewObservation()`,
 * which must be implemented. Minimum time should be spent there, just copy the
 * incoming data (smart pointer). Actual processing can be normally done by any
 * of these two ways:
 * - Wait until `spinOnce()` is called, at the rate specified in the yaml file
 * (default=1 Hz), or
 * - Use your own logic to enqueue a task into a worker thread pool (preferred).
 *
 * \ingroup mola_kernel_interfaces_grp */
class FrontEndBase : public ExecutableBase, public RawDataConsumer
{
  DEFINE_VIRTUAL_MRPT_OBJECT(FrontEndBase, mola)

 public:
  FrontEndBase();

  /** Loads common parameters for all front-ends.
   *
   * These parameters are handled here:
   * - `raw_data_source`: A list of one (e.g. `raw_data_source: 'moduleName'`)
   *   or multiple (e.g. `raw_data_source: [moduleName1, moduleName2]`) MOLA
   *   **module names** to which to subscribe for input raw observations.
   *
   * Recall that module names are given in the `name:` field of the mola-cli
   * launch YAML file.
   *
   * Note that no individual sensor label is read in this abstract class
   * since it is up to the derived classes to specify whether to subscribe
   * to one or more sensor sources, and use descriptive names in the case of
   * multiple sensors.
   *
   *
   */
  void initialize(const Yaml& cfg) override final;

 protected:
  /** Loads children specific parameters */
  virtual void initialize_frontend(const Yaml& cfg) = 0;

 public:
 protected:
  /** A list of one or multiple MOLA **module names** to which to subscribe
   * for input raw observations.
   */
  std::set<std::string> front_end_source_names_;

  /** References to other modules. Populated by initialize_common() */
  VizInterface::Ptr visualizer_;
};

}  // namespace mola
