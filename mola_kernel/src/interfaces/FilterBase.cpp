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
 * @file   FilterBase.cpp
 * @brief  Virtual base class for sensor pipeline filters
 * @author Jose Luis Blanco Claraco
 * @date   Dec 11, 2018
 */

#include <mola_kernel/interfaces/FilterBase.h>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_VIRTUAL_MRPT_OBJECT(FilterBase, ExecutableBase, mola)

FilterBase::FilterBase() = default;

// Virtual interface of any RawDataSource
void FilterBase::initialize_rds(const Yaml& cfg)
{
  if (!cfg.empty())
  {
    MRPT_LOG_WARN_STREAM(
        "`initialize()` not reimplemented by derived class. "
        "Ignoring YAML config block:\n"
        << cfg);
  }
}

void FilterBase::spinOnce()
{
  // Nothing to do by default. We work data-driven via the worker thread.
  // See onNewObservation().
}

// Virtual interface of any RawDataConsumer
void FilterBase::onNewObservation(const CObservation::ConstPtr& o)
{
  const auto obsFut = thread_pool_.enqueue(
      [this](const CObservation::ConstPtr& in)
      {
        try
        {
          // Process the observation:
          CObservation::Ptr out = this->doFilter(in);
          // Forward it:
          if (out)
          {
            this->sendObservationsToFrontEnds(out);
          }
        }
        catch (const std::exception& e)
        {
          MRPT_LOG_ERROR_STREAM(
              "[FilterBase::onNewObservation] Error:\n"
              << mrpt::exception_to_str(e));
        }
      },
      o);

  (void)obsFut;
}
