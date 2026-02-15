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
 * @file   FrontEndBase.cpp
 * @brief  Virtual interface for SLAM front-ends
 * @author Jose Luis Blanco Claraco
 * @date   Dec 11, 2018
 */

#include <mola_kernel/interfaces/FrontEndBase.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/containers/yaml.h>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_VIRTUAL_MRPT_OBJECT(FrontEndBase, ExecutableBase, mola)

FrontEndBase::FrontEndBase() = default;

void FrontEndBase::initialize(const Yaml& cfg)
{
  MRPT_TRY_START

  if (cfg.has("raw_data_source"))
  {
    auto ds_source = cfg["raw_data_source"];

    if (cfg.isSequence())
    {
      for (const auto& v : cfg.asSequence())
      {
        front_end_source_names_.insert(v.as<std::string>());
      }
    }
    else
    {
      front_end_source_names_.insert(ds_source.as<std::string>());
    }

    for (const auto& src_name : front_end_source_names_)
    {
      MRPT_LOG_DEBUG_STREAM(
          "FrontEndBase::initialize(): subscribing to '" << src_name << "' for raw sensor input.");

      ASSERT_(this->nameServer_);

      auto data_src = nameServer_(src_name);
      if (!data_src)
      {
        THROW_EXCEPTION_FMT("Cannot find data source module named `%s`", src_name.c_str());
      }

      auto rdsb = std::dynamic_pointer_cast<RawDataSourceBase>(data_src);
      if (!rdsb)
      {
        THROW_EXCEPTION_FMT(
            "Could not cast data source module named `%s` to RawDataSourceBase", src_name.c_str());
      }

      // Subscribe:
      rdsb->attachToDataConsumer(*this);
    }
  }
  else
  {
    // This may be OK if we are running outside of the context of mola-cli
    MRPT_LOG_DEBUG_STREAM(
        "No 'raw_data_source' entry found in the YAML definition for a "
        "FrontEndBase: YAML contents:\n"
        << cfg << "\n");
  }

  // Optional: attach to the visualizer:
  {
    auto viz = findService<VizInterface>();
    if (viz.size() == 1)
    {
      visualizer_ = std::dynamic_pointer_cast<VizInterface>(viz[0]);
      ASSERT_(visualizer_);
      MRPT_LOG_INFO("Attached to a VizInterface module");
    }
  }

  // children params:
  this->initialize_frontend(cfg);

  MRPT_TRY_END
}
