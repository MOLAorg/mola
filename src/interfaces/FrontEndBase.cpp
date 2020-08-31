/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FrontEndBase.cpp
 * @brief  Virtual interface for SLAM front-ends
 * @author Jose Luis Blanco Claraco
 * @date   Dec 11, 2018
 */

#include <mola-kernel/interfaces/FrontEndBase.h>
#include <mola-kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/containers/yaml.h>
#include <iostream>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_VIRTUAL_MRPT_OBJECT(FrontEndBase, ExecutableBase, mola)

FrontEndBase::FrontEndBase() = default;

void FrontEndBase::initialize_common(const std::string& cfg_block)
{
    MRPT_TRY_START
    auto cfg = mrpt::containers::yaml::FromText(cfg_block);

    // Optional parameter: derived classes may use it or not, so don't throw
    // an exception if not found.
    raw_sensor_label_ =
        cfg.getOrDefault<std::string>("raw_sensor_label", raw_sensor_label_);

    MRPT_TODO("Enable multiple source modules, comma sep list, etc.");
    auto ds_source = cfg["raw_data_source"];
    if (ds_source)
    {
        const auto src_name = ds_source.as<std::string>();

        ASSERT_(this->nameServer_);

        auto data_src = nameServer_(src_name);
        if (!data_src)
            THROW_EXCEPTION_FMT(
                "Cannot find data source module named `%s`", src_name.c_str());

        auto rdsb = std::dynamic_pointer_cast<RawDataSourceBase>(data_src);
        if (!rdsb)
            THROW_EXCEPTION_FMT(
                "Could not cast data source module named `%s` to "
                "RawDataSourceBase",
                src_name.c_str());

        // Subscribe:
        rdsb->attachToDataConsumer(*this);
    }

    // Search for SLAM backend:
    auto fnd_bckends = ExecutableBase::findService<BackEndBase>();
    if (fnd_bckends.empty())
    { MRPT_LOG_WARN("No SLAM back-end found in the system."); }
    else
    {
        if (fnd_bckends.size() > 1)
        {
            MRPT_LOG_WARN(
                "More than one SLAM back-end found in the system! Attaching to "
                "first one.");
        }
        slam_backend_ = std::dynamic_pointer_cast<BackEndBase>(fnd_bckends[0]);
        ASSERT_(slam_backend_);
        MRPT_LOG_INFO_FMT(
            "Attached to SLAM backend module `%s`",
            slam_backend_->getModuleInstanceName().c_str());
    }

    // Search for world model:
    {
        auto wms = findService<WorldModel>();
        if (wms.size() == 1)
        {
            worldmodel_ = std::dynamic_pointer_cast<WorldModel>(wms[0]);
            ASSERT_(worldmodel_);
            MRPT_LOG_INFO_FMT(
                "Attached to WorldModel module `%s`",
                worldmodel_->getModuleInstanceName().c_str());
        }
    }

    MRPT_TRY_END
}
