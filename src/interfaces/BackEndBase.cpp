/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   BackEndBase.cpp
 * @brief  Virtual interface for SLAM back-ends
 * @author Jose Luis Blanco Claraco
 * @date   Dec 21, 2018
 */

#include <mola-kernel/WorldModel.h>
#include <mola-kernel/interfaces/BackEndBase.h>
#include <mola-kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/containers/yaml.h>
#include <iostream>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_VIRTUAL_MRPT_OBJECT(BackEndBase, ExecutableBase, mola)

BackEndBase::BackEndBase() = default;

void BackEndBase::initialize_common([[maybe_unused]] const std::string& cfg)
{
    MRPT_TRY_START

    // attach to world model:
    auto wms = findService<WorldModel>();
    ASSERTMSG_(!wms.empty(), "No WorldModel found in the system!");
    ASSERTMSG_(
        wms.size() == 1, "Only one WorldModel can coexist in the system!");

    worldmodel_ = std::dynamic_pointer_cast<WorldModel>(wms[0]);
    ASSERT_(worldmodel_);
    MRPT_LOG_INFO_FMT(
        "Attached to WorldModel module `%s`",
        worldmodel_->getModuleInstanceName().c_str());

    MRPT_TRY_END
}
