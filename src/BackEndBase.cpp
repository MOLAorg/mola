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

#include <mola-kernel/BackEndBase.h>
#include <mola-kernel/RawDataSourceBase.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

using namespace mola;

BackEndBase::BackEndBase() = default;

void BackEndBase::initialize_common([[maybe_unused]] const std::string& cfg)
{
    MRPT_TRY_START

    MRPT_TODO("Attach to world model?");

    MRPT_TRY_END
}
