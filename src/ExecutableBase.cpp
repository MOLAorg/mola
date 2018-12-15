/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ExecutableBase.h
 * @brief  Virtual interface for objects that can be run into MOLA
 * @author Jose Luis Blanco Claraco
 * @date   Dec 14, 2018
 */

#include <mola-kernel/ExecutableBase.h>

using namespace mola;

ExecutableBase::ExecutableBase() = default;

ExecutableBase::~ExecutableBase() = default;

/** This should be reimplemented to read all the required parameters */
void ExecutableBase::initialize(const std::string& cfg_block)
{
    MRPT_LOG_WARN_STREAM(
        "`initialize()` not reimplemented by derived class. "
        "Ignoring YAML config block:\n"
        << cfg_block);
}
