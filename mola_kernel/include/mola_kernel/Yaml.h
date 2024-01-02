/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Yaml.h
 * @brief  Tiny header to provide a shortcut in NS `mola::` to MRPT yaml class.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2021
 */
#pragma once

#include <mrpt/containers/yaml.h>

namespace mola
{
/** Convenient typedef to save typing in the MOLA project. */
using Yaml = mrpt::containers::yaml;

};  // namespace mola
