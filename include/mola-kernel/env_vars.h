/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   env_vars.h
 * @brief  Parse environment variables in config text blocks
 * @author Jose Luis Blanco Claraco
 * @date   Nov 30, 2018
 */
#pragma once

#include <string>

namespace mola
{
/** Replaces `${VAR}` expressions with their values from environment vars.
 * \ingroup mola_kernel_grp
 */
std::string parseEnvVars(const std::string& text);

}  // namespace mola
