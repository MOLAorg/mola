/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   pretty_print_exception.h
 * @brief  Helper function to print an exception with colors
 * @author Jose Luis Blanco Claraco
 * @date   Aug 13, 2021
 */
#pragma once

#include <stdexcept>
#include <string>

namespace mola
{
void pretty_print_exception(
    const std::exception& e, const std::string& headerLine,
    const bool use_std_cerr = true);

}  // namespace mola
