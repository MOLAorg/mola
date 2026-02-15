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
    const std::exception& e, const std::string& headerLine, const bool use_std_cerr = true);

}  // namespace mola
