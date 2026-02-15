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
 * @file   variant_helper.h
 * @brief  Helper types for std::variant
 * @author Jose Luis Blanco Claraco
 * @date   Jan 09, 2019
 */
#pragma once

namespace mola
{
/** Based on https://en.cppreference.com/w/cpp/utility/variant/visit */
template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};

/** Based on https://en.cppreference.com/w/cpp/utility/variant/visit */
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

}  // namespace mola
