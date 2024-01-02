/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
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
