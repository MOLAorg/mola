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
 * @file   macro_helpers.h
 * @brief  C preprocessor helpers for variadic macro dispatch
 * @author Jose Luis Blanco Claraco
 * @date   Jan 11, 2019
 *
 * Provides the `VFUNC(func, ...)` macro, which dispatches to a suffixed
 * overload of `func` based on the number of variadic arguments supplied.
 *
 * ### Example
 * Given two "overloads":
 * @code
 *   #define MY_MACRO2(a, b)    ...
 *   #define MY_MACRO3(a, b, c) ...
 * @endcode
 * A single entry-point can be declared as:
 * @code
 *   #define MY_MACRO(...) VFUNC(MY_MACRO, __VA_ARGS__)
 * @endcode
 * `MY_MACRO(x, y)` expands to `MY_MACRO2(x, y)` and
 * `MY_MACRO(x, y, z)` expands to `MY_MACRO3(x, y, z)`.
 *
 * Up to 10 arguments are supported.
 *
 * @note Adapted from https://stackoverflow.com/a/26408195/1631514
 */
#pragma once

// ---------------------------------------------------------------------------
// MOLA_NARG: counts the number of arguments passed to a variadic macro.
//
// MOLA_NARG(a, b, c)  =>  3
// MOLA_NARG(a)        =>  1
//
// Works by appending a decreasing sequence and picking the 11th element,
// which corresponds to the actual argument count (up to 10 args supported).
// ---------------------------------------------------------------------------
#define MOLA_NARG(...) MOLA_NARG_I_(__VA_ARGS__, MOLA_RSEQ_N())
#define MOLA_NARG_I_(...) MOLA_ARG_N(__VA_ARGS__)

/** Picks the 11th argument - used internally by MOLA_NARG. */
#define MOLA_ARG_N(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, N, ...) N

/** Decreasing sequence used as the "ruler" for argument counting. */
#define MOLA_RSEQ_N() 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0

// ---------------------------------------------------------------------------
// MOLA_VFUNC / VFUNC: dispatch a macro call to a name##N overload,
// where N is the number of arguments.
//
// VFUNC(FOO, a, b)     =>  FOO2(a, b)
// VFUNC(FOO, a, b, c)  =>  FOO3(a, b, c)
// ---------------------------------------------------------------------------

/** Concatenates `name` and `n` after macro expansion of both. */
#define MOLA_VFUNC_(name, n) name##n

/** Forces an extra expansion pass before concatenation (needed for correctness
 *  when either argument is itself a macro). */
#define MOLA_VFUNC(name, n) MOLA_VFUNC_(name, n)

/**
 * @brief Dispatch a variadic macro call to a numbered overload.
 *
 * `VFUNC(func, args...)` expands to `funcN(args...)` where N is the number
 * of arguments in `args`.
 *
 * @param func  Base name of the macro family (without trailing digit).
 * @param ...   Arguments forwarded verbatim to the selected overload.
 */
#define VFUNC(func, ...) MOLA_VFUNC(func, MOLA_NARG(__VA_ARGS__))(__VA_ARGS__)
