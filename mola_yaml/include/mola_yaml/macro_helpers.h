/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   macro_helpers.h
 * @brief  C preprocessor helpers
 * @author Jose Luis Blanco Claraco
 * @date   Jan 11, 2019
 */
#pragma once

// The following is taken from a great answer here:
// https://stackoverflow.com/a/26408195/1631514

// get number of arguments with MOLA_NARG
#define MOLA_NARG(...) MOLA_NARG_I_(__VA_ARGS__, MOLA_RSEQ_N())
#define MOLA_NARG_I_(...) MOLA_ARG_N(__VA_ARGS__)
#define MOLA_ARG_N(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, N, ...) N
#define MOLA_RSEQ_N() 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0

// general definition for any function name
#define MOLA_VFUNC_(name, n) name##n
#define MOLA_VFUNC(name, n) MOLA_VFUNC_(name, n)
#define VFUNC(func, ...) MOLA_VFUNC(func, MOLA_NARG(__VA_ARGS__))(__VA_ARGS__)
