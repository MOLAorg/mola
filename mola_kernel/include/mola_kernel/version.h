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
 * @file   version.h
 * @brief  Provides macros for checking MOLA version
 * @author Jose Luis Blanco Claraco
 * @date   Aug 18, 2024
 */
#pragma once

/// To be used in #if() checks for >= minimum MOLA versions
#define MOLA_VERSION_CHECK(major, minor, patch)                       \
  ((MOLA_MAJOR_VERSION > (major)) ||                                  \
   (MOLA_MAJOR_VERSION == (major) && MOLA_MINOR_VERSION > (minor)) || \
   (MOLA_MAJOR_VERSION == (major) && MOLA_MINOR_VERSION == (minor) && \
    MOLA_PATCH_VERSION >= (patch)))
