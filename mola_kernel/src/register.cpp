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
 * @file   register.cpp
 * @brief  RTTI & CSerializable registry
 * @author Jose Luis Blanco Claraco
 * @date   Jun 22, 2019
 */

#include <mrpt/core/initializer.h>

// To be run at .so/.dll load:
MRPT_INITIALIZER(do_register_mola_kernel)  // NOLINT(misc-use-anonymous-namespace)
{
  // using namespace mola;

  // Register module:
  // MOLA_REGISTER_MODULE(WorldModel);

  // Register serializable classes:
  // mrpt::rtti::registerClass(CLASS_ID(mola::WorldModelData));
}
