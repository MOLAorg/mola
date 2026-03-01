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
 * `@file`   VizInterface.cpp
 * `@brief`  Virtual visualization interface (see MolaViz)
 * `@author` Jose Luis Blanco Claraco
 * `@date`   Feb 15, 2026
 */

#include <mola_kernel/interfaces/VizInterface.h>

namespace mola
{

VizInterface::~VizInterface() = default;

const std::string VizInterface::BACKEND_NANOGUI = "nanogui";
const std::string VizInterface::BACKEND_IMGUI   = "imgui";

}  // namespace mola