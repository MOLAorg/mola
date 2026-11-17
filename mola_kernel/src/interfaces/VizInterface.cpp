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

std::future<bool> VizInterface::update_viewport_camera(
    [[maybe_unused]] const mrpt::math::TPoint3Df& lookAt, [[maybe_unused]] double azimuth, [[maybe_unused]] double elevation,
    [[maybe_unused]] double zoomDistance, [[maybe_unused]] const std::string& viewportName,
    [[maybe_unused]] const std::string& parentWindow)
{
  std::promise<bool> p;
  p.set_value(false);
  return p.get_future();
}

const std::string VizInterface::BACKEND_NANOGUI = "nanogui";
const std::string VizInterface::BACKEND_IMGUI   = "imgui";

}  // namespace mola