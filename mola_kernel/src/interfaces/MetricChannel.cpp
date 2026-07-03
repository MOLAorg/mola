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
 * @file   MetricChannel.cpp
 * @brief  Handle for streaming timestamped scalar values to the visualizer.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_kernel/interfaces/MetricChannel.h>
#include <mrpt/core/Clock.h>

namespace mola
{

void MetricChannel::push(double value) { push(mrpt::Clock::nowDouble(), value); }

}  // namespace mola
