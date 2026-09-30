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
 * @file   ViewDirectionFilter.h
 * @brief  How cov-to-cov matching uses per-point view directions
 * @author Jose Luis Blanco Claraco
 * @date   Sep 29, 2026
 */
#pragma once

#include <mrpt/typemeta/TEnumType.h>

#include <cstdint>

namespace mola
{
/** How `nn_search_cov2cov()` of `KeyframePointCloudMap` and
 *  `IncrementalPointCloud` uses per-point view directions (`view_x`, `view_y`,
 *  `view_z`: unit vectors pointing FROM each point TOWARD the sensor at
 *  acquisition time) to reject a cov-to-cov pair. The test only runs when both
 *  the map and the query cloud carry those fields, and a zero (missing)
 *  direction never rejects a pair.
 *
 *  Selected by the `view_direction_filter` creation option of both classes,
 *  subject to their `use_view_direction_filter` master switch.
 */
enum class ViewDirectionFilter : uint8_t
{
  /** No filtering. */
  None = 0,
  /** Reject a pair whose two view directions are more than
   *  `max_view_angle_deg` apart. Rejects the two faces of a thin structure,
   *  but also the same surface seen from very different directions on the
   *  same side, e.g. ground observed from opposite azimuths. */
  MaxAngle,
  /** Reject a pair only when both views see the matched map point's surface
   *  clearly (more than ~14.5 deg off its plane) and from opposite sides of
   *  it, as given by the normal of that point's covariance, and the raw (not
   *  regularized) neighborhood of that point is actually flat. Points on
   *  foliage, edges or poles, whose regularized covariance also looks like a
   *  plane but whose normal says nothing about sides, are never rejected;
   *  neither is the same surface seen from very different azimuths on the
   *  same side, e.g. ground. */
  SurfaceSide
};

}  // namespace mola

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mola, mola::ViewDirectionFilter)
MRPT_FILL_ENUM(ViewDirectionFilter::None);
MRPT_FILL_ENUM(ViewDirectionFilter::MaxAngle);
MRPT_FILL_ENUM(ViewDirectionFilter::SurfaceSide);
MRPT_ENUM_TYPE_END()
