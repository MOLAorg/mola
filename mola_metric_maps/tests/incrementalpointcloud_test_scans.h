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
 * @file   incrementalpointcloud_test_scans.h
 * @brief  Synthetic LiDAR scans shared by the IncrementalPointCloud tests
 * @author Jose Luis Blanco Claraco
 * @date   Sep 17, 2026
 */
#pragma once

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <cstddef>

namespace mola_test
{
/** A vehicle-like scan, in the sensor frame: ground plane, two side walls and
 *  some clutter, so the plane-regularized covariances have real structure to
 *  fit.
 */
inline mrpt::maps::CSimplePointsMap::Ptr makeScan(
    mrpt::random::CRandomGenerator& rng, std::size_t n)
{
  auto pts = mrpt::maps::CSimplePointsMap::Create();
  pts->reserve(n);

  for (std::size_t i = 0; i < n; i++)
  {
    const double u = rng.drawUniform(0.0, 1.0);
    if (u < 0.5)
    {
      // Ground plane, up to 50 m away:
      const double r  = rng.drawUniform(1.0, 50.0);
      const double th = rng.drawUniform(-M_PI, M_PI);
      pts->insertPointFast(
          static_cast<float>(r * std::cos(th)), static_cast<float>(r * std::sin(th)),
          static_cast<float>(-1.7 + rng.drawGaussian1D(0, 0.01)));
    }
    else if (u < 0.9)
    {
      // Two walls parallel to the driving direction:
      const double side = (rng.drawUniform(0.0, 1.0) < 0.5) ? -8.0 : 8.0;
      pts->insertPointFast(
          static_cast<float>(rng.drawUniform(-40.0, 40.0)),
          static_cast<float>(side + rng.drawGaussian1D(0, 0.01)),
          static_cast<float>(rng.drawUniform(-1.7, 6.0)));
    }
    else
    {
      // Clutter:
      const double r  = rng.drawUniform(2.0, 60.0);
      const double th = rng.drawUniform(-M_PI, M_PI);
      pts->insertPointFast(
          static_cast<float>(r * std::cos(th)), static_cast<float>(r * std::sin(th)),
          static_cast<float>(rng.drawUniform(-1.7, 8.0)));
    }
  }
  pts->mark_as_modified();
  return pts;
}

/// A gentle curve, so a sliding-window keep box sweeps in both x and y.
inline mrpt::poses::CPose3D poseOfFrame(std::size_t f, double stepPerFrame)
{
  const double s = static_cast<double>(f) * stepPerFrame;
  return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
      s, 20.0 * std::sin(s / 60.0), 0.02 * std::sin(s / 13.0), 0.3 * std::sin(s / 60.0), 0, 0);
}

}  // namespace mola_test
