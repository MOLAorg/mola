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
 * @file   relocalization.cpp
 * @brief  Algorithms for localization starting with large uncertainty.
 * @author Jose Luis Blanco Claraco
 * @date   Apr 2, 2024
 */

#include <mola_relocalization/relocalization.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/version.h>

std::map<double, mrpt::math::TPose2D> mola::find_best_poses_se2(
    const mrpt::poses::CPosePDFGrid& grid, const double percentile)
{
  ASSERT_GT_(percentile, 0.0);
  ASSERT_LT_(percentile, 1.0);

  const double maxLik = *std::max_element(grid.data().begin(), grid.data().end());

  const double threshold = percentile * maxLik;

  std::map<double, mrpt::math::TPose2D> best;

  const size_t N = grid.data().size();
  for (size_t i = 0; i < N; i++)
  {
    const double cell = grid.data()[i];
    if (cell < threshold)
    {
      continue;
    }

    const auto [ix, iy, iphi] = grid.absidx2idx(i);
    const double x            = grid.idx2x(ix);
    const double y            = grid.idx2y(iy);
    const double phi          = grid.idx2phi(iphi);
    best[cell]                = {x, y, phi};
  }
  return best;
}
