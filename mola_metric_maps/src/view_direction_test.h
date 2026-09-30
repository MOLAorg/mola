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
 * @file   view_direction_test.h
 * @brief  The per-pair view-direction test shared by the cov-to-cov map classes
 * @author Jose Luis Blanco Claraco
 * @date   Sep 29, 2026
 */
#pragma once

#include <mola_metric_maps/ViewDirectionFilter.h>
#include <mrpt/core/bits_math.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace mola::internal
{
/** Decides whether a cov-to-cov pair must be rejected from the view directions
 *  of its two points, as configured by a ViewDirectionFilter mode.
 *
 *  Both view vectors, and the covariance of the matched map point, must be
 *  given in the same frame (whichever is cheaper for the caller).
 */
class ViewDirectionTest
{
 public:
  /** `enabled` is the classes' `use_view_direction_filter` master switch. A
   *  MaxAngle threshold of 180 deg or more rejects nothing, so it is reported
   *  as inactive. */
  ViewDirectionTest(bool enabled, ViewDirectionFilter mode, double maxViewAngleDeg)
  {
    const double maxAngle = std::clamp(maxViewAngleDeg, 0.0, 180.0);

    mode_ = enabled ? mode : ViewDirectionFilter::None;
    if (mode_ == ViewDirectionFilter::MaxAngle && maxAngle >= 180.0)
    {
      mode_ = ViewDirectionFilter::None;
    }
    // cos() decreases monotonically on [0, 180] deg, so "angle > max" is "dot < cos(max)":
    cosThreshold_ = static_cast<float>(std::cos(mrpt::DEG2RAD(maxAngle)));
  }

  [[nodiscard]] ViewDirectionFilter mode() const { return mode_; }

  /// Whether the test can reject anything at all.
  [[nodiscard]] bool active() const { return mode_ != ViewDirectionFilter::None; }

  /// Whether rejects() needs the covariance of the matched map point.
  [[nodiscard]] bool needsCovariance() const { return mode_ == ViewDirectionFilter::SurfaceSide; }

  /** True if the pair must be rejected. `covMap` is only read in SurfaceSide
   *  mode, and so is `isFlat`: a callable returning whether the raw (not
   *  regularized) neighborhood of the matched map point is flat, invoked only
   *  for a pair the side test would otherwise reject. A zero (missing)
   *  direction on either side never rejects.
   *
   *  The flatness check is needed because the stored covariances are
   *  plane-regularized: every one that is not the isotropic fallback looks
   *  like a plane, including those of foliage, edges or poles, whose "normal"
   *  says nothing about which side a view comes from. */
  template <class IsFlatFn>
  [[nodiscard]] bool rejects(
      const Eigen::Vector3f& vQuery, const Eigen::Vector3f& vMap, const Eigen::Matrix3f* covMap,
      IsFlatFn&& isFlat) const
  {
    constexpr float MIN_SQR_NORM = 0.25f;  // unit vectors when present
    if (!active() || vQuery.squaredNorm() < MIN_SQR_NORM || vMap.squaredNorm() < MIN_SQR_NORM)
    {
      return false;
    }

    if (mode_ == ViewDirectionFilter::MaxAngle)
    {
      // Written out rather than Eigen's dot(), to keep the exact float
      // evaluation order the map classes used before this was shared:
      const float dot = vQuery.x() * vMap.x() + vQuery.y() * vMap.y() + vQuery.z() * vMap.z();
      return dot < cosThreshold_;
    }

    return covMap != nullptr && seenFromOppositeSides(*covMap, vQuery, vMap) && isFlat();
  }

  /// Overload for the modes that need neither covariance nor flatness.
  [[nodiscard]] bool rejects(const Eigen::Vector3f& vQuery, const Eigen::Vector3f& vMap) const
  {
    return rejects(vQuery, vMap, nullptr, [] { return true; });
  }

  /** True if two view directions see the surface of a point with covariance
   *  `cov` from opposite sides, both clearly (not near grazing incidence). A
   *  covariance without a clear normal (not plane-shaped) never qualifies. */
  [[nodiscard]] static bool seenFromOppositeSides(
      const Eigen::Matrix3f& cov, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
  {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
    es.computeDirect(cov);
    const Eigen::Vector3f& ev = es.eigenvalues();  // ascending
    if (!(ev(0) < 0.1f * ev(1)))
    {
      return false;
    }

    const Eigen::Vector3f n  = es.eigenvectors().col(0);
    const float           s1 = v1.dot(n);
    const float           s2 = v2.dot(n);

    // |v.n| below this is a view within ~14.5 deg of the surface plane, where
    // an error in the normal can flip the sign. A laxer 0.1 (~6 deg) was
    // measured to destabilize some hand-held runs:
    constexpr float MIN_ABS_COS = 0.25f;
    return std::abs(s1) > MIN_ABS_COS && std::abs(s2) > MIN_ABS_COS && (s1 > 0) != (s2 > 0);
  }

  /** Whether the `n` points given by `pointAt(i)` (an Eigen::Vector3d each)
   *  lie close to a plane: smallest scatter eigenvalue under a tenth of the
   *  middle one. Fewer than 3 points never qualify. */
  template <class PointAtFn>
  [[nodiscard]] static bool neighborhoodIsFlat(std::size_t n, PointAtFn&& pointAt)
  {
    if (n < 3)
    {
      return false;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < n; i++)
    {
      mean += pointAt(i);
    }
    mean /= static_cast<double>(n);

    Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
    for (std::size_t i = 0; i < n; i++)
    {
      const Eigen::Vector3d d = pointAt(i) - mean;
      scatter += d * d.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(scatter);
    return es.eigenvalues()(0) < 0.1 * es.eigenvalues()(1);
  }

 private:
  ViewDirectionFilter mode_         = ViewDirectionFilter::None;
  float               cosThreshold_ = -1.0f;
};

}  // namespace mola::internal
