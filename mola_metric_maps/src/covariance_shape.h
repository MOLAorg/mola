/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */

/**
 * @file   covariance_shape.h
 * @brief  How a per-point covariance is shaped before the matcher sees it
 * @author Jose Luis Blanco Claraco
 * @date   Sep 22, 2026
 */
#pragma once

#include <Eigen/Dense>
#include <algorithm>

namespace mola::internal
{
/** Shapes the per-point scatter matrix that becomes a pairing's information.
 *
 * With a positive `lambda` the eigenvalues are replaced by (1, 1, lambda).
 * Every neighborhood then asserts the same confidence along its estimated
 * normal, whether or not the samples support it, and every pairing carries the
 * same total information regardless of how many points were behind it or how
 * spread they were.
 *
 * A non-positive `lambda` keeps the eigenvalues the fit produced, so a sparse
 * or rough neighborhood ends up with a broader covariance, and therefore less
 * weight, than a dense flat one.
 *
 * `|lambda|` means the same thing in both regimes: the smallest eigenvalue as
 * a fraction of the largest. What the sign changes is whether that number is
 * assigned or only bounded. Positive assigns it, so every neighborhood comes
 * out with exactly that ratio. Non-positive bounds it from below, so a
 * neighborhood flatter than the bound is clamped to it and everything else
 * keeps what it had: -0.01 allows up to 100:1, -0.1 up to 10:1, and 0 leaves
 * the ratio alone but for a numerical guard. The bound is what keeps the
 * matrix invertible on a perfectly planar or collinear neighborhood, and it
 * also stops a handful of near-degenerate ones from dominating the solve.
 *
 * Note that the two branches differ in scale as well as in shape: kept
 * eigenvalues carry squared metric units, so the resulting information
 * matrices are not comparable with the regularized ones and anything tuned
 * against their magnitude, such as a matching threshold or a robust kernel,
 * has to be revisited.
 */
inline Eigen::Matrix3d shapePointCovariance(const Eigen::Matrix3d& cov, double lambda)
{
  if (lambda > 0)
  {
    // The shipped form, kept verbatim. SVD sorts the values in decreasing
    // order, so the last one is the normal of a plane.
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3d                   values(1.0, 1.0, lambda);
    return svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
  }

  // Keeping the values found needs the decomposition of a symmetric matrix
  // rather than a general one: on a rank-deficient neighborhood an SVD is free
  // to return opposite signs for the left and right vectors of a null
  // direction, and rebuilding from those two would put a negative value along
  // it. The scatter matrix here is symmetric positive semi-definite by
  // construction, so its eigenvectors are the right tool and the result is
  // positive semi-definite by construction too.
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);

  // This solver sorts ascending, so the largest is last.
  Eigen::Vector3d values  = es.eigenvalues();
  const double    largest = values[2];

  // A neighborhood with no extent carries no usable shape: fall back to the
  // isotropic covariance, as the too-few-neighbors case does.
  if (!(largest > 0))
  {
    return Eigen::Matrix3d::Identity();
  }

  // A bound above one would push the smaller eigenvalues past the largest.
  const double ratio      = std::min(-lambda, 1.0);
  const double floorValue = std::max(ratio * largest, largest * 1e-9);
  values[0]               = std::max(values[0], floorValue);
  values[1]               = std::max(values[1], floorValue);

  return es.eigenvectors() * values.asDiagonal() * es.eigenvectors().transpose();
}
}  // namespace mola::internal
