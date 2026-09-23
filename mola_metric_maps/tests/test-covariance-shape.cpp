/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * ------------------------------------------------------------------------- */

/** Both regimes of the per-point covariance shaping.
 *
 * The regularized one must keep asserting exactly what it used to, since it is
 * the shipped behavior; the unregularized one must preserve the ordering of
 * the eigenvalues the neighborhood produced, which is the whole point of it.
 */

#include <mrpt/core/exceptions.h>

#include <cmath>
#include <iostream>

#include "../src/covariance_shape.h"

namespace
{
/** A flat-ish neighborhood: wide in two directions, thin in the third. */
Eigen::Matrix3d makeScatter(double a, double b, double c)
{
  // An arbitrary rotation, so the test does not accidentally work only on
  // axis-aligned input.
  const Eigen::Matrix3d R = (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()))
                                .toRotationMatrix();
  return R * Eigen::Vector3d(a, b, c).asDiagonal() * R.transpose();
}

Eigen::Vector3d sortedEigenvalues(const Eigen::Matrix3d& m)
{
  Eigen::Vector3d v = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(m).eigenvalues();
  std::sort(v.data(), v.data() + 3, std::greater<double>());
  return v;
}
}  // namespace

int main()
{
  try
  {
    using mola::internal::shapePointCovariance;

    const Eigen::Matrix3d scatter = makeScatter(4.0, 2.0, 0.05);

    // 1) Regularized: the eigenvalues are replaced by (1, 1, lambda),
    //    whatever the neighborhood actually looked like.
    {
      const double          lambda = 1e-3;
      const Eigen::Vector3d ev     = sortedEigenvalues(shapePointCovariance(scatter, lambda));

      ASSERT_NEAR_(ev[0], 1.0, 1e-9);
      ASSERT_NEAR_(ev[1], 1.0, 1e-9);
      ASSERT_NEAR_(ev[2], lambda, 1e-9);
    }

    // A second, very different neighborhood must come out identical: this is
    // the property that makes every pairing carry the same information.
    {
      const Eigen::Vector3d a = sortedEigenvalues(shapePointCovariance(scatter, 1e-3));
      const Eigen::Vector3d b =
          sortedEigenvalues(shapePointCovariance(makeScatter(0.02, 0.01, 1e-4), 1e-3));
      ASSERT_NEAR_((a - b).norm(), 0.0, 1e-9);
    }

    // 2) Unregularized: the eigenvalues found are kept, so a broad
    //    neighborhood stays broad and carries less information.
    {
      const Eigen::Vector3d ev = sortedEigenvalues(shapePointCovariance(scatter, 0.0));

      ASSERT_NEAR_(ev[0], 4.0, 1e-6);
      ASSERT_NEAR_(ev[1], 2.0, 1e-6);
      ASSERT_NEAR_(ev[2], 0.05, 1e-6);
    }

    // 3) The floor applies to the smaller eigenvalues, relative to the
    //    largest, so a degenerate neighborhood stays invertible.
    {
      const Eigen::Matrix3d thin = makeScatter(4.0, 2.0, 0.0);
      const Eigen::Vector3d ev   = sortedEigenvalues(shapePointCovariance(thin, -0.1));

      ASSERT_NEAR_(ev[0], 4.0, 1e-6);
      ASSERT_NEAR_(ev[2], 0.4, 1e-6);  // 0.1 * 4.0

      // The result must stay positive semi-definite even when the
      // neighborhood is exactly flat:
      ASSERT_GT_(ev[2], 0.0);
    }

    // A bound beyond -1 saturates at an isotropic covariance, never above
    // the largest eigenvalue.
    {
      const Eigen::Vector3d ev =
          sortedEigenvalues(shapePointCovariance(makeScatter(4.0, 2.0, 0.0), -2.0));

      ASSERT_NEAR_(ev[0], 4.0, 1e-6);
      ASSERT_NEAR_(ev[1], 4.0, 1e-6);
      ASSERT_NEAR_(ev[2], 4.0, 1e-6);
    }

    // 4) An all-zero neighborhood carries no shape at all, and must fall back
    //    to the isotropic covariance rather than produce a singular one.
    {
      const Eigen::Vector3d ev =
          sortedEigenvalues(shapePointCovariance(Eigen::Matrix3d::Zero(), 0.0));
      ASSERT_NEAR_(ev[0], 1.0, 1e-9);
      ASSERT_NEAR_(ev[2], 1.0, 1e-9);
    }

    std::cout << "covariance shaping: both regimes OK\n";
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
  return 0;
}
