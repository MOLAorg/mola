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
 * @file   learned_point_cov.h
 * @brief  Learned per-point covariances for cov-to-cov (GICP) pairings
 * @author Jose Luis Blanco Claraco
 * @date   Nov 8, 2026
 */
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mola::internal
{
/** Per-point neighborhood statistics a learned covariance model reads.
 *  Everything here depends on the point and its own cloud only; the beam
 *  direction and range of the pairing are supplied at evaluation time.
 */
struct LearnedPointStats
{
  /// Number of static (ray-independent) inputs.
  static constexpr int N_STATIC = 11;

  /// Static inputs, already normalized as the model expects them.
  std::array<float, N_STATIC> feat{};

  /// Eigenvectors (columns, ascending eigenvalues) of the second moment of
  /// the covariance neighborhood about the point. Identity when `ok` is false.
  Eigen::Matrix3f U = Eigen::Matrix3f::Identity();

  /// Geometry class from the 20 nearest neighbors: 0 linear, 1 planar,
  /// 2 scattered.
  uint8_t geomClass = 1;

  /// Whether these statistics were computed for the current point of the slot.
  bool computed = false;

  /// Whether the neighborhood was populated enough to have a shape; if not,
  /// the covariance is isotropic.
  bool ok = false;
};

/** Scan-level inputs of a learned covariance model: a few numbers describing
 *  the query cloud as a whole, the same for all its pairings (used by models
 *  with `scan_inputs > 0`). Computed once per query cloud.
 */
struct LearnedScanContext
{
  static constexpr int N_FEAT = 6;

  /// Already normalized as the model expects them.
  std::array<float, N_FEAT> feat{};
};

/** Scan-level inputs from the range of every point of the query cloud and the
 *  unit normals (smallest-eigenvalue direction) of those with a shape: log
 *  median range, fraction of points within 5 m, log point count, 3D and
 *  horizontal degeneracy of the normals' second moment, and the fraction of
 *  near-vertical normals. Zeros if there are fewer than 10 normals.
 */
LearnedScanContext computeLearnedScanContext(
    const std::vector<float>& ranges, const std::vector<Eigen::Vector3f>& normals);

/** Neighbors of one point, sorted by increasing distance, the point itself
 *  included (at distance 0). */
struct NeighborList
{
  const float*    x      = nullptr;  //!< coordinate arrays of the whole cloud
  const float*    y      = nullptr;
  const float*    z      = nullptr;
  const uint32_t* idx    = nullptr;  //!< neighbor indices into x/y/z
  const float*    distSq = nullptr;  //!< squared distances, ascending
  std::size_t     count  = 0;
};

/** Fills the statistics of the point at `q` from its nearest neighbors (up to
 *  40, no distance limit). The covariance neighborhood is the first
 *  `k` of them within `radius`; `minNeighbors` is the count below which the
 *  point gets no shape.
 */
LearnedPointStats computeLearnedPointStats(
    const Eigen::Vector3d& q, const NeighborList& nb, std::size_t k, double radius,
    std::size_t minNeighbors);

/** A learned per-point covariance model: a lookup table indexed by (geometry
 *  class, incidence bin, range bin), plus a small MLP that corrects each
 *  cell's parameters from continuous per-point inputs. One table and one MLP
 *  per side of a pairing (map point, query point).
 *
 *  The covariance of one side is `exp(s) * U * diag(exp(a0), exp(a1), 1) * U^T`
 *  with `a0 <= a1 <= 0`, i.e. the same convention as the shipped plane
 *  regularization `U * diag(lambda, 1, 1) * U^T`, plus a per-point scale.
 */
class LearnedPointCovModel
{
 public:
  enum Side : int
  {
    MAP   = 0,
    QUERY = 1
  };

  /** Loads a model from its text file (see the exporter in the training
   *  code). Throws on a malformed or incompatible file. */
  static std::shared_ptr<const LearnedPointCovModel> FromFile(const std::string& path);

  /** Covariance of one side, in the frame of `st.U`.
   *  \param rayDir Unit beam direction of the query point, in that same
   *         frame (its sign is irrelevant).
   *  \param range  Distance from the sensor to the query point [m].
   */
  [[nodiscard]] Eigen::Matrix3f sideCovariance(
      Side side, const LearnedPointStats& st, const LearnedScanContext& ctx,
      const Eigen::Vector3f& rayDir, float range) const;

  /// Number of scan-level inputs the model reads (0 for point-only models).
  [[nodiscard]] int numScanInputs() const { return numScan_; }

  /// The covariance neighborhood the model was trained with: the first
  /// `neighbors` nearest points within `radius` [m], and the count below
  /// which a point has no shape. Independent of the map's own settings.
  [[nodiscard]] std::size_t neighbors() const { return neighbors_; }
  [[nodiscard]] double      radius() const { return radius_; }
  [[nodiscard]] std::size_t minNeighbors() const { return minNeighbors_; }

  /// Point inputs, and the most inputs a model can have (point + scan-level).
  static constexpr int N_POINT_INPUTS = LearnedPointStats::N_STATIC + 2;
  static constexpr int N_INPUTS       = N_POINT_INPUTS + LearnedScanContext::N_FEAT;
  static constexpr int N_CLS          = 3;
  static constexpr int N_INC          = 4;
  static constexpr int N_RNG          = 3;
  static constexpr int N_CELLS        = N_CLS * N_INC * N_RNG;
  static constexpr int HIDDEN         = 32;

 private:
  struct Head
  {
    /// Table: free shape parameters per cell, and log-scale per cell plus
    /// one last entry for the isotropic fallback.
    Eigen::Matrix<float, N_CELLS, 2>     shape;
    Eigen::Matrix<float, N_CELLS + 1, 1> logScale;

    // Fixed sizes, so that the evaluation is vectorized and allocation-free.
    Eigen::Matrix<float, HIDDEN, N_INPUTS> W0;
    Eigen::Matrix<float, HIDDEN, 1>        b0;
    Eigen::Matrix<float, HIDDEN, HIDDEN>   W1;
    Eigen::Matrix<float, HIDDEN, 1>        b1;
    Eigen::Matrix<float, 3, HIDDEN>        W2;
    Eigen::Matrix<float, 3, 1>             b2;
  };

  std::array<Head, 2> heads_;
  int                 numScan_      = 0;  //!< scan-level inputs actually used
  bool                bounded_      = false;  //!< squashed outputs (see the .cpp)
  std::size_t         neighbors_    = 20;
  double              radius_       = 2.0;
  std::size_t         minNeighbors_ = 5;
};

}  // namespace mola::internal
