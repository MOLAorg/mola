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
 * @file   learned_point_cov.cpp
 * @brief  Learned per-point covariances for cov-to-cov (GICP) pairings
 * @author Jose Luis Blanco Claraco
 * @date   Nov 8, 2026
 */

#include "learned_point_cov.h"

#include <mrpt/core/exceptions.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

namespace mola::internal
{
namespace
{
constexpr std::array<std::size_t, 3> EIGEN_SCALES = {10, 20, 40};

// Bin edges of the table: incidence angle [deg] and range [m].
constexpr std::array<double, 3> INC_EDGES_DEG = {30.0, 60.0, 75.0};
constexpr std::array<double, 2> RANGE_EDGES   = {5.0, 15.0};

constexpr double NaN     = std::numeric_limits<double>::quiet_NaN();
constexpr double RAD2DEG = 57.29577951308232;

/// Non-finite inputs carry no information: they enter the model as zero.
float finiteOrZero(double v) { return std::isfinite(v) ? static_cast<float>(v) : 0.0f; }

float softplus(float x) { return std::log1p(std::exp(-std::abs(x))) + std::max(x, 0.0f); }

/// x * sigmoid(x), elementwise, in vectorized form.
template <typename VEC>
VEC silu(const VEC& x)
{
  return (x.array() / (1.0f + (-x.array()).exp())).matrix();
}

/// Number of edges at or below `v`, i.e. the bin index of `v`.
template <std::size_t N>
int binOf(double v, const std::array<double, N>& edges)
{
  int b = 0;
  for (const double e : edges)
  {
    if (v >= e)
    {
      b++;
    }
  }
  return b;
}

/// Ascending eigenvalues of the covariance (about their mean) of the first `n`
/// neighbors, or NaN if there are not that many.
Eigen::Vector3d scaleEigenvalues(const NeighborList& nb, std::size_t n)
{
  if (nb.count < n)
  {
    return Eigen::Vector3d::Constant(NaN);
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (std::size_t j = 0; j < n; j++)
  {
    const uint32_t i = nb.idx[j];
    mean += Eigen::Vector3d(nb.x[i], nb.y[i], nb.z[i]);
  }
  mean /= static_cast<double>(n);

  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (std::size_t j = 0; j < n; j++)
  {
    const uint32_t        i = nb.idx[j];
    const Eigen::Vector3d d = Eigen::Vector3d(nb.x[i], nb.y[i], nb.z[i]) - mean;
    C.noalias() += d * d.transpose();
  }
  C /= static_cast<double>(n);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
  es.computeDirect(C, Eigen::EigenvaluesOnly);
  return es.eigenvalues();
}

/// Reads one keyword and checks it is the expected one.
void expectToken(std::istream& in, const std::string& expected, const std::string& path)
{
  std::string tok;
  in >> tok;
  if (!in || tok != expected)
  {
    THROW_EXCEPTION_FMT(
        "Learned point covariance model '%s': expected '%s', found '%s'", path.c_str(),
        expected.c_str(), tok.c_str());
  }
}

template <typename MATRIX>
void readMatrix(std::istream& in, const std::string& name, MATRIX& M, const std::string& path)
{
  expectToken(in, name, path);
  for (Eigen::Index r = 0; r < M.rows(); r++)
  {
    for (Eigen::Index c = 0; c < M.cols(); c++)
    {
      in >> M(r, c);
    }
  }
  if (!in || !M.allFinite())
  {
    THROW_EXCEPTION_FMT(
        "Learned point covariance model '%s': bad values in '%s'", path.c_str(), name.c_str());
  }
}

}  // namespace

LearnedPointStats computeLearnedPointStats(
    const Eigen::Vector3d& q, const NeighborList& nb, std::size_t k, double radius,
    std::size_t minNeighbors)
{
  LearnedPointStats st;

  // The covariance neighborhood: the first k neighbors within the radius.
  const double radiusSq = radius * radius;
  std::size_t  n        = 0;
  while (n < std::min(k, nb.count) && static_cast<double>(nb.distSq[n]) <= radiusSq)
  {
    n++;
  }

  Eigen::Matrix3d M     = Eigen::Matrix3d::Zero();
  double          dSum  = 0;
  std::size_t     nDist = 0;
  for (std::size_t j = 0; j < n; j++)
  {
    const uint32_t        i = nb.idx[j];
    const Eigen::Vector3d d = Eigen::Vector3d(nb.x[i], nb.y[i], nb.z[i]) - q;
    M.noalias() += d * d.transpose();
    if (nb.distSq[j] > 0)
    {
      dSum += std::sqrt(static_cast<double>(nb.distSq[j]));
      nDist++;
    }
  }
  M /= static_cast<double>(std::max<std::size_t>(n, 1));

  st.ok = n >= minNeighbors && M.allFinite();
  if (st.ok)
  {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
    es.computeDirect(M);
    st.U = es.eigenvectors().cast<float>();
  }

  // Shape of the neighborhood at three scales, with no distance limit:
  std::size_t f = 0;
  for (const std::size_t sc : EIGEN_SCALES)
  {
    const Eigen::Vector3d ev = scaleEigenvalues(nb, sc);
    const double          e0 = std::max(ev[0], 1e-12);
    const double          e1 = std::max(ev[1], 1e-12);
    const double          e2 = std::max(ev[2], 1e-12);
    // std::max() keeps a NaN first argument, as intended: it becomes a zero input.
    st.feat[f++] = finiteOrZero(std::clamp(std::log(e0 / e2), -14.0, 0.0) / 5 + 1);
    st.feat[f++] = finiteOrZero(std::clamp(std::log(e1 / e2), -14.0, 0.0) / 5 + 1);
    st.feat[f++] = finiteOrZero((std::log(e2) + 4) / 2);

    if (sc == 20)
    {
      const double d0  = (ev[2] - ev[1]) / ev[2];
      const double d1  = (ev[1] - ev[0]) / ev[2];
      const double d2  = ev[0] / ev[2];
      const bool   fin = std::isfinite(d0) && std::isfinite(d1) && std::isfinite(d2);
      if (fin)
      {
        st.geomClass = d0 >= d1 && d0 >= d2 ? 0 : (d1 >= d2 ? 1 : 2);
      }
    }
  }
  st.feat[f++] = finiteOrZero(std::log(static_cast<double>(std::max<std::size_t>(n, 1))) / 3 - 0.8);
  const double dmean = nDist > 0 ? dSum / static_cast<double>(nDist) : NaN;
  st.feat[f++]       = finiteOrZero(std::log(std::max(dmean, 1e-3)) + 2);

  return st;
}

LearnedScanContext computeLearnedScanContext(
    const std::vector<float>& ranges, const std::vector<Eigen::Vector3f>& normals)
{
  LearnedScanContext ctx;
  if (normals.size() < 10 || ranges.empty())
  {
    return ctx;
  }
  std::vector<float> r   = ranges;
  const auto         mid = r.begin() + static_cast<std::ptrdiff_t>(r.size() / 2);
  std::nth_element(r.begin(), mid, r.end());
  double median = *mid;
  if (r.size() % 2 == 0)
  {
    // The mean of the two central values, as the reference implementation:
    median = 0.5 * (median + *std::max_element(r.begin(), mid));
  }
  std::size_t near = 0;
  for (const float v : ranges)
  {
    near += v < 5.0f ? 1 : 0;
  }
  std::size_t     vertical = 0;
  Eigen::Matrix3d M        = Eigen::Matrix3d::Zero();
  for (const auto& n : normals)
  {
    M.noalias() += (n.cast<double>() * n.cast<double>().transpose());
    vertical += std::abs(n.z()) > 0.9f ? 1 : 0;
  }
  M /= static_cast<double>(normals.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es3(M, Eigen::EigenvaluesOnly);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es2(
      M.topLeftCorner<2, 2>(), Eigen::EigenvaluesOnly);
  const Eigen::Vector3d ev  = es3.eigenvalues();
  const Eigen::Vector2d evh = es2.eigenvalues();

  const auto degeneracy = [](double lo, double hi)
  { return std::clamp(std::log(std::max(lo, 1e-9) / std::max(hi, 1e-9)), -8.0, 0.0) / 4 + 1; };

  ctx.feat[0] = static_cast<float>(std::log(std::max(median, 0.1)) - 2.3);
  ctx.feat[1] =
      static_cast<float>(static_cast<double>(near) / static_cast<double>(ranges.size()) * 2 - 1);
  ctx.feat[2] = static_cast<float>((std::log(static_cast<double>(ranges.size())) - 8.0) / 2);
  ctx.feat[3] = static_cast<float>(degeneracy(ev[0], ev[2]));
  ctx.feat[4] = static_cast<float>(degeneracy(evh[0], evh[1]));
  ctx.feat[5] = static_cast<float>(
      static_cast<double>(vertical) / static_cast<double>(normals.size()) * 2 - 1);
  return ctx;
}

std::shared_ptr<const LearnedPointCovModel> LearnedPointCovModel::FromFile(const std::string& path)
{
  std::ifstream in(path);
  if (!in.is_open())
  {
    THROW_EXCEPTION_FMT("Cannot open learned point covariance model '%s'", path.c_str());
  }

  expectToken(in, "mola_learned_point_cov", path);
  int version = 0;
  in >> version;
  if (version != 1 && version != 2)
  {
    THROW_EXCEPTION_FMT(
        "Learned point covariance model '%s': unsupported version %d", path.c_str(), version);
  }
  int nInputs = 0;
  int hidden  = 0;
  expectToken(in, "inputs", path);
  in >> nInputs;
  expectToken(in, "hidden", path);
  in >> hidden;
  if (!in || nInputs != N_POINT_INPUTS || hidden != HIDDEN)
  {
    THROW_EXCEPTION_FMT(
        "Learned point covariance model '%s': expected %d inputs and %d hidden units, found %d "
        "and %d",
        path.c_str(), N_POINT_INPUTS, HIDDEN, nInputs, hidden);
  }

  auto model = std::make_shared<LearnedPointCovModel>();
  if (version == 2)
  {
    int bounded = 0;
    expectToken(in, "scan_inputs", path);
    in >> model->numScan_;
    expectToken(in, "bounded", path);
    in >> bounded;
    if (!in || model->numScan_ < 0 || model->numScan_ > LearnedScanContext::N_FEAT ||
        (bounded != 0 && bounded != 1))
    {
      THROW_EXCEPTION_FMT(
          "Learned point covariance model '%s': bad version 2 header", path.c_str());
    }
    model->bounded_ = bounded == 1;
  }
  expectToken(in, "neighbors", path);
  in >> model->neighbors_;
  expectToken(in, "radius", path);
  in >> model->radius_;
  expectToken(in, "min_neighbors", path);
  in >> model->minNeighbors_;
  if (!in || model->neighbors_ == 0 || model->neighbors_ > 40 || !(model->radius_ > 0))
  {
    THROW_EXCEPTION_FMT(
        "Learned point covariance model '%s': bad neighborhood definition", path.c_str());
  }
  for (const char* sideName : {"map", "query"})
  {
    Head& h = model->heads_[std::string(sideName) == "map" ? MAP : QUERY];
    expectToken(in, "head", path);
    expectToken(in, sideName, path);
    readMatrix(in, "shape", h.shape, path);
    readMatrix(in, "logscale", h.logScale, path);
    // W0 has one column per input actually used; the rest stay zero:
    Eigen::MatrixXf W0(HIDDEN, N_POINT_INPUTS + model->numScan_);
    readMatrix(in, "W0", W0, path);
    h.W0.setZero();
    h.W0.leftCols(W0.cols()) = W0;
    readMatrix(in, "b0", h.b0, path);
    readMatrix(in, "W1", h.W1, path);
    readMatrix(in, "b1", h.b1, path);
    readMatrix(in, "W2", h.W2, path);
    readMatrix(in, "b2", h.b2, path);
  }
  expectToken(in, "end", path);
  return model;
}

Eigen::Matrix3f LearnedPointCovModel::sideCovariance(
    Side side, const LearnedPointStats& st, const LearnedScanContext& ctx,
    const Eigen::Vector3f& rayDir, float range) const
{
  const Head& h = heads_[side];

  const float cosInc = std::abs(st.U.col(0).dot(rayDir));
  const int   inc =
      binOf(std::acos(std::clamp(static_cast<double>(cosInc), 0.0, 1.0)) * RAD2DEG, INC_EDGES_DEG);
  const int rng  = binOf(static_cast<double>(range), RANGE_EDGES);
  const int cell = (st.geomClass * N_INC + inc) * N_RNG + rng;

  Eigen::Matrix<float, N_INPUTS, 1> x = Eigen::Matrix<float, N_INPUTS, 1>::Zero();
  for (int i = 0; i < LearnedPointStats::N_STATIC; i++)
  {
    x[i] = st.feat[i];
  }
  x[LearnedPointStats::N_STATIC]     = cosInc * 2 - 1;
  x[LearnedPointStats::N_STATIC + 1] = std::log(std::max(range, 0.1f)) - 2;
  for (int i = 0; i < numScan_; i++)
  {
    x[N_POINT_INPUTS + i] = ctx.feat[i];
  }

  using Hidden         = Eigen::Matrix<float, HIDDEN, 1>;
  const Hidden    act1 = silu<Hidden>(h.W0 * x + h.b0);
  const Hidden    act2 = silu<Hidden>(h.W1 * act1 + h.b1);
  Eigen::Vector3f out  = h.W2 * act2 + h.b2;
  if (bounded_)
  {
    // Log-scale residual within +-ln 3, each shape term within +-3:
    out[0] = 3.0f * std::tanh(out[0] / 3.0f);
    out[1] = 3.0f * std::tanh(out[1] / 3.0f);
    out[2] = 1.1f * std::tanh(out[2] / 1.1f);
  }

  const float logScale = h.logScale[st.ok ? cell : N_CELLS] + out[2];
  const float scale    = std::exp(logScale);
  if (!st.ok)
  {
    return Eigen::Matrix3f::Identity() * scale;
  }

  const float a1 = -softplus(h.shape(cell, 1) + out[1]);
  const float a0 = a1 - softplus(h.shape(cell, 0) + out[0]);

  const Eigen::Vector3f D(std::exp(a0) * scale, std::exp(a1) * scale, scale);
  return st.U * D.asDiagonal() * st.U.transpose();
}

}  // namespace mola::internal
