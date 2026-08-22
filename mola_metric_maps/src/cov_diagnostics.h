/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   cov_diagnostics.h
 * @brief  Optional, env-gated instrumentation of cov-to-cov pairing weights.
 *
 * The cov-to-cov pairing weight is `(C_global + C_local)^-1`. Either side
 * falls back to an isotropic identity when fewer than
 * `min_correspondences_for_cov` neighbors are found inside
 * `max_distance_for_cov`.
 *
 * That fallback does not discard the pairing. It leaves it in the sum as a
 * nearly isotropic point-to-point constraint, because the inverse of
 * `U*diag(1,1,1e-3)*U^T + I` is close to a multiple of the identity, so the
 * plane constraint the cov-to-cov form is meant to supply is simply absent
 * for that pairing. Nothing upstream can observe it happening, hence this.
 *
 * Enabled only if MOLA_MM_COV_DIAG_FILE names a writable path. When it is
 * unset nothing here is computed and nothing is allocated.
 */
#pragma once

#include <mp2p_icp/Pairings.h>
#include <mrpt/math/CMatrixFixed.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <ostream>
#include <vector>

namespace mola::cov_diag
{
/** Center the per-point covariance on the neighborhood MEAN instead of on the
 *  query point itself, which is what the reference GICP implementations do
 *  (see DLIO's nano_gicp.cc, `neighbors.colwise() -= neighbors.rowwise()
 *  .mean()`). MOLA switched to the query point in 2025-09, incidentally,
 *  inside a commit about debug viz and a race condition, with no rationale
 *  recorded and no measurement attached.
 *
 *  It matters because the eigenvalues are discarded and replaced by
 *  (1, 1, 1e-3) immediately afterwards, so only the eigenvECTORS survive, and
 *  centering on a point that is itself off the surface rotates them. With
 *  DecimateMethod::FirstPoint the query point is a single raw range return, so
 *  that offset is a full noise sample on every point.
 *
 *  Off by default, so the shipped path is unchanged. Set
 *  MOLA_MM_COV_CENTER_ON_MEAN=1 to enable.
 */
inline bool centerCovarianceOnMean()
{
  static const bool s_on = []()
  {
    const char* v = ::getenv("MOLA_MM_COV_CENTER_ON_MEAN");
    return v != nullptr && v[0] == '1';
  }();
  return s_on;
}

/** Shared output stream, or nullptr when the feature is off. */
inline std::ostream* stream()
{
  static std::unique_ptr<std::ofstream> s_file = []() -> std::unique_ptr<std::ofstream>
  {
    const char* path = ::getenv("MOLA_MM_COV_DIAG_FILE");
    if (path == nullptr)
    {
      return nullptr;
    }
    auto f = std::make_unique<std::ofstream>(path, std::ios::out | std::ios::app);
    if (!f->is_open())
    {
      return nullptr;
    }
    *f << "# map\tcall\tnPairings\tnLocalIdentity\tfracLocalIdentity\tnGlobalIdentity"
          "\tfracGlobalIdentity\taniso_p10\taniso_p50\taniso_p90\tfracAnisoBelow10\n";
    return f;
  }();
  return s_file ? s_file.get() : nullptr;
}

/** Ratio of largest to smallest eigenvalue of a symmetric 3x3 weight matrix.
 *  Around 1000 for a well-conditioned plane pairing whose two sides agree on
 *  the normal, and around 2 once either side has degenerated.
 */
inline double anisotropy(const mrpt::math::CMatrixFloat33& covInv)
{
  const Eigen::Matrix3d                          m = covInv.asEigen().cast<double>();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(m, Eigen::EigenvaluesOnly);
  const auto                                     ev = es.eigenvalues();
  const double                                   lo = std::max(ev.minCoeff(), 1e-12);
  return ev.maxCoeff() / lo;
}

/** True for a covariance that fell back to the isotropic identity. A rotation
 *  leaves the identity unchanged, so this is recognizable in either frame.
 */
inline bool isIsotropicFallback(const mrpt::math::CMatrixFloat33& c)
{
  return c.asEigen().isIdentity(1e-6f);
}

/** Writes one summary row for the pairings appended by one matching call.
 *  `localCov` and `globalCov` are callables mapping a pairing index to the
 *  covariance that side contributed, so both map classes can share this.
 */
template <typename LocalCovFn, typename GlobalCovFn>
void dump(
    std::ostream& os, const char* mapClass, const mp2p_icp::MatchedPointWithCovList& pairings,
    std::size_t firstNew, LocalCovFn&& localCov, GlobalCovFn&& globalCov)
{
  static uint64_t s_call = 0;

  const std::size_t n = pairings.size() - firstNew;
  if (n == 0)
  {
    return;
  }

  std::size_t         nLocalId  = 0;
  std::size_t         nGlobalId = 0;
  std::vector<double> aniso;
  aniso.reserve(n);

  for (std::size_t i = firstNew; i < pairings.size(); i++)
  {
    const auto& p = pairings[i];
    if (isIsotropicFallback(localCov(p)))
    {
      nLocalId++;
    }
    if (isIsotropicFallback(globalCov(p)))
    {
      nGlobalId++;
    }
    aniso.push_back(anisotropy(p.cov_inv));
  }

  std::sort(aniso.begin(), aniso.end());
  const auto quant = [&](double q)
  { return aniso[static_cast<std::size_t>(q * (aniso.size() - 1))]; };
  const std::size_t nBelow10 =
      static_cast<std::size_t>(std::lower_bound(aniso.begin(), aniso.end(), 10.0) - aniso.begin());

  const double dn = static_cast<double>(n);
  os << mapClass << '\t' << s_call++ << '\t' << n << '\t' << nLocalId << '\t'
     << static_cast<double>(nLocalId) / dn << '\t' << nGlobalId << '\t'
     << static_cast<double>(nGlobalId) / dn << '\t' << quant(0.10) << '\t' << quant(0.50) << '\t'
     << quant(0.90) << '\t' << static_cast<double>(nBelow10) / dn << '\n';
}
}  // namespace mola::cov_diag
