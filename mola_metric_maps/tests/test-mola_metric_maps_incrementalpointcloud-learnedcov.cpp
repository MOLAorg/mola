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
 * @file   test-mola_metric_maps_incrementalpointcloud-learnedcov.cpp
 * @brief  Learned per-point covariances of cov-to-cov pairings
 *         (TCreationOptions::learned_cov_model_file)
 * @author Jose Luis Blanco Claraco
 * @date   Nov 8, 2026
 *
 * The model files are written here: a table only (zero MLP weights), so that
 * the expected covariances are known in closed form.
 */

#include <mola_metric_maps/IncrementalPointCloud.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/filesystem.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace
{
constexpr int N_CELLS  = 36;
constexpr int N_RNG    = 3;
constexpr int N_INPUTS = 13;
constexpr int HIDDEN   = 32;

auto& rng = mrpt::random::getRandomGenerator();

/** Writes a table-only model. Every cell gets the shape of the shipped plane
 *  regularization, (lambda, 1, 1) with lambda = 1e-3, and the log-scale of
 *  `logScale(side, cell)`; the last cell index (N_CELLS) is the isotropic
 *  fallback. */
void writeModel(const std::string& path, const std::function<double(int, int)>& logScale)
{
  // a1 = -softplus(p1) ~ 0, a0 = a1 - softplus(p0) = log(1e-3):
  const double p0 = std::log(std::expm1(-std::log(1e-3)));
  const double p1 = -40.0;

  std::ofstream f(path);
  f << "mola_learned_point_cov 1\ninputs " << N_INPUTS << "\nhidden " << HIDDEN
    << "\nneighbors 20\nradius 2.0\nmin_neighbors 5\n";
  const auto zeros = [&](const char* name, int n)
  {
    f << name;
    for (int i = 0; i < n; i++)
    {
      f << " 0";
    }
    f << "\n";
  };
  for (int side = 0; side < 2; side++)
  {
    f << "head " << (side == 0 ? "map" : "query") << "\nshape";
    for (int c = 0; c < N_CELLS; c++)
    {
      f << " " << p0 << " " << p1;
    }
    f << "\nlogscale";
    for (int c = 0; c <= N_CELLS; c++)
    {
      f << " " << logScale(side, c);
    }
    f << "\n";
    zeros("W0", HIDDEN * N_INPUTS);
    zeros("b0", HIDDEN);
    zeros("W1", HIDDEN * HIDDEN);
    zeros("b1", HIDDEN);
    zeros("W2", 3 * HIDDEN);
    zeros("b2", 3);
  }
  f << "end\n";
}

/// A noisy plane z = -1 around the origin, which is taken as the sensor.
mrpt::maps::CSimplePointsMap::Ptr surface()
{
  auto pts = mrpt::maps::CSimplePointsMap::Create();
  for (int ix = -40; ix <= 40; ix++)
  {
    for (int iy = -40; iy <= 40; iy++)
    {
      pts->insertPointFast(
          static_cast<float>(ix * 0.3), static_cast<float>(iy * 0.3),
          static_cast<float>(-1.0 + rng.drawGaussian1D(0, 0.005)));
    }
  }
  pts->mark_as_modified();
  return pts;
}

void fill(mola::IncrementalPointCloud& map, const mrpt::maps::CSimplePointsMap& pts)
{
  // The training neighborhood, so that the shipped-equivalent model below
  // sees the same neighbors as the plane regularization:
  map.creationOptions.k_correspondences_for_cov   = 20;
  map.creationOptions.min_correspondences_for_cov = 5;
  map.creationOptions.max_distance_for_cov        = 2.0;
  for (size_t i = 0; i < pts.size(); i++)
  {
    float x = 0;
    float y = 0;
    float z = 0;
    pts.getPointFast(i, x, y, z);
    map.insertPoint(x, y, z);
  }
}

/// Pairs a copy of `pts` against itself, with the given model and variance scale.
mp2p_icp::MatchedPointWithCovList pairings(
    const mrpt::maps::CSimplePointsMap& pts, const std::string& model, double varianceScale = 1.0)
{
  mola::IncrementalPointCloud global;
  mola::IncrementalPointCloud local;
  fill(global, pts);
  fill(local, pts);
  global.creationOptions.learned_cov_model_file     = model;
  global.creationOptions.learned_cov_variance_scale = varianceScale;

  mp2p_icp::MatchedPointWithCovList out;
  global.nn_search_cov2cov(local, mrpt::poses::CPose3D::Identity(), 0.1f, out);
  ASSERT_EQUAL_(out.size(), pts.size());
  return out;
}

double relDiff(const mrpt::math::CMatrixFloat33& a, const mrpt::math::CMatrixFloat33& b)
{
  return (a.asEigen() - b.asEigen()).norm() / b.asEigen().norm();
}

/** The temporary files of the test, removed when the program ends. */
struct TempFiles
{
  TempFiles() = default;
  ~TempFiles()
  {
    for (const auto& f : files)
    {
      std::remove(f.c_str());
    }
  }
  TempFiles(const TempFiles&)            = delete;
  TempFiles& operator=(const TempFiles&) = delete;

  std::vector<std::string> files;
};
TempFiles tempFiles;

std::string tmpFile(const std::string& name)
{
  // getTempFileName() may create the file it names:
  const std::string base = mrpt::system::getTempFileName();
  tempFiles.files.push_back(base);
  tempFiles.files.push_back(base + "_" + name);
  return tempFiles.files.back();
}

/** Whether pairings() with this model file fails with an error that has the
 *  text `what`. */
bool failsWith(
    const mrpt::maps::CSimplePointsMap& pts, const std::string& path, const std::string& what);

// -------------------------------------------------------------------------
void test_shipped_equivalent_model()
{
  // Unit scale everywhere: the model reproduces the plane regularization.
  const auto path = tmpFile("shipped.txt");
  writeModel(path, [](int, int) { return 0.0; });

  const auto pts     = surface();
  const auto shipped = pairings(*pts, "");
  const auto learned = pairings(*pts, path);

  double worst = 0;
  for (size_t i = 0; i < shipped.size(); i++)
  {
    ASSERT_EQUAL_(shipped[i].local_idx, learned[i].local_idx);
    ASSERT_EQUAL_(shipped[i].global_idx, learned[i].global_idx);
    const double diff = relDiff(learned[i].cov_inv, shipped[i].cov_inv);
    ASSERT_(std::isfinite(diff));
    worst = std::max(worst, diff);
  }
  std::cout << "shipped-equivalent model, worst relative difference: " << worst << "\n";
  // Single precision, on matrices with a condition number of about 1e3:
  ASSERT_LT_(worst, 5e-3);
}

// -------------------------------------------------------------------------
void test_range_bins_and_variance_scale()
{
  // Query side scaled by range bin (edges at 5 and 15 m from the sensor, here
  // the local frame origin, since the cloud has no view directions); the map
  // side made negligible, so the pairing covariance is the query one.
  const auto       unitPath        = tmpFile("unit.txt");
  const auto       binnedPath      = tmpFile("binned.txt");
  constexpr double kMapLogScale    = -30.0;
  const double     binScale[N_RNG] = {1.0, 4.0, 16.0};
  writeModel(unitPath, [&](int side, int) { return side == 0 ? kMapLogScale : 0.0; });
  writeModel(
      binnedPath,
      [&](int side, int cell)
      {
        if (side == 0)
        {
          return kMapLogScale;
        }
        return cell == N_CELLS ? 0.0 : std::log(binScale[cell % N_RNG]);
      });

  const auto pts    = surface();
  const auto unit   = pairings(*pts, unitPath);
  const auto binned = pairings(*pts, binnedPath);
  const auto scaled = pairings(*pts, unitPath, 2.5);

  std::size_t perBin[N_RNG] = {0, 0, 0};
  for (size_t i = 0; i < unit.size(); i++)
  {
    const auto&  p     = unit[i].local;
    const double range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    const int    bin   = range >= 15.0 ? 2 : (range >= 5.0 ? 1 : 0);
    // Stay clear of the bin edges, where float rounding may pick either side:
    if (std::abs(range - 5.0) < 1e-3 || std::abs(range - 15.0) < 1e-3)
    {
      continue;
    }
    perBin[bin]++;

    auto expected = unit[i].cov_inv;
    expected.asEigen() /= static_cast<float>(binScale[bin]);
    ASSERT_LT_(relDiff(binned[i].cov_inv, expected), 1e-4);

    auto expectedScaled = unit[i].cov_inv;
    expectedScaled.asEigen() /= 2.5f;
    ASSERT_LT_(relDiff(scaled[i].cov_inv, expectedScaled), 1e-5);
  }
  for (int b = 0; b < N_RNG; b++)
  {
    ASSERT_GT_(perBin[b], 100U);
  }
}

// -------------------------------------------------------------------------
void test_bad_model_files()
{
  const auto pts = surface();

  ASSERT_(
      failsWith(*pts, tmpFile("does-not-exist.txt"), "Cannot open learned point covariance model"));

  const auto path = tmpFile("bad.txt");
  {
    std::ofstream f(path);
    f << "mola_learned_point_cov 1\ninputs 13\nhidden 16\n";
    f.flush();
    ASSERT_(f.good());
  }
  ASSERT_(failsWith(*pts, path, "hidden units"));
}

bool failsWith(
    const mrpt::maps::CSimplePointsMap& pts, const std::string& path, const std::string& what)
{
  try
  {
    pairings(pts, path);
  }
  catch (const std::exception& e)
  {
    return mrpt::exception_to_str(e).find(what) != std::string::npos;
  }
  return false;
}

}  // namespace

int main()
{
  try
  {
    rng.randomize(1234U);
    test_shipped_equivalent_model();
    test_range_bins_and_variance_scale();
    test_bad_model_files();
    std::cout << "All tests passed.\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
