/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 * ------------------------------------------------------------------------- */

/**
 * Checks that the C++ learned covariance model (file version 2: scan-level
 * inputs and bounded outputs) gives the same scan-level features and side
 * covariances as the training code, on a fixture written by
 * make_cov_fixture.py (random weights, so every layer and the output bounds
 * matter).
 */

#include <mrpt/core/exceptions.h>

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../src/learned_point_cov.h"

namespace
{
using mola::internal::LearnedPointCovModel;
using mola::internal::LearnedPointStats;
using mola::internal::LearnedScanContext;

bool check(double got, double want, double tol, const std::string& what)
{
  if (!(std::abs(got - want) <= tol * std::max(1.0, std::abs(want))))
  {
    std::cerr << "MISMATCH " << what << ": got " << got << ", expected " << want << "\n";
    return false;
  }
  return true;
}

int run(const std::string& dir)
{
  const auto model = LearnedPointCovModel::FromFile(dir + "/learned_cov_v2.model.txt");
  ASSERT_EQUAL_(model->numScanInputs(), LearnedScanContext::N_FEAT);

  std::ifstream in(dir + "/learned_cov_v2.cases.txt");
  ASSERT_(in.is_open());
  std::vector<float>           ranges;
  std::vector<Eigen::Vector3f> normals;
  LearnedScanContext           ctx;
  int                          bad    = 0;
  int                          nCases = 0;
  bool                         ended  = false;

  std::string tag;
  while (in >> tag)
  {
    if (tag == "ranges")
    {
      std::size_t n = 0;
      in >> n;
      ranges.resize(n);
      for (auto& v : ranges)
      {
        in >> v;
      }
    }
    else if (tag == "normals")
    {
      std::size_t n = 0;
      in >> n;
      normals.resize(n);
      for (auto& v : normals)
      {
        in >> v.x() >> v.y() >> v.z();
      }
    }
    else if (tag == "ctx")
    {
      std::array<double, LearnedScanContext::N_FEAT> want{};
      for (auto& v : want)
      {
        in >> v;
      }
      const auto got = mola::internal::computeLearnedScanContext(ranges, normals);
      for (int i = 0; i < LearnedScanContext::N_FEAT; i++)
      {
        bad += check(got.feat[i], want[i], 1e-4, "scan feature " + std::to_string(i)) ? 0 : 1;
      }
      ctx = got;
    }
    else if (tag == "case")
    {
      int side = 0;
      int cls  = 0;
      int ok   = 0;
      in >> side >> cls >> ok;
      LearnedPointStats st;
      for (auto& v : st.feat)
      {
        in >> v;
      }
      for (int r = 0; r < 3; r++)
      {
        for (int c = 0; c < 3; c++)
        {
          in >> st.U(r, c);
        }
      }
      st.geomClass = static_cast<uint8_t>(cls);
      st.ok        = ok != 0;
      Eigen::Vector3f ray;
      float           range = 0;
      in >> ray.x() >> ray.y() >> ray.z() >> range;
      Eigen::Matrix3f want;
      for (int r = 0; r < 3; r++)
      {
        for (int c = 0; c < 3; c++)
        {
          in >> want(r, c);
        }
      }
      // A truncated record must not reach the model:
      ASSERT_(!in.fail());
      const Eigen::Matrix3f got = model->sideCovariance(
          side == 0 ? LearnedPointCovModel::MAP : LearnedPointCovModel::QUERY, st, ctx, ray, range);
      for (int r = 0; r < 3; r++)
      {
        for (int c = 0; c < 3; c++)
        {
          bad += check(
                     got(r, c), want(r, c), 2e-4,
                     "case " + std::to_string(nCases) + " cov(" + std::to_string(r) + "," +
                         std::to_string(c) + ")")
                     ? 0
                     : 1;
        }
      }
      nCases++;
    }
    else if (tag == "end")
    {
      // Nothing may follow the end marker:
      in >> std::ws;
      ASSERT_(in.eof());
      ended = true;
      break;
    }
    else
    {
      THROW_EXCEPTION_FMT("Unknown tag in the fixture: '%s'", tag.c_str());
    }
    ASSERT_(!in.fail());
  }
  // The whole fixture was read:
  ASSERT_(ended);
  ASSERT_(nCases > 0);
  std::cout << nCases << " cases, " << bad << " mismatches\n";
  return bad == 0 ? 0 : 1;
}
}  // namespace

int main(int, char**)
{
  try
  {
    return run(LEARNED_COV_TEST_DATA_DIR);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 2;
  }
}
