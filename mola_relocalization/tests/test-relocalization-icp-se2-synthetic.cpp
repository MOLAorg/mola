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
 * @file   test-relocalization-icp-se2-synthetic.cpp
 * @brief  RelocalizationICP_SE2 and select_relocalization_candidate() on
 *         synthetic worlds generated on the fly.
 */

#include <mola_relocalization/relocalization.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/get_env.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <iostream>

namespace
{
const char* icpPipelineYaml = R"###(
class_name: mp2p_icp::ICP
params:
  maxIterations: 60
  minAbsStep_trans: 1e-4
  minAbsStep_rot: 1e-4
solvers:
  - class: mp2p_icp::Solver_Horn
    params:
      ~
matchers:
  - class: mp2p_icp::Matcher_Points_DistanceThreshold
    params:
      threshold: 1.0
      thresholdAngularDeg: 0
      runFromIteration: 0
      runUpToIteration: 20
      pointLayerMatches:
        - {global: "points", local: "points"}
  - class: mp2p_icp::Matcher_Points_DistanceThreshold
    params:
      threshold: 0.25
      thresholdAngularDeg: 0
      runFromIteration: 21
      runUpToIteration: 0
      pointLayerMatches:
        - {global: "points", local: "points"}
quality:
  - class: mp2p_icp::QualityEvaluator_PairedRatio
    params:
     threshold: 0.10
)###";

using Segment = std::pair<mrpt::math::TPoint2D, mrpt::math::TPoint2D>;

// Vertical walls (0 to 2 m tall) along the segments, sampled every 0.1 m:
mrpt::maps::CSimplePointsMap::Ptr make_world(const std::vector<Segment>& walls)
{
  auto pts = mrpt::maps::CSimplePointsMap::Create();
  for (const auto& [a, b] : walls)
  {
    const double len = (b - a).norm();
    const auto   n   = static_cast<size_t>(std::ceil(len / 0.1));
    for (size_t i = 0; i <= n; i++)
    {
      const auto p = a + (b - a) * (static_cast<double>(i) / static_cast<double>(n));
      for (double z = 0; z <= 2.0; z += 0.25)
      {
        pts->insertPoint(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(z));
      }
    }
  }
  return pts;
}

// A box and a few pillars: no symmetry.
std::vector<Segment> unique_room()
{
  std::vector<Segment> w = {{{0, 0}, {30, 0}},   {{30, 0}, {30, 20}}, {{30, 20}, {0, 20}},
                            {{0, 20}, {0, 0}},   {{8, 0}, {8, 6}},    {{8, 6}, {14, 6}},
                            {{20, 14}, {26, 14}}};
  for (const auto& [x, y] : std::vector<mrpt::math::TPoint2D>{{5, 14}, {17, 9}, {23, 4}})
  {
    w.push_back({{x - 0.3, y - 0.3}, {x + 0.3, y - 0.3}});
    w.push_back({{x + 0.3, y - 0.3}, {x + 0.3, y + 0.3}});
    w.push_back({{x + 0.3, y + 0.3}, {x - 0.3, y + 0.3}});
    w.push_back({{x - 0.3, y + 0.3}, {x - 0.3, y - 0.3}});
  }
  return w;
}

// Long parallel rows every `pitch` meters, with no ends in sight:
std::vector<Segment> periodic_rows(double pitch)
{
  std::vector<Segment> w;
  for (int i = -10; i <= 10; i++)
  {
    w.push_back({{-100, i * pitch}, {100, i * pitch}});
  }
  return w;
}

// What a sensor at `pose` sees: world points within `range`, in its frame,
// with some noise.
mp2p_icp::metric_map_t observe(
    const mrpt::maps::CSimplePointsMap& world, const mrpt::poses::CPose3D& pose, double range)
{
  auto&       rng   = mrpt::random::getRandomGenerator();
  auto        local = mrpt::maps::CSimplePointsMap::Create();
  const auto& xs    = world.getPointsBufferRef_x();
  const auto& ys    = world.getPointsBufferRef_y();
  const auto& zs    = world.getPointsBufferRef_z();
  for (size_t i = 0; i < xs.size(); i++)
  {
    const auto l = pose.inverseComposePoint(mrpt::math::TPoint3D(xs[i], ys[i], zs[i]));
    if (mrpt::math::TPoint2D(l.x, l.y).norm() > range)
    {
      continue;
    }
    local->insertPoint(
        static_cast<float>(l.x + rng.drawGaussian1D(0, 0.02)),
        static_cast<float>(l.y + rng.drawGaussian1D(0, 0.02)), static_cast<float>(l.z));
  }
  mp2p_icp::metric_map_t mm;
  mm.layers["points"] = local;
  return mm;
}

mola::RelocalizationICP_SE2::Input make_input(
    const mrpt::maps::CSimplePointsMap::Ptr& world, const mrpt::poses::CPose3D& truePose,
    const mrpt::math::TPose2D& prior, double halfXY, double halfYaw)
{
  mola::RelocalizationICP_SE2::Input in;
  in.reference_map.layers["points"] = world;
  in.local_map                      = observe(*world, truePose, 12.0);

  for (int i = 0; i < 2; i++)
  {
    const auto [icp, params] =
        mp2p_icp::icp_pipeline_from_yaml(mrpt::containers::yaml::FromText(icpPipelineYaml));
    in.icp_pipeline.push_back(icp);
    in.icp_parameters = params;
  }
  in.icp_minimum_quality = 0.3;

  auto& lat          = in.initial_guess_lattice;
  lat.corner_min     = {prior.x - halfXY, prior.y - halfXY, prior.phi - halfYaw};
  lat.corner_max     = {prior.x + halfXY, prior.y + halfXY, prior.phi + halfYaw};
  lat.resolution_xy  = 1.0;
  lat.resolution_phi = mrpt::DEG2RAD(30.0);
  return in;
}

void expect_near(const mrpt::math::TPose3D& found, const mrpt::poses::CPose3D& truth)
{
  const double dXY  = mrpt::math::TPoint2D(found.x - truth.x(), found.y - truth.y()).norm();
  const double dYaw = std::abs(mrpt::math::wrapToPi(found.yaw - truth.yaw()));
  std::cout << "found: " << found << " truth: " << truth.asString() << " dXY=" << dXY
            << " dYaw=" << mrpt::RAD2DEG(dYaw) << " deg\n";
  ASSERT_LT_(dXY, 0.15);
  ASSERT_LT_(dYaw, mrpt::DEG2RAD(2.0));
}

// GNSS-like prior 2.5 m off, known heading (e.g. from an IMU):
void test_xy_search()
{
  const auto world     = make_world(unique_room());
  const auto truePose  = mrpt::poses::CPose3D::FromXYZYawPitchRoll(11.0, 10.0, 0, 0.3, 0, 0);
  const auto prior     = mrpt::math::TPose2D(12.8, 8.3, 0.3);
  const auto in        = make_input(world, truePose, prior, 3.0, 0.0);
  const auto out       = mola::RelocalizationICP_SE2::run(in);
  const auto selection = mola::select_relocalization_candidate(out.candidates);

  std::cout << "[xy] candidates: " << out.candidates.size() << " modes: " << selection.mode_count
            << " time: " << out.time_cost << " s"
            << " best q=" << selection.best->quality
            << " runner-up q=" << (selection.runner_up ? selection.runner_up->quality : 0) << "\n";
  ASSERT_(selection.best);
  ASSERT_(selection.unambiguous);
  expect_near(selection.best->pose, truePose);
}

// No heading at all (no IMU): search the whole turn too.
void test_xy_yaw_search()
{
  const auto world     = make_world(unique_room());
  const auto truePose  = mrpt::poses::CPose3D::FromXYZYawPitchRoll(22.0, 8.0, 0, -2.4, 0, 0);
  const auto prior     = mrpt::math::TPose2D(20.5, 9.5, 0);
  const auto in        = make_input(world, truePose, prior, 2.0, M_PI);
  const auto out       = mola::RelocalizationICP_SE2::run(in);
  const auto selection = mola::select_relocalization_candidate(out.candidates);

  std::cout << "[xy+yaw] candidates: " << out.candidates.size()
            << " modes: " << selection.mode_count << " time: " << out.time_cost << " s"
            << " best q=" << selection.best->quality
            << " runner-up q=" << (selection.runner_up ? selection.runner_up->quality : 0) << "\n";
  ASSERT_(selection.best);
  ASSERT_(selection.unambiguous);
  expect_near(selection.best->pose, truePose);
}

// Identical rows: shifting by one row is as good a match, so the result
// must be reported as ambiguous.
void test_ambiguous_rows()
{
  const double pitch    = 2.0;
  const auto   world    = make_world(periodic_rows(pitch));
  const auto   truePose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.0, 1.0, 0, 0, 0, 0);
  const auto   prior    = mrpt::math::TPose2D(0.3, 1.2, 0);
  const auto   in       = make_input(world, truePose, prior, 3.0, 0.0);
  const auto   out      = mola::RelocalizationICP_SE2::run(in);

  mola::CandidateSelectionParameters sp;
  sp.same_mode_max_distance = 0.5 * pitch;
  const auto selection      = mola::select_relocalization_candidate(out.candidates, sp);

  std::cout << "[rows] candidates: " << out.candidates.size() << " modes: " << selection.mode_count
            << " best q=" << (selection.best ? selection.best->quality : 0)
            << " runner-up q=" << (selection.runner_up ? selection.runner_up->quality : 0) << "\n";
  ASSERT_(selection.best);
  ASSERT_GE_(selection.mode_count, 2U);
  ASSERT_(!selection.unambiguous);
}

// Lattice over a full turn: no repeated yaw at +-180 deg.
void test_full_turn_lattice()
{
  mola::RelocalizationICP_SE2::Input in;
  in.reference_map.layers["points"] = make_world(unique_room());
  in.local_map                      = observe(
                           *mrpt::ptr_cast<mrpt::maps::CSimplePointsMap>::from(in.reference_map.layers["points"]),
                           mrpt::poses::CPose3D(), 12.0);
  const auto [icp, params] =
      mp2p_icp::icp_pipeline_from_yaml(mrpt::containers::yaml::FromText(icpPipelineYaml));
  in.icp_pipeline                         = {icp};
  in.icp_parameters                       = params;
  in.initial_guess_lattice.corner_min     = {0, 0, -M_PI};
  in.initial_guess_lattice.corner_max     = {0, 0, M_PI};
  in.initial_guess_lattice.resolution_phi = mrpt::DEG2RAD(90.0);
  in.icp_parameters.maxIterations         = 1;

  size_t cells            = 0;
  in.on_progress_callback = [&](const auto& p) { cells = p.total_cells; };
  mola::RelocalizationICP_SE2::run(in);
  ASSERT_EQUAL_(cells, 4U);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    mrpt::random::getRandomGenerator().randomize(1234);
    test_full_turn_lattice();
    test_xy_search();
    test_xy_yaw_search();
    test_ambiguous_rows();
    std::cout << "\nSUCCESS\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
