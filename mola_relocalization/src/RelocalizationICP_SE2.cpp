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
#include <mrpt/math/wrap2pi.h>
#include <mrpt/version.h>

#include <algorithm>
#include <cmath>

namespace
{
// Values from min to max (both included) every `res`. A range spanning a
// full turn (angles) does not repeat its end.
std::vector<double> lattice_values(double min, double max, double res, bool isAngle)
{
  ASSERT_GT_(res, 0);
  ASSERT_GE_(max, min);
  std::vector<double> v;
  if (isAngle && max - min >= 2 * M_PI - 1e-6)
  {
    const auto n = static_cast<size_t>(std::ceil(2 * M_PI / res - 1e-6));
    for (size_t i = 0; i < n; i++)
    {
      v.push_back(mrpt::math::wrapToPi(min + 2 * M_PI * static_cast<double>(i) / n));
    }
    return v;
  }
  const auto n = static_cast<size_t>(std::floor((max - min) / res + 1e-6));
  // Centered in the range:
  const double first = 0.5 * (min + max) - 0.5 * res * static_cast<double>(n);
  for (size_t i = 0; i <= n; i++)
  {
    v.push_back(first + res * static_cast<double>(i));
  }
  return v;
}
}  // namespace

// METHOD: ICP
mola::RelocalizationICP_SE2::Output mola::RelocalizationICP_SE2::run(const Input& in)
{
  mola::RelocalizationICP_SE2::Output result;
  const double                        t0 = mrpt::Clock::nowDouble();

  ASSERT_(!in.reference_map.layers.empty());

  const auto& igl = in.initial_guess_lattice;  // shortcut

  // All the initial guesses:
  const auto xs = lattice_values(igl.corner_min.x, igl.corner_max.x, igl.resolution_xy, false);
  const auto ys = lattice_values(igl.corner_min.y, igl.corner_max.y, igl.resolution_xy, false);
  const auto phis =
      lattice_values(igl.corner_min.phi, igl.corner_max.phi, igl.resolution_phi, true);

  const size_t nCells = xs.size() * ys.size() * phis.size();

  ASSERT_(nCells > 0);

  ASSERT_(!in.icp_pipeline.empty());

  const size_t nPipelines = in.icp_pipeline.size();

  mrpt::WorkerThreadsPool pool(
      nPipelines, mrpt::WorkerThreadsPool::POLICY_FIFO,
      "RelocalizationICP_SE2"  // threads name
  );
  std::vector<std::mutex>        pipelineMtx(nPipelines);
  std::mutex                     resultMtx;
  std::vector<std::future<void>> futs;

  // evaluate over the grid:
  size_t iGlobal = 0;
  for (const double x : xs)
  {
    for (const double y : ys)
    {
      for (const double phi : phis)
      {
        const auto initGuessPose = mrpt::math::TPose3D(x, y, igl.z, phi, igl.pitch, igl.roll);

        auto f = pool.enqueue(
            [initGuessPose, iGlobal, nCells, nPipelines, &in, &resultMtx, &result, &pipelineMtx]()
            {
              const size_t threadIdx = iGlobal % nPipelines;

              auto lck1 = mrpt::lockHelper(pipelineMtx.at(threadIdx));

              mp2p_icp::Results icpResult;

              in.icp_pipeline.at(threadIdx)->align(
                  in.local_map, in.reference_map, initGuessPose, in.icp_parameters, icpResult);

              // report progress to the user, if enabled:
              if (in.on_progress_callback)
              {
                ProgressFeedback p;
                p.cell_init_guess      = initGuessPose;
                p.current_cell         = iGlobal;
                p.total_cells          = nCells;
                p.obtained_icp_quality = icpResult.quality;

                auto lck2 = mrpt::lockHelper(resultMtx);
                in.on_progress_callback(p);
              }

              if (icpResult.quality < in.icp_minimum_quality)
              {
                return;
              }

              // accept result:
              auto       lck2 = mrpt::lockHelper(resultMtx);
              const auto pose = icpResult.optimal_tf.mean.asTPose();
              result.found_poses.insertPose(pose);
              result.candidates.push_back({pose, icpResult.quality, initGuessPose});
            });

        futs.emplace_back(std::move(f));
        iGlobal++;
      }
    }
  }

  // wait for all of them to end:
  for (auto& f : futs)
  {
    f.get();
  }

  std::sort(
      result.candidates.begin(), result.candidates.end(),
      [](const Candidate& a, const Candidate& b) { return a.quality > b.quality; });

  result.time_cost = mrpt::Clock::nowDouble() - t0;

  return result;
}

auto mola::select_relocalization_candidate(
    const std::vector<RelocalizationICP_SE2::Candidate>& candidates,
    const CandidateSelectionParameters&                  params) -> CandidateSelection
{
  using Candidate = RelocalizationICP_SE2::Candidate;

  std::vector<const Candidate*> sorted;
  sorted.reserve(candidates.size());
  for (const auto& c : candidates)
  {
    sorted.push_back(&c);
  }
  std::stable_sort(
      sorted.begin(), sorted.end(),
      [](const Candidate* a, const Candidate* b) { return a->quality > b->quality; });

  // Greedy grouping: each mode is represented by its best candidate.
  std::vector<const Candidate*> modes;
  for (const auto* c : sorted)
  {
    const bool known = std::any_of(
        modes.begin(), modes.end(),
        [&](const Candidate* m)
        {
          const double d =
              mrpt::math::TPoint2D(c->pose.x - m->pose.x, c->pose.y - m->pose.y).norm();
          const double dYaw = std::abs(mrpt::math::wrapToPi(c->pose.yaw - m->pose.yaw));
          return d <= params.same_mode_max_distance && dYaw <= params.same_mode_max_yaw;
        });
    if (!known)
    {
      modes.push_back(c);
    }
  }

  CandidateSelection out;
  out.mode_count = modes.size();
  if (modes.empty())
  {
    return out;
  }
  out.best = *modes[0];
  if (modes.size() > 1)
  {
    out.runner_up = *modes[1];
  }
  out.unambiguous =
      !out.runner_up || out.best->quality - out.runner_up->quality >= params.min_quality_margin;
  return out;
}
