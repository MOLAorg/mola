/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
/**
 * @file   relocalization.cpp
 * @brief  Algorithms for localization starting with large uncertainty.
 * @author Jose Luis Blanco Claraco
 * @date   Apr 2, 2024
 */

#include <mola_relocalization/relocalization.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/version.h>

// METHOD: ICP
mola::RelocalizationICP_SE2::Output mola::RelocalizationICP_SE2::run(const Input& in)
{
  mola::RelocalizationICP_SE2::Output result;
  const double                        t0 = mrpt::Clock::nowDouble();

  ASSERT_(!in.reference_map.layers.empty());

  const auto& igl = in.initial_guess_lattice;  // shortcut

  // Build the grid for all the initial guesses poses:
  const auto grid = mrpt::poses::CPosePDFGrid(
      igl.corner_min.x, igl.corner_max.x, igl.corner_min.y, igl.corner_max.y, igl.resolution_xy,
      igl.resolution_phi, igl.corner_min.phi, igl.corner_max.phi);

  const size_t nX   = grid.getSizeX();
  const size_t nY   = grid.getSizeY();
  const size_t nPhi = grid.getSizePhi();

  const size_t nCells = nX * nY * nPhi;

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
  for (size_t iX = 0, iGlobal = 0; iX < nX; iX++)
  {
    const double x = grid.idx2x(iX);
    for (size_t iY = 0; iY < nY; iY++)
    {
      const double y = grid.idx2y(iY);
      for (size_t iPhi = 0; iPhi < nPhi; iPhi++, iGlobal++)
      {
        const double phi = grid.idx2phi(iPhi);

        const auto initGuessPose = mrpt::math::TPose3D(x, y, 0, phi, 0, 0);

        auto f = pool.enqueue(
            [initGuessPose, iGlobal, nCells, nPipelines, &in, &resultMtx, &result, &pipelineMtx]()
            {
              size_t threadIdx = iGlobal % nPipelines;

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

                in.on_progress_callback(p);
              }

              if (icpResult.quality < in.icp_minimum_quality)
              {
                return;
              }

              // accept result:
              auto lck2 = mrpt::lockHelper(resultMtx);
              result.found_poses.insertPose(icpResult.optimal_tf.mean.asTPose());
            });

        futs.emplace_back(std::move(f));
      }
    }
  }

  // wait for all of them to end:
  for (auto& f : futs) f.get();

  result.time_cost = mrpt::Clock::nowDouble() - t0;

  return result;
}
