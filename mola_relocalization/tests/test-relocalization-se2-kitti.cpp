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
 * @file   test-relocalization-se2-kitti.cpp
 * @brief  Unit tests for mola_relocalization
 * @author Jose Luis Blanco Claraco
 * @date   Apr 2, 2024
 */

#include <mola_relocalization/relocalization.h>
#include <mrpt/core/bits_math.h>  // .0_deg literal
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/system/filesystem.h>

const std::string datasetsRoot = TEST_DATASETS_ROOT;

static void test1()
{
  using namespace mrpt::literals;  // _deg

  mrpt::obs::CRawlog kitti;
  const auto fil = mrpt::system::pathJoin({datasetsRoot, "kitti", "kitti_00_extract.rawlog"});

  ASSERT_FILE_EXISTS_(fil);
  bool readOk = kitti.loadFromRawLogFile(fil);
  ASSERT_(readOk);

  std::cout << "Loaded test kitti dataset entries: " << kitti.size() << "\n";

  auto obs1 =
      std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(kitti.getAsObservation(0));
  auto obs2 =
      std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(kitti.getAsObservation(8));

  ASSERT_(obs1);
  ASSERT_(obs2);

  // Build a reference map with the first observation, and query for
  // relocalization with the second one (using a large decimation factor in
  // the likelihood function):
  mp2p_icp::metric_map_t refMap;
  refMap.layers["raw"] = obs1->pointcloud;

  // These options may be loaded from an INI file, etc.
  auto& likOpts = obs1->pointcloud->likelihoodOptions;

  likOpts.max_corr_distance = 1.5;
  likOpts.decimation        = 1000;
  likOpts.sigma_dist        = 0.2;

  // query observation:
  mrpt::obs::CSensoryFrame querySf;
  querySf.insert(obs2);

  // relocalization:
  mola::RelocalizationLikelihood_SE2::Input in;
  in.corner_min     = {-2.0, -1.0, -30.0_deg};
  in.corner_max     = {+5.0, +1.0, +30.0_deg};
  in.observations   = querySf;
  in.reference_map  = refMap;
  in.resolution_xy  = 0.25;
  in.resolution_phi = mrpt::DEG2RAD(10.0);

  const auto out = mola::RelocalizationLikelihood_SE2::run(in);

  std::cout << "time_cost: " << out.time_cost << "\n";
  std::cout << "max_log_likelihood: " << out.max_log_likelihood << "\n";
  std::cout << "min_log_likelihood: " << out.min_log_likelihood << "\n";

#if 0
    const auto [cov, mean] = out.likelihood_grid.getCovarianceAndMean();
    std::cout << "mean: " << mean << "\n";
    std::cout << "cov:\n" << cov << "\n";
    for (size_t iPhi = 0; iPhi < out.likelihood_grid.getSizePhi(); iPhi++)
    {
        const double phi = out.likelihood_grid.idx2phi(iPhi);

        mrpt::math::CMatrixDouble slice;
        const_cast<mrpt::poses::CPosePDFGrid&>(
            out.likelihood_grid)  // there was a bug in MRPT API
                                  // const-correctness
            .getAsMatrix(phi, slice);

        slice.saveToTextFile(mrpt::format("slice_%.02f.txt", phi));
    }
#endif

  ASSERT_(
      out.likelihood_grid.getByPos(1.5, 0, 0.0_deg) > out.likelihood_grid.getByPos(0, 0, 0.0_deg));

  // search top candidates:
  const auto bestPoses = mola::find_best_poses_se2(out.likelihood_grid, 0.90);

#if 0
    std::cout << "Top poses: " << bestPoses.size() << "\n";
    for (const auto& [lik, pose] : bestPoses)
    {
        printf("%20s : %e\n", pose.asString().c_str(), lik);
    }
#endif

  ASSERT_(!bestPoses.empty());

  // they are sorted by likelihood: take the last one as best:
  const auto& best = bestPoses.rbegin()->second;
  ASSERT_NEAR_(best.x, 1.5, 0.2);
  ASSERT_NEAR_(best.y, 0.0, 0.2);
  ASSERT_NEAR_(best.phi, 0.0, 0.1);

  std::cout << "best pose: " << best << "\n";
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test1();

    std::cout << "Test successful."
              << "\n";
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
