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
 * @file   relocalization.h
 * @brief  Algorithms for localization starting with large uncertainty.
 * @author Jose Luis Blanco Claraco
 * @date   Apr 2, 2024
 */
#pragma once

#include <mola_pose_list/HashedSetSE3.h>
#include <mp2p_icp/ICP.h>
#include <mp2p_icp/Parameters.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPosePDFGrid.h>

#include <map>

namespace mola
{
/** Takes a global metric map, an observation, and a SE(2) ROI, and evaluates
 * the likelihood of the observation in a regular SE(2) lattice.
 *
 * This is based on mrpt maps observationLikelihood() evaluation, so the main
 * parameters that determine the way likelihood is computed must be defined
 * on before hand within each of the metric map layers of the input reference
 * map.
 *
 * At present this algorithm is useful for these sensor/map types:
 * - observations: pointclouds/2d_scan, reference_map: 2D gridmap
 * - observations: pointclouds, reference_map: pointclouds
 *
 * \todo Implement likelihood of observations GNSS datum within georeferenced
 *       maps.
 *
 * \ingroup mola_relocalization_grp
 */
struct RelocalizationLikelihood_SE2
{
  struct Input
  {
    mp2p_icp::metric_map_t   reference_map;
    mrpt::obs::CSensoryFrame observations;
    mrpt::math::TPose2D      corner_min, corner_max;
    double                   resolution_xy  = 0.5;
    double                   resolution_phi = mrpt::DEG2RAD(30.0);

    Input() = default;
  };

  struct Output
  {
    mrpt::poses::CPosePDFGrid likelihood_grid;
    double                    time_cost          = .0;  //!< [s]
    double                    max_log_likelihood = 0;
    double                    min_log_likelihood = 0;

    Output() = default;
  };

  static Output run(const Input& in);
};

/** Takes a global and a local metric map, a SE(2) ROI, and tries to match
 *  the local map in the global map by running ICP from all initial guesses
 *  defined by a regular SE(2) lattice, returning the result as a SE(3) hashed
 *  lattice.
 *
 * This method is based on mp2p_icp ICP pipelines, refer to the project
 * documentation.
 *
 * \ingroup mola_relocalization_grp
 */
struct RelocalizationICP_SE2
{
  struct ProgressFeedback
  {
    ProgressFeedback() = default;

    size_t              current_cell = 0;
    size_t              total_cells  = 0;
    mrpt::math::TPose3D cell_init_guess;
    double              obtained_icp_quality = .0;
  };

  struct Input
  {
    mp2p_icp::metric_map_t reference_map;
    mp2p_icp::metric_map_t local_map;

    /** If provided more than one, several ICP runs will be triggered in
     * parallel threads.
     */
    std::vector<mp2p_icp::ICP::Ptr> icp_pipeline;
    mp2p_icp::Parameters            icp_parameters;
    double                          icp_minimum_quality = 0.50;

    struct InputLattice
    {
      mrpt::math::TPose2D corner_min, corner_max;
      double              resolution_xy  = 1.0;
      double              resolution_phi = mrpt::DEG2RAD(40.0);
    };
    InputLattice initial_guess_lattice;

    struct OutputLattice
    {
      double resolution_xyz   = 0.10;
      double resolution_yaw   = mrpt::DEG2RAD(5.0);
      double resolution_pitch = mrpt::DEG2RAD(5.0);
      double resolution_roll  = mrpt::DEG2RAD(5.0);
    };
    OutputLattice output_lattice;

    std::function<void(const ProgressFeedback&)> on_progress_callback;

    Input() = default;
  };

  struct Output
  {
    mola::HashedSetSE3 found_poses;
    double             time_cost = .0;  //!< [s]

    Output() = default;
  };

  static Output run(const Input& in);
};

/** Finds the SE(2) poses with the top given percentile likelihood, and returns
 *  them sorted by likelihood (higher values are better matches).
 *
 *  \param grid To be used with the output of RelocalizationLikelihood_SE2
 *  \param percentile If set to 0.99, only those poses with a likelihood >=99%
 *         of the whole pdf will be returned.
 *
 *
 * \ingroup mola_relocalization_grp
 */
auto find_best_poses_se2(const mrpt::poses::CPosePDFGrid& grid, const double percentile = 0.99)
    -> std::map<double, mrpt::math::TPose2D>;

}  // namespace mola
