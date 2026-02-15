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
 * @file   NavStateFilter.h
 * @brief  Virtual base class for navigation state estimation algorithms.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */
#pragma once

#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/RawDataConsumer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/topography/data_types.h>

#include <set>
#include <string>

#pragma once

namespace mola
{
/** The state returned by NavStateFilter
 *
 * \ingroup mola_kernel_interfaces_grp */
struct NavState
{
  NavState() = default;

  /** SE(3) pose estimation, including information matrix, given
   *  in the requested frame_id.
   */
  mrpt::poses::CPose3DPDFGaussianInf pose;

  /** Linear and angular velocity estimation, given in the local vehicle
   *  frame. */
  mrpt::math::TTwist3D twist;

  /** Inverse covariance matrix (information) of twist,
   *  with variable order in the matrix: [vx vy vz wx wy wz]
   */
  mrpt::math::CMatrixDouble66 twist_inv_cov;

  std::string asString() const;
};

/** Virtual API for kinematic state filtering algorithms,
 *  fusing information from multiple odometry or twist sources, IMU, GNSS, etc.
 *
 *  Refer to the "state estimation section"
 *  in [the docs](https://docs.mola-slam.org/)
 *
 * \ingroup mola_kernel_interfaces_grp */
class NavStateFilter : public mola::ExecutableBase, public RawDataConsumer
{
 public:
  NavStateFilter();

  /** Resets the estimator state to an initial state */
  virtual void reset() = 0;

  /** initialize(): loads "raw_data_source".
   * If reimplemented in a derived class, remember to call the base class.
   */
  void initialize(const Yaml& cfg) override;

  /** Integrates new SE(3) pose estimation of the vehicle wrt a given
   * frame_id.
   */
  virtual void fuse_pose(
      const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
      const std::string& frame_id) = 0;

  /** Integrates new wheels-based odometry observations into the estimator.
   *  This is a convenience method that internally ends up calling
   *  fuse_pose(), but computing the uncertainty of odometry increments
   *  according to a given motion model.
   */
  virtual void fuse_odometry(
      const mrpt::obs::CObservationOdometry& odom, const std::string& odomName = "odom_wheels") = 0;

  /** Integrates new IMU observations into the estimator */
  virtual void fuse_imu(const mrpt::obs::CObservationIMU& imu) = 0;

  /** Integrates new GNSS observations into the estimator */
  virtual void fuse_gnss(const mrpt::obs::CObservationGPS& gps) = 0;

  /** Integrates new twist estimation (in the odom frame) */
  virtual void fuse_twist(
      const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
      const mrpt::math::CMatrixDouble66& twistCov) = 0;

  /** Computes the estimated vehicle state at a given timestep using the
   * observations in the time window. A std::nullopt is returned if there is
   * no valid observations yet, or if requested a timestamp out of the model
   * validity time window (e.g. too far in the future to be trustful).
   */
  virtual std::optional<NavState> estimated_navstate(
      const mrpt::Clock::time_point& timestamp, const std::string& frame_id) = 0;

  /** (Optional virtual method) Returns the estimated trajectory (sequence of
   * timestamped poses) between two time points, in the given frame_id.
   *  \return std::nullopt if not implemented or not able to compute for the
   *          requested time interval.
   */
  virtual std::optional<mrpt::poses::CPose3DInterpolator> estimated_trajectory(
      [[maybe_unused]] const mrpt::Clock::time_point& start_time,  // NOLINT
      [[maybe_unused]] const mrpt::Clock::time_point& end_time,
      [[maybe_unused]] const std::string&             frame_id)
  {
    return {};  // Default: none
  }

  /** (Optional virtual method) Returns true if the estimator has converged
   *  to a stable state with uncertainty below reasonable thresholds.
   *  \param[out] pose The current estimated pose with covariance (in the "map" frame_id)
   *  \return true if converged, false otherwise or if not implemented.
   */
  virtual bool has_converged_localization(
      [[maybe_unused]] mrpt::poses::CPose3DPDFGaussian& pose_in_map) const
  {
    return false;  // Default: not implemented
  }

 private:
  /// A list of one or multiple MOLA **module names** to which to subscribe
  std::set<std::string> navstate_source_names_;
};

}  // namespace mola
