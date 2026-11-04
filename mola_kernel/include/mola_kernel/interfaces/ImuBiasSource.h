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
 * @file   ImuBiasSource.h
 * @brief  Virtual interface for modules that estimate IMU biases.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mrpt/core/Clock.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>

#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mola
{
/** The estimated biases of one IMU, identified by its sensor label.
 *  Providers always fill both covariances (with the prior ones before any
 *  measurement). */
struct ImuBias
{
  ImuBias() = default;

  /** Sensor label of the IMU (as in its mrpt::obs::CObservationIMU). */
  std::string sensor_label;

  /** Gyroscope bias [rad/s], in the IMU sensor frame: subtract it from the raw
   *  angular velocity readings. It includes the Earth rotation. */
  mrpt::math::TVector3D gyro = {0, 0, 0};
  /** Accelerometer bias [m/s^2], in the IMU sensor frame: subtract it from the raw
   *  specific force readings. */
  mrpt::math::TVector3D acc = {0, 0, 0};

  /** Covariances of the two biases above [(rad/s)^2] and [(m/s^2)^2]. A large
   *  entry means that component is barely known (e.g. the accelerometer bias
   *  perpendicular to gravity, which at rest cannot be told from a tilt). */
  mrpt::math::CMatrixDouble33 gyro_cov;
  mrpt::math::CMatrixDouble33 acc_cov;

  /** Random walk densities of the provider's bias model [rad/s/sqrt(s)] and
   *  [m/s^2/sqrt(s)], so consumers can propagate a cached estimate: its
   *  covariance grows by density^2 * dt per axis. Zero if unknown. */
  double gyro_random_walk = 0;
  double acc_random_walk  = 0;

  /** Time the estimate (mean and covariances) refers to. */
  mrpt::Clock::time_point timestamp;

  /** Time of the newest measurement used. Unset while the estimate is still the
   *  prior (zero biases, prior covariances). */
  std::optional<mrpt::Clock::time_point> last_update;
};

/** Virtual interface for modules that estimate the biases of one or more IMUs,
 *  so other modules can use them: e.g. a state estimator that tracks them, read
 *  by a LiDAR odometry front end to correct its IMU-based deskew or motion prior.
 *
 *  Consumers find providers with `ExecutableBase::findService<ImuBiasSource>()`
 *  (if several are found, which one to use is up to the consumer), and either
 *  query them with imu_bias() from any thread, or subscribe to their updates.
 *  Callbacks run in the provider thread, one advertised update at a time, and
 *  may call imu_bias() or (un)subscribe. They must return quickly.
 *
 *  Providers implement imu_bias_sensor_labels() and imu_bias_impl(), both
 *  thread-safe, and call advertise_updated_imu_bias() after each update, without
 *  holding the lock that imu_bias_impl() takes.
 *
 *  Downstream code can detect this interface with
 *  `__has_include(<mola_kernel/interfaces/ImuBiasSource.h>)`.
 *
 * \ingroup mola_kernel_interfaces_grp
 */
class ImuBiasSource
{
 public:
  ImuBiasSource() = default;
  virtual ~ImuBiasSource();

  ImuBiasSource(const ImuBiasSource&)            = delete;
  ImuBiasSource& operator=(const ImuBiasSource&) = delete;
  ImuBiasSource(ImuBiasSource&&)                 = delete;
  ImuBiasSource& operator=(ImuBiasSource&&)      = delete;

  /** Sensor labels of the IMUs with a bias estimate. */
  [[nodiscard]] virtual std::vector<std::string> imu_bias_sensor_labels() const = 0;

  /** The current bias estimate of one IMU, or nothing if this provider does not
   *  know that sensor label. If `t` is given and not older than the estimate,
   *  the estimate is propagated to it by the provider's bias model, and its
   *  `timestamp` is exactly `t`. Otherwise, `timestamp` is the time of the
   *  provider's newest data for that IMU. */
  [[nodiscard]] std::optional<ImuBias> imu_bias(
      const std::string&                            sensorLabel,
      const std::optional<mrpt::Clock::time_point>& t = std::nullopt) const
  {
    // Non-virtual, so the default argument cannot differ between implementations.
    return imu_bias_impl(sensorLabel, t);
  }

  using imu_bias_callback_t = std::function<void(const ImuBias&)>;
  using subscription_id_t   = std::uint64_t;

  /** Registers a callback invoked every time an IMU bias estimate is updated.
   *  \return an id for unsubscribe_from_imu_bias_updates().
   *  \exception std::invalid_argument if the callback is empty. */
  subscription_id_t subscribe_to_imu_bias_updates(const imu_bias_callback_t& callback);

  /** Removes a callback, e.g. before its captured objects are destroyed. Once
   *  this returns, the callback is neither running nor started again: it waits
   *  for an update being advertised in another thread, so do not call it while
   *  holding a lock that the callback takes. It can be called from within the
   *  callback itself. Unknown ids are ignored. */
  void unsubscribe_from_imu_bias_updates(subscription_id_t id);

 protected:
  [[nodiscard]] bool any_imu_bias_subscriber() const;

  /** To be called by providers after each update of an estimate. An exception in
   *  one callback does not prevent the others from running. */
  void advertise_updated_imu_bias(const ImuBias& b);

 private:
  /** Implemented by providers: see imu_bias(). */
  [[nodiscard]] virtual std::optional<ImuBias> imu_bias_impl(
      const std::string& sensorLabel, const std::optional<mrpt::Clock::time_point>& t) const = 0;

  std::map<subscription_id_t, imu_bias_callback_t> imuBiasSubs_;
  subscription_id_t                                imuBiasNextId_ = 1;
  mutable std::mutex                               imuBiasSubsMtx_;
  /// Held while advertising, so unsubscribing waits for callbacks in progress.
  std::recursive_mutex imuBiasDispatchMtx_;
};

}  // namespace mola
