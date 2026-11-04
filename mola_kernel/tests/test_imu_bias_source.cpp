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
 * @file   test_imu_bias_source.cpp
 * @brief  Unit tests for the ImuBiasSource interface
 * @author Jose Luis Blanco Claraco
 * @date   Nov 4, 2026
 */

#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mola_kernel/interfaces/ImuBiasSource.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/system/datetime.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace mola
{
/** A provider like a state estimator would be: it averages the gyroscope readings
 *  of each IMU over windows of readings taken at rest, and models the bias as a
 *  random walk between updates. */
class MockBiasEstimator : public ExecutableBase, public ImuBiasSource
{
  DEFINE_MRPT_OBJECT(MockBiasEstimator, mola)
 public:
  MockBiasEstimator()           = default;
  ~MockBiasEstimator() override = default;

  void initialize(const mrpt::containers::yaml&) override {}
  void spinOnce() override {}

  static constexpr std::size_t WINDOW_SAMPLES = 100;
  static constexpr double      PRIOR_SIGMA    = 0.02;  // [rad/s]
  static constexpr double      PRIOR_VAR      = PRIOR_SIGMA * PRIOR_SIGMA;
  static constexpr double      RANDOM_WALK    = 2e-5;  // [rad/s/sqrt(s)]

  /// Feeds one raw gyroscope reading taken at rest.
  void add_reading(
      const std::string& label, const mrpt::Clock::time_point& t, const mrpt::math::TVector3D& w)
  {
    ImuBias updated;
    {
      auto  lck = mrpt::lockHelper(mtx_);
      auto& s   = imus_[label];
      s.newest  = t;
      s.sum += w;
      s.sum_sq += mrpt::math::TVector3D(w.x * w.x, w.y * w.y, w.z * w.z);
      s.n++;
      if (s.n < WINDOW_SAMPLES)
      {
        return;
      }
      // Window closed: its mean replaces the bias (variance of the mean).
      const double n = static_cast<double>(s.n);
      s.bias         = s.sum * (1.0 / n);
      for (int i = 0; i < 3; i++)
      {
        const double var = s.sum_sq[i] / n - s.bias[i] * s.bias[i];
        s.var[i]         = std::max(var, 1e-12) / n;
      }
      s.last_update = t;
      s.n           = 0;
      s.sum         = {0, 0, 0};
      s.sum_sq      = {0, 0, 0};
      updated       = to_bias(label, s, t);
    }
    // Outside the lock, as the interface requires:
    advertise_updated_imu_bias(updated);
  }

  std::vector<std::string> imu_bias_sensor_labels() const override
  {
    auto                     lck = mrpt::lockHelper(mtx_);
    std::vector<std::string> labels;
    for (const auto& [label, s] : imus_)
    {
      labels.push_back(label);
    }
    return labels;
  }

 private:
  std::optional<ImuBias> imu_bias_impl(
      const std::string&                            sensorLabel,
      const std::optional<mrpt::Clock::time_point>& t) const override
  {
    auto       lck = mrpt::lockHelper(mtx_);
    const auto it  = imus_.find(sensorLabel);
    if (it == imus_.end())
    {
      return std::nullopt;
    }
    const auto& s = it->second;
    return to_bias(sensorLabel, s, t && *t >= s.newest ? *t : s.newest);
  }

  struct PerImu
  {
    mrpt::math::TVector3D                  bias{0, 0, 0};
    mrpt::math::TVector3D                  var{PRIOR_VAR, PRIOR_VAR, PRIOR_VAR};
    std::optional<mrpt::Clock::time_point> last_update;
    mrpt::Clock::time_point                newest;

    std::size_t           n = 0;
    mrpt::math::TVector3D sum{0, 0, 0};
    mrpt::math::TVector3D sum_sq{0, 0, 0};
  };

  static ImuBias to_bias(const std::string& label, const PerImu& s, mrpt::Clock::time_point t)
  {
    ImuBias b;
    b.sensor_label     = label;
    b.gyro             = s.bias;
    b.acc              = {0, 0, 0};
    b.timestamp        = t;
    b.last_update      = s.last_update;
    b.gyro_random_walk = RANDOM_WALK;
    const double dt =
        s.last_update ? std::max(0.0, mrpt::system::timeDifference(*s.last_update, t)) : 0.0;
    for (int i = 0; i < 3; i++)
    {
      b.gyro_cov(i, i) = s.var[i] + RANDOM_WALK * RANDOM_WALK * dt;
      b.acc_cov(i, i)  = 0.3 * 0.3;
    }
    return b;
  }

  std::map<std::string, PerImu> imus_;
  mutable std::mutex            mtx_;
};

/** A module that is not a bias provider. */
class MockOtherModule : public ExecutableBase
{
  DEFINE_MRPT_OBJECT(MockOtherModule, mola)
 public:
  void initialize(const mrpt::containers::yaml&) override {}
  void spinOnce() override {}
};

}  // namespace mola
IMPLEMENTS_MRPT_OBJECT(MockBiasEstimator, mola::ExecutableBase, mola)
IMPLEMENTS_MRPT_OBJECT(MockOtherModule, mola::ExecutableBase, mola)

namespace
{
// Biases like those of a LiDAR internal IMU and a separate attitude IMU [rad/s]:
const mrpt::math::TVector3D OUSTER_BIAS{
    mrpt::DEG2RAD(0.02), mrpt::DEG2RAD(-0.29), mrpt::DEG2RAD(-0.20)};
const mrpt::math::TVector3D HANDSFREE_BIAS{mrpt::DEG2RAD(0.01), 0.0, mrpt::DEG2RAD(-0.03)};
constexpr double            DITHER = 0.003;  // [rad/s] deterministic noise, zero mean per window
constexpr double            RATE   = 100.0;  // [Hz]

// A realistic epoch, where double seconds do not round-trip time points exactly:
const auto T0 = mrpt::Clock::fromDouble(1.8e9 + 0.123);

mrpt::Clock::time_point at(int i)
{
  return T0 + std::chrono::microseconds(static_cast<int64_t>(i * 1e6 / RATE));
}

mrpt::math::TVector3D reading(const mrpt::math::TVector3D& bias, int i)
{
  const double d = (i % 2 == 0) ? DITHER : -DITHER;
  return {bias.x + d, bias.y - d, bias.z + d};
}

/// Feeds readings [from, to) of a robot at rest.
void feed_standstill(
    mola::MockBiasEstimator& est, const std::string& label, const mrpt::math::TVector3D& bias,
    int from, int to)
{
  for (int i = from; i < to; i++)
  {
    est.add_reading(label, at(i), reading(bias, i));
  }
}

std::shared_ptr<mola::ImuBiasSource> find_provider(const mola::ExecutableBase& consumer)
{
  const auto found = consumer.findService<mola::ImuBiasSource>();
  if (found.size() != 1)
  {
    return {};
  }
  return std::dynamic_pointer_cast<mola::ImuBiasSource>(found.front());
}

void test_discovery()
{
  auto                         provider = std::make_shared<mola::MockBiasEstimator>();
  auto                         other    = std::make_shared<mola::MockOtherModule>();
  mola::MinimalModuleContainer container({other, provider});

  // Seen from the other module (a front end looking for a bias source):
  const auto found = other->findService<mola::ImuBiasSource>();
  ASSERT_EQUAL_(found.size(), 1u);
  ASSERT_(found.front() == provider);
  ASSERT_(find_provider(*other));
}

void test_unknown_label_and_prior()
{
  mola::MockBiasEstimator est;
  ASSERT_(est.imu_bias_sensor_labels().empty());
  ASSERT_(!est.imu_bias("ouster_imu").has_value());

  // A few readings, not yet a whole window: the sensor is known, still at its prior.
  feed_standstill(est, "ouster_imu", OUSTER_BIAS, 0, 10);
  const auto b = est.imu_bias("ouster_imu");
  ASSERT_(b.has_value());
  ASSERT_EQUAL_(b->sensor_label, std::string("ouster_imu"));
  ASSERT_(!b->last_update.has_value());
  ASSERT_(b->timestamp == at(9));  // the newest data
  ASSERT_EQUAL_(b->gyro.norm(), 0.0);
  ASSERT_NEAR_(std::sqrt(b->gyro_cov(2, 2)), mola::MockBiasEstimator::PRIOR_SIGMA, 1e-12);
  ASSERT_GT_(b->acc_cov(0, 0), 0.0);
}

/** The intended use: a front end corrects its raw gyroscope readings with the
 *  biases of the provider, per sensor label, and its integrated heading stops
 *  drifting. */
void test_consumer_corrects_heading_drift()
{
  auto                         provider = std::make_shared<mola::MockBiasEstimator>();
  auto                         consumer = std::make_shared<mola::MockOtherModule>();
  mola::MinimalModuleContainer container({provider, consumer});

  // The robot waits 5 s at startup:
  feed_standstill(*provider, "ouster_imu", OUSTER_BIAS, 0, 500);
  feed_standstill(*provider, "handsfree_imu", HANDSFREE_BIAS, 0, 500);

  const auto source = find_provider(*consumer);
  ASSERT_(source);
  ASSERT_EQUAL_(source->imu_bias_sensor_labels().size(), 2u);

  const auto b_ouster = source->imu_bias("ouster_imu");
  const auto b_hf     = source->imu_bias("handsfree_imu");
  ASSERT_(b_ouster && b_ouster->last_update);
  ASSERT_(b_hf && b_hf->last_update);
  ASSERT_LT_((b_ouster->gyro - OUSTER_BIAS).norm(), 1e-12);
  ASSERT_LT_((b_hf->gyro - HANDSFREE_BIAS).norm(), 1e-12);

  // Then it drives 2 min straight: integrate the yaw rate, raw vs corrected.
  double yaw_raw       = 0;
  double yaw_corrected = 0;
  for (int i = 0; i < static_cast<int>(120 * RATE); i++)
  {
    const auto w = reading(OUSTER_BIAS, i);
    yaw_raw += w.z / RATE;
    yaw_corrected += (w.z - b_ouster->gyro.z) / RATE;
  }
  ASSERT_NEAR_(yaw_raw, OUSTER_BIAS.z * 120, 1e-9);  // -24 deg
  ASSERT_LT_(std::abs(yaw_corrected), 1e-9);
}

void test_timestamps_and_propagation()
{
  mola::MockBiasEstimator est;
  feed_standstill(est, "ouster_imu", OUSTER_BIAS, 0, 150);

  // No time given: the newest data, after the last update.
  const auto b_now = est.imu_bias("ouster_imu");
  ASSERT_(b_now && b_now->last_update);
  ASSERT_(*b_now->last_update == at(99));
  ASSERT_(b_now->timestamp == at(149));

  // An hour (and one clock tick) later: exactly that time, less certain by the
  // random walk.
  const auto t_later = *b_now->last_update + std::chrono::seconds(3600) + mrpt::Clock::duration(1);
  const auto b_later = est.imu_bias("ouster_imu", t_later);
  ASSERT_(b_later);
  ASSERT_(b_later->timestamp == t_later);
  ASSERT_(b_later->last_update == b_now->last_update);
  ASSERT_EQUAL_(b_later->gyro.x, b_now->gyro.x);
  const double dt = mrpt::system::timeDifference(b_now->timestamp, t_later);
  const double q  = mrpt::square(b_later->gyro_random_walk);
  ASSERT_NEAR_(b_later->gyro_cov(2, 2) - b_now->gyro_cov(2, 2), q * dt, 1e-15);

  // A query in the past gets the newest estimate instead:
  const auto b_past = est.imu_bias("ouster_imu", at(10));
  ASSERT_(b_past);
  ASSERT_(b_past->timestamp == b_now->timestamp);
  ASSERT_EQUAL_(b_past->gyro_cov(2, 2), b_now->gyro_cov(2, 2));
}

void test_callbacks()
{
  mola::MockBiasEstimator est;

  std::map<std::string, int>    count;
  std::map<std::string, double> last_z;
  bool                          all_had_update = true;
  // Faulty subscribers must not prevent the others from being called:
  est.subscribe_to_imu_bias_updates([](const mola::ImuBias&)
                                    { throw std::runtime_error("faulty subscriber"); });
  est.subscribe_to_imu_bias_updates([](const mola::ImuBias&) { throw 42; });
  const auto id = est.subscribe_to_imu_bias_updates(
      [&](const mola::ImuBias& b)
      {
        count[b.sensor_label]++;
        last_z[b.sensor_label] = b.gyro.z;
        all_had_update         = all_had_update && b.last_update.has_value();
      });

  // 3 s at rest, readings at 100 Hz: one update per 100 readings and IMU.
  feed_standstill(est, "ouster_imu", OUSTER_BIAS, 0, 300);
  feed_standstill(est, "handsfree_imu", HANDSFREE_BIAS, 0, 300);
  ASSERT_EQUAL_(count["ouster_imu"], 3);
  ASSERT_EQUAL_(count["handsfree_imu"], 3);
  ASSERT_(all_had_update);

  // The advertised value is the one returned by a query:
  ASSERT_EQUAL_(last_z["ouster_imu"], est.imu_bias("ouster_imu")->gyro.z);

  // Unsubscribed: no more calls.
  est.unsubscribe_from_imu_bias_updates(id);
  est.unsubscribe_from_imu_bias_updates(12345);  // unknown: ignored
  feed_standstill(est, "ouster_imu", OUSTER_BIAS, 300, 400);
  ASSERT_EQUAL_(count["ouster_imu"], 3);
}

/** A callback may query the provider and (un)subscribe: it runs without the
 *  interface lock held. */
void test_reentrant_callbacks()
{
  mola::MockBiasEstimator est;

  int                                    queried = 0;
  int                                    late    = 0;
  mola::ImuBiasSource::subscription_id_t selfId  = 0;
  selfId                                         = est.subscribe_to_imu_bias_updates(
      [&](const mola::ImuBias& b)
      {
        // Query the provider, as a consumer caching all IMUs would:
        if (est.imu_bias(b.sensor_label, b.timestamp))
        {
          queried++;
        }
        // Subscribe another one and remove itself, from within the callback:
        est.subscribe_to_imu_bias_updates([&](const mola::ImuBias&) { late++; });
        est.unsubscribe_from_imu_bias_updates(selfId);
      });
  feed_standstill(est, "ouster_imu", OUSTER_BIAS, 0, 300);
  ASSERT_EQUAL_(queried, 1);
  ASSERT_EQUAL_(late, 2);  // windows 2 and 3
}

/** Queries and (un)subscriptions from other threads (e.g. the LiDAR odometry
 *  thread) while the provider updates and advertises its estimates. */
void test_concurrent_access()
{
  auto                         provider = std::make_shared<mola::MockBiasEstimator>();
  auto                         consumer = std::make_shared<mola::MockOtherModule>();
  mola::MinimalModuleContainer container({provider, consumer});

  std::atomic_int updates{0};
  provider->subscribe_to_imu_bias_updates([&](const mola::ImuBias&) { updates++; });

  std::atomic_bool done{false};
  std::thread      producer(
      [&]()
      {
        feed_standstill(*provider, "ouster_imu", OUSTER_BIAS, 0, 3000);
        done = true;
      });

  const auto       source = find_provider(*consumer);
  std::atomic_bool ok{true};
  auto             reader = [&]()
  {
    std::optional<mrpt::Clock::time_point> last;
    while (!done)
    {
      const auto b = source->imu_bias("ouster_imu");
      if (b && b->last_update)
      {
        // Estimates only move forward in time:
        if (last && *b->last_update < *last)
        {
          ok = false;
        }
        last = b->last_update;
      }
      std::this_thread::yield();
    }
  };
  auto churn = [&]()
  {
    while (!done)
    {
      const auto id = source->subscribe_to_imu_bias_updates([](const mola::ImuBias&) {});
      std::this_thread::yield();
      source->unsubscribe_from_imu_bias_updates(id);
    }
  };
  std::thread r1(reader);
  std::thread r2(reader);
  std::thread c1(churn);
  producer.join();
  r1.join();
  r2.join();
  c1.join();

  ASSERT_(ok);
  ASSERT_EQUAL_(updates.load(), 30);
  ASSERT_LT_((source->imu_bias("ouster_imu")->gyro - OUSTER_BIAS).norm(), 1e-12);
}
}  // namespace

int main()
{
  try
  {
    test_discovery();
    test_unknown_label_and_prior();
    test_consumer_corrects_heading_drift();
    test_timestamps_and_propagation();
    test_callbacks();
    test_reentrant_callbacks();
    test_concurrent_access();
    std::cout << "All ImuBiasSource tests passed.\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Test failed: " << e.what() << "\n";
    return 1;
  }
}
