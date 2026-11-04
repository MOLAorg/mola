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
 * @file   test_local_map_correction_source.cpp
 * @brief  Unit tests for mola::LocalMapCorrectionSource
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_kernel/interfaces/LocalMapCorrectionSource.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/lock_helper.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{
using mola::LocalMapCorrection;
using mola::LocalMapKeyframe;
using mrpt::poses::CPose3D;

/** A provider like a global mapper would be: keyframes along a straight line,
 *  one per meter, whose poses a "loop closure" shifts sideways. */
class MockMapper : public mola::LocalMapCorrectionSource
{
 public:
  explicit MockMapper(std::size_t n)
  {
    for (std::size_t i = 0; i < n; i++)
    {
      poses_.emplace_back(static_cast<double>(i), 0, 0, 0, 0, 0);
    }
  }

  void correct(double lateralShift, std::size_t newLoops)
  {
    {
      auto lck = mrpt::lockHelper(mtx_);
      for (std::size_t i = 0; i < poses_.size(); i++)
      {
        // A growing lateral shift: relative poses change, not only the origin.
        poses_[i] =
            CPose3D(poses_[i].x(), lateralShift * static_cast<double>(i) / 10.0, 0, 0, 0, 0);
      }
    }
    LocalMapCorrection c;
    c.timestamp         = mrpt::Clock::fromDouble(static_cast<double>(poses_.size() - 1));
    c.new_loop_closures = newLoops;
    advertise_local_map_correction(c);
  }

  [[nodiscard]] bool has_subscribers() const { return any_local_map_correction_subscriber(); }

 private:
  [[nodiscard]] std::optional<std::vector<LocalMapKeyframe>> local_map_keyframes_impl(
      mola::SharedKeyframeMap::KeyFrameID reference, double radius) const override
  {
    auto lck = mrpt::lockHelper(mtx_);
    if (reference >= poses_.size())
    {
      return std::nullopt;
    }
    const auto&                   ref = poses_.at(reference);
    std::vector<LocalMapKeyframe> out;
    for (std::size_t i = 0; i < poses_.size(); i++)
    {
      if ((poses_[i].translation() - ref.translation()).norm() > radius)
      {
        continue;
      }
      LocalMapKeyframe kf;
      kf.id                  = i;
      kf.timestamp           = mrpt::Clock::fromDouble(static_cast<double>(i));
      kf.pose_from_reference = poses_[i] - ref;
      out.push_back(kf);
    }
    return out;
  }

  mutable std::mutex   mtx_;
  std::vector<CPose3D> poses_;
};

void test_query_relative_to_reference()
{
  MockMapper m(20);
  ASSERT_(!m.local_map_keyframes(20, 5.0).has_value());

  const auto kfs = m.local_map_keyframes(10, 3.5);
  ASSERT_(kfs.has_value());
  ASSERT_EQUAL_(kfs->size(), 7U);  // 7..13
  ASSERT_EQUAL_(kfs->front().id, 7U);
  ASSERT_LT_(std::abs(kfs->front().pose_from_reference.x() + 3.0), 1e-9);

  // Oldest first, and the reference itself at the origin of the poses:
  bool referenceFound = false;
  for (std::size_t i = 0; i < kfs->size(); i++)
  {
    const auto& kf = kfs->at(i);
    if (i > 0)
    {
      ASSERT_LT_(kfs->at(i - 1).id, kf.id);
    }
    if (kf.id == 10)
    {
      referenceFound = true;
      ASSERT_LT_(kf.pose_from_reference.translation().norm(), 1e-9);
    }
  }
  ASSERT_(referenceFound);
}

/** Callbacks can query the provider, (un)subscribe themselves, and an
 *  exception in one does not stop the others. */
void test_callbacks()
{
  MockMapper m(20);

  bool threw = false;
  try
  {
    m.subscribe_to_local_map_corrections({});
  }
  catch (const std::invalid_argument&)
  {
    threw = true;
  }
  ASSERT_(threw);

  std::vector<double>     lateralSeen;
  std::size_t             loopsSeen = 0;
  mrpt::Clock::time_point lastStamp;
  const auto              idQuery = m.subscribe_to_local_map_corrections(
      [&](const LocalMapCorrection& c)
      {
        loopsSeen += c.new_loop_closures;
        lastStamp      = c.timestamp;
        const auto kfs = m.local_map_keyframes(0, 100.0);  // no deadlock
        ASSERT_(kfs.has_value());
        lateralSeen.push_back(kfs->back().pose_from_reference.y());
      });
  m.subscribe_to_local_map_corrections([](const LocalMapCorrection&)
                                       { throw std::runtime_error("faulty subscriber"); });

  mola::LocalMapCorrectionSource::subscription_id_t idOnce    = 0;
  int                                               onceCalls = 0;
  idOnce = m.subscribe_to_local_map_corrections(
      [&](const LocalMapCorrection&)
      {
        onceCalls++;
        m.unsubscribe_from_local_map_corrections(idOnce);
      });

  // State captured by value in a callback persists across corrections:
  std::vector<int> countsSeen;
  m.subscribe_to_local_map_corrections([&countsSeen, count = 0](const LocalMapCorrection&) mutable
                                       { countsSeen.push_back(++count); });

  m.correct(1.0, 2);
  m.correct(2.0, 1);
  ASSERT_EQUAL_(countsSeen.size(), 2U);
  ASSERT_EQUAL_(countsSeen.at(1), 2);

  ASSERT_EQUAL_(onceCalls, 1);
  ASSERT_EQUAL_(loopsSeen, 3U);
  ASSERT_(lastStamp == mrpt::Clock::fromDouble(19.0));  // the newest keyframe
  ASSERT_EQUAL_(lateralSeen.size(), 2U);
  ASSERT_LT_(std::abs(lateralSeen.at(0) - 1.9), 1e-9);  // keyframe 19: 1.0 * 19 / 10
  ASSERT_LT_(std::abs(lateralSeen.at(1) - 3.8), 1e-9);

  m.unsubscribe_from_local_map_corrections(idQuery);
  m.unsubscribe_from_local_map_corrections(12345);  // unknown: ignored
  m.correct(3.0, 1);
  ASSERT_EQUAL_(lateralSeen.size(), 2U);
}

/** Unsubscribing from another thread waits for the callback in progress, so
 *  the objects it captures can be destroyed right after. */
void test_unsubscribe_waits_for_callback()
{
  MockMapper        m(5);
  std::atomic<bool> inCallback{false};
  std::atomic<bool> finished{false};
  const auto        id = m.subscribe_to_local_map_corrections(
      [&](const LocalMapCorrection&)
      {
        inCallback = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        finished = true;
      });

  std::thread provider([&]() { m.correct(1.0, 1); });
  while (!inCallback)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  m.unsubscribe_from_local_map_corrections(id);
  ASSERT_(finished);
  ASSERT_(!m.has_subscribers());
  provider.join();
}

}  // namespace

int main()
{
  try
  {
    test_query_relative_to_reference();
    test_callbacks();
    test_unsubscribe_waits_for_callback();
    std::cout << "All tests passed.\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
