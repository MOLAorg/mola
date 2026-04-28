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
 * @file   test-searchable-pose-list.cpp
 * @brief  Unit tests for mola_pose_list / SearchablePoseList
 * @author Jose Luis Blanco Claraco
 * @date   Mar 5, 2024
 */

#include <mola_pose_list/SearchablePoseList.h>
#include <mrpt/poses/Lie/SE.h>

#include <cmath>
#include <iostream>

namespace
{
using mrpt::poses::CPose3D;

CPose3D xyz(double x, double y, double z) { return CPose3D::FromXYZYawPitchRoll(x, y, z, 0, 0, 0); }

// Populate a list with 25 KFs spaced 1 m along x, ids = id_base..id_base+24.
// Returns the id of the KF whose x is targetX.
mola::SearchablePoseList::KFID populateLine(
    mola::SearchablePoseList& list, mola::SearchablePoseList::KFID id_base = 100)
{
  for (mola::SearchablePoseList::KFID i = 0; i < 25; ++i)
  {
    list.insert(xyz(static_cast<double>(i), 0, 0), id_base + i);
  }
  return id_base;
}

// ── 1. id-keyed insertion + setPoseById updates the NN search ──────────────
void test_insert_with_id_and_set_pose()
{
  mola::SearchablePoseList list(false /*from_last_only*/);
  const auto               idBase = populateLine(list);  // KFs at x=0..24

  ASSERT_EQUAL_(list.size(), 25UL);

  // Query at (5.1, 0, 0): nearest is KF (idBase+5) at (5,0,0).
  {
    auto [isFirst, dist] = list.check(xyz(5.1, 0, 0));
    ASSERT_(!isFirst);
    ASSERT_NEAR_(dist.translation().norm(), 0.1, 1e-5);
  }

  // Move that KF to (1000, 0, 0): query at (5.1,0,0) now snaps to KF at (4,0,0)
  // or (6,0,0); both are at distance ~1.1 / ~0.9. Closest is (6,0,0): dist 0.9.
  list.setPoseById(idBase + 5, xyz(1000, 0, 0));
  {
    auto [isFirst, dist] = list.check(xyz(5.1, 0, 0));
    ASSERT_(!isFirst);
    ASSERT_NEAR_(dist.translation().norm(), 0.9, 1e-5);
  }

  // setPoseById on an unknown id is a no-op.
  list.setPoseById(999999, xyz(0, 0, 0));
  ASSERT_EQUAL_(list.size(), 25UL);
}

// ── 2. removeAllFartherThan preserves the id->index mapping ────────────────
void test_remove_preserves_ids()
{
  mola::SearchablePoseList list(false);
  const auto               idBase = populateLine(list);  // KFs at x=0..24

  // Drop everything farther than 10 m from (12,0,0): keeps KFs at x=2..22
  // (21 KFs).
  list.removeAllFartherThan(xyz(12, 0, 0), 10.0);
  ASSERT_EQUAL_(list.size(), 21UL);

  // Surviving id (idBase+12) is still addressable: move it to (1000,0,0),
  // confirm the query no longer snaps to it.
  list.setPoseById(idBase + 12, xyz(1000, 0, 0));
  {
    auto [isFirst, dist] = list.check(xyz(11.5, 0, 0));
    ASSERT_(!isFirst);
    // Closest is now (11,0,0) → 0.5
    ASSERT_NEAR_(dist.translation().norm(), 0.5, 1e-5);
  }

  // An evicted id (idBase+0 was at x=0, distance 12 from anchor → removed)
  // is silently a no-op.
  list.setPoseById(idBase + 0, xyz(0, 0, 0));
  ASSERT_EQUAL_(list.size(), 21UL);
}

// ── 3. legacy insert(p) (no id) keeps working alongside id-tagged inserts ──
void test_legacy_insert_no_id()
{
  mola::SearchablePoseList list(false);
  // 24 untagged + 1 tagged
  for (int i = 0; i < 24; ++i)
  {
    list.insert(xyz(static_cast<double>(i), 0, 0));  // no id
  }
  list.insert(xyz(50, 0, 0), 42);

  ASSERT_EQUAL_(list.size(), 25UL);

  // Move the tagged one far away: query at (50.1,0,0) used to snap there.
  list.setPoseById(42, xyz(1000, 0, 0));
  auto [isFirst, dist] = list.check(xyz(50.1, 0, 0));
  ASSERT_(!isFirst);
  // Closest among the untagged line (max x = 23) is (23, 0, 0): dist ≈ 27.1
  ASSERT_NEAR_(dist.translation().norm(), 50.1 - 23.0, 1e-3);
}

// ── 4. Regression: check() works correctly when fewer than k=20 KFs exist.
// Before the fix, nn_multiple_search returned sized-20 vectors with trailing
// garbage when the cloud had fewer points, corrupting the best-match pick.
void test_check_with_few_kfs()
{
  mola::SearchablePoseList list(false);

  // Just 3 KFs along x.
  list.insert(xyz(0, 0, 0), 1);
  list.insert(xyz(5, 0, 0), 2);
  list.insert(xyz(10, 0, 0), 3);

  // Query near (5,0,0) must select the (5,0,0) KF.
  auto [isFirst, dist] = list.check(xyz(4.9, 0, 0));
  ASSERT_(!isFirst);
  ASSERT_NEAR_(dist.translation().norm(), 0.1, 1e-5);
}

// ── 5. from_last_only mode: id-keyed APIs are no-ops ──────────────────────
void test_from_last_only_is_noop_for_id_api()
{
  mola::SearchablePoseList list(true /*from_last_only*/);

  list.insert(xyz(1, 2, 3), 7);  // id ignored, behaves like insert(p)
  ASSERT_EQUAL_(list.size(), 1UL);

  // setPoseById is a no-op; the stored last-pose is unchanged.
  list.setPoseById(7, xyz(99, 99, 99));
  auto [isFirst, dist] = list.check(xyz(1, 2, 3));
  ASSERT_(!isFirst);
  ASSERT_NEAR_(dist.translation().norm(), 0.0, 1e-9);
}
// ── 6. countNearby: basic translation-only counting ───────────────────────
void test_count_nearby_translation()
{
  mola::SearchablePoseList list(false);

  // KFs at x = 0, 1, 2, 3, 4 m
  for (int i = 0; i < 5; ++i)
  {
    list.insert(xyz(static_cast<double>(i), 0, 0));
  }

  // Query at (2, 0, 0), threshold 1.5 m trans, large rot.
  // Expected: x=1,2,3 are within 1.5 m => count=3
  const uint32_t count = list.countNearby(xyz(2, 0, 0), 1.5, M_PI);
  ASSERT_EQUAL_(count, 3u);

  // Tight threshold: only the exact match
  const uint32_t count2 = list.countNearby(xyz(2, 0, 0), 0.1, M_PI);
  ASSERT_EQUAL_(count2, 1u);

  // No matches
  const uint32_t count3 = list.countNearby(xyz(100, 0, 0), 1.0, M_PI);
  ASSERT_EQUAL_(count3, 0u);
}

// ── 7. countNearby: rotation threshold is applied ─────────────────────────
void test_count_nearby_rotation()
{
  mola::SearchablePoseList list(false);

  // Two KFs at the same translation but different yaw
  list.insert(mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0, 0, 0));
  list.insert(mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, M_PI, 0, 0));

  // Large rot threshold: both match
  const uint32_t both = list.countNearby(
      mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, M_PI / 2, 0, 0), 0.1, M_PI);
  ASSERT_EQUAL_(both, 2u);

  // Tight rot threshold: only the one with yaw=pi/2 should survive...
  // but none of the stored poses is within pi/4 of pi/2; let's use yaw=0:
  // query at yaw=0.1, threshold=0.2 rad => only stored yaw=0 is within 0.2 rad.
  const uint32_t one =
      list.countNearby(mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0.1, 0, 0), 0.1, 0.2);
  ASSERT_EQUAL_(one, 1u);
}

// ── 8. countNearby: from_last_only mode ───────────────────────────────────
void test_count_nearby_from_last_only()
{
  mola::SearchablePoseList list(true /*from_last_only*/);
  list.insert(xyz(5, 0, 0));

  // Within threshold
  ASSERT_EQUAL_(list.countNearby(xyz(5.05, 0, 0), 0.1, M_PI), 1u);

  // Outside threshold
  ASSERT_EQUAL_(list.countNearby(xyz(10, 0, 0), 0.1, M_PI), 0u);
}

// ── 9. countNearby: empty list returns 0 ─────────────────────────────────
void test_count_nearby_empty()
{
  mola::SearchablePoseList list(false);
  ASSERT_EQUAL_(list.countNearby(xyz(0, 0, 0), 100.0, M_PI), 0u);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    std::cout << "test_insert_with_id_and_set_pose ...\n";
    test_insert_with_id_and_set_pose();

    std::cout << "test_remove_preserves_ids ...\n";
    test_remove_preserves_ids();

    std::cout << "test_legacy_insert_no_id ...\n";
    test_legacy_insert_no_id();

    std::cout << "test_check_with_few_kfs ...\n";
    test_check_with_few_kfs();

    std::cout << "test_from_last_only_is_noop_for_id_api ...\n";
    test_from_last_only_is_noop_for_id_api();

    std::cout << "test_count_nearby_translation ...\n";
    test_count_nearby_translation();

    std::cout << "test_count_nearby_rotation ...\n";
    test_count_nearby_rotation();

    std::cout << "test_count_nearby_from_last_only ...\n";
    test_count_nearby_from_last_only();

    std::cout << "test_count_nearby_empty ...\n";
    test_count_nearby_empty();

    std::cout << "Test successful."
              << "\n";
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
