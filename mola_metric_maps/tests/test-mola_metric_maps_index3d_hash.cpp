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
 * @file   test-mola_metric_maps_index3d_hash.cpp
 * @brief  Test the spatial hash used by every voxel map class
 * @author Jose Luis Blanco Claraco
 * @date   Sep 02, 2026
 */

#include <mola_metric_maps/index3d_t.h>
#include <mrpt/core/exceptions.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>

namespace
{
using key_t = mola::index3d_t<int32_t>;

/// A cube of voxel indices, offset so the negative-coordinate path is used.
std::vector<key_t> denseBox(int side, int offset)
{
  std::vector<key_t> v;
  v.reserve(static_cast<size_t>(side) * side * side);
  for (int x = 0; x < side; x++)
  {
    for (int y = 0; y < side; y++)
    {
      for (int z = 0; z < side; z++)
      {
        v.emplace_back(x + offset, y + offset, z + offset);
      }
    }
  }
  return v;
}

/** The hash must not be capped to a fixed width.
 *
 * A point-based voxel map holds one cell per occupied voxel and never came
 * near a million of them, so a 20-bit hash went unnoticed for years. A field
 * map fills a band around every surface and passes that in seconds, and past
 * the cap every new key collides with an existing one: the container's probing
 * degrades without bound, which is a memory blow-up rather than a slowdown.
 */
void test_hash_is_not_capped()
{
  const auto           keys = denseBox(128, -64);
  mola::index3d_hash<> h;

  std::vector<uint64_t> hs;
  hs.reserve(keys.size());
  for (const auto& k : keys)
  {
    hs.push_back(h(k));
  }
  std::sort(hs.begin(), hs.end());
  const size_t distinct = static_cast<size_t>(std::unique(hs.begin(), hs.end()) - hs.begin());

  const double frac = double(distinct) / double(keys.size());
  std::cout << "test_hash_is_not_capped: " << distinct << " distinct of " << keys.size()  //
            << " (" << 100.0 * frac << "%)\n";

  // A 20-bit hash cannot exceed 2^20 distinct values, i.e. 50% of this set.
  ASSERT_GT_(frac, 0.95);
}

/** The low bits must be mixed, because those are the ones that index buckets.
 *
 * `tsl::robin_map` uses a power-of-two growth policy, so the bucket of a key is
 * the low bits of its hash. The low bits of a sum of odd-prime multiples of the
 * coordinates depend on very few input bits, so grid-aligned keys pile into a
 * few buckets even when the full-width hash is well spread.
 */
void test_low_bits_are_mixed()
{
  const auto           keys = denseBox(128, -64);
  mola::index3d_hash<> h;

  // Occupancy of a power-of-two table at a load factor of 0.5, as the
  // container would size it.
  size_t bits = 1;
  while ((1ULL << bits) < keys.size() * 2)
  {
    bits++;
  }
  const uint64_t nBuckets = 1ULL << bits;
  const uint64_t mask     = nBuckets - 1;

  std::vector<uint32_t> occupancy(nBuckets, 0);
  for (const auto& k : keys)
  {
    occupancy[h(k) & mask]++;
  }

  const double expected = double(keys.size()) / double(nBuckets);
  double       chi2     = 0;
  uint32_t     worst    = 0;
  for (const uint32_t o : occupancy)
  {
    const double d = o - expected;
    chi2 += d * d / expected;
    worst = std::max(worst, o);
  }
  // Normalized so that a uniform hash gives 1.0 and worse is higher.
  const double chi2n = chi2 / double(nBuckets);

  std::cout << "test_low_bits_are_mixed: chi2n = " << chi2n << ", longest chain = " << worst
            << "\n";

  ASSERT_LT_(chi2n, 1.3);
  ASSERT_LT_(worst, 12U);
}

/// The same functor doubles as the `std::map` comparator, which the hash
/// change must leave alone.
void test_comparator_is_a_strict_weak_ordering()
{
  mola::index3d_hash<> c;

  const key_t a(1, 2, 3);
  const key_t b(1, 2, 4);
  const key_t d(1, 3, 0);
  const key_t e(2, 0, 0);

  ASSERT_(c(a, b));
  ASSERT_(!c(b, a));
  ASSERT_(c(b, d));
  ASSERT_(c(d, e));
  ASSERT_(!c(a, a));
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test_hash_is_not_capped();
    test_low_bits_are_mixed();
    test_comparator_is_a_strict_weak_ordering();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
  return 0;
}
