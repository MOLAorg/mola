/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */

/**
 * @file   closest_matches.h
 * @brief  Bounded list of the closest nearest-neighbor candidates
 * @author Jose Luis Blanco Claraco
 * @date   Oct 24, 2026
 */
#pragma once

#include <mrpt/math/TPoint3D.h>

#include <array>
#include <cstdint>
#include <vector>

namespace mola::internal
{
/** The `capacity` closest candidates seen so far, sorted by ascending distance
 * (ties keep the order they were added in), for the k-NN and the capped radius
 * searches of the voxel maps.
 *
 * The first `STACK_CAPACITY` matches live on the stack, so the usual small-k
 * queries do not allocate; the heap only takes the matches beyond that, so a
 * large capacity costs nothing unless that many candidates are found.
 */
class ClosestMatches
{
 public:
  struct Match
  {
    mrpt::math::TPoint3Df pt;
    float                 sqrDist;
    uint64_t              id;
  };

  constexpr static size_t STACK_CAPACITY = 32;

  explicit ClosestMatches(size_t capacity) : capacity_(capacity) {}

  // data_ may point into this object:
  ClosestMatches(const ClosestMatches&)            = delete;
  ClosestMatches& operator=(const ClosestMatches&) = delete;

  void add(float sqrDist, const mrpt::math::TPoint3Df& pt, uint64_t id)
  {
    // After every match at the same or a shorter distance. When full, a
    // candidate farther than all of them is rejected on the first comparison.
    size_t i = size_;
    while (i > 0 && sqrDist < data_[i - 1].sqrDist)
    {
      i--;
    }
    if (i >= capacity_)
    {
      return;
    }

    if (size_ < capacity_)
    {
      growByOne();
    }

    // Move [i, size-1) one position up, starting from the back so that nothing
    // is overwritten before being moved. If the list was full, its last match
    // drops out.
    for (size_t j = size_ - 1; j > i; j--)
    {
      data_[j] = data_[j - 1];
    }
    data_[i] = {pt, sqrDist, id};
  }

  size_t size() const { return size_; }

  /// Appends the matches, closest first, to the output vectors.
  void appendTo(
      std::vector<mrpt::math::TPoint3Df>& pts, std::vector<float>& sqrDists,
      std::vector<uint64_t>& ids) const
  {
    for (size_t k = 0; k < size_; k++)
    {
      pts.push_back(data_[k].pt);
      sqrDists.push_back(data_[k].sqrDist);
      ids.push_back(data_[k].id);
    }
  }

 private:
  void growByOne()
  {
    if (size_ >= STACK_CAPACITY)
    {
      if (heap_.empty())
      {
        heap_.assign(stack_.begin(), stack_.end());
      }
      heap_.emplace_back();
      data_ = heap_.data();
    }
    size_++;
  }

  size_t                            capacity_;
  size_t                            size_ = 0;
  std::array<Match, STACK_CAPACITY> stack_;
  std::vector<Match>                heap_;
  Match*                            data_ = stack_.data();
};
}  // namespace mola::internal
