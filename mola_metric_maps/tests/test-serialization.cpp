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
 * @file   test-serialization.cpp
 * @brief  Test serialization of map classes
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */

#include <mola_metric_maps/HashedVoxelPointCloud.h>
#include <mola_metric_maps/KeyframePointCloudMap.h>
#include <mola_metric_maps/OccGrid.h>
#include <mola_metric_maps/SparseVoxelPointCloud.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>

#include <iostream>

template <typename T>
class TraitsTest
{
  TraitsTest()
  {
    static_assert(std::is_move_constructible<T>(), "Can't move construct");
    static_assert(std::is_copy_constructible<T>(), "Can't copy construct");
    static_assert(std::is_move_assignable<T>(), "Can't move assign");
    static_assert(std::is_copy_assignable<T>(), "Can't copy assign");
  }
};

#define TEST_CLASS_MOVE_COPY_CTORS(_classname) template class TraitsTest<_classname>

TEST_CLASS_MOVE_COPY_CTORS(mola::SparseVoxelPointCloud);
TEST_CLASS_MOVE_COPY_CTORS(mola::OccGrid);

// Create a set of classes, then serialize and deserialize to test possible
// bugs:
void test_serialization()
{
  const mrpt::rtti::TRuntimeClassId* lstClasses[] = {
      CLASS_ID(mola::OccGrid),
      CLASS_ID(mola::SparseVoxelPointCloud),
      CLASS_ID(mola::HashedVoxelPointCloud),
      CLASS_ID(mola::KeyframePointCloudMap),
  };

  for (auto& c : lstClasses)
  {
    try
    {
      mrpt::io::CMemoryStream buf;
      auto                    arch = mrpt::serialization::archiveFrom(buf);
      {
        auto o = mrpt::ptr_cast<mrpt::serialization::CSerializable>::from(c->createObject());
        arch << *o;
        o.reset();
      }

      mrpt::serialization::CSerializable::Ptr recons;
      buf.Seek(0);
      arch >> recons;
    }
    catch (const std::exception& e)
    {
      THROW_EXCEPTION_FMT(
          "Exception during serialization test for class '%s': %s", c->className, e.what());
    }
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test_serialization();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
