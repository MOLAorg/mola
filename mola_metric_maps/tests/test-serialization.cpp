/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-serialization.cpp
 * @brief  Test serialization of map classes
 * @author Jose Luis Blanco Claraco
 * @date   Oct 31, 2023
 */

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

#define TEST_CLASS_MOVE_COPY_CTORS(_classname) \
    template class TraitsTest<_classname>

TEST_CLASS_MOVE_COPY_CTORS(mola::SparseVoxelPointCloud);
TEST_CLASS_MOVE_COPY_CTORS(mola::OccGrid);

// Create a set of classes, then serialize and deserialize to test possible
// bugs:
void test_serialization()
{
    const mrpt::rtti::TRuntimeClassId* lstClasses[] = {
        CLASS_ID(mola::OccGrid),
        CLASS_ID(mola::SparseVoxelPointCloud),
    };

    for (auto& c : lstClasses)
    {
        try
        {
            mrpt::io::CMemoryStream buf;
            auto arch = mrpt::serialization::archiveFrom(buf);
            {
                auto o =
                    mrpt::ptr_cast<mrpt::serialization::CSerializable>::from(
                        c->createObject());
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
                "Exception during serialization test for class '%s': %s",
                c->className, e.what());
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
