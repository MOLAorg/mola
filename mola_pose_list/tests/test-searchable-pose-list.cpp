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
 * @brief  Unit tests for mola_pose_list
 * @author Jose Luis Blanco Claraco
 * @date   Mar 5, 2024
 */

#include <mola_pose_list/SearchablePoseList.h>
#include <mrpt/poses/Lie/SE.h>

#include <iostream>

static void test1()
{
  // write me!
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test1();

    std::cout << "Test successful."
              << "\n";
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
