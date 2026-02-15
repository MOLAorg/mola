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

#include <mrpt/core/exceptions.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>

#include <iostream>

int main(int argc, char** argv)
{
  try
  {
    if (argc != 4)
    {
      std::cerr << "Usage: " << argv[0]
                << " INPUT.tum OUTPUT.tum \"[x y z yaw_deg pitch_deg "
                   "roll_deg]\""
                << "\n";
      return 1;
    }

    const std::string sIn  = argv[1];
    const std::string sOut = argv[2];
    const std::string sTF  = argv[3];

    mrpt::poses::CPose3DInterpolator pIn;
    pIn.loadFromTextFile_TUM(sIn);

    std::cout << "Loaded: " << pIn.size() << " poses.\n";
    ASSERT_(!pIn.empty());

    auto in0 = pIn.begin()->second;

    // Apply tf:
    const auto newStartPose = mrpt::poses::CPose3D::FromString(sTF);
    std::cout << "newStartPose: " << newStartPose << "\n";

    auto tf = newStartPose - mrpt::poses::CPose3D(in0);

    for (auto& [t, pose] : pIn)  //
      pose = (tf + mrpt::poses::CPose3D(pose)).asTPose();

    // save:
    pIn.saveToTextFile_TUM(sOut);

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
    return 1;
  }
}
