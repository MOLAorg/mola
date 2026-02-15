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
#include <mrpt/poses/CPose3DInterpolator.h>

#include <iostream>

int main(int argc, char** argv)
{
  try
  {
    if (argc != 3)
    {
      std::cerr << "Usage: " << argv[0] << " INPUT.tum  OUTPUT.ypr"
                << "\n";
      return 1;
    }

    const std::string sIn  = argv[1];
    const std::string sOut = argv[2];

    mrpt::poses::CPose3DInterpolator p;
    p.loadFromTextFile_TUM(sIn);
    p.saveToTextFile(sOut);

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
    return 1;
  }
}
