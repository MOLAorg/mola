/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/core/exceptions.h>
#include <mrpt/poses/CPose3DInterpolator.h>

#include <iostream>

int main(int argc, char** argv)
{
    try
    {
        if (argc != 3)
        {
            std::cerr << "Usage: " << argv[0] << " INPUT.ypr  OUTPUT.tum"
                      << std::endl;
            return 1;
        }

        const std::string sIn  = argv[1];
        const std::string sOut = argv[2];

        mrpt::poses::CPose3DInterpolator p;
        p.loadFromTextFile(sIn);
        p.saveToTextFile_TUM(sOut);

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}
