/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mola-myapp
 * @brief  Blah blah
 * @author XXX
 * @date   XXX
 */

#include <mrpt/core/exceptions.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <iostream>

// Declare supported cli switches ===========
static TCLAP::CmdLine cmd("mola-myapp");

static TCLAP::ValueArg<std::string> argParam1(
    "p", "param",
    "Explanation",
    false, "", "", cmd);

static void do_my_app()
{
    using namespace std::string_literals;

}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.
        do_my_app();
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exit due to exception:\n"
                  << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
