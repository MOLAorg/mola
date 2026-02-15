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
 * @file   pretty_print_exception.cpp
 * @brief  Helper function to print an exception with colors
 * @author Jose Luis Blanco Claraco
 * @date   Aug 13, 2021
 */

#include <mola_kernel/pretty_print_exception.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/version.h>

#include <iostream>

void mola::pretty_print_exception(
    const std::exception& e, const std::string& headerLine, const bool use_std_cerr)
{
  using namespace mrpt::system;

  auto& o = use_std_cerr ? std::cerr : std::cout;

  const auto setFormatRed = [=](const ConsoleTextStyle style)
  {
    consoleColorAndStyle(
        ConsoleForegroundColor::RED, ConsoleBackgroundColor::DEFAULT, style, use_std_cerr);
  };

  const auto resetFormat = [=]()
  {
    consoleColorAndStyle(
        ConsoleForegroundColor::DEFAULT, ConsoleBackgroundColor::DEFAULT, ConsoleTextStyle::REGULAR,
        use_std_cerr);
  };

  if (!headerLine.empty())
  {
    setFormatRed(ConsoleTextStyle::BOLD);
    o << headerLine << "\n";
    resetFormat();
  }

  std::vector<std::string> lines;
  mrpt::system::tokenize(mrpt::exception_to_str(e), "\r\n", lines);

  for (const auto& line : lines)
  {
    if (mrpt::system::strStarts(line, "Message:") || mrpt::system::strStarts(line, "Location:"))
    {
      setFormatRed(ConsoleTextStyle::UNDERLINED);
      o << line.substr(0, line.find(":") + 1);
      resetFormat();
      o << line.substr(line.find(":") + 1);
    }
    else if (mrpt::system::strStarts(line, "==== MRPT exception"))
    {
      setFormatRed(ConsoleTextStyle::BLINKING);
      o << line;
      resetFormat();
    }
    else
      o << line;
    o << "\n";
  }
}
