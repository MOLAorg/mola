/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   yaml_helpers.cpp
 * @brief  YAML processing helper utilities
 * @author Jose Luis Blanco Claraco
 * @date   Nov 30, 2018
 */

#include <mola-kernel/yaml_helpers.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <algorithm>
#include <cstdlib>
#include <iostream>

MRPT_TODO("Add a more generic YAML file parser: includes, etc.");

static std::string parseEnvVars(const std::string& text)
{
    MRPT_TRY_START

    const auto start = text.find("${");
    if (start == std::string::npos) return text;

    const std::string pre  = text.substr(0, start);
    const std::string post = text.substr(start + 2);

    const auto post_end = post.find('}');
    if (post_end == std::string::npos)
    {
        THROW_EXCEPTION_FMT(
            "Column=%u: Cannot find matching `}` for `${` in: `%s`",
            static_cast<unsigned int>(start), text.c_str());
    }

    const auto  varname = post.substr(0, post_end);
    std::string varvalue;
    const char* v = ::getenv(varname.c_str());
    if (v != nullptr)
        varvalue = std::string(v);
    else
    {
        THROW_EXCEPTION_FMT(
            "YAML parseEnvVars(): Undefined variable found: ${%s}",
            varname.c_str());
    }

    return parseEnvVars(pre + varvalue + post.substr(post_end + 1));
    MRPT_TRY_END
}

static std::string parseCmdRuns(const std::string& text)
{
    MRPT_TRY_START

    const auto start = text.find("$(");
    if (start == std::string::npos) return text;

    const std::string pre  = text.substr(0, start);
    const std::string post = text.substr(start + 2);

    const auto post_end = post.find(')');
    if (post_end == std::string::npos)
    {
        THROW_EXCEPTION_FMT(
            "Column=%u: Cannot find matching `)` for `$(` in: `%s`",
            static_cast<unsigned int>(start), text.c_str());
    }

    const auto cmd = post.substr(0, post_end);

    // Launch command and get console output:
    std::string cmdOut;

    int ret = mrpt::system::executeCommand(cmd, &cmdOut);
    if (ret != 0)
    {
        THROW_EXCEPTION_FMT(
            "Error (retval=%i) executing external command: `%s`", ret,
            cmd.c_str());
    }
    // Clear whitespaces:
    cmdOut = mrpt::system::trim(cmdOut);
    cmdOut.erase(std::remove(cmdOut.begin(), cmdOut.end(), '\r'), cmdOut.end());
    cmdOut.erase(std::remove(cmdOut.begin(), cmdOut.end(), '\n'), cmdOut.end());

    return parseCmdRuns(pre + cmdOut + post.substr(post_end + 1));
    MRPT_TRY_END
}

std::string mola::parseYaml(const std::string& text)
{
    std::string s;

    s = parseCmdRuns(text);
    s = parseEnvVars(s);

    return s;
}
