/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   env_vars.cpp
 * @brief  Parse environment variables in config text blocks
 * @author Jose Luis Blanco Claraco
 * @date   Nov 30, 2018
 */

#include <mola-kernel/env_vars.h>
#include <mrpt/core/exceptions.h>
#include <cstdlib>

MRPT_TODO("Add a more generic YAML file parser: includes, etc.");

std::string mola::parseEnvVars(const std::string& text)
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
    if (v != nullptr) varvalue = std::string(v);

    return parseEnvVars(pre + varvalue + post.substr(post_end + 1));
    MRPT_TRY_END
}
