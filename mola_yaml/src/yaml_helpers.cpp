/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   yaml_helpers.cpp
 * @brief  YAML processing helper utilities
 * @author Jose Luis Blanco Claraco
 * @date   Nov 30, 2018
 */

#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>

#if STD_FS_IS_EXPERIMENTAL
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

// The format of MOLA YAML files is explained in:
// https://docs.mola-slam.org/latest/concept-slam-configuration-file.html

/** \defgroup mola_yaml_grp mola-yaml: YAML parsing helper functions
 */

using mrpt::containers::yaml;

static std::string::size_type findClosing(
    size_t pos, const std::string& s, const char searchEndChar,
    const char otherStartChar)
{
    int openEnvs = 1;
    for (; pos < s.size(); pos++)
    {
        const char ch = s[pos];
        if (ch == otherStartChar)
            openEnvs++;
        else if (ch == searchEndChar)
        {
            openEnvs--;
            if (openEnvs == 0) { return pos; }
        }
    }

    // not found:
    return std::string::npos;
}

// "foo|bar" -> {"foo","bar"}
static std::tuple<std::string, std::string> splitVerticalBar(
    const std::string& s)
{
    const auto posBar = s.find("|");
    if (posBar == std::string::npos) return {s, {}};

    return {s.substr(0, posBar), s.substr(posBar + 1)};
}

static std::string trimWSNL(const std::string& s)
{
    std::string str = s;
    mrpt::system::trim(str);
    str.erase(std::remove(str.begin(), str.end(), '\r'), str.end());
    str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
    return str;
}
std::string mola::yaml_to_string(const mrpt::containers::yaml& cfg)
{
    std::stringstream ss;
    ss << cfg;
    return ss.str();
}

static std::string parseEnvVars(
    const std::string& text, const mola::YAMLParseOptions& opts)
{
    MRPT_TRY_START

    const auto start = text.find("${");
    if (start == std::string::npos) return text;

    const std::string pre  = text.substr(0, start);
    const std::string post = text.substr(start + 2);

    const auto post_end = findClosing(0, post, '}', '{');
    if (post_end == std::string::npos)
    {
        THROW_EXCEPTION_FMT(
            "Column=%u: Cannot find matching `}` for `${` in: `%s`",
            static_cast<unsigned int>(start), text.c_str());
    }

    const auto varnameOrg = post.substr(0, post_end);

    const auto [varname, defaultValue] = splitVerticalBar(varnameOrg);

    std::string varvalue;
    const char* v = ::getenv(varname.c_str());
    if (v != nullptr)
        varvalue = std::string(v);
    else
    {
        // Handle special variable names:
        // ${CURRENT_YAML_FILE_PATH}
        if (varname == "CURRENT_YAML_FILE_PATH")
            varvalue = opts.includesBasePath;
        else if (!defaultValue.empty())
        {
            varvalue = defaultValue;
        }
        else
        {
            THROW_EXCEPTION_FMT(
                "YAML parseEnvVars(): Undefined environment variable: ${%s}",
                varname.c_str());
        }
    }

    return parseEnvVars(pre + varvalue + post.substr(post_end + 1), opts);
    MRPT_TRY_END
}

static std::string parseCmdRuns(
    const std::string& text, const mola::YAMLParseOptions& opts)
{
    MRPT_TRY_START

    const auto start = text.find("$(");
    if (start == std::string::npos) return text;

    const std::string pre  = text.substr(0, start);
    const std::string post = text.substr(start + 2);

    const auto post_end = findClosing(0, post, ')', '(');
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
    cmdOut = trimWSNL(cmdOut);

    return parseCmdRuns(pre + cmdOut + post.substr(post_end + 1), opts);
    MRPT_TRY_END
}

static void recursiveParseNodeForIncludes(
    yaml::node_t& n, const mola::YAMLParseOptions& opts)
{
    if (n.isScalar())
    {
        //
        std::string text = n.as<std::string>();

        const auto start = text.find("$include{");
        if (start == std::string::npos) return;

        const std::string pre  = text.substr(0, start);
        const std::string post = text.substr(start + 9);

        const auto post_end = findClosing(0, post, '}', '{');
        if (post_end == std::string::npos)
        {
            THROW_EXCEPTION_FMT(
                "Column=%u: Cannot find matching `{` for `$include{` in: `%s`",
                static_cast<unsigned int>(start), text.c_str());
        }

        auto expr = post.substr(0, post_end);
        // Solve for possible variables, etc:
        expr = trimWSNL(mola::parse_yaml(expr));

        // Relative vs absolute paths:
        std::string newIncludeBaseDir = opts.includesBasePath;
        if (!opts.includesBasePath.empty())
        {
            fs::path f = expr;
            if (f.is_relative())
            {
                f    = fs::path(opts.includesBasePath) / f;
                expr = f;

                newIncludeBaseDir = fs::path(f).remove_filename();
            }
        }

        // Read external file:
        if (!mrpt::system::fileExists(expr))
        {
            THROW_EXCEPTION_FMT(
                "Error: cannot find $include{}'d YAML file with path `%s`",
                expr.c_str());
        }

        if (getenv("VERBOSE"))
            std::cout << "[recursiveParseNodeForIncludes] Including yaml from `"
                      << expr << "`\n";

        auto filData = yaml::FromFile(expr);

        // Handle possible recursive expressions & replace contents:
        auto newOpts             = opts;
        newOpts.includesBasePath = newIncludeBaseDir;

        n = yaml::FromText(
            mola::parse_yaml(mola::yaml_to_string(filData), newOpts));

        if (getenv("VERBOSE"))
            std::cout << "[recursiveParseNodeForIncludes] Include done ok.\n";
    }
    else if (n.isSequence())
    {
        for (auto& e : n.asSequence()) recursiveParseNodeForIncludes(e, opts);
    }
    else if (n.isMap())
    {
        for (auto& e : n.asMap()) recursiveParseNodeForIncludes(e.second, opts);
    }
}

static std::string parseIncludes(
    const std::string& text, const mola::YAMLParseOptions& opts)
{
    MRPT_TRY_START

    yaml root = yaml::FromText(text);

    recursiveParseNodeForIncludes(root.node(), opts);

    return mola::yaml_to_string(root);

    MRPT_TRY_END
}

mrpt::containers::yaml mola::parse_yaml(
    const mrpt::containers::yaml& input, const mola::YAMLParseOptions& opts)
{
    return mrpt::containers::yaml::FromText(
        parse_yaml(yaml_to_string(input), opts));
}

std::string mola::parse_yaml(
    const std::string& text, const YAMLParseOptions& opts)
{
    std::string s = text;

    // 1) Parse "$include{}"s
    if (opts.doIncludes) s = parseIncludes(s, opts);

    // 2) Parse "$()"s
    if (opts.doCmdRuns) s = parseCmdRuns(s, opts);

    // 3) Parse "${}"s
    if (opts.doEnvVars) s = parseEnvVars(s, opts);

    return s;
}

/* This is equivalent to calling mrpt::containers::yaml::FromFile(), setting the
 * relative path in YAMLParseOptions, calling parseYaml(), and reparsing as a
 * mrpt::containers::yaml class again. \sa parseYaml
 */
mrpt::containers::yaml mola::load_yaml_file(
    const std::string& fileName, const YAMLParseOptions& opts)
{
    MRPT_START
    const auto rawYaml = mrpt::containers::yaml::FromFile(fileName);

    auto optsMod             = opts;
    optsMod.includesBasePath = mrpt::system::extractFileDirectory(fileName);

    return mrpt::containers::yaml::FromText(
        parse_yaml(yaml_to_string(rawYaml), optsMod));
    MRPT_END
}
