/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   yaml_helpers.h
 * @brief  YAML processing helper utilities
 * @author Jose Luis Blanco Claraco
 * @date   Nov 30, 2018
 */
#pragma once

#include <mola_yaml/macro_helpers.h>
#include <mrpt/containers/yaml.h>

#include <sstream>
#include <string>

namespace mola
{
/** \addtogroup mola_yaml_grp
 * @{*/

/** For use in parseYaml() */
struct YAMLParseOptions
{
    bool doIncludes{true};  //!< "$include{}"s
    bool doCmdRuns{true};  //!< "$()"s
    bool doEnvVars{true};  //!< "${}"s

    /** If not empty, base reference path which respect to "$include{}"s are
     * specified. Automatically filled in by load_yaml_file() */
    std::string includesBasePath;
};

/** Parses: system run expressions `$(cmd)`, environment variables `${VAR}`.
 */
[[nodiscard]] std::string parse_yaml(
    const std::string& text, const YAMLParseOptions& opts = YAMLParseOptions());

/** \overload (Version taking an mrpt::containers::yaml as input and output)
 */
[[nodiscard]] mrpt::containers::yaml parse_yaml(
    const mrpt::containers::yaml& input,
    const YAMLParseOptions&       opts = YAMLParseOptions());

/** Loads and parses a YAML file.
 *
 * This is equivalent to calling mrpt::containers::yaml::FromFile(), setting the
 * relative path in YAMLParseOptions, calling parseYaml(), and reparsing as a
 * mrpt::containers::yaml class again. \sa parseYaml
 */
[[nodiscard]] mrpt::containers::yaml load_yaml_file(
    const std::string&      fileName,
    const YAMLParseOptions& opts = YAMLParseOptions());

/** Converts a yaml node into a string
 */
[[nodiscard]] std::string yaml_to_string(const mrpt::containers::yaml& cfg);

#define ENSURE_YAML_ENTRY_EXISTS(_c, _name) \
    ASSERTMSG_(                             \
        _c.has(_name),                      \
        mrpt::format(                       \
            "Missing YAML required entry: `%s`", std::string(_name).c_str()))

/** Loads (optional) variable named "_varname" from the YAML config named `cfg`
 * into the variable `_param_str._varname` */
#define YAML_LOAD_OPT3(_param_str, _varname, _type) \
    _param_str._varname =                           \
        cfg.getOrDefault<_type>(#_varname, _param_str._varname)

#define YAML_LOAD_OPT2(_varname, _type) \
    _varname = cfg.getOrDefault<_type>(#_varname, _varname)

#define YAML_LOAD_OPT_DEG2(_varname, _type) \
    _varname = mrpt::DEG2RAD(               \
        cfg.getOrDefault<_type>(#_varname, mrpt::RAD2DEG(_varname)))

/** Use `YAML_LOAD_MEMBER_OPT(foo,double);` to load YAML var `foo` into `foo_`
 */
#define YAML_LOAD_MEMBER_OPT(_varname, _type) \
    _varname##_ = cfg.getOrDefault<_type>(#_varname, _varname##_)

/** Loads (required) variable named "_varname" from the YAML config named `cfg`
 * into the variable `_param_str._varname` */
#define YAML_LOAD_REQ3(_param_str, _varname, _type) \
    ENSURE_YAML_ENTRY_EXISTS(cfg, #_varname);       \
    YAML_LOAD_OPT3(_param_str, _varname, _type)

#define YAML_LOAD_REQ2(_varname, _type)       \
    ENSURE_YAML_ENTRY_EXISTS(cfg, #_varname); \
    YAML_LOAD_OPT2(_varname, _type)

#define YAML_LOAD_REQ_DEG2(_varname, _type)   \
    ENSURE_YAML_ENTRY_EXISTS(cfg, #_varname); \
    YAML_LOAD_OPT_DEG2(_varname, _type)

/** Use `YAML_LOAD_MEMBER_REQ(foo,double);` to load YAML var `foo` into `foo_`
 */
#define YAML_LOAD_MEMBER_REQ(_varname, _type) \
    ENSURE_YAML_ENTRY_EXISTS(cfg, #_varname); \
    YAML_LOAD_MEMBER_OPT(_varname, _type)

/** like YAML_LOAD_OPT, values in the YAML file in "degrees" stored in rads */
#define YAML_LOAD_OPT_DEG3(_param_str, _varname, _type)       \
    _param_str._varname = mrpt::RAD2DEG(_param_str._varname); \
    YAML_LOAD_OPT3(_param_str, _varname, _type);              \
    _param_str._varname = mrpt::DEG2RAD(_param_str._varname)

/** like YAML_LOAD_REQ, values in the YAML file in "degrees" stored in rads */
#define YAML_LOAD_REQ_DEG3(_param_str, _varname, _type)       \
    _param_str._varname = mrpt::RAD2DEG(_param_str._varname); \
    YAML_LOAD_REQ(_param_str, _varname, _type);               \
    _param_str._varname = mrpt::DEG2RAD(_param_str._varname)

#define YAML_LOAD_OPT(...) VFUNC(YAML_LOAD_OPT, __VA_ARGS__)
#define YAML_LOAD_REQ(...) VFUNC(YAML_LOAD_REQ, __VA_ARGS__)
#define YAML_LOAD_OPT_DEG(...) VFUNC(YAML_LOAD_OPT_DEG, __VA_ARGS__)
#define YAML_LOAD_REQ_DEG(...) VFUNC(YAML_LOAD_REQ_DEG, __VA_ARGS__)

/** @} */

}  // namespace mola
