/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   yaml_helpers.h
 * @brief  YAML processing helper utilities
 * @author Jose Luis Blanco Claraco
 * @date   Nov 30, 2018
 */
#pragma once

#include <sstream>
#include <string>
#include "macro_helpers.h"

namespace mola
{
/** For use in parseYaml() */
struct YAMLParseOptions
{
    bool doIncludes{true};  //!< "$include{}"s
    bool doCmdRuns{true};  //!< "$()"s
    bool doEnvVars{true};  //!< "${}"s
};

/** Parses: system run expressions `$(cmd)`, environment variables `${VAR}`.
 * \ingroup mola_kernel_grp
 */
std::string parseYaml(
    const std::string& text, const YAMLParseOptions& opts = YAMLParseOptions());

/** Converts a yamlcpp node into a string
 * \ingroup mola_kernel_grp
 */
template <class YAML_CLASS>
inline std::string yaml2string(const YAML_CLASS& cfg)
{
    std::stringstream ss;
    ss << cfg;
    return ss.str();
}

#define ENSURE_YAML_ENTRY_EXISTS(_c, _name) \
    ASSERTMSG_(_c[_name], "Missing YAML required entry: `" _name "`")

/** Loads (optional) variable named "_varname" from the YAML config named `cfg`
 * into the variable `_param_str._varname` */
#define YAML_LOAD_OPT3(_param_str, _varname, _type) \
    _param_str._varname = cfg[#_varname].as<_type>(_param_str._varname)

#define YAML_LOAD_OPT2(_varname, _type) \
    _varname = cfg[#_varname].as<_type>(_varname)

/** Use `YAML_LOAD_MEMBER_OPT(foo,double);` to load YAML var `foo` into `foo_`
 */
#define YAML_LOAD_MEMBER_OPT(_varname, _type) \
    _varname##_ = cfg[#_varname].as<_type>(_varname##_)

/** Loads (required) variable named "_varname" from the YAML config named `cfg`
 * into the variable `_param_str._varname` */
#define YAML_LOAD_REQ3(_param_str, _varname, _type) \
    ENSURE_YAML_ENTRY_EXISTS(cfg, #_varname);       \
    YAML_LOAD_OPT3(_param_str, _varname, _type)

#define YAML_LOAD_REQ2(_varname, _type)       \
    ENSURE_YAML_ENTRY_EXISTS(cfg, #_varname); \
    YAML_LOAD_OPT2(_varname, _type)

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

}  // namespace mola
