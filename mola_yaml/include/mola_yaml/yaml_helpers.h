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
 * @file   yaml_helpers.h
 * @brief  YAML processing helper utilities
 * @author Jose Luis Blanco Claraco
 * @date   Nov 30, 2018
 *
 * Provides:
 *  - `mola::parse_yaml()`: string/node pre-processor that expands
 *    `$include{}`, `$()`, and `${}` expressions (in that order).
 *  - `mola::load_yaml_file()`: load and fully pre-process a YAML file.
 *  - `mola::yaml_to_string()`: serialize an `mrpt::containers::yaml` node.
 *  - A family of `YAML_LOAD_OPT` / `YAML_LOAD_REQ` helper macros that
 *    reduce boilerplate when reading parameters from a `cfg` YAML node.
 */
#pragma once

#include <mola_yaml/macro_helpers.h>
#include <mrpt/containers/yaml.h>

#include <map>
#include <string>

namespace mola
{
/** \addtogroup mola_yaml_grp
 * @{*/

// ---------------------------------------------------------------------------
// YAMLParseOptions
// ---------------------------------------------------------------------------

/**
 * @brief Options controlling which pre-processing steps `parse_yaml()` runs.
 *
 * Pre-processing is performed in the following fixed order:
 *  1. **`$include{path}`** - replaced with the contents of the referenced
 *     YAML file (recursive; relative paths resolved against
 *     `includesBasePath`).
 *  2. **`$(cmd)`** - replaced with the trimmed stdout of the shell command
 *     `cmd` (exit code ≠ 0 throws).
 *  3. **`${VAR}`** or **`${VAR|default}`** - replaced with the value of
 *     an environment variable, a custom `variables` entry, or the built-in
 *     `CURRENT_YAML_FILE_PATH` token (falls back to `default` when given,
 *     otherwise throws if the variable is not defined).
 *
 * Any of these steps can be individually disabled via the boolean flags
 * below.
 */
struct YAMLParseOptions
{
  /** If `true` (default), process `$include{path}` directives. */
  bool doIncludes{true};

  /** If `true` (default), process `$(command)` shell-run substitutions. */
  bool doCmdRuns{true};

  /** If `true` (default), process `${VAR}` / `${VAR|default}` variable
   *  substitutions from the environment, `variables`, and built-in tokens. */
  bool doEnvVars{true};

  /**
   * @brief User-defined variable bindings for `${name}` substitutions.
   *
   * Entries in this map are checked *after* the real environment variables
   * and the built-in `CURRENT_YAML_FILE_PATH` token, but *before* the
   * `|default` fallback syntax.
   */
  std::map<std::string, std::string> variables;

  /**
   * @brief Base directory used to resolve relative `$include{}` paths.
   *
   * When non-empty, any relative path inside a `$include{}` expression is
   * resolved relative to this directory.  `load_yaml_file()` sets this
   * automatically to the directory of the file being loaded.
   */
  std::string includesBasePath;
};

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/**
 * @brief Pre-process a raw YAML string, expanding `$include{}`, `$()`, and
 *        `${}` expressions according to `opts`.
 *
 * The three substitution passes are always run in the order listed in
 * `YAMLParseOptions` (includes → command runs → variables), regardless of
 * the order in which the expressions appear in the text.  Lines that begin
 * with a YAML comment character (`#`) are skipped for variable expansion.
 *
 * @param text  Raw YAML text to process.
 * @param opts  Controls which substitutions are active and supplies
 *              custom variables / the include base path.
 * @return      Fully substituted YAML text (still a plain string; call
 *              `mrpt::containers::yaml::FromText()` to parse it).
 * @throws      `std::exception` if a required variable is undefined, a
 *              shell command exits with a non-zero status, or a referenced
 *              include file cannot be found.
 */
[[nodiscard]] std::string parse_yaml(
    const std::string& text, const YAMLParseOptions& opts = YAMLParseOptions());

/**
 * @brief Overload that accepts and returns an `mrpt::containers::yaml` node.
 *
 * Serializes `input` to text, runs the same pre-processing as the
 * string overload, then re-parses the result back into a
 * `mrpt::containers::yaml` node.
 *
 * @param input  Source YAML node.
 * @param opts   See string overload.
 * @return       New node with all substitutions applied.
 */
[[nodiscard]] mrpt::containers::yaml parse_yaml(
    const mrpt::containers::yaml& input, const YAMLParseOptions& opts = YAMLParseOptions());

/**
 * @brief Load a YAML file from disk and fully pre-process it.
 *
 * This is a convenience wrapper equivalent to:
 *  1. `mrpt::containers::yaml::FromFile(fileName)`
 *  2. Setting `YAMLParseOptions::includesBasePath` to the file's directory.
 *  3. `parse_yaml(yaml_to_string(raw), opts)`
 *  4. Re-parsing the result with `mrpt::containers::yaml::FromText()`.
 *
 * @param fileName  Path to the YAML file to load.
 * @param opts      Additional parse options merged with the auto-detected
 *                  `includesBasePath`.
 * @return          Parsed and fully pre-processed YAML node.
 */
[[nodiscard]] mrpt::containers::yaml load_yaml_file(
    const std::string& fileName, const YAMLParseOptions& opts = YAMLParseOptions());

/**
 * @brief Serialize an `mrpt::containers::yaml` node to a string.
 *
 * Uses the node's built-in stream operator.  The result can be re-parsed
 * with `mrpt::containers::yaml::FromText()`.
 *
 * @param cfg  Node to serialize.
 * @return     YAML text representation.
 */
[[nodiscard]] std::string yaml_to_string(const mrpt::containers::yaml& cfg);

// ---------------------------------------------------------------------------
// Helper macros for reading parameters from a `cfg` YAML node
// ---------------------------------------------------------------------------

/**
 * @brief Assert that a required key exists in the YAML node `_c`.
 *
 * Throws a descriptive error when the key `_name` is missing.
 *
 * @param _c     `mrpt::containers::yaml` node to check (must support `.has()`).
 * @param _name  Key name as a string literal or `std::string`.
 */
#define ENSURE_YAML_ENTRY_EXISTS(_c, _name) \
  ASSERTMSG_(                               \
      _c.has(_name),                        \
      mrpt::format("Missing YAML required entry: `%s`", std::string(_name).c_str()))

// ---------------------------------------------------------------------------
// 3-argument forms (load into a sub-struct field):
//   YAML_LOAD_OPT(_struct, _field, _type)
//   YAML_LOAD_REQ(_struct, _field, _type)
// ---------------------------------------------------------------------------

/**
 * @brief **Optional** load: `_param_str._varname = cfg[_varname]` (or keep
 *        the current default when the key is absent).
 *
 * @param _param_str  Struct whose field will receive the value.
 * @param _varname    Field / YAML key name (used as both identifier and string).
 * @param _type       C++ type for the `getOrDefault` call.
 */
#define YAML_LOAD_OPT3(_param_str, _varname, _type) \
  _param_str._varname = cfg.getOrDefault<_type>(#_varname, _param_str._varname)

/**
 * @brief **Required** load into a struct field - throws if the key is absent.
 *
 * @param _param_str  Struct whose field will receive the value.
 * @param _varname    Field / YAML key name.
 * @param _type       C++ type for the `getOrDefault` call.
 */
#define YAML_LOAD_REQ3(_param_str, _varname, _type) \
  do                                                \
  {                                                 \
    ENSURE_YAML_ENTRY_EXISTS(cfg, #_varname);       \
    YAML_LOAD_OPT3(_param_str, _varname, _type);    \
  } while (0)

// ---------------------------------------------------------------------------
// 2-argument forms (load into a local / member variable directly):
//   YAML_LOAD_OPT(_varname, _type)
//   YAML_LOAD_REQ(_varname, _type)
// ---------------------------------------------------------------------------

/**
 * @brief **Optional** load: `_varname = cfg[_varname]` (keeps default when
 *        the key is absent).
 *
 * @param _varname  Variable name - also used verbatim as the YAML key.
 * @param _type     C++ type for the `getOrDefault` call.
 */
#define YAML_LOAD_OPT2(_varname, _type) _varname = cfg.getOrDefault<_type>(#_varname, _varname)

/**
 * @brief **Required** load into a local variable - throws if the key is absent.
 *
 * @param _varname  Variable name / YAML key.
 * @param _type     C++ type for the `getOrDefault` call.
 */
#define YAML_LOAD_REQ2(_varname, _type)       \
  do                                          \
  {                                           \
    ENSURE_YAML_ENTRY_EXISTS(cfg, #_varname); \
    YAML_LOAD_OPT2(_varname, _type);          \
  } while (0)

// ---------------------------------------------------------------------------
// DEG variants: YAML value is in degrees, stored variable is in radians.
// ---------------------------------------------------------------------------

/**
 * @brief **Optional** load of an angle: YAML stores degrees, variable holds
 *        radians (2-arg form - local variable).
 */
#define YAML_LOAD_OPT_DEG2(_varname, _type) \
  _varname = mrpt::DEG2RAD(cfg.getOrDefault<_type>(#_varname, mrpt::RAD2DEG(_varname)))

/**
 * @brief **Required** load of an angle: YAML stores degrees, variable holds
 *        radians (2-arg form - local variable).  Throws if key is absent.
 */
#define YAML_LOAD_REQ_DEG2(_varname, _type)   \
  do                                          \
  {                                           \
    ENSURE_YAML_ENTRY_EXISTS(cfg, #_varname); \
    YAML_LOAD_OPT_DEG2(_varname, _type);      \
  } while (0)

/**
 * @brief **Optional** load of an angle into a struct field (3-arg form).
 *        YAML stores degrees; `_param_str._varname` holds radians.
 *
 * Uses a temporary to avoid leaving `_param_str._varname` mutated to degrees
 * if the YAML read throws mid-way.
 */
#define YAML_LOAD_OPT_DEG3(_param_str, _varname, _type)                     \
  do                                                                        \
  {                                                                         \
    const _type _mola_deg_tmp_ = cfg.getOrDefault<_type>(                   \
        #_varname, static_cast<_type>(mrpt::RAD2DEG(_param_str._varname))); \
    _param_str._varname = mrpt::DEG2RAD(_mola_deg_tmp_);                    \
  } while (0)

/**
 * @brief **Required** load of an angle into a struct field (3-arg form).
 *        YAML stores degrees; `_param_str._varname` holds radians.
 *        Throws if the key is absent.
 *
 * Uses a temporary so `_param_str._varname` is never left in degrees if
 * `ENSURE_YAML_ENTRY_EXISTS` or the YAML read throws.
 */
#define YAML_LOAD_REQ_DEG3(_param_str, _varname, _type)                     \
  do                                                                        \
  {                                                                         \
    ENSURE_YAML_ENTRY_EXISTS(cfg, #_varname);                               \
    const _type _mola_deg_tmp_ = cfg.getOrDefault<_type>(                   \
        #_varname, static_cast<_type>(mrpt::RAD2DEG(_param_str._varname))); \
    _param_str._varname = mrpt::DEG2RAD(_mola_deg_tmp_);                    \
  } while (0)

// ---------------------------------------------------------------------------
// Member variable forms: `foo_` ← `cfg["foo"]`
// ---------------------------------------------------------------------------

/**
 * @brief **Optional** load into a member variable `_varname_`.
 *
 * Use `YAML_LOAD_MEMBER_OPT(foo, double)` inside a class method to load
 * the YAML key `"foo"` into the member `foo_`.
 *
 * @param _varname  Base name (YAML key = `#_varname`; member = `_varname##_`).
 * @param _type     C++ type for the `getOrDefault` call.
 */
#define YAML_LOAD_MEMBER_OPT(_varname, _type) \
  _varname##_ = cfg.getOrDefault<_type>(#_varname, _varname##_)

/**
 * @brief **Required** load into a member variable `_varname_`.
 *
 * Throws when the key is absent.
 *
 * @param _varname  Base name (YAML key = `#_varname`; member = `_varname##_`).
 * @param _type     C++ type for the `getOrDefault` call.
 */
#define YAML_LOAD_MEMBER_REQ(_varname, _type) \
  do                                          \
  {                                           \
    ENSURE_YAML_ENTRY_EXISTS(cfg, #_varname); \
    YAML_LOAD_MEMBER_OPT(_varname, _type);    \
  } while (0)

// ---------------------------------------------------------------------------
// Variadic entry-points: dispatch to the 2- or 3-argument form automatically.
// ---------------------------------------------------------------------------

/**
 * @brief Variadic `YAML_LOAD_OPT`: 2-arg or 3-arg form selected by count.
 *
 * - `YAML_LOAD_OPT(var, Type)`           → `YAML_LOAD_OPT2`
 * - `YAML_LOAD_OPT(params, var, Type)`   → `YAML_LOAD_OPT3`
 */
#define YAML_LOAD_OPT(...) VFUNC(YAML_LOAD_OPT, __VA_ARGS__)

/**
 * @brief Variadic `YAML_LOAD_REQ`: 2-arg or 3-arg form selected by count.
 *
 * - `YAML_LOAD_REQ(var, Type)`           → `YAML_LOAD_REQ2`
 * - `YAML_LOAD_REQ(params, var, Type)`   → `YAML_LOAD_REQ3`
 */
#define YAML_LOAD_REQ(...) VFUNC(YAML_LOAD_REQ, __VA_ARGS__)

/**
 * @brief Variadic angle load (optional): 2-arg or 3-arg form selected by
 *        count.
 */
#define YAML_LOAD_OPT_DEG(...) VFUNC(YAML_LOAD_OPT_DEG, __VA_ARGS__)

/**
 * @brief Variadic angle load (required): 2-arg or 3-arg form selected by
 *        count.
 */
#define YAML_LOAD_REQ_DEG(...) VFUNC(YAML_LOAD_REQ_DEG, __VA_ARGS__)

/** @} */

}  // namespace mola