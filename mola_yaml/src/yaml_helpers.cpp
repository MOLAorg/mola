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
#include <optional>
#include <sstream>
#include <vector>

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

// ---------------------------------------------------------------------------
// Anonymous-namespace helpers
// ---------------------------------------------------------------------------
namespace
{

/**
 * Find the closing character that balances the *already-open* `otherStartChar`
 * encountered just before position `pos`.
 *
 * The search starts at `pos` inside string `s` with a nesting depth of 1
 * (one opening bracket is assumed to already be open).  Every subsequent
 * `otherStartChar` increments the depth; every `searchEndChar` decrements it.
 * The position of the `searchEndChar` that brings the depth to zero is
 * returned, or `std::string::npos` when no such character is found.
 *
 * Example: findClosing(2, "a${b${c}d}e", '}', '{') returns 9.
 *
 * @param pos            Index in `s` from which to start scanning.
 * @param s              The string to search.
 * @param searchEndChar  The closing character (e.g. `}`).
 * @param otherStartChar The opening character that increases nesting (e.g. `{`).
 * @return               Absolute index of the matching closer, or
 *                       `std::string::npos` if not found.
 */
[[nodiscard]] std::string::size_type findClosing(
    size_t pos, const std::string& s, const char searchEndChar, const char otherStartChar) noexcept
{
  int depth = 1;
  for (const size_t len = s.size(); pos < len; ++pos)
  {
    const char ch = s[pos];
    if (ch == otherStartChar)
    {
      ++depth;
    }
    else if (ch == searchEndChar && --depth == 0)
    {
      return pos;
    }
  }
  return std::string::npos;
}

/**
 * Split `"name|default"` into `{"name", default}`.
 *
 * The second element is `std::nullopt` when no `|` separator is present,
 * and an (possibly empty) `std::string` when one is.  This lets callers
 * distinguish `${VAR}` (no default - throw if undefined) from `${VAR|}`
 * (explicit empty-string default - resolve to `""` rather than throwing).
 */
[[nodiscard]] std::pair<std::string, std::optional<std::string>> splitVerticalBar(
    const std::string& s)
{
  const auto pos = s.find('|');
  if (pos == std::string::npos)
  {
    return {s, std::nullopt};
  }
  return {s.substr(0, pos), s.substr(pos + 1)};
}

/**
 * Strip leading and trailing whitespace (including CR/LF) from a string.
 * Internal content - including embedded newlines in multi-line command output -
 * is preserved unchanged.
 *
 * Used to clean the stdout of `$(command)` substitutions and to normalise
 * `$include{}` path expressions.
 */
[[nodiscard]] std::string trimWSNL(const std::string& s)
{
  std::string str = s;
  // mrpt::system::trim strips leading/trailing ASCII whitespace (space, tab,
  // CR, LF).  The explicit boundary-only erase below handles any platform
  // edge cases where only spaces/tabs are stripped.
  mrpt::system::trim(str);

  // Strip any remaining leading CR/LF (boundary only, not internal).
  const auto isNewline = [](unsigned char c) { return c == '\r' || c == '\n'; };
  str.erase(str.begin(), std::find_if_not(str.begin(), str.end(), isNewline));
  str.erase(std::find_if_not(str.rbegin(), str.rend(), isNewline).base(), str.end());
  return str;
}

/**
 * Return `true` when the character at `pos` in `text` lies on a line whose
 * first non-whitespace content before `pos` is a YAML comment marker (`#`).
 *
 * Scans backwards from `pos` to the beginning of the current line.
 */
[[nodiscard]] bool linePositionIsCommentedOut(const std::string& text, size_t pos) noexcept
{
  while (true)
  {
    if (text[pos] == '#')
    {
      return true;
    }
    if (pos == 0 || text[pos] == '\n' || text[pos] == '\r')
    {
      return false;
    }
    --pos;
  }
}

// ---------------------------------------------------------------------------
// parseVars
// ---------------------------------------------------------------------------

/**
 * Single-pass iterative substitution of `${VAR}` / `${VAR|default}` tokens.
 *
 * Resolution order for each token (first match wins):
 *  1. Real environment variables (`::getenv`).
 *  2. Built-in token `CURRENT_YAML_FILE_PATH` → `opts.includesBasePath`.
 *  3. User-supplied `opts.variables` map.
 *  4. Inline default value after `|` (e.g. `${KEY|fallback}`).
 *  5. Exception - variable not found.
 *
 * Tokens that appear after a `#` comment marker on the same line are left
 * intact (the `${` is rewritten as `$ {` so they won't be matched again).
 *
 * Unlike the previous recursive implementation, substituted values are
 * emitted as-is and are not re-scanned.  This prevents inadvertent
 * second-pass expansion and eliminates the risk of unbounded recursion when
 * a variable's value itself contains `${...}`.
 */
[[nodiscard]] std::string parseVars(const std::string& text, const mola::YAMLParseOptions& opts)
{
  MRPT_TRY_START

  std::string result;
  result.reserve(text.size());  // avoid reallocations for the common case
  size_t pos = 0;

  while (true)
  {
    // Find the next candidate token
    const auto tokenStart = text.find("${", pos);
    if (tokenStart == std::string::npos)
    {
      // No more tokens - flush the remainder and exit
      result.append(text, pos, std::string::npos);
      break;
    }

    // Append literal text that precedes this token
    result.append(text, pos, tokenStart - pos);

    // Locate the matching closing brace (handles nesting such as
    // ${OUTER${INNER}})
    const size_t exprStart = tokenStart + 2;  // first char after "${"
    const auto   exprEnd   = findClosing(exprStart, text, '}', '{');
    if (exprEnd == std::string::npos)
    {
      THROW_EXCEPTION_FMT(
          "Column=%u: Cannot find matching `}` for `${` in: `%s`",
          static_cast<unsigned int>(tokenStart), text.c_str());
    }

    const std::string varExpr = text.substr(exprStart, exprEnd - exprStart);
    pos                       = exprEnd + 1;  // advance past the closing `}`

    // If the token is inside a YAML comment, neutralise it by breaking the
    // `${` pattern and emit it verbatim.
    if (linePositionIsCommentedOut(text, tokenStart))
    {
      result += "$ {";
      result += varExpr;
      result += '}';
      continue;
    }

    const auto [varname, defaultValue] = splitVerticalBar(varExpr);

    // -- Resolution order --
    std::string value;
    if (const char* env = ::getenv(varname.c_str()); env != nullptr)
    {
      value = env;
    }
    else if (varname == "CURRENT_YAML_FILE_PATH")
    {
      value = opts.includesBasePath;
    }
    else if (const auto it = opts.variables.find(varname); it != opts.variables.end())
    {
      value = it->second;
    }
    else if (defaultValue.has_value())
    {
      // Use the inline default (possibly an empty string for `${VAR|}`).
      value = *defaultValue;
    }
    else
    {
      THROW_EXCEPTION_FMT("YAML parseVars(): Undefined variable: ${%s}", varname.c_str());
    }

    result += value;
  }

  return result;
  MRPT_TRY_END
}

// ---------------------------------------------------------------------------
// parseCmdRuns
// ---------------------------------------------------------------------------

/**
 * Single-pass iterative substitution of `$(command)` shell-run tokens.
 *
 * Each token is replaced by the trimmed standard output of `command`.
 * A non-zero exit code causes an exception.  Substituted output is emitted
 * as-is and not re-scanned.
 *
 * Tokens that appear after a `#` comment marker on the same line are left
 * intact (the `$(` is rewritten as `$ (` so they won't be matched again).
 * This mirrors the comment-skipping behaviour of `parseVars` and prevents
 * spurious command execution when `doIncludes` is disabled and the raw YAML
 * text (which still contains comments) is processed directly.
 */
[[nodiscard]] std::string parseCmdRuns(
    const std::string& text, const mola::YAMLParseOptions& /*opts*/)
{
  MRPT_TRY_START

  std::string result;
  result.reserve(text.size());
  size_t pos = 0;

  while (true)
  {
    const auto tokenStart = text.find("$(", pos);
    if (tokenStart == std::string::npos)
    {
      result.append(text, pos, std::string::npos);
      break;
    }

    result.append(text, pos, tokenStart - pos);

    const size_t exprStart = tokenStart + 2;
    const auto   exprEnd   = findClosing(exprStart, text, ')', '(');
    if (exprEnd == std::string::npos)
    {
      THROW_EXCEPTION_FMT(
          "Column=%u: Cannot find matching `)` for `$(` in: `%s`",
          static_cast<unsigned int>(tokenStart), text.c_str());
    }

    const std::string cmd = text.substr(exprStart, exprEnd - exprStart);
    pos                   = exprEnd + 1;

    // If the token is inside a YAML comment, neutralise it verbatim.
    if (linePositionIsCommentedOut(text, tokenStart))
    {
      result += "$ (";
      result += cmd;
      result += ')';
      continue;
    }

    std::string cmdOut;
    const int   ret = mrpt::system::executeCommand(cmd, &cmdOut);
    if (ret != 0)
    {
      THROW_EXCEPTION_FMT("Error (retval=%i) executing external command: `%s`", ret, cmd.c_str());
    }

    result += trimWSNL(cmdOut);
  }

  return result;
  MRPT_TRY_END
}

// ---------------------------------------------------------------------------
// Include processing (tree-walk, inherently recursive over YAML nodes)
// ---------------------------------------------------------------------------

// Thread-local stack of canonical paths currently being included.
// Each entry is the absolute, weakly-canonicalized path of one file in the
// active include chain.  Used to detect and break circular $include{} cycles.
thread_local std::vector<std::string> g_activeIncludes;

/**
 * RAII helper that pushes a canonical path onto `g_activeIncludes` on
 * construction and pops it on destruction (even when an exception unwinds).
 */
struct IncludeGuard
{
  explicit IncludeGuard(const std::string& path) { g_activeIncludes.push_back(path); }
  ~IncludeGuard() { g_activeIncludes.pop_back(); }

  // Non-copyable, non-movable.
  IncludeGuard(const IncludeGuard&)            = delete;
  IncludeGuard& operator=(const IncludeGuard&) = delete;
};

/**
 * Recursively walk a `yaml::node_t` tree and expand any scalar nodes whose
 * text matches `$include{path}`.
 *
 * When such a scalar is found:
 *  1. The path expression is itself pre-processed with `parse_yaml` so that
 *     variable/command substitutions within the path work correctly.
 *  2. The referenced file is loaded, fully pre-processed (including nested
 *     includes), and the current node is replaced by the loaded YAML.
 *
 * Relative paths are resolved against `opts.includesBasePath`.  The base
 * path is updated on each recursive descent into an included file, so that
 * includes *within* included files resolve relative to their own location.
 *
 * @note  Recursion here is over the YAML node tree (depth bounded by the
 *        document structure), not over the string - safe in practice.
 */
void recursiveProcessIncludes(yaml::node_t& n, const mola::YAMLParseOptions& opts)
{
  if (n.isScalar())
  {
    const std::string text = n.as<std::string>();

    const auto tokenStart = text.find("$include{");
    if (tokenStart == std::string::npos) return;

    const size_t exprStart = tokenStart + 9;  // length of "$include{"
    const auto   exprEnd   = findClosing(exprStart, text, '}', '{');
    if (exprEnd == std::string::npos)
    {
      THROW_EXCEPTION_FMT(
          "Column=%u: Cannot find matching `}` for `$include{` in: `%s`",
          static_cast<unsigned int>(tokenStart), text.c_str());
    }

    // Resolve any variable/command expressions inside the path itself
    std::string expr = text.substr(exprStart, exprEnd - exprStart);
    expr             = trimWSNL(mola::parse_yaml(expr, opts));  // pass opts!

    // Resolve relative paths against the current include base.
    if (!opts.includesBasePath.empty())
    {
      fs::path p = expr;
      if (p.is_relative())
      {
        p    = fs::path(opts.includesBasePath) / p;
        expr = p.string();
      }
    }

    // Always derive the nested base dir from the resolved (possibly absolute)
    // expr, not from the parent's base path.  This ensures that nested
    // relative $include{} directives inside the included file resolve relative
    // to *that* file's own directory, regardless of whether expr was absolute
    // or relative from the caller's perspective.
    const std::string newIncludeBaseDir = fs::path(expr).remove_filename().string();

    if (!mrpt::system::fileExists(expr))
    {
      THROW_EXCEPTION_FMT("Cannot find $include{}'d YAML file: `%s`", expr.c_str());
    }

    // Detect circular includes: resolve to a canonical path and check whether
    // it is already in the active include chain for this thread.
    const std::string canonicalExpr = fs::weakly_canonical(fs::path(expr)).string();

    for (const auto& active : g_activeIncludes)
    {
      if (active == canonicalExpr)
      {
        THROW_EXCEPTION_FMT(
            "Circular $include{} detected: `%s` is already being included", canonicalExpr.c_str());
      }
    }

    // Push this file onto the active-include stack; popped automatically on
    // scope exit (including exception unwind) via IncludeGuard.
    IncludeGuard includeGuard(canonicalExpr);

    if (::getenv("VERBOSE"))
    {
      std::cout << "[mola::parse_yaml] $include{ \"" << expr << "\" }\n";
    }

    // Load the file, update the base path for nested includes, pre-process
    // the included content, then replace the current node.
    const auto filData = yaml::FromFile(expr);

    auto nestedOpts             = opts;
    nestedOpts.includesBasePath = newIncludeBaseDir;

    n = yaml::FromText(mola::parse_yaml(mola::yaml_to_string(filData), nestedOpts));

    if (::getenv("VERBOSE"))
    {
      std::cout << "[mola::parse_yaml] $include done: \"" << expr << "\"\n";
    }
  }
  else if (n.isSequence())
  {
    for (auto& element : n.asSequence())
    {
      recursiveProcessIncludes(element, opts);
    }
  }
  else if (n.isMap())
  {
    for (auto& [key, value] : n.asMap())
    {
      recursiveProcessIncludes(value, opts);
    }
  }
}

/**
 * Entry point for the include-expansion pass.
 *
 * Parses `text` into a temporary YAML tree, walks it with
 * `recursiveProcessIncludes`, then serializes the result back to a string
 * for the subsequent variable/command-substitution passes.
 */
[[nodiscard]] std::string parseIncludes(const std::string& text, const mola::YAMLParseOptions& opts)
{
  MRPT_TRY_START

  yaml root = yaml::FromText(text);
  recursiveProcessIncludes(root.node(), opts);
  return mola::yaml_to_string(root);

  MRPT_TRY_END
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

std::string mola::yaml_to_string(const mrpt::containers::yaml& cfg)
{
  std::ostringstream ss;
  ss << cfg;
  return ss.str();
}

mrpt::containers::yaml mola::parse_yaml(
    const mrpt::containers::yaml& input, const mola::YAMLParseOptions& opts)
{
  return mrpt::containers::yaml::FromText(parse_yaml(yaml_to_string(input), opts));
}

std::string mola::parse_yaml(const std::string& text, const YAMLParseOptions& opts)
{
  std::string s = text;

  // Pass 1: Expand $include{} directives.
  // Must run first so that variable/command substitutions in included files
  // are handled by the subsequent passes on the fully assembled text.
  if (opts.doIncludes) s = parseIncludes(s, opts);

  // Pass 2: Expand $(command) shell-run substitutions.
  if (opts.doCmdRuns) s = parseCmdRuns(s, opts);

  // Pass 3: Expand ${VAR} / ${VAR|default} variable substitutions.
  if (opts.doEnvVars) s = parseVars(s, opts);

  return s;
}

mrpt::containers::yaml mola::load_yaml_file(
    const std::string& fileName, const YAMLParseOptions& opts)
{
  MRPT_START

  const auto rawYaml = mrpt::containers::yaml::FromFile(fileName);

  // Derive the base path from the file's own directory, but only when the
  // caller did not already supply one.  A non-empty caller-provided path
  // takes precedence so that callers can anchor includes to a different
  // root (e.g. a workspace directory that differs from the file location).
  auto effectiveOpts = opts;
  if (effectiveOpts.includesBasePath.empty())
  {
    effectiveOpts.includesBasePath = mrpt::system::extractFileDirectory(fileName);
  }

  return mrpt::containers::yaml::FromText(parse_yaml(yaml_to_string(rawYaml), effectiveOpts));

  MRPT_END
}