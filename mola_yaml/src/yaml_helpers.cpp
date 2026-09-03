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
#include <mrpt/core/get_env.h>
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
 * Recursively deep-merge `overlay` on top of `base` (in place).
 *
 *  - When both nodes are maps, keys are merged recursively: a key present only
 *    in `overlay` is added; a key present in both is merged by recursing.
 *  - For any other combination (scalar, sequence, or type mismatch), `overlay`
 *    REPLACES `base` wholesale.
 *
 * This is the override semantics of the `$import` directive: sibling entries
 * override the imported base, and nested maps are merged deeply so a single
 * deep key can be overridden without restating its whole subtree.
 */
void deepMergeNode(yaml::node_t& base, const yaml::node_t& overlay)
{
  if (base.isMap() && overlay.isMap())
  {
    // `yaml::map_t` is a vector-backed, insertion-order structure (not
    // associative), so lookups go through the `yaml_ref` wrapper's has()/
    // operator[] rather than through asMap() directly.
    mrpt::containers::yaml_ref baseRef(base);
    for (const auto& [key, value] : overlay.asMap())
    {
      const auto keyStr = key.as<std::string>();
      if (baseRef.has(keyStr))
      {
        deepMergeNode(baseRef[keyStr].node(), value);
      }
      else
      {
        baseRef[keyStr] = yaml(value);
      }
    }
  }
  else
  {
    base = overlay;
  }
}

/**
 * Resolve `pathExpr` (which may itself contain `${}` / `$()` expressions and be
 * a relative path) and load + fully pre-process the referenced YAML file,
 * returning the resulting node.
 *
 * Shared by the `$include{path}` scalar directive (whole-node replacement) and
 * the `$import` map directive (structural merge). Relative paths resolve against
 * `opts.includesBasePath`; nested includes/imports inside the loaded file
 * resolve against that file's own directory. Circular references are detected
 * via the thread-local `g_activeIncludes` stack.
 */
[[nodiscard]] yaml::node_t loadExternalYaml(
    const std::string& pathExpr, const mola::YAMLParseOptions& opts)
{
  // Resolve any variable/command expressions inside the path itself.
  std::string expr = trimWSNL(mola::parse_yaml(pathExpr, opts));

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

  // Derive the nested base dir from the resolved path so that relative
  // includes/imports inside the loaded file resolve relative to its own dir.
  const std::string newIncludeBaseDir = fs::path(expr).remove_filename().string();

  if (!mrpt::system::fileExists(expr))
  {
    THROW_EXCEPTION_FMT("Cannot find referenced YAML file: `%s`", expr.c_str());
  }

  // Detect circular references via the active include/import chain.
  const std::string canonicalExpr = fs::weakly_canonical(fs::path(expr)).string();
  for (const auto& active : g_activeIncludes)
  {
    if (active == canonicalExpr)
    {
      THROW_EXCEPTION_FMT(
          "Circular include/import detected: `%s` is already being processed",
          canonicalExpr.c_str());
    }
  }
  IncludeGuard includeGuard(canonicalExpr);

  if (mrpt::get_env<bool>("MOLA_YAML_VERBOSE", false))
  {
    std::cout << "[mola::parse_yaml] loading external YAML: \"" << expr << "\"\n";
  }

  const auto filData = yaml::FromFile(expr);

  auto nestedOpts             = opts;
  nestedOpts.includesBasePath = newIncludeBaseDir;

  return yaml::FromText(mola::parse_yaml(mola::yaml_to_string(filData), nestedOpts)).node();
}

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
void recursiveProcessIncludes(yaml::node_t& n, const mola::YAMLParseOptions& opts);

/**
 * Handle the directives of a MAP node: the `$import` merge, or plain recursion
 * into the children. Split out of `recursiveProcessIncludes()` so that the
 * `$define` scope handling can wrap it (the `$import` branch returns early).
 */
void processMapDirectives(yaml::node_t& n, const mola::YAMLParseOptions& opts)
{
  auto& m = n.asMap();

  // `$import` directive: a map whose `$import` key names one (scalar) or
  // several (sequence) external YAML files is REPLACED by the deep-merge of
  // those files (in listed order) with the map's REMAINING keys overlaid on
  // top. So the imported file(s) act as a base, and the sibling entries
  // OVERRIDE particular entries (nested maps merge deeply; scalars/sequences
  // replace). This is what lets `params:` reference a shared file and then
  // tweak just a few keys, instead of duplicating the whole block.
  // `yaml::map_t` is a vector-backed, insertion-order structure (not
  // associative), so lookups use std::find_if over key names rather than
  // `map_t::find()`, which does not exist.
  const auto itImport = std::find_if(
      m.begin(), m.end(),
      [](const auto& kv)
      { return kv.first.isScalar() && kv.first.template as<std::string>() == "$import"; });
  if (itImport != m.end())
  {
    std::vector<std::string> importPaths;
    const auto&              importVal = itImport->second;
    if (importVal.isScalar())
    {
      importPaths.push_back(importVal.as<std::string>());
    }
    else if (importVal.isSequence())
    {
      for (const auto& element : importVal.asSequence())
      {
        importPaths.push_back(element.as<std::string>());
      }
    }
    else
    {
      THROW_EXCEPTION("`$import` value must be a file path string or a sequence of paths.");
    }

    // Base: deep-merge of the imported file(s), in the order listed.
    yaml::node_t merged = yaml::Map();
    for (const auto& path : importPaths)
    {
      const yaml::node_t loaded = loadExternalYaml(path, opts);
      deepMergeNode(merged, loaded);
    }

    // Overlay: the remaining sibling keys (recursively pre-processed first),
    // which override the imported base.
    for (auto& [key, value] : m)
    {
      if (key.isScalar() && key.as<std::string>() == "$import")
      {
        continue;
      }
      recursiveProcessIncludes(value, opts);
      yaml::node_t               single = yaml::Map();
      mrpt::containers::yaml_ref singleRef(single);
      singleRef[key.as<std::string>()] = yaml(value);
      deepMergeNode(merged, single);
    }

    n = merged;
    return;
  }

  for (auto& [key, value] : m)
  {
    recursiveProcessIncludes(value, opts);
  }
}

/**
 * Consume a `$define` key from map `m`, returning a copy of `opts` augmented
 * with its `NAME: VALUE` pairs as `${NAME}` variables, and erasing the key from
 * `m` so it does not reach the output. Returns `std::nullopt` when the map has
 * no `$define` key.
 *
 * The values may themselves contain `${}` / `$()` expressions; they are resolved
 * against the OUTER scope, so the result never depends on the order in which the
 * map entries happen to be visited.
 *
 * OUTER-WINS ACROSS NESTING: a name already present in `opts.variables` -
 * whether set by an ancestor `$define` (however many `$import` levels up) or
 * supplied by the caller before parsing started - is left untouched; this
 * block's own entry for that name is skipped. This mirrors, one level up,
 * "environment > $define > inline default" for a single `${VAR|default}`
 * token: the scope closer to the document root / the caller has final say,
 * not the file it happens to import. Without this, a reusable imported
 * fragment that `$define`s one of its own hooks (to change ONLY its inline
 * default) would silently and permanently shadow that same hook for every
 * file that imports it, with no way for an importer to override it short of
 * restating the whole target key as a literal sibling value.
 */
[[nodiscard]] std::optional<mola::YAMLParseOptions> consumeDefineBlock(
    yaml::map_t& m, const mola::YAMLParseOptions& opts)
{
  // `yaml::map_t` is a vector-backed, insertion-order structure (not
  // associative), so lookups use std::find_if over key names.
  const auto itDefine = std::find_if(
      m.begin(), m.end(),
      [](const auto& kv)
      { return kv.first.isScalar() && kv.first.template as<std::string>() == "$define"; });
  if (itDefine == m.end())
  {
    return std::nullopt;
  }

  auto scopedOpts = opts;

  if (!itDefine->second.isMap())
  {
    THROW_EXCEPTION("`$define` value must be a map of `NAME: VALUE` entries.");
  }

  // Expand the values with the outer scope, but without the include pass: the
  // values are plain scalars, not YAML documents.
  auto valueOpts       = opts;
  valueOpts.doIncludes = false;

  for (const auto& [key, value] : itDefine->second.asMap())
  {
    if (!key.isScalar() || !value.isScalar())
    {
      THROW_EXCEPTION("`$define` entries must be scalar `NAME: VALUE` pairs.");
    }
    const std::string varName = key.as<std::string>();

    // Already set by an outer scope: that definition wins (see OUTER-WINS
    // note above), so this file's own entry for the same name is a no-op.
    if (opts.variables.count(varName) != 0)
    {
      continue;
    }

    scopedOpts.variables[varName] = trimWSNL(mola::parse_yaml(value.as<std::string>(), valueOpts));
  }

  m.erase(itDefine);

  return scopedOpts;
}

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

    // Load + fully pre-process the referenced file, then REPLACE this node with
    // the loaded content (whole-node substitution).
    const std::string expr = text.substr(exprStart, exprEnd - exprStart);
    n                      = loadExternalYaml(expr, opts);
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
    // `$define` directive: a sibling map of `NAME: VALUE` pairs that become
    // `${NAME}` variables for this map's whole subtree, including the files
    // pulled in by a sibling `$import` / `$include{}`. This lets a launcher
    // select a variant of an imported pipeline through the `${VAR|default}`
    // hooks that file already exposes, instead of duplicating a whole block
    // just to change one nested value.
    // The resolution order in parseVars() is unchanged, so the effective
    // priority is: real environment > `$define` > inline `|default`. A variable
    // exported on the command line therefore still overrides the YAML file.
    // Across NESTED `$define` scopes for the SAME name (e.g. an imported file
    // `$define`s one of its own hooks), the OUTER one wins: consumeDefineBlock
    // skips re-defining a name already present in the incoming `opts`, so the
    // scope closest to the document root - not the file it imports - has
    // final say (see its doc comment).
    const auto  defineOpts = consumeDefineBlock(n.asMap(), opts);
    const auto& scopedOpts = defineOpts.has_value() ? *defineOpts : opts;

    processMapDirectives(n, scopedOpts);

    if (defineOpts.has_value())
    {
      // The variable pass runs later over the whole document with the OUTER
      // options, so it would not see these definitions: expand them now, over
      // this subtree only. Includes/commands were already resolved above.
      auto varOpts       = scopedOpts;
      varOpts.doIncludes = false;
      varOpts.doCmdRuns  = false;
      n = yaml::FromText(mola::parse_yaml(mola::yaml_to_string(yaml(n)), varOpts)).node();
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