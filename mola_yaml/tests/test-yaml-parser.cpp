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
 * @file   test-yaml-parser.cpp
 * @brief  Unit tests for mola-yaml functionality
 *
 * @author Jose Luis Blanco Claraco
 * @date   Jan 14, 2021
 */

#include <mola_yaml/yaml_helpers.h>

#include <cstdlib>  // setenv / unsetenv (POSIX), _putenv_s (Windows)
#include <filesystem>
#include <fstream>

// Platform-portable helpers for temporarily setting environment variables
// in tests.  On POSIX, setenv()/unsetenv() are used.  On Windows the CRT
// provides _putenv_s(); clearing is done by setting the value to "".
#ifdef _WIN32
#define MOLA_TEST_SETENV(name, val) ::_putenv_s(name, val)
#define MOLA_TEST_UNSETENV(name) ::_putenv_s(name, "")
#else
#define MOLA_TEST_SETENV(name, val) ::setenv(name, val, /*overwrite=*/1)
#define MOLA_TEST_UNSETENV(name) ::unsetenv(name)
#endif

#include <iostream>

using namespace std::string_literals;

namespace
{
void test_yaml2string()
{
  {
    const mrpt::containers::yaml data = mrpt::containers::yaml::Map({{"A", 1.0}, {"B", 3}});

    const auto str = mola::yaml_to_string(data);
    ASSERT_EQUAL_(str, "A: 1.0\nB: 3\n");
  }
  {
    using mrpt::containers::vkcp;
    mrpt::containers::yaml data;
    data << vkcp("w", 1.5, "Width") << vkcp("h", 2.5, "Height");
    const auto str = mola::yaml_to_string(data);
    ASSERT_EQUAL_(str, "# Width\nw: 1.5\n# Height\nh: 2.5\n");
  }
}

const std::string txt1 = R"###(# sample yaml
a: 1.0
b: "foo"
c:
  - a
  - b
  - c
d:
  va: 'z'
e: '${foo|default1}'
)###";

const std::string txt2 = R"###(# sample yaml
a: 3
f: '${zoo}'
)###";

const std::string txt3 = R"###(# sample yaml
a: 3
b: 3  #f: '${zoo}'
)###";

void test_parseSimple()
{
  {
    const auto y = mrpt::containers::yaml::FromText(txt1);
    ASSERT_(y.isMap());
    ASSERT_EQUAL_(y["a"].as<int>(), 1);
    ASSERT_EQUAL_(y["b"].as<std::string>(), "foo");
    ASSERT_EQUAL_(y["c"](2).as<std::string>(), "c");
    ASSERT_EQUAL_(y["d"]["va"].as<std::string>(), "z");
  }
}

void test_parseCustomVars()
{
  {
    const auto y = mola::parse_yaml(mrpt::containers::yaml::FromText(txt1));
    ASSERT_(y.isMap());
    ASSERT_EQUAL_(y["e"].as<std::string>(), "default1");
  }
  {
    mola::YAMLParseOptions opts;
    opts.doEnvVars = false;

    const auto y = mola::parse_yaml(mrpt::containers::yaml::FromText(txt1), opts);
    ASSERT_(y.isMap());
    ASSERT_EQUAL_(y["e"].as<std::string>(), "${foo|default1}");
  }
  {
    mola::YAMLParseOptions opts;
    const std::string      se = "Something Else";
    opts.variables["foo"]     = se;

    const auto y = mola::parse_yaml(mrpt::containers::yaml::FromText(txt1), opts);
    ASSERT_(y.isMap());
    ASSERT_EQUAL_(y["e"].as<std::string>(), se);
  }

  // catch undefined variables:
  {
    bool did_throw = false;
    try
    {
      const auto y = mola::parse_yaml(mrpt::containers::yaml::FromText(txt2));
    }
    catch (const std::exception&)
    {
      did_throw = true;
    }
    ASSERT_(did_throw);
  }

  // do not throw if they are commented out:
  {
    bool did_throw = false;
    try
    {
      const auto y = mola::parse_yaml(mrpt::containers::yaml::FromText(txt3));
    }
    catch (const std::exception&)
    {
      did_throw = true;
    }
    ASSERT_(!did_throw);
  }
}

void test_parseIncludes()
{
  {
    const auto file = MOLA_MODULE_SOURCE_DIR + "/test_include1.yaml"s;
    const auto y    = mola::load_yaml_file(file);
    ASSERT_(y.isMap());
    ASSERT_(y.has("params"));
    ASSERT_EQUAL_(y["params"]["a"].as<int>(), 10);
    ASSERT_EQUAL_(y["params"]["b"].as<std::string>(), "20");
  }
  {
    const auto file = MOLA_MODULE_SOURCE_DIR + "/test_include2.yaml"s;
    const auto y    = mola::load_yaml_file(file);
    ASSERT_(y.isMap());
    ASSERT_(y.has("map1"));
    ASSERT_(y.has("map2"));
    ASSERT_EQUAL_(y["map1"]["a"].as<int>(), 10);
    ASSERT_EQUAL_(y["map2"]["c"].as<int>(), 30);
  }
  {
    const auto file = MOLA_MODULE_SOURCE_DIR + "/test_include3.yaml"s;
    const auto y    = mola::load_yaml_file(file);
    ASSERT_(y.has("map1"));
    ASSERT_(y.has("map2"));
    ASSERT_(y.has("map3"));
    ASSERT_EQUAL_(y["map1"]["p3"].as<bool>(), true);
    ASSERT_EQUAL_(y["map2"]["a"].as<int>(), 10);
    ASSERT_EQUAL_(y["map3"]["foo"].as<std::string>(), "bar");
    ASSERT_EQUAL_(y["map3"]["config4"]["p4"].as<bool>(), true);
    ASSERT_EQUAL_(y["map3"]["config2"]["c"].as<int>(), 30);
  }
  {
    const auto file = MOLA_MODULE_SOURCE_DIR + "/d1/test3b.yaml"s;
    const auto y    = mola::load_yaml_file(file);
    ASSERT_EQUAL_(y["foo"].as<std::string>(), "bar");
    ASSERT_EQUAL_(y["config2"]["c"].as<int>(), 30);
    ASSERT_EQUAL_(y["config4"]["p4"].as<bool>(), true);
  }
}

// ---------------------------------------------------------------------------
// 1. parse_yaml STRING overload
//
// Verify that the string overload performs the same substitutions.
// ---------------------------------------------------------------------------
void test_parseStringOverload()
{
  // Variable substitution via the string overload
  {
    mola::YAMLParseOptions opts;
    opts.variables["greeting"] = "hello";
    const std::string input    = "msg: '${greeting}'";
    const std::string result   = mola::parse_yaml(input, opts);

    const auto y = mrpt::containers::yaml::FromText(result);
    ASSERT_EQUAL_(y["msg"].as<std::string>(), "hello");
  }

  // With all substitutions disabled the text must come back verbatim
  {
    mola::YAMLParseOptions opts;
    opts.doIncludes = false;
    opts.doCmdRuns  = false;
    opts.doEnvVars  = false;

    const std::string input = "x: '${UNDEFINED}'\ny: '$(false)'";
    // None of the tokens are processed, so no exception should be raised.
    bool did_throw = false;
    try
    {
      const std::string result = mola::parse_yaml(input, opts);
      ASSERT_EQUAL_(result, input);
    }
    catch (const std::exception&)
    {
      did_throw = true;
    }
    ASSERT_(!did_throw);
  }
}

// ---------------------------------------------------------------------------
// 2. Real environment variables
//
// Verifies the first step of the resolution order (env > CURRENT_YAML_FILE_PATH
// > opts.variables > |default).
// ---------------------------------------------------------------------------
void test_parseEnvVar()
{
  // --- basic resolution ---
  {
    MOLA_TEST_SETENV("MOLA_TEST_VAR_BASIC", "from_env");

    mola::YAMLParseOptions opts;
    const auto             y =
        mola::parse_yaml(mrpt::containers::yaml::FromText("v: '${MOLA_TEST_VAR_BASIC}'"), opts);
    ASSERT_EQUAL_(y["v"].as<std::string>(), "from_env");

    MOLA_TEST_UNSETENV("MOLA_TEST_VAR_BASIC");
  }

  // --- env var takes priority over opts.variables ---
  // Resolution order: env first, user variables second.  If both define the
  // same name the environment value must win.
  {
    MOLA_TEST_SETENV("MOLA_TEST_VAR_PRIO", "env_value");

    mola::YAMLParseOptions opts;
    opts.variables["MOLA_TEST_VAR_PRIO"] = "map_value";  // should be shadowed

    const auto y =
        mola::parse_yaml(mrpt::containers::yaml::FromText("v: '${MOLA_TEST_VAR_PRIO}'"), opts);
    ASSERT_EQUAL_(y["v"].as<std::string>(), "env_value");

    MOLA_TEST_UNSETENV("MOLA_TEST_VAR_PRIO");
  }

  // --- env var takes priority even when a |default is present ---
  {
    MOLA_TEST_SETENV("MOLA_TEST_VAR_DEF", "env_wins");

    mola::YAMLParseOptions opts;
    const auto             y = mola::parse_yaml(
                    mrpt::containers::yaml::FromText("v: '${MOLA_TEST_VAR_DEF|ignored_default}'"), opts);
    ASSERT_EQUAL_(y["v"].as<std::string>(), "env_wins");

    MOLA_TEST_UNSETENV("MOLA_TEST_VAR_DEF");
  }
}

// ---------------------------------------------------------------------------
// 3. CURRENT_YAML_FILE_PATH built-in token
//
// This built-in resolves to YAMLParseOptions::includesBasePath.  It is
// normally set automatically by load_yaml_file(), but can be injected
// directly for unit-testing without requiring a file on disk.
// ---------------------------------------------------------------------------
void test_parseCurrentYamlFilePath()
{
  mola::YAMLParseOptions opts;
  opts.includesBasePath = "/some/project/config";

  const auto y =
      mola::parse_yaml(mrpt::containers::yaml::FromText("path: '${CURRENT_YAML_FILE_PATH}'"), opts);
  ASSERT_EQUAL_(y["path"].as<std::string>(), "/some/project/config");
}

// ---------------------------------------------------------------------------
// 4. Multiple ${} tokens in a single scalar
//
// Tests that the iterative parser handles more than one substitution
// token per value string.
// ---------------------------------------------------------------------------
void test_parseMultipleVarsInScalar()
{
  mola::YAMLParseOptions opts;
  opts.variables["FIRST"]  = "hello";
  opts.variables["SECOND"] = "world";

  // Two tokens separated by a literal space
  const auto y =
      mola::parse_yaml(mrpt::containers::yaml::FromText("msg: '${FIRST} ${SECOND}'"), opts);
  ASSERT_EQUAL_(y["msg"].as<std::string>(), "hello world");

  // Three tokens; the middle one uses a |default fallback
  const auto y2 = mola::parse_yaml(
      mrpt::containers::yaml::FromText("msg: '${FIRST}-${MISSING|mid}-${SECOND}'"), opts);
  ASSERT_EQUAL_(y2["msg"].as<std::string>(), "hello-mid-world");

  // Explicit empty default `${VAR|}` must resolve to "" rather than throwing.
  // (Distinguishes "no default provided" from "default is the empty string".)
  const auto y3 =
      mola::parse_yaml(mrpt::containers::yaml::FromText("msg: '${TOTALLY_ABSENT_VAR_XYZ|}'"), opts);
  ASSERT_EQUAL_(y3["msg"].as<std::string>(), "");
}

// ---------------------------------------------------------------------------
// 5. $(command) substitution
//
// The command-run feature has zero test coverage in the existing suite.
// ---------------------------------------------------------------------------
void test_parseCmdRuns()
{
#ifdef _WIN32
  // POSIX commands (`echo` producing exact output, `false` with exit-code 1)
  // are not portably available on Windows.  Skip this test on that platform.
  return;
#endif
  // --- basic substitution ---
  // `echo` is POSIX-mandated; its output is trimmed of trailing newline.
  {
    const auto y = mola::parse_yaml(mrpt::containers::yaml::FromText("v: '$(echo hello_mola)'"));
    ASSERT_EQUAL_(y["v"].as<std::string>(), "hello_mola");
  }

  // --- multi-word output (whitespace preserved, trailing newlines stripped) ---
  {
    const auto y = mola::parse_yaml(mrpt::containers::yaml::FromText("v: '$(echo foo bar)'"));
    ASSERT_EQUAL_(y["v"].as<std::string>(), "foo bar");
  }

  // --- doCmdRuns=false: token kept verbatim, no execution ---
  {
    mola::YAMLParseOptions opts;
    opts.doCmdRuns = false;

    const auto y = mola::parse_yaml(mrpt::containers::yaml::FromText("v: '$(echo hello)'"), opts);
    // The token is not expanded, so the literal string is preserved.
    ASSERT_EQUAL_(y["v"].as<std::string>(), "$(echo hello)");
  }

  // --- failing command throws ---
  {
    bool did_throw = false;
    try
    {
      // `false` is a POSIX command that always exits with status 1.
      const auto res = mola::parse_yaml(mrpt::containers::yaml::FromText("v: '$(false)'"));
      (void)res;
    }
    catch (const std::exception&)
    {
      did_throw = true;
    }
    ASSERT_(did_throw);
  }
}

// ---------------------------------------------------------------------------
// 6. doIncludes=false: $include{} tokens are left intact
//
// The include step is the first pass; disabling it must prevent any file I/O
// and leave the token as a plain string.
// ---------------------------------------------------------------------------
void test_parseIncludesDisabled()
{
  mola::YAMLParseOptions opts;
  opts.doIncludes = false;

  // Use a path that does not exist - if includes were processed this would
  // throw; with doIncludes=false it must silently pass through.
  const std::string input     = "cfg:\n  $include{/does/not/exist.yaml}";
  bool              did_throw = false;
  try
  {
    const std::string result = mola::parse_yaml(input, opts);
    // The include token must survive verbatim in the serialized output.
    ASSERT_(result.find("$include{") != std::string::npos);
  }
  catch (const std::exception&)
  {
    did_throw = true;
  }
  ASSERT_(!did_throw);
}

// ---------------------------------------------------------------------------
// 7. Malformed syntax: unclosed delimiters must throw
//
// Covers the error-reporting paths of findClosing() for each token type.
// ---------------------------------------------------------------------------
void test_parseMalformedInput()
{
  // Unclosed ${
  {
    bool did_throw = false;
    try
    {
      const auto ret = mola::parse_yaml("v: '${unclosed'");
      (void)ret;
    }
    catch (const std::exception&)
    {
      did_throw = true;
    }
    ASSERT_(did_throw);
  }

  // Unclosed $(
  {
    bool did_throw = false;
    try
    {
      const auto ret = mola::parse_yaml("v: '$(unclosed'");
      (void)ret;
    }
    catch (const std::exception&)
    {
      did_throw = true;
    }
    ASSERT_(did_throw);
  }
}

// ---------------------------------------------------------------------------
// 8. Missing $include file must throw
// ---------------------------------------------------------------------------
void test_parseMissingIncludeFile()
{
  // Build a minimal YAML file path that is guaranteed not to exist.
  const std::string yaml_with_bad_include =
      "cfg:\n  $include{/this/path/absolutely/does/not/exist_xyz.yaml}";

  bool did_throw = false;
  try
  {
    const auto ret = mola::parse_yaml(yaml_with_bad_include);
    (void)ret;
  }
  catch (const std::exception&)
  {
    did_throw = true;
  }
  ASSERT_(did_throw);
}

// ---------------------------------------------------------------------------
// 9. opts.variables propagated through load_yaml_file
//
// Write real temporary YAML files containing ${} tokens, then call
// load_yaml_file() with a custom opts.variables map to verify the variables
// are forwarded correctly through the full file-load pipeline.
// ---------------------------------------------------------------------------
void test_parseVarsInLoadedFile()
{
  namespace fs = std::filesystem;

  const fs::path    tmpDir = fs::temp_directory_path();
  const std::string file1  = (tmpDir / "mola_test_vars1.yaml").string();
  const std::string file2  = (tmpDir / "mola_test_vars2.yaml").string();

  // Write temp files - cleaned up at the end of this test.
  {
    std::ofstream f(file1);
    f << "v: '${MOLA_TEST_CUSTOM}'\n";
  }
  {
    std::ofstream f(file2);
    // |wrong_default must be overridden by opts.variables.
    f << "v: '${MOLA_TEST_CUSTOM|wrong_default}'\n";
  }

  mola::YAMLParseOptions opts;
  opts.variables["MOLA_TEST_CUSTOM"] = "custom_value";

  // Plain variable substitution via load_yaml_file
  {
    const auto y = mola::load_yaml_file(file1, opts);
    ASSERT_EQUAL_(y["v"].as<std::string>(), "custom_value");
  }

  // Custom variable overrides the |default fallback inside load_yaml_file
  {
    const auto y2 = mola::load_yaml_file(file2, opts);
    ASSERT_EQUAL_(y2["v"].as<std::string>(), "custom_value");
  }

  // Clean up temp files.
  fs::remove(file1);
  fs::remove(file2);
}

// ---------------------------------------------------------------------------
// 10. yaml_to_string - additional node shapes
//
// The existing test covers Map.  Check Sequence and the empty-map case so
// that round-trips through yaml_to_string are verified for more node types.
// ---------------------------------------------------------------------------
void test_yaml2stringAdditional()
{
  // Empty map round-trips without error
  {
    const mrpt::containers::yaml empty = mrpt::containers::yaml::Map({});

    const std::string s = mola::yaml_to_string(empty);
    // Re-parse: must yield an empty / null-ish node (no throw).
    [[maybe_unused]] const auto reparsed = mrpt::containers::yaml::FromText(s);
  }

  // Sequence round-trip
  {
    mrpt::containers::yaml seq = mrpt::containers::yaml::Sequence();
    seq.push_back(1.0);
    seq.push_back(2.0);
    seq.push_back(3.0);
    const std::string s        = mola::yaml_to_string(seq);
    const auto        reparsed = mrpt::containers::yaml::FromText(s);
    ASSERT_(reparsed.isSequence());
    ASSERT_EQUAL_(reparsed(0).as<int>(), 1);
    ASSERT_EQUAL_(reparsed(2).as<int>(), 3);
  }

  // Nested map round-trip preserves structure
  {
    const mrpt::containers::yaml inner = mrpt::containers::yaml::Map({{"x", 10}, {"y", 20}});
    mrpt::containers::yaml       outer =
        mrpt::containers::yaml::Map({{"inner", std::make_shared<mrpt::containers::yaml>(inner)}});

    const auto s = mola::yaml_to_string(outer);
    const auto y = mrpt::containers::yaml::FromText(s);
    ASSERT_EQUAL_(y["inner"]["x"].as<int>(), 10);
    ASSERT_EQUAL_(y["inner"]["y"].as<int>(), 20);
  }
}

// ---------------------------------------------------------------------------
// 11. `$import` directive: import a base file and OVERRIDE particular entries
//
// A map with a `$import` key (scalar path or sequence of paths) is replaced by
// the deep-merge of the imported file(s) with the map's remaining keys overlaid
// on top (siblings override the base; nested maps merge deeply).
// ---------------------------------------------------------------------------
void test_parseImportOverride()
{
  using mrpt::containers::yaml;

  mola::YAMLParseOptions opts;
  opts.includesBasePath = MOLA_MODULE_SOURCE_DIR;

  // --- single import + scalar override + new key + deep-nested override ---
  {
    const std::string input =
        "params:\n"
        "  $import: test_import_base.yaml\n"
        "  b: 99\n"  // override an existing scalar
        "  c: 3\n"  // add a brand-new key
        "  nested:\n"
        "    y: 200\n";  // deep-override one nested key (x must survive)
    const auto y = mola::parse_yaml(yaml::FromText(input), opts);
    ASSERT_(y.isMap());
    ASSERT_EQUAL_(y["params"]["a"].as<int>(), 1);  // from base
    ASSERT_EQUAL_(y["params"]["b"].as<int>(), 99);  // overridden
    ASSERT_EQUAL_(y["params"]["c"].as<int>(), 3);  // added
    ASSERT_EQUAL_(y["params"]["nested"]["x"].as<int>(), 10);  // kept (deep merge)
    ASSERT_EQUAL_(y["params"]["nested"]["y"].as<int>(), 200);  // overridden
    ASSERT_EQUAL_(y["params"]["list"](0).as<std::string>(), "p");  // from base
  }

  // --- sequence import: later file overrides earlier; sibling overrides both ---
  {
    const std::string input =
        "config:\n"
        "  $import:\n"
        "    - test1.yaml\n"  // a:10 b:20
        "    - test_import_o2.yaml\n"  // b:777 e:5
        "  a: 1000\n";  // sibling override
    const auto y = mola::parse_yaml(yaml::FromText(input), opts);
    ASSERT_EQUAL_(y["config"]["a"].as<int>(), 1000);  // sibling wins over imports
    ASSERT_EQUAL_(y["config"]["b"].as<int>(), 777);  // 2nd import wins over 1st
    ASSERT_EQUAL_(y["config"]["e"].as<int>(), 5);
  }

  // --- via load_yaml_file(): relative import resolved against the file's dir ---
  {
    const auto file = MOLA_MODULE_SOURCE_DIR + "/test_import_top.yaml"s;
    const auto y    = mola::load_yaml_file(file);
    ASSERT_EQUAL_(y["params"]["a"].as<int>(), 1);  // from base
    ASSERT_EQUAL_(y["params"]["b"].as<int>(), 42);  // overridden
    ASSERT_EQUAL_(y["params"]["nested"]["x"].as<int>(), 10);  // kept
    ASSERT_EQUAL_(y["params"]["nested"]["y"].as<int>(), 222);  // overridden
  }
}

// ---------------------------------------------------------------------------
// 12. `$import` of a missing file must throw
// ---------------------------------------------------------------------------
void test_parseImportMissingFile()
{
  mola::YAMLParseOptions opts;
  opts.includesBasePath = MOLA_MODULE_SOURCE_DIR;

  bool did_throw = false;
  try
  {
    const std::string input = "p:\n  $import: this_file_does_not_exist_xyz.yaml\n";
    const auto        y     = mola::parse_yaml(mrpt::containers::yaml::FromText(input), opts);
    (void)y;
  }
  catch (const std::exception&)
  {
    did_throw = true;
  }
  ASSERT_(did_throw);
}

// ---------------------------------------------------------------------------
// 13. `$define` directive: bind ${VAR} variables for a subtree
//
// A `$define` map key binds `${NAME}` variables for the whole subtree of the
// map it appears in, INCLUDING the files pulled in by a sibling `$import` /
// `$include{}`. Priority: real environment > $define > inline `|default`.
// ---------------------------------------------------------------------------
void test_parseDefine()
{
  using mrpt::containers::yaml;

  mola::YAMLParseOptions opts;
  opts.includesBasePath = MOLA_MODULE_SOURCE_DIR;

  // --- the imported file's ${VAR|default} hooks are driven by $define ---
  {
    const std::string input =
        "params:\n"
        "  $define:\n"
        "    MOLA_TEST_DEFINE_METHOD: from_define\n"
        "    MOLA_TEST_DEFINE_DEEP: deep_from_define\n"
        "  $import: test_define_base.yaml\n";
    const auto y = mola::parse_yaml(yaml::FromText(input), opts);

    ASSERT_EQUAL_(y["params"]["method"].as<std::string>(), "from_define");
    ASSERT_EQUAL_(y["params"]["nested"]["deep"].as<std::string>(), "deep_from_define");
    // Untouched hooks keep their inline default:
    ASSERT_EQUAL_(y["params"]["gain"].as<std::string>(), "1");
    // Reaches inside sequences too - which plain $import overrides cannot patch:
    ASSERT_EQUAL_(y["params"]["steps"](0)["mode"].as<std::string>(), "from_define");
    ASSERT_EQUAL_(y["params"]["steps"](1)["mode"].as<std::string>(), "fixed");
    // The directive itself must not survive into the output:
    ASSERT_(!y["params"].has("$define"));
  }

  // --- without $define, the imported file falls back to its inline defaults ---
  {
    const std::string input = "params:\n  $import: test_define_base.yaml\n";
    const auto        y     = mola::parse_yaml(yaml::FromText(input), opts);
    ASSERT_EQUAL_(y["params"]["method"].as<std::string>(), "default_method");
  }

  // --- a real environment variable still wins over $define ---
  {
    MOLA_TEST_SETENV("MOLA_TEST_DEFINE_METHOD", "from_env");

    const std::string input =
        "params:\n"
        "  $define:\n"
        "    MOLA_TEST_DEFINE_METHOD: from_define\n"
        "  $import: test_define_base.yaml\n";
    const auto y = mola::parse_yaml(yaml::FromText(input), opts);
    ASSERT_EQUAL_(y["params"]["method"].as<std::string>(), "from_env");

    MOLA_TEST_UNSETENV("MOLA_TEST_DEFINE_METHOD");
  }

  // --- $define also reaches SIBLING keys of the same map, not just imports ---
  {
    const std::string input =
        "params:\n"
        "  $define:\n"
        "    MY_VAR: sibling_value\n"
        "  local: '${MY_VAR}'\n"
        "  deeper:\n"
        "    also: '${MY_VAR}'\n";
    const auto y = mola::parse_yaml(yaml::FromText(input), opts);
    ASSERT_EQUAL_(y["params"]["local"].as<std::string>(), "sibling_value");
    ASSERT_EQUAL_(y["params"]["deeper"]["also"].as<std::string>(), "sibling_value");
  }

  // --- scope is limited to the subtree: a sibling map does NOT see it ---
  {
    const std::string input =
        "a:\n"
        "  $define:\n"
        "    SCOPED_VAR: inside\n"
        "  v: '${SCOPED_VAR}'\n"
        "b:\n"
        "  v: '${SCOPED_VAR|outside}'\n";
    const auto y = mola::parse_yaml(yaml::FromText(input), opts);
    ASSERT_EQUAL_(y["a"]["v"].as<std::string>(), "inside");
    ASSERT_EQUAL_(y["b"]["v"].as<std::string>(), "outside");
  }

  // --- $define values may themselves use ${} / $() expressions ---
  {
    mola::YAMLParseOptions o2 = opts;
    o2.variables["OUTER"]     = "composed";

    const std::string input =
        "params:\n"
        "  $define:\n"
        "    INNER: '${OUTER}-suffix'\n"
        "  v: '${INNER}'\n";
    const auto y = mola::parse_yaml(yaml::FromText(input), o2);
    ASSERT_EQUAL_(y["params"]["v"].as<std::string>(), "composed-suffix");
  }

  // --- nested scopes: an OUTER $define wins over a more deeply imported
  //     file's own $define for the SAME name; a name the outer scope never
  //     touches still gets set by the inner file's own $define ---
  {
    const std::string input =
        "$define:\n"
        "  MOLA_TEST_DEFINE_METHOD: from_outer\n"
        "top:\n"
        "  $import: test_define_nested.yaml\n";
    const auto y = mola::parse_yaml(yaml::FromText(input), opts);
    // test_define_nested.yaml's `inner` subtree tries to re-define the SAME
    // variable; the outer (root) definition wins for both `inner` and `outer`:
    ASSERT_EQUAL_(y["top"]["inner"]["method"].as<std::string>(), "from_outer");
    ASSERT_EQUAL_(y["top"]["outer"]["method"].as<std::string>(), "from_outer");
    // A name the outer scope never touches still gets set by the more deeply
    // imported file's own $define:
    ASSERT_EQUAL_(y["top"]["inner"]["gain"].as<std::string>(), "from_inner_file_gain");
    // ... and, absent any $define for it anywhere, falls back to the inline
    // default, same as always:
    ASSERT_EQUAL_(y["top"]["outer"]["gain"].as<std::string>(), "1");
  }

  // --- outer-wins holds across a genuine multi-level $import CHAIN too, not
  //     just nested same-document scopes: a reusable fragment that $define's
  //     one of its own hooks (test_define_chain_middle.yaml, itself imported
  //     via a plain, un-$define'd $import here) must not permanently shadow
  //     that hook from every file that imports it ---
  {
    const std::string input =
        "$define:\n"
        "  MOLA_TEST_DEFINE_METHOD: from_outer_chain\n"
        "$import: test_define_chain_middle.yaml\n";
    const auto y = mola::parse_yaml(yaml::FromText(input), opts);
    ASSERT_EQUAL_(y["method"].as<std::string>(), "from_outer_chain");
    // Untouched by either layer's $define: still the inline default.
    ASSERT_EQUAL_(y["gain"].as<std::string>(), "1");
  }

  // --- without an outer override, the middle file's own $define still
  //     applies (it is not merely inert) ---
  {
    const std::string input = "$import: test_define_chain_middle.yaml\n";
    const auto        y     = mola::parse_yaml(yaml::FromText(input), opts);
    ASSERT_EQUAL_(y["method"].as<std::string>(), "from_middle_file");
  }

  // --- a non-map $define value is an error ---
  {
    bool did_throw = false;
    try
    {
      const auto y = mola::parse_yaml(yaml::FromText("p:\n  $define: not_a_map\n"), opts);
      (void)y;
    }
    catch (const std::exception&)
    {
      did_throw = true;
    }
    ASSERT_(did_throw);
  }
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test_yaml2string();
    test_parseSimple();
    test_parseIncludes();
    test_parseCustomVars();
    test_parseStringOverload();
    test_parseEnvVar();
    test_parseCurrentYamlFilePath();
    test_parseMultipleVarsInScalar();
    test_parseCmdRuns();
    test_parseIncludesDisabled();
    test_parseMalformedInput();
    test_parseMissingIncludeFile();
    test_parseVarsInLoadedFile();
    test_yaml2stringAdditional();
    test_parseImportOverride();
    test_parseImportMissingFile();
    test_parseDefine();

    std::cout << "Test successful.\n";
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
}