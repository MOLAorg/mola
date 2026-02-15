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

#include <iostream>

using namespace std::string_literals;

namespace
{
void test_yaml2string()
{
  {
    const auto data = mrpt::containers::yaml::Map({{"A", 1.0}, {"B", 3}});
    const auto str  = mola::yaml_to_string(data);
    ASSERT_EQUAL_(str, "A: 1\nB: 3\n");
  }
  {
    using mrpt::containers::vkcp;
    mrpt::containers::yaml data;
    data << vkcp("w", 1.5, "Width") << vkcp("h", 2.5, "Height");
    const auto str = mola::yaml_to_string(data);
    ASSERT_EQUAL_(str, "# Height\nh: 2.5\n# Width\nw: 1.5\n");
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

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  try
  {
    test_yaml2string();
    test_parseSimple();
    test_parseIncludes();
    test_parseCustomVars();

    std::cout << "Test successful."
              << "\n";
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
