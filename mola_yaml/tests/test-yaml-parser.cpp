/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

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

static void test_yaml2string()
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
)###";

static void test_parseSimple()
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

static void test_parseIncludes()
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

MRPT_TODO("Possible bug: #$include{} shouldn't be parsed")
MRPT_TODO("bug: #${var} shouldn't be parsed")

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_yaml2string();
        test_parseSimple();
        test_parseIncludes();
        // test_parseEnvSimple();

        std::cout << "Test successful." << std::endl;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
