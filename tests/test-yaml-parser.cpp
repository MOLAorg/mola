/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-yaml-parser.cpp
 * @brief  Unit tests for mola-yaml functionality
 *
 * @author Jose Luis Blanco Claraco
 * @date   Jan 14, 2021
 */

#include <mola-yaml/yaml_helpers.h>

#include <iostream>

static void test_yaml2string()
{
    {
        const auto data = mrpt::containers::yaml::Map({{"A", 1.0}, {"B", 3}});
        const auto str  = mola::yaml2string(data);
        ASSERT_EQUAL_(str, "A: 1\nB: 3\n");
    }
    {
        using mrpt::containers::vkcp;
        mrpt::containers::yaml data;
        data << vkcp("w", 1.5, "Width") << vkcp("h", 2.5, "Height");
        const auto str = mola::yaml2string(data);
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

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_yaml2string();
        test_parseSimple();
        // test_parseEnvSimple();

        std::cout << "Test successful." << std::endl;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
