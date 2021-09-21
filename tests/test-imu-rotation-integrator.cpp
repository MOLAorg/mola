/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-imu-rotation-integrator.cpp
 * @brief  Unit tests for mola-imu-preintegration functionality
 * @author Jose Luis Blanco Claraco
 * @date   Sep 21, 2021
 */

#include <mola-imu-preintegration/RotationIntegrator.h>

#include <iostream>

using namespace std::string_literals;

const std::string yamlRotIntParams1 =
    R"###(# Config for gtsam::RotationIntegrationParams
gyroBias: [-1.0e-4, 2.0e-4, -3.0e-4]
sensorLocationInVehicle:
  quaternion: [0.0, 0.0, 0.0, 1.0]
  translation: [0.0, 0.0, 0.0]
)###";

static void test_rotation_integration()
{
    mola::RotationIntegrator ri;
    ri.initialize(mola::Yaml::FromText(yamlRotIntParams1));

    ASSERT_EQUAL_(ri.params_.gyroBias.x, -1.0e-4);
    ASSERT_EQUAL_(ri.params_.gyroBias.y, +2.0e-4);
    ASSERT_EQUAL_(ri.params_.gyroBias.z, -3.0e-4);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_rotation_integration();

        std::cout << "Test successful." << std::endl;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
