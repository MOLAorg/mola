# Unit tests:
mola_add_test(
  TARGET  test-imu-rotation-integrator
  SOURCES test-imu-rotation-integrator.cpp
  LINK_LIBRARIES
    mola::mola_imu_preintegration
)
