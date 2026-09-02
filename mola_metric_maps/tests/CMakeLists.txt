# Unit tests:
mola_add_test(
  TARGET  test-mola_metric_maps_occgrid
  SOURCES test-occgrid.cpp
  LINK_LIBRARIES
  mola_metric_maps
)

mola_add_test(
  TARGET  test-mola_metric_maps_sparsevoxelpointcloud
  SOURCES test-sparsevoxelpointcloud.cpp
  LINK_LIBRARIES
  mola_metric_maps
)

mola_add_test(
  TARGET  test-mola_metric_maps_hashedvoxels
  SOURCES test-mola_metric_maps_hashedvoxels.cpp
  LINK_LIBRARIES
  mola_metric_maps
)

mola_add_test(
  TARGET  test-mola_metric_maps_ndt
  SOURCES test-mola_metric_maps_ndt.cpp
  LINK_LIBRARIES
  mola_metric_maps
)

mola_add_test(
  TARGET  test-mola_metric_maps_tsdf
  SOURCES test-mola_metric_maps_tsdf.cpp
  LINK_LIBRARIES
  mola_metric_maps
)

mola_add_test(
  TARGET  test-mola_metric_maps_serialization
  SOURCES test-serialization.cpp
  LINK_LIBRARIES
  mola_metric_maps
)

mola_add_test(
  TARGET  test-mola_metric_maps_keyframemap
  SOURCES test-mola_metric_maps_keyframemap.cpp
  LINK_LIBRARIES
  mola_metric_maps
)

mola_add_test(
  TARGET  test-mola_metric_maps_options_ini_io
  SOURCES test-options-ini-io.cpp
  LINK_LIBRARIES
  mola_metric_maps
)

if(MOLA_HAS_INCREMENTAL_POINT_CLOUD)
  mola_add_test(
    TARGET  test-mola_metric_maps_incrementalpointcloud
    SOURCES test-mola_metric_maps_incrementalpointcloud.cpp
    LINK_LIBRARIES
    mola_metric_maps
  )

  # LiDAR-odometry-like stress test. Meant to be run under a sanitizer or
  # valgrind too, see tests/README-sanitizers.md
  mola_add_test(
    TARGET  test-mola_metric_maps_incrementalpointcloud_stress
    SOURCES test-mola_metric_maps_incrementalpointcloud-stress.cpp
    LINK_LIBRARIES
    mola_metric_maps
  )
endif()

