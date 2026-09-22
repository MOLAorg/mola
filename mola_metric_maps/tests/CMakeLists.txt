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
  TARGET  test-mola_metric_maps_index3d_hash
  SOURCES test-mola_metric_maps_index3d_hash.cpp
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

  # Per-point covariance cache admission gate: behavior checks, plus a
  # benchmark sweep that only runs when given a frame count in argv.
  # Eigen is linked explicitly: the library keeps it private, so a test that
  # reasons about the covariance matrices does not inherit its include path.
  mola_add_test(
    TARGET  test-mola_metric_maps_incrementalpointcloud_covcache
    SOURCES test-mola_metric_maps_incrementalpointcloud-covcache.cpp
    LINK_LIBRARIES
    mola_metric_maps
    Eigen3::Eigen
  )

  # Part of the test above runs on real LiDAR geometry, when available:
  if(mola_test_datasets_FOUND)
    target_compile_definitions(test-mola_metric_maps_incrementalpointcloud_covcache
      PRIVATE TEST_DATASETS_ROOT=\"${mola_test_datasets_DIR}/../datasets/\")
  endif()
endif()

mola_add_test(
  TARGET  test-mola_metric_maps_covariance_shape
  SOURCES test-covariance-shape.cpp
  LINK_LIBRARIES
  mola_metric_maps
  Eigen3::Eigen
)
