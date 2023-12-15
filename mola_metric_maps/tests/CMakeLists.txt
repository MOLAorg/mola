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
  TARGET  test-mola_metric_maps_serialization
  SOURCES test-serialization.cpp
  LINK_LIBRARIES
  mola_metric_maps
)
