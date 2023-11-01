# Unit tests:
mola_add_test(
  TARGET  test-mola_metric_maps_occgrid
  SOURCES test-occgrid.cpp
  LINK_LIBRARIES
  mola_metric_maps
)

# Unit tests:
mola_add_test(
  TARGET  test-mola_metric_maps_dualvoxelpointcloud
  SOURCES test-dualvoxelpointcloud.cpp
  LINK_LIBRARIES
  mola_metric_maps
)

mola_add_test(
  TARGET  test-mola_metric_maps_serialization
  SOURCES test-serialization.cpp
  LINK_LIBRARIES
  mola_metric_maps
)
