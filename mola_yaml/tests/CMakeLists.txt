#Unit tests:
mola_add_test(
  TARGET  test-mola_yaml-parser
  SOURCES test-yaml-parser.cpp
  LINK_LIBRARIES
    mola_yaml
)
