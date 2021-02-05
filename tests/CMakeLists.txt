#Unit tests:
mola_add_test(
  TARGET  test-mola-yaml-parser
  SOURCES test-yaml-parser.cpp
  LINK_LIBRARIES
    mola-yaml
)
