#Unit tests:
mola_add_executable(
  DONT_INSTALL
  TARGET  test-mola-yaml-parser
  SOURCES test-yaml-parser.cpp
  LINK_LIBRARIES
    mola-yaml
)
add_test(test-mola-yaml-parser ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test-mola-yaml-parser)
