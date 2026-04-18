# Unit tests for mola_kernel

mola_add_test(
  TARGET  test-diagnostics-provider
  SOURCES test_diagnostics_provider.cpp
  LINK_LIBRARIES
    mola::mola_kernel
)
