"""No-op stand-in for the doxyrest Sphinx extension.

The published docs are built with doxygen + doxyrest, which turn the C++
headers into thousands of generated .rst files. That toolchain is heavy and
is not what pull requests need to be checked against: CI only validates the
hand-written pages. Registering this shim keeps conf.py unmodified so CI
builds the same configuration as production, minus the generated half.
"""


def setup(app):
    return {
        "version": "0.0.0-ci-shim",
        "parallel_read_safe": True,
        "parallel_write_safe": True,
    }
