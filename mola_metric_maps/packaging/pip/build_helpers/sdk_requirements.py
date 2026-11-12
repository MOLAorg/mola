"""scikit-build-core metadata provider of the wheel "dependencies".

The C++ code in the wheel links to the libraries of the mp2p-icp and mrpt3
wheels it was built against (their C++ SDKs), so the requirements follow
their versions in the build environment: the same minor series, and at least
that patch version (e.g. "mp2p-icp>=3.1.0,<3.2", "mrpt3>=3.6.2,<3.7"). Other
requirements come from the "requires" setting (see pyproject.toml).
"""

from __future__ import annotations

import re
from importlib.metadata import version

# Distributions whose C++ SDK the wheel is built against:
SDK_DISTRIBUTIONS = ("mp2p-icp", "mrpt3")


def minor_series_requirement(dist: str) -> str:
    v = version(dist)
    m = re.match(r"(\d+)\.(\d+)\.(\d+)", v)
    if not m:
        raise ValueError(f"Unexpected {dist} version: '{v}'")
    major, minor, patch = (int(x) for x in m.groups())
    return f"{dist}>={major}.{minor}.{patch},<{major}.{minor + 1}"


def dynamic_metadata(field, settings):
    if field != "dependencies":
        raise ValueError(f"This provider only fills 'dependencies', not '{field}'")
    return [*settings.get("requires", []), *map(minor_series_requirement, SDK_DISTRIBUTIONS)]
