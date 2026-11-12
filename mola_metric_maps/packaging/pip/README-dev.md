# mola-metric-maps Python wheel: developer notes

PyPI name `mola-metric-maps`, import name `mola_metric_maps`. `pyproject.toml`
(scikit-build-core) runs the superbuild in `CMakeLists.txt`, which builds
`mola_metric_maps` (the C++ library and its Python bindings, `../../python`;
no apps nor tests) as an ExternalProject, against the C++ SDKs of two wheels,
both build requirements:

* [`mp2p-icp`](https://pypi.org/project/mp2p-icp/) (`site-packages/mp2p_icp/sdk`):
  `mp2p_icp_map` and the `mola_common` CMake functions (`mola_add_library()`...).
  They are not rebuilt here.
* [`mrpt3`](https://pypi.org/project/mrpt3/) (`site-packages/mrpt/sdk`): MRPT,
  Eigen, nanoflann and oneTBB.

The wheel holds only the `mola_metric_maps` Python package:

```
site-packages/
  mola_metric_maps/   __init__.py, .pyi stubs, _core.*.so, libmola_metric_maps.so
```

It ships no C++ SDK (headers, CMake files): add one, as mp2p-icp does, if
other wheels ever need to build C++ code against `mola_metric_maps`.

* **mp2p_icp, MRPT and oneTBB are never bundled.** MRPT keeps a single class
  registry per process (used, e.g., to deserialize `.mm` files), and Python
  types are shared with the `mrpt` and `mp2p_icp` Python modules, so the wheel
  must load the very same `libmp2p_icp*.so`, `libmrpt_*.so` and `libtbb.so.12`
  as the installed `mp2p-icp` and `mrpt3` wheels. The RUNPATH of both binaries
  is `$ORIGIN:$ORIGIN/../mp2p_icp/sdk/lib:$ORIGIN/../mrpt/sdk/lib`, and
  auditwheel is told to exclude those libraries and the OpenGL (libglvnd)
  ones (`repair-wheel-command` in `pyproject.toml`).
* pybind11 is pinned to the version of the mrpt3 and mp2p-icp wheels
  (`mrpt.sdk.PYBIND11_VERSION`, `mp2p_icp.sdk.PYBIND11_VERSION`), or Python
  types would not be shared; the superbuild checks it.
* The library is built without SOVERSION (`MOLA_NO_SOVERSION=ON`), since wheels
  cannot hold symlinks.
* nanoflann comes from the MRPT SDK (the superbuild requires >= 1.10, for the
  `IncrementalPointCloud` k-d tree index). The k-d tree baking also needs
  nanoflann >= 1.11: `mola_metric_maps.HAS_INCREMENTAL_KDTREE_BAKE` tells
  whether it is available (without it, baked k-d trees in `.mm` files are
  ignored on load, and rebuilt).
* The version comes from `mola_metric_maps/package.xml` (as
  `mola_metric_maps.__version__`).

## Requirements, ABI, and the release order

The wheel requires `mp2p-icp>=X.Y.Z,<X.(Y+1)` and `mrpt3>=X.Y.Z,<X.(Y+1)`, where
`X.Y.Z` are the versions it was built against (computed at build time by
`build_helpers/sdk_requirements.py`). The build uses the exact versions pinned
in `[build-system] requires`: update them (and the pybind11 pin, if it
changed) to build against newer ones.

This assumes that patch releases of mp2p-icp and mrpt3 keep the C++ ABI of
their libraries (no SOVERSION, see the mp2p-icp README-dev.md). A new minor
version of any of them always needs a new mola-metric-maps build.

**mp2p-icp must be on PyPI first**: the CI builds and tests against the
mp2p-icp wheels from PyPI, so publish the mp2p-icp version pinned in
`pyproject.toml` before building (or releasing) this wheel. Until then, the CI
jobs fail when installing the build requirements.

## Local build (manylinux image)

The reference build is the one in CI (next section). To reproduce it locally
(needs Docker), from the repository root, with
`git submodule update --init mola_metric_maps/3rdparty/robin-map` done:

```bash
python3 -m venv /tmp/cibw && /tmp/cibw/bin/pip install cibuildwheel
CIBW_BUILD=cp312-manylinux_x86_64 /tmp/cibw/bin/cibuildwheel --platform linux \
  --output-dir /tmp/wheelhouse mola_metric_maps/packaging/pip
```

If the pinned mp2p-icp is not on PyPI yet, point pip to a directory with its
wheels (the containers see the host file system under `/host`):

```bash
PIP_FIND_LINKS=/host/path/to/mp2p_icp/wheelhouse CIBW_ENVIRONMENT_PASS_LINUX=PIP_FIND_LINKS \
  CIBW_BUILD=cp312-manylinux_x86_64 /tmp/cibw/bin/cibuildwheel ...
```

That builds, repairs and tests the wheel. To test it again in a clean system,
also with the map of the `mola_test_datasets` repository:

```bash
docker run --rm -v $PWD:/src:ro -v /tmp/wheelhouse:/wh:ro \
  -v /path/to/mola_test_datasets/datasets:/datasets:ro -e MOLA_TEST_DATASETS_DIR=/datasets \
  python:3.12-slim bash -c "apt-get update -qq && apt-get install -y -qq binutils > /dev/null && \
   pip install pytest /wh/mola_metric_maps-*-cp312-*.whl && \
   python /src/mola_metric_maps/packaging/pip/tests/test_wheel.py"
```

Building outside the manylinux images (`python -m build --wheel
mola_metric_maps/packaging/pip`) also works, if the compiler is ABI compatible
with the one used for the mrpt3 and mp2p-icp wheels, but do not source a ROS
environment first: the superbuild ignores `$CMAKE_PREFIX_PATH`, yet other
variables may still leak in. The build dir
(`mola_metric_maps/packaging/pip/build/<wheel tag>`) is kept for incremental
rebuilds.

`tests/test_wheel.py` (run by cibuildwheel) checks that the libraries come from
the mp2p-icp and mrpt3 wheels and are not bundled, the RUNPATH and
dependencies of the binaries (if `readelf` and `ldd` are found), maps built
from NumPy points, queries, a `.mm` round trip, and that `mp2p_icp` imports
`mola_metric_maps` by itself to load a map with an `IncrementalPointCloud`
layer. With `MOLA_TEST_DATASETS_DIR` set, it also loads
`ual/husky_benches/husky_benches_map.mm`. Then it runs the pytest suite of
`mola_metric_maps/python/tests`, so it needs the source tree. Tests of that
suite that need fixes or features of MRPT Python bindings newer than the
installed mrpt3 wheel are deselected, after a run-time probe that prints a
`[KNOWN]` line (`known_upstream_issues()`): with mrpt3 3.6.2, `test_bases`
(`CMetricMap.GetRuntimeClass()` double free), `test_occgrid` and
`test_incremental_point_cloud_fields_and_transform` (newer bindings API).

To check the binaries of a wheel:

```bash
auditwheel show mola_metric_maps-*.whl   # libmp2p_icp*, libmrpt_*, libtbb: "not found" here is expected
unzip -o -d /tmp/w mola_metric_maps-*.whl && readelf -d /tmp/w/mola_metric_maps/*.so | grep -E "NEEDED|RUNPATH"
```

## Type stubs

The committed `mola_metric_maps/python/mola_metric_maps/_core.pyi` is generated
by `mola_metric_maps/python/scripts/generate_stubs.py` (pybind11-stubgen) from
a module built with the pybind11 version of the mrpt3 wheel: other versions
write different annotations. With a wheel built as above, from the repository
root:

```bash
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v $PWD:/src -v /tmp/wheelhouse:/wh:ro \
  python:3.12-slim bash -c "pip install --user /wh/mola_metric_maps-*-cp312-*.whl \
  pybind11-stubgen==3.0.0 && cd / && python /src/mola_metric_maps/python/scripts/generate_stubs.py"
```

## CI and publishing

`.github/workflows/python-wheels.yml` builds and tests the wheels with
cibuildwheel, in `manylinux_2_28` containers, on native x86_64 and aarch64
runners, with the map of `mola_test_datasets` (sparse checkout):

* Pull requests touching `mola_metric_maps` (the library, the bindings or this
  packaging) or the workflow: one Python version per platform.
* Weekly, on release tags and by hand: Python 3.10 to 3.14.
* Release tags of this repository (`X.Y.Z`, as the rest of the release
  process, all packages share the version) upload the wheels to PyPI. A manual
  run can also upload to TestPyPI or PyPI (`publish` input). The tag must match
  the version in `mola_metric_maps/package.xml`.

Uploads use PyPI Trusted Publishing (no tokens). **One-time setup**, by a
maintainer: on pypi.org, create the `mola-metric-maps` project, or a "pending
publisher" for it, with owner `MOLAorg`, repository `mola`, workflow
`python-wheels.yml` and environment `pypi`; then create that `pypi`
environment in the GitHub repository settings (optionally, with required
reviewers). For TestPyPI, the same on test.pypi.org with environment
`testpypi` (its dependencies come from PyPI: `pip install --index-url
https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple/ mola-metric-maps`).
PyPI never accepts the same version twice: try changes on TestPyPI first.
