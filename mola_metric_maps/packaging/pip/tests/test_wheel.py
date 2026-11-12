#!/usr/bin/env python3
"""Checks an installed mola-metric-maps wheel: it imports, it uses the
libraries of the mp2p-icp and mrpt3 wheels (no bundled copies), and it works
(maps from NumPy points, queries, .mm round trips, mp2p_icp loading maps with
these classes without importing mola_metric_maps). Then it runs the Python
tests of the source tree (if pytest is found), except those that need newer
MRPT Python bindings than the installed mrpt3 wheel.

Usage: python test_wheel.py   (with the wheel installed in the environment)

A map built with MOLA (ual/husky_benches/husky_benches_map.mm, from the
mola_test_datasets repository) is also tested, if found in the directory set
in the MOLA_TEST_DATASETS_DIR environment variable.
"""

import importlib.metadata
import importlib.util
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

SRC_DIR = Path(__file__).resolve().parents[3]  # mola_metric_maps/
TESTS_DIR = SRC_DIR / "python" / "tests"

# A map built with MOLA, whose layer "localmap" is an IncrementalPointCloud:
HUSKY_MAP = "ual/husky_benches/husky_benches_map.mm"
HUSKY_MAP_POINTS = 217841

# Libraries that must come from the mp2p-icp and mrpt3 wheels, never from this
# one:
SDK_LIB_PREFIXES = ("libmp2p_icp", "libmrpt_", "libtbb")

# RUNPATH of the binaries in site-packages/mola_metric_maps/:
EXPECTED_RUNPATH = "$ORIGIN:$ORIGIN/../mp2p_icp/sdk/lib:$ORIGIN/../mrpt/sdk/lib"


def husky_map_file():
    d = os.environ.get("MOLA_TEST_DATASETS_DIR")
    if d and (Path(d) / HUSKY_MAP).is_file():
        return Path(d) / HUSKY_MAP
    return None


def make_scene(n: int = 3000, seed: int = 0) -> np.ndarray:
    """Points on the ground plane z=0 and on a wall x=5, (N,3) float32."""
    rng = np.random.default_rng(seed)
    ground = np.column_stack(
        [rng.uniform(-8, 8, n), rng.uniform(-8, 8, n), rng.normal(0, 0.005, n)]
    )
    wall = np.column_stack(
        [5 + rng.normal(0, 0.005, n), rng.uniform(-8, 8, n), rng.uniform(0, 3, n)]
    )
    return np.vstack([ground, wall]).astype(np.float32)


def check_install(failed: list) -> None:
    import mrpt.sdk
    import mp2p_icp.sdk
    import mola_metric_maps as mmm

    pkg_dir = Path(mmm.__file__).resolve().parent
    mp2p_lib_dir = Path(mp2p_icp.sdk.get_library_dir()).resolve()
    mrpt_lib_dir = Path(mrpt.sdk.get_library_dir()).resolve()
    print("mola_metric_maps", mmm.__version__, "from", pkg_dir)
    print("mp2p_icp", mp2p_icp.__version__, "SDK from", mp2p_lib_dir)
    print("MRPT SDK", mrpt.sdk.MRPT_VERSION, "from", mrpt_lib_dir)

    dist_version = importlib.metadata.version("mola-metric-maps")
    if mmm.__version__ != dist_version:
        failed.append(f"__version__ {mmm.__version__} != wheel version {dist_version}")
    m = re.search(r"<version>([0-9.]+)</version>", (SRC_DIR / "package.xml").read_text())
    if m and m.group(1) != dist_version:
        failed.append(f"wheel version {dist_version} != package.xml version {m.group(1)}")

    # No copies of the SDK libraries in the wheel:
    for f in importlib.metadata.files("mola-metric-maps") or []:
        if Path(f).name.startswith(SDK_LIB_PREFIXES):
            failed.append(f"SDK library bundled in the wheel: {f}")

    # Where the shared libraries of this process were loaded from, each one
    # only once (extension modules, e.g. "_core", may share names):
    loaded = {}
    for line in Path("/proc/self/maps").read_text().splitlines():
        path = line.split()[-1]
        if path.startswith("/") and ".so" in path:
            p = Path(path).resolve()
            loaded.setdefault(p.name, set()).add(p.parent)
    for name, dirs in sorted(loaded.items()):
        if name.startswith("lib") and len(dirs) > 1:
            failed.append(f"{name} loaded from several dirs: {sorted(map(str, dirs))}")
        d = next(iter(dirs))
        if name.startswith(("libmrpt_", "libtbb")) and d != mrpt_lib_dir:
            failed.append(f"{name} loaded from {d}, not from the MRPT SDK")
        if name.startswith("libmp2p_icp") and d != mp2p_lib_dir:
            failed.append(f"{name} loaded from {d}, not from the mp2p_icp SDK")
        if name.startswith("libmola_metric_maps") and d != pkg_dir:
            failed.append(f"{name} loaded from {d}, not from the wheel")
    for expected in ["libmola_metric_maps.so", "libmp2p_icp_map.so", "libmrpt_maps.so", "libtbb.so.12"]:
        if expected not in loaded:
            failed.append(f"{expected} not loaded")

    # nanoflann >= 1.10 (MRPT SDK) enables the k-d tree index:
    if not mmm.HAS_INCREMENTAL_KDTREE:
        failed.append("Built without the IncrementalPointCloud k-d tree index")
    print(
        f"[OK] install (k-d tree index: {mmm.HAS_INCREMENTAL_KDTREE}, "
        f"k-d tree baking: {mmm.HAS_INCREMENTAL_KDTREE_BAKE})"
    )


def check_binaries(failed: list) -> None:
    """RUNPATH of the binaries, and that all their dependencies are found."""
    import mola_metric_maps as mmm

    if not shutil.which("readelf") or not shutil.which("ldd"):
        print("[SKIP] binaries: readelf or ldd not found")
        return
    pkg_dir = Path(mmm.__file__).resolve().parent
    files = sorted(pkg_dir.glob("*.so"))
    names = [f.name for f in files]
    if "libmola_metric_maps.so" not in names or not any(n.startswith("_core") for n in names):
        failed.append(f"Unexpected binaries in the package: {names}")
    for f in files:
        dyn = subprocess.run(["readelf", "-d", str(f)], capture_output=True, text=True).stdout
        m = re.search(r"\((?:RUNPATH|RPATH)\).*\[(.*)\]", dyn)
        runpath = m.group(1) if m else ""
        if runpath != EXPECTED_RUNPATH:
            failed.append(f"{f.name}: RUNPATH '{runpath}', expected '{EXPECTED_RUNPATH}'")
        deps = subprocess.run(["ldd", str(f)], capture_output=True, text=True).stdout
        if "not found" in deps:
            failed.append(f"{f.name}: missing dependencies:\n{deps}")
    print("[OK] binaries:", ", ".join(names))


def check_maps(failed: list, tmp: Path) -> tuple:
    """Maps from NumPy points, queries, and a .mm round trip. Returns the
    saved .mm file, with an IncrementalPointCloud layer, and its size."""
    import mp2p_icp
    import mola_metric_maps as mmm

    pts = make_scene()
    ipc = mmm.IncrementalPointCloud()
    ipc.insertPointCloud(pts, robotPose=[0, 0, 0, 0, 0, 0])
    if ipc.livePointCount() == 0:
        failed.append("IncrementalPointCloud: no points inserted")
    q = ipc.nn_single_search_batch(pts[:50])
    if not q["found"].all() or not np.allclose(q["dist_sqr"], 0, atol=1e-6):
        failed.append("IncrementalPointCloud: nn_single_search_batch() did not find the points")
    idx, nn, d2 = ipc.nn_multiple_search([0.0, 0.0, 0.0], 5)
    if len(idx) != 5:
        failed.append(f"IncrementalPointCloud: nn_multiple_search() gave {len(idx)} points")

    ndt = mmm.NDT(1.0)
    ndt.insertPointCloud(pts, robotPose=[0, 0, 1.5, 0, 0, 0])
    voxels = ndt.getVoxelsAsNumpy()
    hashed = mmm.HashedVoxelPointCloud(0.5)
    hashed.insertPointCloud(pts)
    if not len(voxels["indices"]) or not len(hashed.getPointsAsNumpy()):
        failed.append("NDT/HashedVoxelPointCloud: empty after insertion")

    mm = mp2p_icp.MetricMap()
    mm.layers["localmap"] = ipc
    mm.layers["ndt"] = ndt
    mm.layers["voxels"] = hashed
    f = tmp / "synthetic.mm"
    mm.save(f)
    back = mp2p_icp.MetricMap.from_file(f)
    for name, layer in mm.layers.items():
        b = back.layers[name]
        if type(b) is not type(layer) or repr(b) != repr(layer):
            failed.append(f".mm round trip, layer '{name}': {b!r} vs {layer!r}")
    np.testing.assert_array_equal(
        back.layers["localmap"].getPointsAsNumpy(), ipc.getPointsAsNumpy()
    )
    print("[OK] maps and .mm round trip:", {n: repr(x) for n, x in back.layers.items()})
    return f, ipc.size()


def check_auto_import(failed: list, mm_file: Path, points: int, layer="localmap") -> None:
    """mp2p_icp imports mola_metric_maps when a map needs its classes. Run in
    a new process, where nothing imported mola_metric_maps."""
    code = (
        "import sys\n"
        "import mp2p_icp\n"
        "assert 'mola_metric_maps' not in sys.modules\n"
        "m = mp2p_icp.MetricMap.from_file(sys.argv[1])\n"
        "assert 'mola_metric_maps' in sys.modules, 'not imported'\n"
        "import mola_metric_maps\n"
        f"layer = m.layers[{layer!r}]\n"
        "assert type(layer) is mola_metric_maps.IncrementalPointCloud, type(layer)\n"
        "print(layer.size())\n"
    )
    with tempfile.TemporaryDirectory() as cwd:
        r = subprocess.run(
            [sys.executable, "-c", code, str(mm_file)], capture_output=True, text=True, cwd=cwd
        )
    if r.returncode != 0 or r.stdout.split()[-1:] != [str(points)]:
        failed.append(f"auto-import from mp2p_icp ({mm_file.name}):\n{r.stdout}\n{r.stderr}")
        return
    print(f"[OK] auto-import from mp2p_icp: {mm_file.name}, {points} points")


def check_husky_map(failed: list, mm_file: Path) -> None:
    import mp2p_icp
    import mola_metric_maps as mmm

    m = mp2p_icp.MetricMap.from_file(mm_file)
    layer = m.layers["localmap"]
    if type(layer) is not mmm.IncrementalPointCloud or layer.size() != HUSKY_MAP_POINTS:
        failed.append(f"husky map: unexpected layer {layer!r}")
        return
    fields = mp2p_icp.layer_to_numpy(layer)
    xyz = np.column_stack([fields["x"], fields["y"], fields["z"]])
    q = layer.nn_single_search_batch(xyz[::1000])
    if xyz.shape != (HUSKY_MAP_POINTS, 3) or not q["found"].all():
        failed.append("husky map: unexpected NumPy fields or queries")
    print(f"[OK] husky map: {layer!r}, fields {sorted(fields)}")


def known_upstream_issues() -> list:
    """pytest arguments to deselect the tests of the source tree that need
    fixes or features of MRPT Python bindings newer than the installed mrpt3
    wheel. Each probe stops deselecting once the mrpt3 wheel has them."""
    import mrpt.maps
    import mrpt.obs

    skipped = []
    # CMetricMap.GetRuntimeClass() freed the static class ID it returns:
    probe = "import mrpt.maps; mrpt.maps.CSimplePointsMap().GetRuntimeClass()"
    if subprocess.run([sys.executable, "-c", probe], capture_output=True).returncode != 0:
        skipped.append(("test_bases", "CMetricMap.GetRuntimeClass() crashes"))
    if not hasattr(mrpt.maps.CPointsMap, "getPointFieldsAsNumpy"):
        skipped.append(
            ("test_incremental_point_cloud_fields_and_transform", "no getPointFieldsAsNumpy()")
        )
    if not hasattr(mrpt.obs.CObservation2DRangeScan, "loadFromVectors"):
        skipped.append(("test_occgrid", "no CObservation2DRangeScan.loadFromVectors()"))
    for test, why in skipped:
        print(f"[KNOWN] mrpt3 bindings: {why}, deselecting {test}")
    return ["-k", " and ".join(f"not {t}" for t, _ in skipped)] if skipped else []


def run_pytest(failed: list) -> None:
    if importlib.util.find_spec("pytest") is None:
        print("[SKIP] Python tests of the source tree: pytest not found")
        return
    # Run from elsewhere, so the package in the source tree is not imported
    # instead of the installed one:
    with tempfile.TemporaryDirectory() as cwd:
        r = subprocess.run(
            [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", str(TESTS_DIR)]
            + known_upstream_issues(),
            cwd=cwd,
        )
    if r.returncode != 0:
        failed.append(f"Python tests of the source tree failed ({TESTS_DIR})")


def main() -> int:
    # Keep the order of the output of this script and its subprocesses:
    sys.stdout.reconfigure(line_buffering=True)
    failed = []
    husky = husky_map_file()
    with tempfile.TemporaryDirectory() as tmp:
        checks = [
            lambda: check_auto_import(failed, *check_maps(failed, Path(tmp))),
        ]
        if husky:
            checks += [
                lambda: check_auto_import(failed, husky, HUSKY_MAP_POINTS),
                lambda: check_husky_map(failed, husky),
            ]
        else:
            print(f"[SKIP] husky map: set MOLA_TEST_DATASETS_DIR to test {HUSKY_MAP}")
        # Last, so libraries loaded on demand are checked too:
        checks += [lambda: check_binaries(failed), lambda: check_install(failed)]
        for check in checks:
            try:
                check()
            except Exception as e:  # noqa: BLE001
                failed.append(f"{type(e).__name__}: {e}")
    run_pytest(failed)

    if failed:
        print("\nFAILURES:", *failed, sep="\n- ")
        return 1
    print("\nAll checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
