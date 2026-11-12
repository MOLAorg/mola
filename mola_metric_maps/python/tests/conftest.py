"""Common fixtures for the mola_metric_maps Python tests."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import numpy as np
import pytest

import mola_metric_maps as mmm
import mp2p_icp

# A map built with MOLA, whose layer "localmap" is an IncrementalPointCloud:
HUSKY_MAP = "ual/husky_benches/husky_benches_map.mm"
HUSKY_MAP_POINTS = 217841

# All the CMetricMap classes, with a constructor argument suitable for the
# synthetic scene below:
MAP_FACTORIES = {
    "HashedVoxelPointCloud": lambda: mmm.HashedVoxelPointCloud(0.5),
    "SparseVoxelPointCloud": lambda: mmm.SparseVoxelPointCloud(0.2),
    "SparseTreesPointCloud": lambda: mmm.SparseTreesPointCloud(5.0),
    "NDT": lambda: mmm.NDT(1.0),
    "TSDF": lambda: mmm.TSDF(0.3),
    "KeyframePointCloudMap": lambda: mmm.KeyframePointCloudMap(),
    "IncrementalPointCloud": lambda: mmm.IncrementalPointCloud(),
}


def _mola_test_datasets_dir() -> Optional[Path]:
    """Datasets of the optional `mola_test_datasets` package, if available.
    MOLA_TEST_DATASET_DIR is the name used by the C++ tests."""
    env = os.environ.get("MOLA_TEST_DATASETS_DIR") or os.environ.get("MOLA_TEST_DATASET_DIR")
    if env:
        return Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("mola_test_datasets")) / "datasets"
    except Exception:
        return None


@pytest.fixture(scope="session")
def husky_map_file() -> Path:
    d = _mola_test_datasets_dir()
    if d is None or not (d / HUSKY_MAP).is_file():
        pytest.skip("mola_test_datasets not found (set MOLA_TEST_DATASETS_DIR)")
    return d / HUSKY_MAP


@pytest.fixture(scope="session")
def render_or_skip():
    """mp2p_icp.render(), skipping the test if off-screen OpenGL is not available."""

    def render(*args, **kwargs) -> np.ndarray:
        # Not pytest.importorskip(): it only skips if the module is missing, not
        # if it fails to load the system OpenGL libraries.
        try:
            import mrpt.opengl  # noqa: F401
        except ImportError as e:
            pytest.skip(f"mrpt.opengl not available: {e}")
        try:
            return mp2p_icp.render(*args, **kwargs)
        except RuntimeError as e:
            if "EGL" in str(e) or "OpenGL" in str(e) or "context" in str(e):
                pytest.skip(f"Off-screen OpenGL rendering not available: {e}")
            raise

    return render


def make_scene(n: int = 4000, seed: int = 42) -> np.ndarray:
    """Points on the ground plane z=0 and on a wall x=5, as an (N,3) float32
    array: two planes, so plane fits and nearest neighbors are predictable."""
    rng = np.random.default_rng(seed)
    ground = np.column_stack(
        [rng.uniform(-8, 8, n), rng.uniform(-8, 8, n), rng.normal(0, 0.005, n)]
    )
    wall = np.column_stack(
        [5 + rng.normal(0, 0.005, n), rng.uniform(-8, 8, n), rng.uniform(0, 3, n)]
    )
    return np.vstack([ground, wall]).astype(np.float32)


@pytest.fixture
def scene() -> np.ndarray:
    return make_scene()


@pytest.fixture(params=list(MAP_FACTORIES))
def map_name(request) -> str:
    return request.param


def filled_map(name: str, points: np.ndarray):
    """A map of the given class, with the scene inserted as seen from a robot
    (the sensor) 1.5 m above the origin. TSDF gets several scans, to end its
    warm-up."""
    m = MAP_FACTORIES[name]()
    local = points - np.array([0, 0, 1.5], dtype=np.float32)
    scans = 12 if name == "TSDF" else 1
    for _ in range(scans):
        assert m.insertPointCloud(local, robotPose=[0, 0, 1.5, 0, 0, 0])
    return m
