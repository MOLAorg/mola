"""Tests with a map built with MOLA, whose layer "localmap" is an
IncrementalPointCloud. Skipped if the dataset (mola_test_datasets) is not
available."""

from __future__ import annotations

import subprocess
import sys

import numpy as np
import pytest

import mrpt.maps

import mola_metric_maps as mmm
import mp2p_icp

from conftest import HUSKY_MAP_POINTS


@pytest.fixture(scope="module")
def husky_map(husky_map_file):
    return mp2p_icp.MetricMap.from_file(husky_map_file)


def test_load_without_plugins(husky_map_file, tmp_path):
    # Importing the package is enough to load the map, with the actual layer
    # class. Run in a new process, where no plugin was loaded by other tests:
    code = (
        "import sys\n"
        "import mola_metric_maps\n"
        "import mp2p_icp\n"
        "m = mp2p_icp.MetricMap.from_file(sys.argv[1])\n"
        "layer = m.layers['localmap']\n"
        "assert type(layer) is mola_metric_maps.IncrementalPointCloud, type(layer)\n"
        "print(layer.size())\n"
    )
    r = subprocess.run(
        [sys.executable, "-c", code, str(husky_map_file)],
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )
    assert r.returncode == 0, f"{r.stdout}\n{r.stderr}"
    assert r.stdout.split()[-1] == str(HUSKY_MAP_POINTS)


def test_layer_type(husky_map):
    assert list(husky_map.layers) == ["localmap"]
    layer = husky_map.layers["localmap"]
    assert type(layer) is mmm.IncrementalPointCloud
    assert isinstance(layer, mrpt.maps.CGenericPointsMap)
    assert isinstance(layer, mrpt.maps.CPointsMap)
    assert layer.size() == HUSKY_MAP_POINTS
    assert layer.livePointCount() == HUSKY_MAP_POINTS
    assert husky_map.size() == HUSKY_MAP_POINTS
    assert (
        repr(layer)
        == f"IncrementalPointCloud(points={HUSKY_MAP_POINTS}, storage={HUSKY_MAP_POINTS})"
    )
    assert isinstance(layer.creationOptions, mmm.IncrementalPointCloud.TCreationOptions)


def test_layer_to_numpy(husky_map):
    layer = husky_map.layers["localmap"]
    fields = mp2p_icp.layer_to_numpy(layer)
    for name in ["x", "y", "z"]:
        assert fields[name].shape == (HUSKY_MAP_POINTS,)
        assert np.all(np.isfinite(fields[name]))
    xyz = np.column_stack([fields["x"], fields["y"], fields["z"]])
    np.testing.assert_array_equal(xyz, layer.getPointsAsNumpy())


def test_queries(husky_map):
    layer = husky_map.layers["localmap"]
    pts = layer.getPointsAsNumpy()
    sample = pts[:: len(pts) // 100]
    r = layer.nn_single_search_batch(sample)
    assert r["found"].all()
    np.testing.assert_allclose(r["dist_sqr"], 0, atol=1e-10)
    np.testing.assert_array_equal(r["points"], sample)

    idx, nn, d2 = layer.nn_multiple_search(sample[0], 10)
    assert len(idx) == 10 and d2[0] == 0


def test_save_load(husky_map, tmp_path):
    f = tmp_path / "husky.mm"
    husky_map.save(f)
    layer = mp2p_icp.MetricMap.from_file(f).layers["localmap"]
    assert type(layer) is mmm.IncrementalPointCloud
    assert layer.size() == HUSKY_MAP_POINTS


def test_render(husky_map, render_or_skip):
    img = render_or_skip(husky_map, 320, 240, background=(1, 1, 1))
    assert img.shape == (240, 320, 3)
    # Some non-white (point) pixels:
    assert np.any(img < 200, axis=2).mean() > 0.01
