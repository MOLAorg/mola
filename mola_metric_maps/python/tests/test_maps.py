"""Tests common to all the map classes: construction, insertion from NumPy,
options, queries, serialization and copies."""

from __future__ import annotations

import copy
import pickle

import numpy as np
import pytest

import mrpt.config
import mrpt.maps
import mrpt.poses
import mrpt.viz

import mola_metric_maps as mmm
import mp2p_icp

from conftest import MAP_FACTORIES, filled_map, make_scene

# Classes implementing the nn_*() queries, and those whose 3D k-NN queries work:
NN_CLASSES = [
    "HashedVoxelPointCloud",
    "SparseVoxelPointCloud",
    "SparseTreesPointCloud",
    "NDT",
    "KeyframePointCloudMap",
    "IncrementalPointCloud",
]
KNN_CLASSES = [c for c in NN_CLASSES if c != "SparseVoxelPointCloud"]


def _on_scene(pts: np.ndarray, tol: float) -> np.ndarray:
    """Whether each point lies on the ground or on the wall of the scene."""
    return (np.abs(pts[:, 2]) < tol) | (np.abs(pts[:, 0] - 5) < tol)


def _prepare_queries(m) -> None:
    if isinstance(m, mmm.KeyframePointCloudMap):
        m.icp_get_prepared_as_global(None)


def _assert_same_contents(a, b) -> None:
    assert type(a) is type(b)
    assert repr(a) == repr(b)
    pa = a.getPointsAsNumpy()
    pb = b.getPointsAsNumpy()
    assert pa.shape == pb.shape
    # Voxel maps may visit their contents in another order:
    np.testing.assert_array_equal(np.unique(pa, axis=0), np.unique(pb, axis=0))


def test_import_registers_classes():
    import mrpt.rtti

    for name in MAP_FACTORIES:
        assert mrpt.rtti.findRegisteredClass(f"mola::{name}") is not None
    assert mmm.__version__.count(".") == 2


def test_bases(map_name):
    m = MAP_FACTORIES[map_name]()
    assert isinstance(m, mrpt.maps.CMetricMap)
    if map_name == "IncrementalPointCloud":
        assert isinstance(m, mrpt.maps.CGenericPointsMap)
    assert m.GetRuntimeClass().className == f"mola::{map_name}"
    assert m.isEmpty()
    assert repr(m).startswith(f"{map_name}(")


def test_insert_numpy(map_name, scene):
    m = filled_map(map_name, scene)
    assert not m.isEmpty()
    pts = m.getPointsAsNumpy()
    assert pts.ndim == 2 and pts.shape[1] == 3 and pts.dtype == np.float32
    assert len(pts) > 1000
    # The TSDF surface is interpolated between voxel centers:
    tol = 0.2 if map_name == "TSDF" else 0.05
    assert _on_scene(pts, tol).mean() > 0.95
    bb = m.boundingBox()
    assert bb.min.x < -7 and bb.max.x > 4.9


@pytest.mark.parametrize(
    "points",
    [
        "array",
        "list",
        "dict",
        "CPointsMap",
    ],
)
def test_insert_point_formats(points, scene):
    if points == "array":
        data = scene
    elif points == "list":
        data = scene[:100].tolist()
    elif points == "dict":
        data = {"x": scene[:, 0], "y": scene[:, 1], "z": scene[:, 2]}
    else:
        data = mp2p_icp.layer_from_numpy(scene)
    m = mmm.HashedVoxelPointCloud(0.5)
    assert m.insertPointCloud(data)
    assert not m.isEmpty()


@pytest.mark.parametrize(
    "pose",
    [
        mrpt.poses.CPose3D(1.0, 2.0, 0.0, 0.0, 0.0, 0.0),
        [1.0, 2.0, 0.0, 0.0, 0.0, 0.0],
        np.array([[1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float),
    ],
)
def test_insert_with_robot_pose(pose):
    m = mmm.HashedVoxelPointCloud(0.5)
    m.insertPointCloud(np.array([[0.1, 0.1, 0.1]], dtype=np.float32), robotPose=pose)
    np.testing.assert_allclose(m.getPointsAsNumpy(), [[1.1, 2.1, 0.1]], atol=1e-6)


def test_insert_bad_input():
    m = mmm.HashedVoxelPointCloud()
    with pytest.raises(ValueError):
        m.insertPointCloud(np.zeros((10, 2)))
    with pytest.raises(TypeError):
        m.insertPointCloud(np.zeros((1, 3)), robotPose="not a pose")


@pytest.mark.parametrize("map_name", NN_CLASSES)
def test_nn_queries(map_name, scene):
    m = filled_map(map_name, scene)
    _prepare_queries(m)
    q = [1.0, 1.0, 0.05]

    idx, pts, d2 = m.nn_radius_search(q, 0.6**2)
    assert len(idx) == len(pts) == len(d2) > 0
    assert pts.shape[1] == 3
    assert np.all(d2 <= 0.6**2 + 1e-6)
    np.testing.assert_allclose(((pts - q) ** 2).sum(axis=1), d2, rtol=1e-4, atol=1e-6)

    if map_name not in KNN_CLASSES:
        return

    r = m.nn_single_search(q)
    assert r is not None
    _, p, dist2 = r
    assert abs(p.z) < 0.05
    assert dist2 < 0.5**2

    idx, pts, d2 = m.nn_multiple_search(q, 5)
    assert len(idx) == 5 and pts.shape == (5, 3)
    assert np.all(np.diff(d2) >= 0)

    queries = np.array([q, [5.05, 0.0, 1.0]], dtype=np.float32)
    b = m.nn_single_search_batch(queries)
    assert set(b) == {"found", "indices", "points", "dist_sqr"}
    assert b["found"].all()
    assert b["points"].shape == (2, 3)
    assert b["dist_sqr"][0] == pytest.approx(dist2, rel=1e-5)
    assert abs(b["points"][1, 0] - 5) < 0.05


@pytest.mark.parametrize("map_name", ["NDT", "TSDF"])
def test_point_to_plane(map_name, scene):
    m = filled_map(map_name, scene)
    if map_name == "TSDF":
        assert not m.isBootstrapping()
    r = m.nn_search_pt2pl([1.0, 1.0, 0.1], 1.0)
    assert r is not None
    patch, dist = r
    assert isinstance(patch, mp2p_icp.PlanePatch)
    assert dist == pytest.approx(0.1, abs=0.05)
    normal = np.array(list(patch.plane.coefs)[:3])
    normal /= np.linalg.norm(normal)
    assert abs(abs(normal[2]) - 1) < 0.05
    assert m.nn_search_pt2pl([100.0, 100.0, 100.0], 1.0) is None


def test_options(map_name):
    m = MAP_FACTORIES[map_name]()
    by_name = m.optionsByName()
    assert "creationOptions" in by_name
    for name, opts in by_name.items():
        assert isinstance(opts, mrpt.config.CLoadableOptions)
        if map_name == "IncrementalPointCloud" and name != "creationOptions":
            # The options of mrpt.maps.CPointsMap, not bound by MRPT yet:
            continue
        assert repr(opts).startswith(f"{map_name}.")
        assert str(opts)
        if name != "creationOptions":
            # The live objects:
            assert type(getattr(m, name)) is type(opts)

    # creationOptions is a copy, applied when assigned back:
    co = m.creationOptions
    assert type(co) is type(m).TCreationOptions
    assert m.trySetCreationOptions(co)
    m.creationOptions = co


def test_live_options():
    m = mmm.HashedVoxelPointCloud(0.5)
    m.insertionOptions.max_points_per_voxel = 2
    assert m.optionsByName()["insertionOptions"].max_points_per_voxel == 2
    m.insertPointCloud(make_scene())
    assert m.getVoxelsAsNumpy()["num_points"].max() == 2

    ins = mmm.HashedVoxelPointCloud.TInsertionOptions()
    ins.max_points_per_voxel = 7
    m.insertionOptions = ins
    assert m.insertionOptions.max_points_per_voxel == 7


@pytest.mark.parametrize(
    "map_name,field,new_value",
    [
        ("HashedVoxelPointCloud", "voxel_size", 0.25),
        ("SparseVoxelPointCloud", "voxel_size", 0.4),
        ("SparseTreesPointCloud", "grid_size", 2.0),
        ("NDT", "voxel_size", 2.0),
        ("TSDF", "voxel_size", 0.2),
    ],
)
def test_structural_creation_options(map_name, field, new_value, scene):
    m = MAP_FACTORIES[map_name]()
    co = m.creationOptions
    setattr(co, field, new_value)
    m.creationOptions = co
    assert getattr(m.creationOptions, field) == pytest.approx(new_value)

    # Not possible once the map has contents:
    m = filled_map(map_name, scene)
    old = getattr(m.creationOptions, field)
    co = m.creationOptions
    setattr(co, field, 3 * old)
    assert not m.trySetCreationOptions(co)
    with pytest.raises(ValueError):
        m.creationOptions = co
    assert getattr(m.creationOptions, field) == pytest.approx(old)

    # Reading the property gives a copy:
    copy_of_opts = m.creationOptions
    setattr(copy_of_opts, field, 123.0)
    assert getattr(m.creationOptions, field) == pytest.approx(old)


def test_creation_options_from_config():
    m = mmm.NDT(1.0)
    cfg = mrpt.config.CConfigFileMemory()
    cfg.setContent("[opts]\nvoxel_size = 0.75\n")
    assert m.trySetCreationOptions(cfg, "opts")
    assert m.creationOptions.voxel_size == pytest.approx(0.75)


def test_visualization(map_name, scene):
    m = filled_map(map_name, scene)
    viz = m.getVisualization()
    assert isinstance(viz, mrpt.viz.CSetOfObjects)


def test_mm_round_trip(map_name, scene, tmp_path):
    m = filled_map(map_name, scene)
    mm = mp2p_icp.MetricMap()
    mm.layers["layer"] = m
    f = tmp_path / "map.mm"
    mm.save(f)

    loaded = mp2p_icp.MetricMap.from_file(f).layers["layer"]
    assert isinstance(loaded, type(m))
    _assert_same_contents(m, loaded)
    assert repr(loaded.creationOptions) == repr(m.creationOptions)


def test_pickle_and_copies(map_name, scene):
    m = filled_map(map_name, scene)
    for other in [pickle.loads(pickle.dumps(m)), copy.copy(m), copy.deepcopy(m)]:
        _assert_same_contents(m, other)

    # Copies are independent:
    c = copy.deepcopy(m)
    n = len(m.getPointsAsNumpy())
    c.clear()
    assert c.isEmpty()
    assert len(m.getPointsAsNumpy()) == n


def test_mrpt_bytes_round_trip(map_name, scene):
    import mrpt.serialization

    m = filled_map(map_name, scene)
    b = mrpt.serialization.objectToBytes(m)
    other = mrpt.serialization.bytesToObject(b)
    assert isinstance(other, type(m))
    _assert_same_contents(m, other)


def test_layer_to_numpy(scene):
    # Classes that provide a point cloud view of their contents:
    for name in [
        "HashedVoxelPointCloud",
        "NDT",
        "TSDF",
        "KeyframePointCloudMap",
        "IncrementalPointCloud",
    ]:
        m = filled_map(name, scene)
        f = mp2p_icp.layer_to_numpy(m)
        assert len(f["x"]) == len(m.getPointsAsNumpy())
