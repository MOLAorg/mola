"""Tests of the methods specific to each map class."""

from __future__ import annotations

import numpy as np
import pytest

import mrpt.maps
import mrpt.math
import mrpt.obs
import mrpt.poses
import mrpt.viz

import mola_metric_maps as mmm

from conftest import filled_map, make_scene


def _wall(n: int = 2000, x: float = 0.0, seed: int = 1) -> np.ndarray:
    """A vertical wall at x, 4x2 m."""
    rng = np.random.default_rng(seed)
    return np.column_stack([np.full(n, x), rng.uniform(-2, 2, n), rng.uniform(0, 2, n)]).astype(
        np.float32
    )


# -----------------------------------------------------------------------------
# Voxel maps
# -----------------------------------------------------------------------------
def test_hashed_voxels(scene):
    m = filled_map("HashedVoxelPointCloud", scene)
    v = m.getVoxelsAsNumpy()
    assert set(v) == {"indices", "centers", "num_points"}
    n = len(v["num_points"])
    assert n == m.voxelCount()
    assert v["indices"].shape == (n, 3) and v["indices"].dtype == np.int32
    assert v["centers"].shape == (n, 3)
    assert v["num_points"].sum() == len(m.getPointsAsNumpy())
    # Centers of voxels of side 0.5, which span [i, i+1) * size:
    np.testing.assert_allclose(v["centers"], (v["indices"] + 0.5) * 0.5, atol=1e-5)

    m.setVoxelProperties(0.25)
    assert m.isEmpty() and m.creationOptions.voxel_size == pytest.approx(0.25)
    m.insertPoint([0.1, 0.2, 0.3])
    m.insertPoint(mrpt.math.TPoint3D(0.1, 0.2, 0.35))
    assert m.voxelCount() == 1
    assert len(m.getPointsAsNumpy()) == 2


def test_sparse_voxels(scene):
    m = filled_map("SparseVoxelPointCloud", scene)
    v = m.getVoxelsAsNumpy()
    assert set(v) == {"indices", "centers", "num_points", "means"}
    assert np.all(v["num_points"] > 0)
    assert v["num_points"].sum() == len(m.getPointsAsNumpy())
    # Voxels are centered at i * size, and their means lie inside them:
    np.testing.assert_allclose(v["centers"], v["indices"] * 0.2, atol=1e-5)
    assert np.all(np.abs(v["means"] - v["centers"]) <= 0.1 + 1e-5)

    m.likelihoodOptions.match_mean = True
    _, pts, _ = m.nn_radius_search([1.0, 1.0, 0.0], 0.5**2)
    assert len(pts) > 0


def test_sparse_trees(scene):
    m = filled_map("SparseTreesPointCloud", scene)
    g = m.getGridsAsNumpy()
    assert g["num_points"].sum() == len(m.getPointsAsNumpy())
    np.testing.assert_allclose(g["centers"], g["indices"] * 5.0, atol=1e-5)

    n_before = len(g["num_points"])
    m.eraseGridsFartherThan([-8.0, -8.0, 0.0], 6.0)
    assert 0 < len(m.getGridsAsNumpy()["num_points"]) < n_before


def test_ndt_voxels(scene):
    m = filled_map("NDT", scene)
    v = m.getVoxelsAsNumpy()
    n = len(v["num_points"])
    assert n == m.voxelCount()
    for k in ["means", "eigenvalues", "normals"]:
        assert v[k].shape == (n, 3)
    fitted = v["has_ndt"]
    assert fitted.sum() > n // 2
    assert np.all(np.isnan(v["means"][~fitted]))
    # Ground voxels are planes with a vertical normal:
    ground = fitted & (np.abs(v["means"][:, 2]) < 0.05) & (np.abs(v["means"][:, 0] - 5) > 1)
    assert ground.sum() > 10
    assert v["is_plane"][ground].mean() > 0.9
    planes = ground & v["is_plane"]
    np.testing.assert_allclose(np.abs(v["normals"][planes, 2]), 1.0, atol=0.05)
    # Eigenvalues in ascending order:
    assert np.all(np.diff(v["eigenvalues"][fitted], axis=1) >= 0)

    m2 = mmm.NDT(1.0)
    m2.insertPoint([0.5, 0.5, 0.0], [0.0, 0.0, 2.0])
    assert m2.voxelCount() == 1


def test_tsdf(scene):
    m = filled_map("TSDF", scene)
    assert not m.isBootstrapping()
    assert m.lastFieldReadyFraction() > 0.5
    assert m.voxelSize() == pytest.approx(0.3)
    assert len(m) == m.size() > 0

    q = m.queryField([1.0, 1.0, 0.1])
    assert q is not None
    dist, grad = q
    # Positive towards the sensor, above the ground (the field is biased away
    # from the sensor by oblique rays, a fraction of a voxel):
    assert 0 < dist < 0.3
    assert grad.shape == (3,) and grad[2] > 0
    assert m.queryField([100.0, 100.0, 100.0]) is None

    b = m.queryFieldBatch(np.array([[1.0, 1.0, 0.1], [100.0, 100.0, 100.0]]))
    assert b["valid"].tolist() == [True, False]
    assert b["dist"][0] == pytest.approx(dist)
    assert np.isnan(b["dist"][1])
    assert b["gradient"].shape == (2, 3)

    v = m.getVoxelsAsNumpy()
    assert len(v["dist"]) == len(v["weight"]) == m.size()
    np.testing.assert_allclose(v["centers"], (v["indices"] + 0.5) * 0.3, atol=1e-5)

    pts, w = m.getZeroCrossingsAsNumpy()
    assert pts.shape == (len(w), 3)
    np.testing.assert_array_equal(pts, m.getPointsAsNumpy())

    m.renderOptions.render_as_mesh = True
    assert isinstance(m.getVisualization(), mrpt.viz.CSetOfObjects)


def test_tsdf_insert_point():
    m = mmm.TSDF(0.2)
    m.insertionOptions.bootstrap_with_points = False
    for y in np.linspace(-1, 1, 41):
        for z in np.linspace(-1, 1, 41):
            m.insertPoint([2.0, y, z], [0.0, 0.0, 0.0])
    q = m.queryField([1.9, 0.0, 0.0])
    assert q is not None
    assert q[0] == pytest.approx(0.1, abs=0.05)


# -----------------------------------------------------------------------------
# KeyframePointCloudMap
# -----------------------------------------------------------------------------
def test_keyframes():
    m = mmm.KeyframePointCloudMap()
    assert m.lastInsertedKeyFrameID() is None
    assert m.nextFreeKeyFrameID() == 0

    poses = [[0, 0, 0, 0, 0, 0], [2, 0, 0, 0.1, 0, 0], [4, 1, 0, 0.2, 0, 0]]
    for p in poses:
        assert m.insertPointCloud(_wall(x=3.0), robotPose=p)
    assert m.lastInsertedKeyFrameID() == 2
    assert m.nextFreeKeyFrameID() == 3
    assert repr(m) == "KeyframePointCloudMap(keyframes=3)"

    kf = m.keyframePoses()
    assert sorted(kf) == [0, 1, 2]
    assert isinstance(kf[1], mrpt.poses.CPose3D)
    assert kf[1].x == pytest.approx(2.0)

    ids, mats = m.getKeyframePosesAsNumpy()
    assert ids.tolist() == [0, 1, 2]
    assert mats.shape == (3, 4, 4)
    np.testing.assert_allclose(
        mats[2], mrpt.poses.CPose3D(4, 1, 0, 0.2, 0, 0).getHomogeneousMatrix(), atol=1e-12
    )

    m.setKeyframePose(1, [10, 0, 0, 0, 0, 0])
    assert m.keyframePoses()[1].x == pytest.approx(10.0)

    m.transform_map_left_multiply([0, 0, 1, 0, 0, 0])
    assert m.getKeyframePosesAsNumpy()[1][:, 2, 3] == pytest.approx([1.0, 1.0, 1.0])

    # Global points: the wall of each keyframe, transformed by its pose:
    assert len(m.getPointsAsNumpy()) == 3 * 2000


def test_keyframes_eviction_and_queries():
    m = mmm.KeyframePointCloudMap()
    m.insertionOptions.remove_frames_farther_than = 5.0
    m.insertPointCloud(_wall(x=3.0), robotPose=[0, 0, 0, 0, 0, 0])
    m.insertPointCloud(_wall(x=3.0), robotPose=[1, 0, 0, 0, 0, 0])
    m.insertPointCloud(_wall(x=3.0), robotPose=[20, 0, 0, 0, 0, 0])
    assert sorted(m.drainEvictedKeyFrameIDs()) == [0, 1]
    assert m.drainEvictedKeyFrameIDs() == []
    assert list(m.keyframePoses()) == [2]

    m.icp_get_prepared_as_global([20, 0, 0, 0, 0, 0])
    assert m.point_count() == 2000
    r = m.nn_single_search([23.1, 0.0, 1.0])
    assert r is not None
    assert r[1].x == pytest.approx(23.0, abs=1e-4)
    m.icp_cleanup()


def test_keyframes_merge_and_regroup():
    a = mmm.KeyframePointCloudMap()
    b = mmm.KeyframePointCloudMap()
    for x in [0.0, 1.0, 2.0]:
        a.insertPointCloud(_wall(x=3.0, seed=int(x)), robotPose=[x, 0, 0, 0, 0, 0])
    b.insertPointCloud(_wall(x=3.0), robotPose=[0, 0, 0, 0, 0, 0])
    a.merge_with(b, [0, 5, 0, 0, 0, 0])
    assert len(a.keyframePoses()) == 4

    params = mmm.KeyframePointCloudMap.RegroupParams()
    params.unify_all = True
    assert "unify_all=True" in repr(params)
    messages = []
    regrouped = a.regroupKeyframes(params, messages.append)
    assert isinstance(regrouped, mmm.KeyframePointCloudMap)
    assert len(regrouped.keyframePoses()) == 1
    assert len(a.keyframePoses()) == 4
    assert all(isinstance(msg, str) for msg in messages)
    assert len(a.regroupKeyframes(params).keyframePoses()) == 1


def test_keyframes_creation_options():
    m = mmm.KeyframePointCloudMap()
    co = m.creationOptions
    co.max_search_keyframes = 5
    co.view_direction_filter = mmm.ViewDirectionFilter.SurfaceSide
    m.creationOptions = co
    assert m.creationOptions.max_search_keyframes == 5
    assert m.creationOptions.view_direction_filter == mmm.ViewDirectionFilter.SurfaceSide
    # Always applicable, even with contents:
    m.insertPointCloud(_wall())
    co.k_correspondences_for_cov = 10
    m.creationOptions = co
    assert m.creationOptions.k_correspondences_for_cov == 10


# -----------------------------------------------------------------------------
# IncrementalPointCloud
# -----------------------------------------------------------------------------
@pytest.mark.parametrize("index", ["VoxelHash", "KDTree"])
def test_incremental_point_cloud(index):
    if index == "KDTree" and not mmm.HAS_INCREMENTAL_KDTREE:
        pytest.skip("Built without the incremental k-d tree (nanoflann < 1.10)")
    m = mmm.IncrementalPointCloud()
    co = m.creationOptions
    co.index_type = getattr(mmm.PointIndexType, index)
    co.remove_points_farther_than = 10.0
    m.creationOptions = co
    assert m.creationOptions.index_type == getattr(mmm.PointIndexType, index)

    scene = make_scene()
    m.insertPointCloud(scene, robotPose=[0, 0, 0, 0, 0, 0])
    assert m.livePointCount() == len(scene)

    # The sliding window removes what is farther than 10 m from the robot:
    m.insertPointCloud(scene, robotPose=[15, 0, 0, 0, 0, 0])
    live = m.liveCompactedCopy()
    assert isinstance(live, mrpt.maps.CGenericPointsMap)
    assert live.size() == m.livePointCount()
    pts = live.getPointsAsNumpy()
    assert np.all(pts[:, 0] >= 5.0 - 1e-4)
    assert m.size() >= m.livePointCount()

    m.compact()
    assert m.size() == m.livePointCount()
    assert np.all(np.isfinite(m.getPointsAsNumpy()))

    r = m.nn_single_search([16.0, 1.0, 0.05])
    assert r is not None and r[2] < 0.5**2
    b = m.nn_single_search_batch(np.array([[16.0, 1.0, 0.05]]))
    assert b["found"][0] and b["indices"][0] == r[0]

    m.keepOnlyPointsNear([20.0, 0.0, 0.0], 2.0)
    pts = m.liveCompactedCopy().getPointsAsNumpy()
    assert len(pts) > 0 and np.all(np.abs(pts[:, 0] - 20) <= 2 + 1e-4)


def test_incremental_point_cloud_fields_and_transform():
    m = mmm.IncrementalPointCloud()
    scene = make_scene(500)
    intensity = np.arange(len(scene), dtype=np.float32)
    m.insertPointCloud(
        {"x": scene[:, 0], "y": scene[:, 1], "z": scene[:, 2], "intensity": intensity}
    )
    fields = m.getPointFieldsAsNumpy()
    np.testing.assert_array_equal(fields["intensity"], intensity)

    m.changeCoordinatesReference(mrpt.poses.CPose3D(1.0, 0, 0, 0, 0, 0))
    np.testing.assert_allclose(m.getPointsAsNumpy()[:, 0], scene[:, 0] + 1, atol=1e-5)
    r = m.nn_single_search(mrpt.math.TPoint3D(*(scene[0] + [1.0, 0, 0])))
    assert r is not None and r[2] < 1e-8

    assert m.covarianceComputations() == 0
    assert m.cachedCovarianceCount() == 0
    assert m.recyclableSlotCount() == 0


# -----------------------------------------------------------------------------
# OccGrid
# -----------------------------------------------------------------------------
def test_occgrid():
    import mrpt.serialization

    g = mmm.OccGrid()
    assert isinstance(g, mrpt.serialization.CSerializable)
    g.setSize([-10, -10], [10, 10], 0.1)
    assert g.grid().getSizeX() == 200
    assert "200x200" in repr(g)

    g.insertionParameters.maxDistanceInsertion = 20.0
    assert g.insertionParameters.maxDistanceInsertion == pytest.approx(20.0)
    assert "maxDistanceInsertion=20" in repr(g.insertionParameters)

    scan = mrpt.obs.CObservation2DRangeScan()
    scan.aperture = np.pi
    scan.maxRange = 20.0
    scan.loadFromVectors(np.full(181, 5.0, dtype=np.float32), np.ones(181, dtype=bool))
    g.insertObservation(scan, mrpt.math.TPose2D(0, 0, 0))
    assert g.grid().getCell(0, 0) == pytest.approx(0.5)  # unknown corner
    assert g.grid().getPos(2.0, 0.0) > 0.5  # free space in front

    g.resizeGrid([-20, -10], [10, 10])
    assert g.grid().getSizeX() >= 300
    assert g.grid().getPos(2.0, 0.0) > 0.5

    import pickle

    g2 = pickle.loads(pickle.dumps(g))
    assert g2.grid().getSizeX() == g.grid().getSizeX()
