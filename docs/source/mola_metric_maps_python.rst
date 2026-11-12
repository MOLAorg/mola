.. _mola_metric_maps_python:

==============================
Python API: mola_metric_maps
==============================

The ``mola_metric_maps`` Python package wraps the map classes of the
`mola_metric_maps <https://github.com/MOLAorg/mola/tree/develop/mola_metric_maps>`_
C++ library. They are the layers of the ``.mm`` maps built by MOLA, and are used together
with the ``mp2p_icp`` Python package (``MetricMap``, file formats, ICP) and the ``mrpt``
one (MRPT 3: points, poses, base map classes).

Install
=======

**pip** (Linux x86_64 and aarch64, Python 3.10 to 3.14): ``pip install mola-metric-maps``.
It installs the ``mp2p-icp`` and ``mrpt3`` wheels too, whose C++ libraries it uses.

**ROS 2**: the bindings are built with ``mola_metric_maps`` whenever ``pybind11`` is found
(``pybind11-dev`` on Debian/Ubuntu). After ``source install/setup.bash`` (or the ROS
distribution setup file), ``import mola_metric_maps`` just works. It needs the ``mrpt`` and
``mp2p_icp`` Python packages, built the same way.

Classes
=======

.. list-table::
   :widths: 30 25 45
   :header-rows: 1

   * - Class
     - Python base
     - What it is
   * - ``IncrementalPointCloud``
     - ``mrpt.maps.CGenericPointsMap``
     - Sliding-window local map for LiDAR odometry, with an incremental spatial index.
   * - ``KeyframePointCloudMap``
     - ``mrpt.maps.CMetricMap``
     - Point clouds anchored to keyframe poses, for SLAM and localization.
   * - ``HashedVoxelPointCloud``
     - ``mrpt.maps.CMetricMap``
     - Points in a hashed grid of voxels, up to 32 per voxel.
   * - ``SparseVoxelPointCloud``
     - ``mrpt.maps.CMetricMap``
     - Points in a two-level voxel grid, with per-voxel means.
   * - ``SparseTreesPointCloud``
     - ``mrpt.maps.CMetricMap``
     - Point cloud split into blocks with their own k-d trees (experimental).
   * - ``NDT``
     - ``mrpt.maps.CMetricMap``
     - Normal Distributions Transform: per-voxel Gaussians and planes.
   * - ``TSDF``
     - ``mrpt.maps.CMetricMap``
     - Truncated signed distance field.
   * - ``OccGrid``
     - ``mrpt.serialization.CSerializable``
     - 2D occupancy grid with a likelihood cache (experimental).

Each class has its option structures as nested classes (``TCreationOptions``,
``TInsertionOptions``, ``TLikelihoodOptions``, ``TRenderOptions``), with the fields
documented in the :doc:`C++ API <doxygen-index>`.

Loading maps built with MOLA
============================

Importing the package loads the C++ library, which registers its classes for
deserialization: there is no need to call ``mp2p_icp.load_plugins()``, and layers come out
with their actual Python class, including all the methods of their base classes. Even the
import is optional: ``mp2p_icp`` imports ``mola_metric_maps`` by itself the first time a map
needs one of its classes.

.. code-block:: python

    import mola_metric_maps
    import mp2p_icp

    m = mp2p_icp.MetricMap.from_file("map.mm")
    layer = m.layers["localmap"]          # a mola_metric_maps.IncrementalPointCloud
    print(layer.livePointCount(), layer.creationOptions)

    fields = mp2p_icp.layer_to_numpy(layer)  # x, y, z, intensity...: NumPy arrays

Building maps
=============

``insertPointCloud()`` inserts a point cloud given in the robot frame (an ``(N,3)`` array, a
dict of per-point fields, or an ``mrpt.maps.CPointsMap``), with the robot at a given pose
(``mrpt.poses.CPose3D``, a 4x4 matrix or ``[x, y, z, yaw, pitch, roll]``). It works as
``insertObservation()``, so insertion options apply. Maps that trace rays (``TSDF``,
``NDT``) take the robot position as their origin, and ``KeyframePointCloudMap`` creates a
new keyframe on each call.

.. code-block:: python

    import numpy as np
    import mola_metric_maps as mmm
    import mp2p_icp

    points = np.random.rand(1000, 3).astype(np.float32) * 10

    ndt = mmm.NDT(voxel_size=1.0)
    ndt.insertionOptions.max_eigen_ratio_for_planes = 0.02
    ndt.insertPointCloud(points, robotPose=[0, 0, 1.5, 0, 0, 0])

    voxels = ndt.getVoxelsAsNumpy()  # indices, centers, means, normals, is_plane...

    # Saved as a layer of a .mm file:
    mm = mp2p_icp.MetricMap()
    mm.layers["ndt"] = ndt
    mm.save("ndt.mm")

``insertionOptions``, ``likelihoodOptions`` and ``renderOptions`` are the live objects.
``creationOptions`` define the structure of the map, so reading it returns a **copy**:
modify it and assign it back, which raises ``ValueError`` if it cannot be applied without
discarding the map contents (e.g. a new voxel size), as ``trySetCreationOptions()`` does.

.. code-block:: python

    ipc = mmm.IncrementalPointCloud()
    opts = ipc.creationOptions
    opts.index_type = mmm.PointIndexType.KDTree
    opts.remove_points_farther_than = 50.0
    ipc.creationOptions = opts

Queries
=======

- ``nn_single_search()``, ``nn_multiple_search()``, ``nn_radius_search()``: nearest
  neighbors, with the same results as for ``mrpt.maps.CPointsMap``. Queries may be
  ``mrpt.math.TPoint3D`` or ``[x, y, z]``.
- ``nn_single_search_batch()``: the closest point to each row of an ``(N,3)`` array, as a
  dict of NumPy arrays, without holding the GIL.
- ``nn_search_pt2pl()`` (``NDT``, ``TSDF``): the closest plane, as an
  ``mp2p_icp.PlanePatch`` and its distance.
- ``TSDF.queryField()`` / ``queryFieldBatch()``: interpolated signed distance and gradient.
- ``KeyframePointCloudMap`` answers neighbor queries on the keyframes selected around a pose
  by ``icp_get_prepared_as_global()``.

NumPy access
============

- ``getPointsAsNumpy()``: all points, ``(N,3)``. For ``TSDF``, its zero level set; for
  ``KeyframePointCloudMap``, decimated as set by its ``renderOptions``.
- ``getVoxelsAsNumpy()`` (``getGridsAsNumpy()`` for ``SparseTreesPointCloud``): one row per
  voxel, with its contents (point count, mean, normal, signed distance...).
- ``KeyframePointCloudMap.getKeyframePosesAsNumpy()``: keyframe IDs and ``(N,4,4)`` poses.

Maps can also be copied (``copy.deepcopy()``) and pickled.

.. note::
   ``IncrementalPointCloud`` point indices are storage slots: ``size()`` and the NumPy
   conversions inherited from ``CPointsMap`` include freed slots (NaN) of removed points.
   ``livePointCount()`` and ``liveCompactedCopy()`` give the live points only, and
   ``compact()`` drops the freed slots.
