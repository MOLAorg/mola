"""
mola_metric_maps: Python API of the mola_metric_maps C++ library, part of MOLA.

Metric map classes, usable as layers of ``mp2p_icp.MetricMap`` (``.mm``
files) or on their own:

- ``IncrementalPointCloud``: sliding-window local map for LiDAR odometry, an
  ``mrpt.maps.CGenericPointsMap`` with an incremental spatial index.
- ``HashedVoxelPointCloud``, ``SparseVoxelPointCloud``,
  ``SparseTreesPointCloud``: voxelized point clouds.
- ``NDT``: Normal Distributions Transform map.
- ``TSDF``: truncated signed distance field.
- ``KeyframePointCloudMap``: point clouds anchored to keyframe poses.
- ``OccGrid``: 2D occupancy grid (experimental).

Importing this package loads the C++ library, which registers its classes, so
``.mm`` files with layers of these classes can be loaded without
``mp2p_icp.load_plugins()``, and their layers come out with their actual
Python type.

Points and poses are MRPT objects (``mrpt.math``, ``mrpt.poses``) or NumPy
arrays, as in the ``mrpt`` and ``mp2p_icp`` Python packages.
"""

# The MRPT and mp2p_icp Python modules register the base classes and other
# types used by the extension:
import mrpt.rtti  # noqa: F401
import mrpt.serialization  # noqa: F401
import mrpt.config  # noqa: F401
import mrpt.math  # noqa: F401
import mrpt.img  # noqa: F401
import mrpt.poses  # noqa: F401
import mrpt.viz  # noqa: F401
import mrpt.obs  # noqa: F401
import mrpt.maps  # noqa: F401
import mp2p_icp  # noqa: F401

from ._core import (
    HAS_INCREMENTAL_KDTREE,
    HAS_INCREMENTAL_KDTREE_BAKE,
    HashedVoxelPointCloud,
    IncrementalPointCloud,
    KeyframePointCloudMap,
    NDT,
    OccGrid,
    PointIndexType,
    SparseTreesPointCloud,
    SparseVoxelPointCloud,
    TSDF,
    ViewDirectionFilter,
    __version__,
)

__all__ = [
    "HAS_INCREMENTAL_KDTREE",
    "HAS_INCREMENTAL_KDTREE_BAKE",
    "HashedVoxelPointCloud",
    "IncrementalPointCloud",
    "KeyframePointCloudMap",
    "NDT",
    "OccGrid",
    "PointIndexType",
    "SparseTreesPointCloud",
    "SparseVoxelPointCloud",
    "TSDF",
    "ViewDirectionFilter",
    "__version__",
]
