# mola_metric_maps for Python

Metric map classes of [MOLA](https://github.com/MOLAorg/mola), the LiDAR
odometry and mapping framework: incremental point clouds, voxel maps, NDT,
TSDF and keyframe maps. They are the layers of the `.mm` metric maps built by
MOLA, and work together with [`mp2p-icp`](https://pypi.org/project/mp2p-icp/)
(metric maps, ICP) and [`mrpt3`](https://pypi.org/project/mrpt3/) (points,
poses, base map classes).

```bash
pip install mola-metric-maps
```

Load a map built with MOLA: its layers come out with their actual classes
(`mp2p_icp` imports `mola_metric_maps` by itself when a map needs it):

```python
import mp2p_icp
import mola_metric_maps as mmm

m = mp2p_icp.MetricMap.from_file("map.mm")
layer = m.layers["localmap"]             # a mmm.IncrementalPointCloud
print(layer.livePointCount(), layer.creationOptions)

fields = mp2p_icp.layer_to_numpy(layer)  # x, y, z, intensity...: NumPy arrays
print(fields["x"].shape, list(fields))
```

Build maps from NumPy points (in the robot frame, with the robot at
`[x, y, z, yaw, pitch, roll]`, a 4x4 matrix or an `mrpt.poses.CPose3D`), query
them, and save them as `.mm` layers:

```python
import numpy as np

points = np.random.default_rng(0).uniform(-10, 10, (5000, 3)).astype(np.float32)

ipc = mmm.IncrementalPointCloud()
ipc.insertPointCloud(points, robotPose=[0, 0, 0, 0, 0, 0])
nn = ipc.nn_single_search_batch(points[:10] + 0.01)  # dict: found, points, dist_sqr...
idx, nn_points, dist_sqr = ipc.nn_multiple_search([1.0, 2.0, 0.0], 5)

ndt = mmm.NDT(voxel_size=1.0)
ndt.insertPointCloud(points, robotPose=[0, 0, 1.5, 0, 0, 0])
voxels = ndt.getVoxelsAsNumpy()  # indices, means, normals, is_plane...

out = mp2p_icp.MetricMap()
out.layers["points"] = ipc
out.layers["ndt"] = ndt
out.save("out.mm")
```

Classes: `IncrementalPointCloud`, `HashedVoxelPointCloud`,
`SparseVoxelPointCloud`, `SparseTreesPointCloud`, `NDT`, `TSDF`,
`KeyframePointCloudMap` and `OccGrid`, each with its option structures
(`creationOptions`, `insertionOptions`...). This wheel uses the libraries of
the installed `mp2p-icp` and `mrpt3` wheels, which are dependencies.

Requirements: Linux (glibc 2.28 or newer), x86_64 or aarch64, Python 3.11 to
3.14.

See the [Python API guide](https://docs.mola-slam.org/latest/mola_metric_maps_python.html)
and the [MOLA documentation](https://docs.mola-slam.org/latest/).

## License

GPL-3.0. **Commercial products that use MOLA need a commercial license**,
unless the entire product is released under GPL-3.0: see the
[plans and pricing](https://docs.mola-slam.org/latest/pricing.html).

## Other ways to install mola_metric_maps

mola_metric_maps is also available as a ROS 2 package
(`ros-$ROS_DISTRO-mola-metric-maps`), which includes these Python bindings
and the command line tools. Avoid mixing this wheel with those Python
bindings in the same environment.
