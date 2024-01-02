# `mola_metric_maps`
Advanced metric map classes, using the generic `mrpt::maps::CMetricMap` interface,
for use in other MOLA odometry and SLAM modules.

## Contents
This repository provides a pure C++ library `mola_metric_maps` that extends
`mrpt-maps` with additional metric map classes:

- `mola::OccGrid`: Extends MRPT's occupancy grid with super-resolution likelihood field.
- `mola::SparseVoxelPointCloud`: a pointcloud stored in two dual hash'ed voxel maps,
  one for decimation purposes only, and another for nearest-neighbor search.

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## License
Copyright (C) 2018-2024 Jose Luis Blanco <jlblanco@ual.es>, University of Almeria

This package is released under the GNU GPL v3 license as open source for research
and evaluation purposes only. Commercial licenses available upon request, for this
package alone or in combination with the complete SLAM system.
