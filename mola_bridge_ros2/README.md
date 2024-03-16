# mola_input_ros2
RawDataSource acting as a bridge: ROS2 -> MOLA

Can be used to interface a real sensor using a ROS driver node, or a dataset
in rosbag format; at present, datasets must be replayed externally
using `ros2 bag play`.

Building this module requires ROS 2 to be installed, and its `setup.bash`
activation script being sourced **before** invoking CMake to configure and build MOLA.

See package docs for instructions and options to install ROS prerequisites.

Provided MOLA modules:
* `BridgeROS2`, type RawDataSourceBase.

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## Docs and examples
See this package page [in the documentation](https://docs.mola-slam.org/latest/modules.html).

## License
This package is released under the BSD 3-clause license.
