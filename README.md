# mola-input-ros1
RawDataSource acting as a bridge: ROS1 -> MOLA

Can be used to interface a real sensor using a ROS driver node, or a dataset
in rosbag format; at present, datasets must be replayed externally
using `rosbag play`.

Building this module requires ROS1 to be installed, and the its initialization
script being sourced **before** invoking CMake to configure MOLA.

See package docs for instructions and options to install ROS prerequisites.

Provided MOLA modules:
* `InputROS1`, type RawDataSourceBase.

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## Docs and examples
See this package page [in the documentation](https://docs.mola-slam.org/latest/modules.html).

## License
This package is released under the GNU GPL v3 license.
