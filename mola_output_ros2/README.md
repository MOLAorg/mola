# mola_output_ros2
Bridge: MOLA -> ROS2

This package can be used to: 
- Publish datasets from any of the MOLA dataset input modules to ROS 2.
- Expose the results of a MOLA SLAM/odometry system to the rest of a ROS 2 system.

Building this module requires ROS 2 to be installed, and its `setup.bash`
activation script being sourced **before** invoking CMake to configure and build MOLA.

See package docs for instructions and options to install ROS prerequisites.

Provided MOLA modules:
* `OutputROS2`

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## Docs and examples
See this package page [in the documentation](https://docs.mola-slam.org/latest/modules.html).

## License
This package is released under the BSD 3-clause license.
