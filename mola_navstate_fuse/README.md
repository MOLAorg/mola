# mola_navstate_fuse
SE(3) pose and twist path data fusion estimator.

This repository provides:
* `NavStateFuse`: C++ class to integrate odometry, IMU, and pose/twist estimations.

Note: At present this package just extrapolates the latest pose/twist estimations, taking into account
random-walk uncertainty. TODO: Proper integration of IMU and odometry.

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## License
This package is released under the GNU GPL v3 license. Other options available upon request.
