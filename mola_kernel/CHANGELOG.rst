^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_kernel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2024-03-19)
------------------
* add methods to query for subscribers
* New interfaces
* Refactor initialize()
* mola_kernel: new UI interface for datasets
* New option to shutdown automatically mola-cli after dataset ends
* viz API: add enqueue_custom_nanogui_code()
* mola_viz: show console messages
* Correct usage of mola:: namespace in cmake targets
* copyright update
* mola_viz: support visualizing velodyne observations
* Add look_at() viz interface
* Fewer mutex locking()
* dont force by default load() lazy-load observations
* FrontEndBase: attach to VizInterface too
* Fix loss of yaml key/values when using import-from-file feature
* kitti eval cli moves to its own package
* port to mrpt::lockHelper()
* reorganize as monorepo
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
* Correct references to the license.
* viz interface: new service update_3d_object()
* Fix const-correctness of observations
* FIX missing dependency on mrpt::gui for public header
* Contributors: Jose Luis Blanco-Claraco

0.2.1 (2023-09-02)
------------------

* Add virtual interface for dataset groundtruth
* Update copyright date
* Update to new colcon ROS2 build system
* Contributors: Jose Luis Blanco-Claraco
