.. _concept_simulation_real_time:

=============================================
Simulation vs. real time
=============================================

Goal
------

MOLA modules should be agnostic to whether sensory inputs come from live sensors
or offline datasets, including potential actions like pause/resume and changing
the playback rate.

This enables easy debugging of sensor front-end modules and SLAM back-ends, as
well as unit testing of complex SLAM algorithms on slow machines, e.g. in build farms.


Rules to follow
-----------------

- **Never** use the system clock for anything related to SLAM, determining key-frame selection, etc.

- **Only** rely on the timestamp associated to input observation objects. In
  particular, since any input sensory piece of data should be represented as a
  class derived from `mrpt::obs::CObservation`, use its field
  `mrpt::obs::CObservation::timestamp` as the actual time at which the data was
  grabbed by the robot.
