.. _gui_vs_cli:

===========================================
GUI or offline CLI: which one, and why
===========================================

MOLA-LO ships two ways to process a recorded dataset, and they are not
interchangeable. Picking the wrong one does not produce an error; it produces
a slightly different trajectory, which is worse.

.. list-table::
   :widths: 22 39 39
   :header-rows: 1

   * -
     - ``mola-lo-gui-*``
     - ``mola-lo-cli-*``
   * - Runs through
     - ``mola-cli`` with the 3D GUI
     - ``mola-lidar-odometry-cli``, no window
   * - Pacing
     - Real time, as if the sensor were live
     - As fast as the machine allows
   * - Drops scans under load
     - **Yes**
     - No
   * - Reproducible run to run
     - **No**
     - Yes, with the precautions below
   * - Use it for
     - Looking at your data, sanity checks, demos
     - Benchmarks, published numbers, regression runs

Use the GUI to *see* what is happening. Use the CLI to *measure* anything.


Why the GUI is not reproducible
--------------------------------

The GUI paces playback against the wall clock, exactly as a live sensor
would. When a scan takes longer to process than the sensor period, the
pipeline keeps the freshest scan and drops the stale one, because that is the
correct behavior for a robot that has to keep up with reality.

For a benchmark it is the wrong behavior twice over. Scans go missing, and
the motion prior for the scans that remain is extrapolated across the
queueing delay rather than across one sensor period. Both effects depend on
what else your machine was doing at the time.

This is not a small correction. On one sequence, the difference between
real-time-paced and batch processing moved absolute pose error by several
times.


Making a CLI run bit-identical
-------------------------------

The CLI removes the pacing problem. ``mola-lidar-odometry-cli`` also turns
off, by default, the real-time behaviors that would make an offline run depend
on timing. An explicit setting in your environment still takes precedence:

- ``MOLA_ASYNC_BACKEND=false``: the smoother state estimator serves queries
  synchronously instead of from its non-deterministic asynchronous path. This
  was measured to be accuracy-neutral across 49 sequences, so it buys
  reproducibility, not accuracy.
- ``MOLA_DROP_STALE_SCANS=false``: every scan is processed, none is dropped.
- ``MOLA_INCREMENTAL_MAP_ASYNC_REBUILD=false``: when the local map is
  ``mola::IncrementalPointCloud``, its k-d tree is rebalanced synchronously.
  With it on, the rebuilds run on a background thread that nearest-neighbor
  queries never wait for, so the tree a query sees depends on the scheduler,
  and ties between equidistant neighbors can resolve differently from one run
  to the next. The pipelines default it to ``true``, the right choice in real
  time. The default map class, ``mola::KeyframePointCloudMap``, is unaffected,
  but some dataset wrappers switch to the incremental map
  (``mola-lo-cli-kitti`` does).

State these yourself for other offline entry points. The last one is also
needed with mola_lidar_odometry 3.3.0 and older, whose CLI does not default it:

.. code-block:: bash

   MOLA_INCREMENTAL_MAP_ASYNC_REBUILD=false mola-lo-cli-kitti 00

**Thread count does not matter.** The parallel sums in the ICP solver use a
fixed partition, and pairings are sorted into a fixed order before they reach
it. The result is therefore bit-identical for any number of threads and any
scheduling, with no need to pin the process. This holds from mp2p_icp 3.0.0
and MOLA 3.2.0 onward. With older releases, pin the process to one core
(``taskset -c 0 ...``).

Confirm it rather than assuming it: run twice and compare the output
trajectories with ``md5sum``. If they do not match, nothing downstream of
them is comparable either.

.. note::
   Since mola_lidar_odometry 3.2.0, ``mola-lidar-odometry-cli`` links the
   smoother state estimator in at build time whenever
   ``mola_state_estimation_smoother`` is installed, so
   ``--state-estimator mola::state_estimation_smoother::StateEstimationSmoother``
   works without loading a plugin. On older releases, or on a build that did
   not find that package, add ``-l libmola_state_estimation_smoother.so``, or
   the class factory fails with ``unknown class name``. See
   :ref:`troubleshooting`.


Before you believe a difference
--------------------------------

Two habits, both of which have caught real mistakes:

- **Characterize the noise floor first.** Run the same configuration twice
  and see how far apart the results are. A difference between two configs
  that is smaller than that gap is not a result.
- **Check the estimate covers the whole ground-truth timespan.** A run that
  quietly stopped early can look excellent on any alignment-based metric.

And read ``no_motion_model`` from the run summary next to whatever error
number you computed. A non-zero value means part of the trajectory was
registered with no motion prior at all, whatever the error says. The
proportion of dropped scans is not in that line; it is published separately
as the ``dropped_ratio`` diagnostic, which is the one to watch when running
through the GUI.
