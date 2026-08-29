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

The CLI removes the pacing problem. Two further sources of run-to-run
variation remain, and both matter if you intend to compare numbers.

**Pin the threads.** ``tbb::parallel_reduce`` sums in whatever order the
threads finish, and floating-point addition is not associative, so the
schedule changes the result:

.. code-block:: bash

   taskset -c 0 mola-lo-cli-mulran KAIST01

Confirm it worked rather than assuming it: run twice and compare the output
trajectories with ``md5sum``. If they do not match, nothing downstream of
them is comparable either.

**Turn off asynchronous backend serving** if you are using the smoother state
estimator, whose async path is non-deterministic:

.. code-block:: bash

   MOLA_ASYNC_BACKEND=false ...

``mola-lidar-odometry-cli`` already defaults this to ``false``, since it is a
batch tool with no real-time deadline; it only needs stating for other
offline entry points. It was measured to be accuracy-neutral across 49
sequences before being adopted, so it buys reproducibility, not accuracy.

.. note::
   The smoother also needs its plugin loaded explicitly, with
   ``-l libmola_state_estimation_smoother.so``. The CLI does not load it by
   default, and without it the class factory fails with ``unknown class
   name``. See :ref:`troubleshooting`.


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
