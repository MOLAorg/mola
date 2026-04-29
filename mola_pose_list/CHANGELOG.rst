^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_pose_list
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.0 (2026-04-29)
------------------
* Merge pull request `#141 <https://github.com/MOLAorg/mola/issues/141>`_ from MOLAorg/feat/pose-list-multiple
  feat: pose lists now support multiple nearby poses (Useful for non-repetitive scanners)
* feat: pose lists now support multiple nearby poses (Useful for Livox scanners)
* Merge pull request `#138 <https://github.com/MOLAorg/mola/issues/138>`_ from MOLAorg/fix/kf-map
  fix: robustify edge cases from last API changes
* fix: robustify edge cases from last API changes
* Merge pull request `#137 <https://github.com/MOLAorg/mola/issues/137>`_ from MOLAorg/feat/metric-map-changes-for-lo-grav-align
  mola_metric_maps: per-KF pose plumbing for online gravity rebake
* fix(mola_pose_list): cap k at cloud size in SearchablePoseList::check
  nn_multiple_search resizes its output vectors to the requested k
  regardless of how many neighbours actually exist, leaving trailing
  entries uninitialized when the cloud has fewer than k points. The
  best-match selection in check() then iterated over those garbage
  distances and indices, producing a wrong nearest-pose result whenever
  the list held fewer than 20 entries.
  Cap k to the actual cloud size before calling nn_multiple_search.
  Adds a regression test (test_check_with_few_kfs) that previously failed
  with the stale code: 3 KFs along x, query at (4.9, 0, 0) used to snap
  to the (0, 0, 0) entry instead of the correct (5, 0, 0).
  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
* feat(mola_pose_list): id-keyed SearchablePoseList for online gravity rebake
  Add an optional KFID tag to SearchablePoseList entries plus an in-place
  pose updater. This lets the LIO online gravity-rebake update its
  distance-checkers when per-KF poses are corrected, without rebuilding
  the whole list.
  New API:
  - insert(pose, id) overload
  - setPoseById(id, new_pose): updates the stored pose and the kd-tree
  point in place; no-op for unknown ids or in from_last_only mode
  removeAllFartherThan now also maintains the id->index map for surviving
  entries.
  Exports MOLA_POSE_LIST_HAS_ID_KEYED_API so downstream packages in
  separate repos (mola_lidar_odometry) can guard usage with __has_include
  + this macro and stay buildable against older mola_pose_list checkouts.
  Adds unit tests for the new id-keyed API in test-searchable-pose-list
  (also fills in the previously-empty test fixture).
  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
* Contributors: Jose Luis Blanco-Claraco

2.7.0 (2026-04-22)
------------------

2.6.1 (2026-04-02)
------------------

2.6.0 (2026-03-12)
------------------
* Fix clang-tidy warnings
* Merge pull request `#107 <https://github.com/MOLAorg/mola/issues/107>`_ from MOLAorg/fix/viz-decay-clouds
  Fix/viz-decay-clouds
* fix clang-tidy warning: avoid std::endl
* Update coyright notes
* Contributors: Jose Luis Blanco-Claraco

2.5.0 (2026-02-14)
------------------

2.4.0 (2025-12-28)
------------------

2.3.0 (2025-12-15)
------------------

2.2.1 (2025-11-08)
------------------

2.2.0 (2025-10-28)
------------------

2.1.0 (2025-10-20)
------------------

2.0.0 (2025-10-13)
------------------
* Merge pull request `#93 <https://github.com/MOLAorg/mola/issues/93>`_ from MOLAorg/feature/better-lio
  Changes for new LIO
* clang-format
* Fix typos and clang-tidy hints
* Contributors: Jose Luis Blanco-Claraco

1.9.1 (2025-07-07)
------------------

1.9.0 (2025-06-06)
------------------

1.8.1 (2025-05-28)
------------------

1.8.0 (2025-05-25)
------------------
* Update copyright year
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-05-06)
------------------

1.6.4 (2025-04-23)
------------------
* modernize clang-format
* Contributors: Jose Luis Blanco-Claraco

1.6.3 (2025-03-15)
------------------

1.6.2 (2025-02-22)
------------------

1.6.1 (2025-02-13)
------------------

1.6.0 (2025-01-21)
------------------

1.5.1 (2024-12-29)
------------------

1.5.0 (2024-12-26)
------------------

1.4.1 (2024-12-20)
------------------

1.4.0 (2024-12-18)
------------------

1.3.0 (2024-12-11)
------------------

1.2.1 (2024-09-29)
------------------

1.2.0 (2024-09-16)
------------------

1.1.3 (2024-08-28)
------------------
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

1.1.2 (2024-08-26)
------------------

1.1.1 (2024-08-23)
------------------

1.1.0 (2024-08-18)
------------------
* Merge pull request `#62 <https://github.com/MOLAorg/mola/issues/62>`_ from MOLAorg/docs-fixes
  Docs fixes
* Fix ament_xmllint warnings in package.xml
* Contributors: Jose Luis Blanco-Claraco

1.0.8 (2024-07-29)
------------------
* ament_lint_cmake: clean warnings
* Contributors: Jose Luis Blanco-Claraco

1.0.7 (2024-07-24)
------------------
* Add sanity asserts
* Contributors: Jose Luis Blanco-Claraco

1.0.6 (2024-06-21)
------------------

1.0.5 (2024-05-28)
------------------

1.0.4 (2024-05-14)
------------------
* bump cmake_minimum_required to 3.5
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* Add new HashedSetSE3 data structure
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------

1.0.0 (2024-03-19)
------------------
* Refactor SearchablePoseList into its own package
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------