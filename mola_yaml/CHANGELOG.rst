^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_yaml
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge remote-tracking branch 'origin/feat/map-frame-gauge-change' into feat/map-frame-gauge-change
* Merge branch 'develop' into feat/map-frame-gauge-change
* Contributors: Jose Luis Blanco-Claraco

3.1.1 (2026-08-10)
------------------

3.1.0 (2026-08-06)
------------------
* Merge pull request `#187 <https://github.com/MOLAorg/mola/issues/187>`_ from MOLAorg/feat/incremental-point-cloud-kdtree-bake
  Bake IncrementalPointCloud's k-d tree index (mm-ipc-bake-kdtree)
* changelog
* Merge pull request `#182 <https://github.com/MOLAorg/mola/issues/182>`_ from MOLAorg/fix/yaml-define-outer-wins
  fix(mola_yaml): outer $define wins over a more deeply imported file's own $define
* style: apply clang-format-14 to the outer-wins $define fix
* fix(mola_yaml): outer $define wins over a more deeply imported file's own $define
  Nested $define blocks for the same variable name did not compose the way
  $import sibling overrides do: the more deeply imported file's own $define
  silently won, discarding an outer file's override for that same name. This
  made it impossible for a launcher to retune a hook that a reusable imported
  fragment also $defines its own default for, short of restating the whole
  target key as a literal sibling value.
  consumeDefineBlock() now skips a $define entry whose name is already present
  in the inherited scope, so the scope closest to the document root (or the
  caller's initial variables) has final say, mirroring "environment > $define >
  inline default" one level up. Updated the existing nested-$define test to the
  new (opposite) contract and added a dedicated multi-level $import regression
  test for the exact reusable-fragment shape that surfaced this.
* Merge pull request `#181 <https://github.com/MOLAorg/mola/issues/181>`_ from MOLAorg/feat/yaml-define-directive
  mola_yaml: new `$define` directive to set variables for a subtree
* docs: clarify outer scope only is used to expand vars
* mola_yaml: new `$define` directive to set variables for a subtree
  Shared pipeline files already expose their tunable settings as
  `${VAR|default}` hooks, but until now a launcher had no way to drive them
  from the YAML itself: the value had to come from the real environment.
  Working around that meant duplicating whole blocks after an `$import` just
  to change one nested field, which is especially painful for values inside a
  YAML sequence, since `$import`'s deep-merge replaces sequences wholesale.
  `$define` is a map key holding `NAME: VALUE` pairs that are bound as
  `${NAME}` variables for the subtree of the map it appears in, including the
  files pulled in by a sibling `$import` / `$include{}`:
  $define:
  MOLA_DESKEW_METHOD: "MotionCompensationMethod::IMU"
  $import: lidar3d-default.yaml
  The bindings go into `YAMLParseOptions::variables`, so the resolution order
  in parseVars() is unchanged and the effective priority is
  `environment > $define > inline |default`: a variable exported on the
  command line still overrides the file. This is deliberately not a setenv:
  the scope is the YAML subtree, so nothing leaks into the process
  environment, into `$()` sub-processes, or into a sibling module's import.
  Since the variable pass runs later over the whole document with the outer
  options, a subtree carrying a `$define` is expanded eagerly so the
  definitions reach the plain sibling keys too, not only the imported files.
  Includes unit tests and user documentation.
* Contributors: Jose Luis Blanco-Claraco

* feat: new `$define` YAML directive to bind variables for a subtree
  (including imported files), letting launchers tune shared pipeline
  settings without duplicating blocks.
* fix(mola_yaml): an outer file's `$define` now correctly wins over a more
  deeply imported file's own `$define`.
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2026-07-17)
------------------
* docs: list all debug/trace env variables
* Merge pull request `#161 <https://github.com/MOLAorg/mola/issues/161>`_ from MOLAorg/feat/yaml-import-override
  feat(mola_yaml): $import map directive for base-file + overrides
* feat(mola_yaml): add $import map directive for base-file + overrides
  A map with an `$import` key (a path, or a sequence of paths) is replaced by
  the deep-merge of the imported file(s) with the map's remaining keys overlaid
  on top, so sibling entries OVERRIDE the imported base (nested maps merge
  deeply; scalars/sequences replace). This complements `$include{}` (which
  replaces a whole node) and lets near-identical launch files share a common
  params block while tweaking just a few values, instead of duplicating it.
  - Resolved in the same first pass as `$include{}`, reusing its path
  pre-processing, relative-path resolution and circular-reference detection.
  - Unit tests: single import, multi-file sequence (later file wins), sibling
  override including deep-nested, load_yaml_file path, and missing-file throw.
  - Docs: new section + processing-order table row in the SLAM config format.
* Contributors: Jose Luis Blanco-Claraco

2.9.0 (2026-05-11)
------------------
* Merge pull request `#143 <https://github.com/MOLAorg/mola/issues/143>`_ from MOLAorg/bump-cmake-version
  bump min req cmake version to 3.22
* bump min req cmake version to 3.22
* Contributors: Jose Luis Blanco-Claraco

2.8.0 (2026-04-29)
------------------

2.7.0 (2026-04-22)
------------------

2.6.1 (2026-04-02)
------------------
* Merge pull request `#114 <https://github.com/MOLAorg/mola/issues/114>`_ from MOLAorg/feat/refactor-yaml-parser
  Refactor mola_yaml parser (faster, less memory usage, non recursive)
* Refactor mola_yaml parser (faster, less memory usage, non recursive)
* Contributors: Jose Luis Blanco-Claraco

2.6.0 (2026-03-12)
------------------
* Merge pull request `#107 <https://github.com/MOLAorg/mola/issues/107>`_ from MOLAorg/fix/viz-decay-clouds
  Fix/viz-decay-clouds
* fix clang-tidy warning: avoid std::endl
* Update coyright notes
* Contributors: Jose Luis Blanco-Claraco

2.5.0 (2026-02-14)
------------------
* Merge pull request `#100 <https://github.com/MOLAorg/mola/issues/100>`_ from MOLAorg/fix/remove-mrpt-deprecated-maps
  Remove use of mrpt deprecated maps
* Avoid use of deprecated mrpt::maps classes
* Contributors: Jose Luis Blanco-Claraco

2.4.0 (2025-12-28)
------------------

2.3.0 (2025-12-15)
------------------

2.2.1 (2025-11-08)
------------------
* Fix embedded FindFilesystem.cmake to avoid cmake warnings
* Contributors: Jose Luis Blanco-Claraco

2.2.0 (2025-10-28)
------------------

2.1.0 (2025-10-20)
------------------

2.0.0 (2025-10-13)
------------------
* Modernize copyright notice
* Use ament linters
* Contributors: Jose Luis Blanco-Claraco

1.9.1 (2025-07-07)
------------------

1.9.0 (2025-06-06)
------------------

1.8.1 (2025-05-28)
------------------

1.8.0 (2025-05-25)
------------------
* Update license tag to "BSD-3-Clause"
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
* FIX: parser bug; it should not try to parse commented-out env variables
* Contributors: Jose Luis Blanco-Claraco

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
* Parser now supports replacing custom variables
* Contributors: Jose Luis Blanco-Claraco

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
* Update clang-format style; add reformat bash script
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
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------

1.0.0 (2024-03-19)
------------------
* copyright update
* reorganize as monorepo
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
* Fix package name in docs
* Contributors: Jose Luis Blanco-Claraco

0.2.1 (2023-09-02)
------------------

0.2.0 (2023-08-24)
------------------
* First public release as ROS 2 package.
