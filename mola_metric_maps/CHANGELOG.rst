^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_metric_maps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#195 <https://github.com/MOLAorg/mola/issues/195>`_ from MOLAorg/feat/cov2cov-ambiguity-gating
  mola_metric_maps: adopt mp2p_icp::MatchingDistanceProfile in nn_search_cov2cov
* Merge branch 'develop' into feat/cov2cov-ambiguity-gating
* mola_metric_maps: remove the ambiguity gate from nn_search_cov2cov()
  Companion to the removal in mp2p_icp`#89 <https://github.com/MOLAorg/mola/issues/89>`_: firstToSecondDistanceMin/
  firstToSecondMinRange were not yet justified by results. Drop the
  gate logic (and the k=2 / radius-inflated query it required) from
  IncrementalPointCloud and KeyframePointCloudMap's exact and
  approximate-cov paths, the flat compat stand-in, and the
  corresponding test coverage, keeping the range-adaptive matching
  distance.
* mola_metric_maps: build against mp2p_icp releases without MatchingDistanceProfile
  rosdep resolves mp2p_icp to the last released binary package in every CI job, so
  this tree has to compile against an mp2p_icp that predates
  MatchingDistanceProfile.
  Adds MatchingDistanceProfileCompat.h, which either aliases the real type or, when
  the header is absent, supplies a flat-only stand-in with the same small surface.
  The search implementations are written against that alias and so stay free of
  preprocessor branches; only the public overload, and the tests that exercise the
  ambiguity gate, are guarded by MP2P_ICP_HAS_MATCHING_DISTANCE_PROFILE.
  Both map classes now implement the flat-threshold overload (still the pure
  virtual upstream) as a forwarder into a shared private nn_search_cov2cov_impl(),
  so the two public entry points cannot drift apart.
* mola_metric_maps: adopt mp2p_icp::MatchingDistanceProfile in nn_search_cov2cov
  Follows the mp2p_icp interface change: nn_search_cov2cov() now receives a
  MatchingDistanceProfile instead of a flat float search distance. Implemented in
  IncrementalPointCloud and in both KeyframePointCloudMap paths (exact and
  approximate-cov).
  The flat, ungated default keeps a dedicated fast path in all three: no
  per-point range is computed and the KD-tree query stays k=1, so the previous
  behavior is reproduced exactly and at the same cost.
  When the ambiguity test is active for a query point, the search radius is
  inflated by the ratio, so any runner-up able to disqualify the winner is
  guaranteed to lie inside it, and the best two candidates are kept. In the
  approximate-cov path the runner-up may live in a different keyframe than the
  winner, so the best two are folded across all active keyframes rather than per
  keyframe.
  Range is measured in the query point's own untransformed (sensor) frame.
  Tests added for the ambiguity gate in both map classes.
* Merge pull request `#193 <https://github.com/MOLAorg/mola/issues/193>`_ from MOLAorg/fix/incremental-map-pairing-order
  Give IncrementalPointCloud::nn_search_cov2cov() a canonical pairing order
* Give IncrementalPointCloud::nn_search_cov2cov() a canonical pairing order
  The same defect fixed for KeyframePointCloudMap in bfe6cb2e, which did not
  cover this map class: the parallel path accumulates correspondences into a
  tbb::enumerable_thread_specific and merges it by iteration, whose order is
  unspecified. The permutation is not cosmetic, it reaches the solver, which
  sums the normal equations over the pairing list in order.
  This class needs a stronger fix than the keyframe map did, because here the
  *sequential* path was not canonical either. The live local points come from
  snapshotLiveIndices(), a depth-first walk of the k-d tree, so they arrive in
  tree-topology order rather than in slot order. Tombstones and rebuilds change
  that shape, so with async_rebuild enabled the pairing order varied between
  runs even single-threaded. The sort is therefore applied to the shared
  intermediate match list, before the pairings are assembled, which pins both
  paths to the same order and makes the result independent of the tree shape a
  rebuild happened to leave behind. Two sibling call sites in this file already
  sort that snapshot for the same reason.
  Sorting the intermediate list rather than the output pairings also keeps the
  comparison on an 8-byte key instead of a full pairing, and gives the assembly
  pass ascending access into the coordinate and covariance buffers.
  Each local point yields at most one match, so its slot is a unique key and the
  resulting order is total.
  test_pairing_order_is_canonical asserts ascending local_idx and identical order
  across repeated calls, over a cloud with tombstones so tree order really does
  diverge from slot order, and large enough that TBB splits the range across
  workers. It fails without this change with "Pairings are not in canonical
  (ascending local_idx) order".
* silent a gcc warning (safe)
* Merge remote-tracking branch 'origin/feat/map-frame-gauge-change' into feat/map-frame-gauge-change
* Merge branch 'develop' into feat/map-frame-gauge-change
* Contributors: Jose Luis Blanco-Claraco

3.1.1 (2026-08-10)
------------------
* mola_metric_maps: fix calloc arg order and nodiscard warnings on newer GCC.
* Give nn_search_cov2cov() a canonical pairing order. The parallel path merged
  per-thread correspondences in unspecified order, making ICP results
  non-deterministic run to run; now sorted by local_idx to match the
  sequential path.
* IncrementalPointCloud: rebuild the k-d tree on a global SE(3) re-map.
  changeCoordinatesReference() rewrote coordinates in place without resizing,
  so the k-d tree kept stale split planes and nearest-neighbor queries
  silently returned wrong results. Fixes `#186 <https://github.com/MOLAorg/mola/issues/186>`_.
* mola_metric_maps: depend on nanoflann_vendor instead of nanoflann, since the
  rosdep key otherwise resolves to the distro's older libnanoflann-dev.
* fix(mola_metric_maps): don't require nanoflann>=1.5.1 for KeyframePointCloudMap
  covariances. Ubuntu jammy's older nanoflann made every use of
  mola::KeyframePointCloudMap throw on Humble; now falls back to a plain kNN
  truncated at the same radius, verified numerically equivalent.
* Contributors: Jose Luis Blanco-Claraco

3.1.0 (2026-08-06)
------------------
* Merge pull request `#187 <https://github.com/MOLAorg/mola/issues/187>`_ from MOLAorg/feat/incremental-point-cloud-kdtree-bake
  Bake IncrementalPointCloud's k-d tree index (mm-ipc-bake-kdtree)
* address review comments
* Add k-d tree baking for IncrementalPointCloud + mm-ipc-bake-kdtree tool
  Serializes the incremental k-d tree index alongside an IncrementalPointCloud
  layer's points (TCreationOptions::serialize_kdtree), so it does not have to
  be rebuilt (an O(N log M) bulk build) on every load. Unlike
  KeyframePointCloudMap's baked static trees, nanoflann's incremental index had
  no save/load support at all; this depends on saveIndex()/loadIndex() added
  upstream (nanoflann >= 1.11.0, see the companion nanoflann PR), gated behind
  MOLA_METRIC_MAPS_HAS_INCREMENTAL_KDTREE_BAKE so older builds keep working
  with the option as a documented no-op.
  Serialization always writes/reads the compacted (tombstone-free) point
  order, so baking builds a throwaway index over that exact order rather than
  reusing the live index (whose slots may not match after tombstones/slot
  recycling).
  Adds the mm-ipc-bake-kdtree CLI tool (analogous to mm-kf-bake-kdtrees) and a
  shared mm_cli_utils.h generic layer-iteration helper for it.
  Unit tests cover: bake/load round-trip through memory and through a real
  temporary file, k-d tree parameters differing between bake and load time,
  clearing and re-inserting into a loaded (baked) map, further
  insertions/trims on a loaded map, and serialize_kdtree=false remaining a
  no-op -- all independent of whether this build's nanoflann actually supports
  baking.
* changelog
* chore: document and fix some multithreading issues
* Merge pull request `#185 <https://github.com/MOLAorg/mola/issues/185>`_ from MOLAorg/chore/remove-keyframe-map-capable
  Remove the KeyframeMapCapable interface
* docs: drop the KeyframeMapCapable references left behind
* chore: remove the KeyframeMapCapable interface
  This mixin was introduced to expose per-KF pose plumbing to
  mola_lidar_odometry's trajectory-rebake experiment, which corrected
  accumulated tilt by re-integrating the keyframe chain. That experiment is
  being removed: it was never wired in, and rotating map keyframes without
  transforming the trajectory consistently leaks vertical position.
  The interface had exactly one implementation and no callers, so it is
  removed along with the two methods that existed only for the rebake path,
  `oldestActiveKeyframeID()` and `applyPivotTransform()`.
  `keyframePoses()` is kept, since the regroup tests already use it as
  ordinary map API, and the duplicate `cloneKFPoses()` (whose only difference
  was not being the virtual one) is folded into it.
* Merge pull request `#184 <https://github.com/MOLAorg/mola/issues/184>`_ from MOLAorg/feat/incremental-pointcloud-map
  feat(metric_maps): add mola::IncrementalPointCloud (incremental k-d tree local map)
* docs: correct the trySetCreationOptions contract
  The k-d tree parameters used to require an empty map, with
  trySetCreationOptions() returning false rather than discarding points.
  It now compacts and rebuilds instead, so it always succeeds and keeps the
  map contents; only the previously returned point indices are invalidated.
  The TCreationOptions docs still described the old behaviour.
* style: apply clang-format-14
* feat(metric_maps): degrade gracefully on distros with an old nanoflann
  The incremental k-d tree index needs nanoflann >= 1.10.0. Previously the
  whole class was compiled out below that, so a YAML naming
  mola::IncrementalPointCloud failed with MRPT's generic "no such registered
  CMetricMap class", which reads like a typo or a missing plugin rather than
  a missing feature.
  Now the class is always declared, compiled and registered; only the k-d
  tree factory is conditional. Without a suitable nanoflann,
  IncrementalKDTree_stub.cpp provides a factory that throws an explanatory
  std::runtime_error naming the version actually found and pointing at
  mola::KeyframePointCloudMap as the alternative. CMake emits a warning at
  configure time instead of a silent status message.
  MOLA_METRIC_MAPS_HAS_INCREMENTAL_POINT_CLOUD keeps advertising whether the
  feature is functional, and the unit test is still only built when it is.
  Verified by configuring against the nanoflann 1.9.0 shipped by ROS Humble:
  builds clean, class registers, and construction throws the expected message.
* feat(metric_maps): add mola::IncrementalPointCloud
  A single-global-frame, sliding-window local map for LiDAR (inertial)
  odometry, backed by one incremental self-balancing nanoflann k-d tree
  instead of a static tree rebuilt on every scan.
  It derives from mrpt::maps::CGenericPointsMap, so points arrive through
  the usual insertObservation()/insertAnotherMap() entry points with all
  per-point channels preserved, and the index picks up appended points
  lazily. Points far from the robot are evicted on insertion
  (creationOptions.remove_points_farther_than, a cube half-side), and the
  slots the index reclaims are recycled so storage stays bounded under
  churn. GICP matching is supported via mp2p_icp::NearestPointWithCovCapable
  with lazily computed, cached, plane-regularized per-point covariances.
  This is an odometry local map only: a single global tree cannot absorb a
  loop-closure re-map, so KeyframePointCloudMap remains the choice there.
  Notes:
  - Requires nanoflann >= 1.10.0 (the release that introduced the
  incremental index). The class is simply not built otherwise, gated by
  the CMake-set MOLA_METRIC_MAPS_HAS_INCREMENTAL_POINT_CLOUD.
  - nanoflann is included by src/IncrementalKDTree.cpp alone, by absolute
  path and with its namespace renamed, and is never exposed through a
  public header. MRPT bundles its own, usually older, copy of the same
  header and the two differ in non-template entities that share mangled
  names, so letting both reach one program is an ODR violation. Include
  path ordering cannot select ours either: MRPT exports its copy as an
  -isystem directory, and a directory listed both as -I and -isystem is
  deduplicated by GCC in favour of the system entry.
  - Removal is lazy, so the inherited size() counts live plus not-yet-reused
  slots; livePointCount() and compact() expose the live set. Reclaimed
  slots are blanked to NaN so that generic CPointsMap walkers (notably
  insertAnotherMap(), used to copy layers out for visualization) cannot
  resurrect evicted geometry.
  - With async_rebuild the balancing rebuilds run on a background thread.
  Its worker reads the coordinate buffers, so reserve()/resize()/setSize()
  are overridden to wait for it before those can move, and insertion keeps
  spare capacity for one batch so a raw insertPointFast() loop cannot
  reallocate either.
  Tested against a brute-force nearest-neighbour oracle (both index
  variants), for bounded memory under churn, cov2cov pairings,
  serialization round-trip and copy.
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

* chore: documented and fixed some multithreading issues; removed the unused
  KeyframeMapCapable interface and its stale docs.
* feat(metric_maps): added mola::IncrementalPointCloud, a nanoflann-based
  incremental k-d tree local map for LiDAR odometry, with graceful
  degradation on distros with an old nanoflann. Two different thresholds
  apply: the incremental k-d tree itself requires nanoflann >= 1.10.0 (with
  older versions the map falls back to a stub and is unavailable at runtime),
  while baking (`creationOptions.serialize_kdtree`, `mm-ipc-bake-kdtree`) and
  loading baked k-d tree indexes additionally require nanoflann >= 1.11.0.
  Thus, with nanoflann 1.10.x only the incremental functionality works, and
  baking is a documented no-op.
* docs: corrected trySetCreationOptions docs to match its new
  compact-and-rebuild (always-succeeds) behavior.
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2026-07-17)
------------------
* fix: dont crash on empty maps
* feat: --unify-all flag for mm-kf-regroup
* fix(mola_metric_maps): don't restore stale icp_search_kfs cache from serialized maps
  icp_search_kfs was serialized purely for post-mortem debugging, but
  serializeFrom() restored it straight into the live cache. Since
  icp_search_submap (the actual prepared search structure it gates) is
  never serialized, this let icp_get_prepared_as_global()'s already-up-to-date
  fast path skip rebuilding the submap whenever the freshly computed
  active-keyframe selection happened to coincide with the stale, reloaded
  set. The next nn_search_cov2cov() call would then fail its
  'icp_get_prepared_as_global() first' assertion. Reproduced reliably when
  chaining multi-session/multi-robot maps (loading a previously saved map
  as the starting point for a new robot/session).
* fix: sort by indices to avoid gcc warnings on ABI changes
* chore: tune default island param
* debug: add env-gated point-density match-stats trace
  Adds MOLA_KEYFRAME_MAP_DEBUG_MATCH_STATS (off by default), counting per-call
  accepted / no-candidate-in-range / rejected-by-view-filter totals in
  nn_search_cov2cov_approximate(). Used to diagnose a localization stall on a
  real dataset: distinguishes a genuine point-density gap in the map (points
  with no nearby candidate) from a view-direction-filter rejection or a
  keyframe-selection problem, which look identical from the ICP goodness
  trace alone.
* fix(mola_metric_maps): bound island-merge distance; never starve the diverse KF slot
  Address CodeRabbit review on `#174 <https://github.com/MOLAorg/mola/issues/174>`_ plus a second, distinct tracking-loss
  mode found during full-bag testing:
  - regroupKeyframes(): cap island absorption distance at
  extent_factor * (seed radii sum) so a tiny cluster is not glued to a
  spatially disjoint neighbor merely because it's the "nearest" one
  left; islands beyond that bound stay standalone.
  - icp_get_prepared_as_global(): the diverse-keyframe slot's distance
  limit (3x the nearest primary candidate) could reject every other
  candidate when a map has only a handful of large super-keyframes,
  leaving that slot empty and ICP running against a single keyframe.
  At the edge of that keyframe's own coverage this can permanently
  stall tracking (quality stuck just under threshold, no fallback).
  Now falls back to the best-diversity candidate regardless of
  distance rather than leaving the slot unfilled.
* fix(mola_metric_maps): density-aware nearest-keyframe selection + island merging in regroup
  A sparse, under-merged keyframe (e.g. a leftover cluster from
  regroupKeyframes()) could outrank a much better-populated keyframe in
  icp_get_prepared_as_global()'s proximity search purely because its pose
  happened to be closer, starving ICP of real geometry and causing
  localization to get stuck with zero correspondences.
  - KeyframePointCloudMap: add a distance penalty for keyframes below
  density_penalty_min_points, so sparse candidates no longer win over
  well-populated ones just by pose proximity.
  - regroupKeyframes(): absorb clusters below island_merge_fraction of the
  largest cluster's point count into their nearest neighbor instead of
  emitting them as standalone, near-empty super-keyframes.
  - mm-kf-regroup: expose --island-merge-fraction.
  - Adds env-gated debug traces (MOLA_KEYFRAME_MAP_DEBUG_ACTIVE_KFS,
  MOLA_KEYFRAME_MAP_DEBUG_DUMP_KFS_ON_LOAD) used to diagnose this.
* Merge pull request `#173 <https://github.com/MOLAorg/mola/issues/173>`_ from MOLAorg/perf/mm-kf-regroup-tbb-parallel-superkf
  mm-kf-regroup: parallelize super-keyframe cloud building with TBB
* mm-kf-regroup: parallelize super-keyframe cloud building with TBB
  Building each super-keyframe's merged/decimated point cloud in
  regroupKeyframes() was the most expensive step but ran single-threaded.
  Clusters are independent, so parallelize with tbb::parallel_for
  (gated by MOLA_METRIC_MAPS_USE_TBB, already a mola_metric_maps
  dependency), falling back to a serial loop when TBB is unavailable.
* better defaults for regroup KF algorithm
* feat: mm-kf-bake-kdtrees sets approximate cov by default
* Merge pull request `#171 <https://github.com/MOLAorg/mola/issues/171>`_ from MOLAorg/feat/keyframe-map-persist-covariances-local-tree
  feat(mola_metric_maps): persist per-KF covariances + query baked local KD-tree in approximate cov2cov
* fix(mola_metric_maps): handle v3 in serializeFrom + clang-format
  - Add `case 3:` to KeyframePointCloudMap::serializeFrom(): serializeGetVersion()
  returns 3, so v3 maps (with baked covariances) were hitting
  MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION on load and never reaching the new
  covariance-read block. (CodeRabbit)
  - Apply clang-format-14 to the changed files (CI clang-format-check).
* feat(mola_metric_maps): persist per-KF covariances and query baked local KD-tree in approximate cov2cov
  Two changes that make localization-only startup with a baked .mm actually
  fast, eliminating the occasional multi-second stalls seen when a fresh
  keyframe first enters the active set:
  A) approximate_cov now queries each active keyframe's own LOCAL-frame cloud
  and KD-tree directly (kf.pointcloud(), the one baked by mm-kf-bake-kdtrees).
  The query point is transformed into each KF's local frame before the lookup
  and the match composed back to global for the pairing. A KD-tree's structure
  is invariant under the rigid KF pose, so this avoids ever materializing a
  per-KF global-frame cloud or rebuilding a KD-tree on it. Previously the baked
  (local) index never touched this hot path, which used a separate global-frame
  tree rebuilt from scratch on first activation. Also robust to online KF pose
  nudges (LIO gravity-tilt correction), which no longer invalidate a global cloud.
  B) New serialize_covariances creationOption: bakes each keyframe's per-point
  local covariances into the .mm (map serialization bumped to v3), so the
  plane-regularized K-NN + SVD pass is not repeated on load. This is the single
  most expensive part of warming a keyframe and needs no special MRPT API, so it
  works on any build (unlike serialize_kdtrees). The cheap per-pose global
  rotation is still done at runtime.
  mm-kf-bake-kdtrees now bakes both KD-trees and covariances by default
  (--no-kdtrees / --no-covariances to select, --disable to strip both).
  Adds a round-trip test asserting a covariance-baked map yields byte-identical
  cov2cov pairings (points + cov_inv) to a runtime-computed one.
* Merge pull request `#169 <https://github.com/MOLAorg/mola/issues/169>`_ from MOLAorg/feat/keyframe-map-approximate-cov2cov
  feat(mola_metric_maps): approximate cov2cov mode for KeyframePointCloudMap
* fix(mola_metric_maps): tolerate concurrent KF eviction in approximate cov2cov
  nn_search_cov2cov_approximate() looked up each active KF id via
  keyframes\_.at(), which throws std::out_of_range if the KF was evicted
  by a concurrent insertObservation() (remove_frames_farther_than)
  between the earlier icp_get_prepared_as_global() snapshot and this
  call. The exact (merged-submap) path is immune since it deep-copies
  points at prepare time; the approximate path intentionally keeps live
  references into keyframes\_ instead, so it needs to check existence.
  Found by CodeRabbit review on `MOLAorg/mola#169 <https://github.com/MOLAorg/mola/issues/169>`_.
* feat(mola_metric_maps): approximate cov2cov mode for KeyframePointCloudMap
  Add TCreationOptions::approximate_cov: when enabled, nn_search_cov2cov()
  skips building the merged, multi-keyframe submap in
  icp_get_prepared_as_global() and instead queries each active keyframe's
  own already-cached KD-tree and per-point covariances directly (N
  per-KF KD-tree lookups instead of one on a merged cloud). This trades
  covariance exactness (estimated only from within-KF neighbors) for
  speed (no merge/KD-tree rebuild), and is opt-in (default false,
  unchanged exact behavior).
* fix: stable re-coloring
* Merge pull request `#168 <https://github.com/MOLAorg/mola/issues/168>`_ from MOLAorg/feat/kf-viz-color-by-kf
  feat(mola_metric_maps): debug env var to color each keyframe distinctly
* feat(mola_metric_maps): debug env var to color each keyframe distinctly
  Add MOLA_KEYFRAME_MAP_VIZ_COLOR_BY_KF: when set, KeyframePointCloudMap
  renders every keyframe with a single, distinct color (golden-ratio hue
  spacing) so regrouped super-keyframes / clusters are easy to tell apart
  visually while debugging.
  KeyFrame::getViz() gains an optional overrideColor argument; the override
  path paints all points uniformly and is not cached, so toggling the env
  var takes effect on the next render.
* Merge pull request `#167 <https://github.com/MOLAorg/mola/issues/167>`_ from MOLAorg/feat/keyframe-map-regroup-and-kdtree-persist
  Keyframe map regrouping + optional KD-tree persistence (localization-only speedups)
* address review: clearer error when --layer is not present in the map
* address review: shared CLI helper, unsupported-build warning, decimation test
  - Extract the duplicated plugin-load / map-load / KeyframePointCloudMap-layer
  iteration logic of the two CLI tools into a shared apps/kf_cli_utils.h.
  - mm-kf-bake-kdtrees now warns when built against an MRPT lacking the KD-tree
  save/load index API (baking would silently be a no-op otherwise).
  - Add a regroup test exercising merge_decimate_voxel > 0: the duplicated points
  collapse and the per-point view-direction fields survive decimation (verified
  via the cov-to-cov view filter).
* feat(mola_metric_maps): keyframe regrouping + optional kd-tree persistence
  Two offline optimizations of a KeyframePointCloudMap layer for fast
  localization-only operation:
  - regroupKeyframes() + mm-kf-regroup CLI: group many small keyframes into
  fewer, larger, deliberately-overlapping "super-keyframes" so the ICP active
  set stays size 1 (no per-scan keyframe switching). Graph-theoretic: keyframe
  adjacency graph with voxel Jaccard-min overlap edge weights, greedy
  overlapping set-cover clustering auto-sized from each keyframe bounding box,
  merged clouds voxel-decimated (view-direction fields carried).
  - serialize_kdtrees creationOption + mm-kf-bake-kdtrees CLI: cache each
  keyframe's per-cloud 3D KD-tree index inside the .mm so it is not rebuilt on
  every load. Bumps map serialization to v2 (self-describing per-KF flag byte +
  KD-tree blob). Requires the MRPT KD-tree save/load index API, feature-detected
  via MRPT_HAS_KDTREE_SAVE_LOAD_INDEX; when absent the option is a no-op on write
  and readers skip any stored blob, so .mm files stay interoperable across MRPT
  builds.
  Adds unit tests (regroup reduction/coverage/roundtrip, kd-tree serialization
  roundtrip) and updates agents.md.
* docs: clarify clouds are deskwed
* remove not used macros
* Add mm2ini/ini2mm CLI tools to export/import metric map layer options (`#158 <https://github.com/MOLAorg/mola/issues/158>`_)
  * Add mm2ini/ini2mm CLI tools to export/import metric map layer options
  mm2ini exports the CLoadableOptions (creation/insertion/likelihood/render
  options) of every layer in a .mm file to a human-editable .ini file;
  ini2mm applies such a file back, overwriting only the options sections
  present in it while leaving map data untouched. Supported layer classes
  are identified via dynamic_cast in OptionsIniIO.h, covering this
  library's own map types plus basic mrpt::maps classes (CPointsMap and
  derivatives, COccupancyGridMap2D).
  Also works around a few known dumpToTextStream()/loadFromConfigFile()
  inconsistencies in upstream MRPT (COccupancyGridMap2D's
  horizontalTolerance unit mismatch, rayTracing_decimation being write-only,
  and bare Y/N booleans) without requiring any MRPT changes.
  Adds test-options-ini-io.cpp, which builds a synthetic in-memory
  CSimpleMap and round-trips the options of all 7 supported classes.
  * Add MapOptionsCapable interface for generic, safe creation-options handling
  Introduces mola::MapOptionsCapable, a mixin interface implemented by all
  map classes in this library, with two methods:
  - optionsByName(): lists every CLoadableOptions group by name, so callers
  (e.g. OptionsIniIO/mm2ini/ini2mm) no longer need per-class knowledge of
  which option structs a map defines.
  - trySetCreationOptions(): applies new creation options (e.g. voxel/grid
  size) only if doing so doesn't require discarding the map's current
  contents; otherwise returns false and leaves the map untouched.
  NDT, HashedVoxelPointCloud, SparseVoxelPointCloud and SparseTreesPointCloud
  gain a real TCreationOptions (their voxel/grid size, previously only a
  constructor argument), kept in sync with the internal structure via
  setVoxelProperties()/setGridProperties(). KeyframePointCloudMap's existing
  TCreationOptions are pure runtime thresholds, so it always accepts changes.
  OptionsIniIO.cpp's five near-duplicate per-class dynamic_cast blocks
  collapse into one generic path for any MapOptionsCapable map, keeping a
  dynamic_cast fallback only for classes outside this library
  (mrpt::maps::CPointsMap, COccupancyGridMap2D). ini2mm now reports rejected
  creation-option changes explicitly instead of a generic warning.
  * Rename MapOptionsCapable to OptionsCapable
  * Fix critical data-wipe bug and stale-cache issue found by CodeRabbit
  - NDT/HashedVoxelPointCloud/SparseVoxelPointCloud/SparseTreesPointCloud's
  trySetCreationOptions() called setVoxelProperties()/setGridProperties()
  unconditionally, which clears the map -- so re-applying an unchanged
  voxel/grid size (e.g. a routine ini2mm run that doesn't touch that field)
  silently wiped all map contents. Now only called when the value actually
  changes.
  - KeyframePointCloudMap::trySetCreationOptions() updated the top-level
  creationOptions but left already-inserted KeyFrames with their own
  cached k_correspondences_for_cov/min_correspondences_for_cov/
  max_distance_for_cov (frozen at insertion time for lazy per-point
  covariance computation), so existing keyframes kept using stale values.
  Added KeyFrame::updateCovarianceParams() and propagate the new values to
  every existing keyframe, under state_mtx\_ like other mutators.
  - Added regression tests for both: re-applying an unchanged voxel_size on
  a non-empty map must be a no-op (not a wipe), and clarified the
  agents.md wording on which classes implement OptionsCapable.
* Merge pull request `#157 <https://github.com/MOLAorg/mola/issues/157>`_ from MOLAorg/fix/keyframemap-view-vector-rotation
  Fix view-direction vector frame mismatch in KeyframePointCloudMap
* Fix compat shim: real if constexpr discarding requires a template
  The previous compat shim used if constexpr inside a non-template member
  function (updatePointsGlobal). Per the standard, if constexpr only skips
  type-checking the untaken branch inside a template -- in ordinary code
  both branches are fully compiled. Against a real old mp2p_icp checkout
  (2.9.0, lacking rotateViewDirectionFields()), this made the discarded
  "call the real helper" branch resolve to our own ellipsis fallback and
  try to pass CPointsMap by value through it, which is ill-formed since
  CPointsMap's copy constructor is deleted -- a hard build failure caught
  by CI (x86_64 / humble stable job on PR `#157 <https://github.com/MOLAorg/mola/issues/157>`_).
  Fix: move the dispatch into a small template function
  (rotateViewDirectionFieldsOrFallback<Pts, Pose>), and factor the legacy
  rotation loop into its own function (rotateViewDirectionFieldsLegacy) so
  it isn't duplicated between the "header missing" and "header present but
  function missing" cases. With a genuine template parameter, if constexpr
  now actually discards the untaken branch without instantiating it.
  Verified: rebuilt and ran mola_metric_maps' full test suite (12/12 pass)
  against the real (new) mp2p_icp, and syntax-checked the file with
  -fsyntax-only against a hand-crafted header lacking
  rotateViewDirectionFields() to confirm the fallback path compiles too.
* Keep building against older mp2p_icp checkouts lacking rotateViewDirectionFields()
  Detect the helper's availability purely in C++ via a SFINAE trait (the
  containing header predates the function, so __has_include() alone can't
  tell old and new mp2p_icp_map apart) and fall back to the original
  open-coded rotation loop when it's absent. MRPT_TODO marks the fallback
  for removal once the minimum required mp2p_icp_map version provides the
  helper (`MOLAorg/mp2p_icp#70 <https://github.com/MOLAorg/mp2p_icp/issues/70>`_).
* Fix view-direction vector frame mismatch in KeyframePointCloudMap
  KeyFrame::updatePointsGlobal() rotated a keyframe's view_x/y/z vectors to
  the global frame assuming they were stored in the local KF frame, but
  mp2p_icp_filters::FilterMerge was leaving them in the global frame after
  inserting into a KeyframePointCloudMap, causing a double rotation that
  made the cov-to-cov view-angle pairing filter reject valid pairs as the
  keyframe heading diverged from the map's build-time reference.
  Replaces the open-coded rotation loop with the new shared
  mp2p_icp::rotateViewDirectionFields() helper (also used by the FilterMerge
  fix in `MOLAorg/mp2p_icp#70 <https://github.com/MOLAorg/mp2p_icp/issues/70>`_), documents the local-KF-frame contract in
  KeyframePointCloudMap.h, and adds a heading-sweep regression test.
* Merge pull request `#156 <https://github.com/MOLAorg/mola/issues/156>`_ from MOLAorg/feat/cov-viz
  feat: add debug viz of map covariances
* feat: add debug viz of map covariances
* Merge pull request `#155 <https://github.com/MOLAorg/mola/issues/155>`_ from MOLAorg/fix/more-robust-kf-map-covariances
  fix: more robust KF map convariances
* fix: more robust KF map convariances
* docs(mola_metric_maps): expand class-level Doxygen and complete README
  - HashedVoxelPointCloud: document SSO, robin_map backend, global ID packing
  - SparseVoxelPointCloud: document two-level hierarchy, voxel-mean ICP option
  - NDT: document dual-interface design (point-to-point vs. point-to-plane)
  - KeyframePointCloudMap: full description of keyframe storage, ICP integration,
  and implemented interfaces
  - FixedDenseGrid3D: document calloc rationale, cell layout, trivially-copyable
  requirement
  - index3d_t / index3d_hash: document coordinate range, hash algorithm reference,
  dual-role as hash and comparator
  - OccGrid: mark as experimental/incomplete (likelihood cache not populated)
  - SparseTreesPointCloud: document per-block KD-tree design; mark as experimental
  - README: replace two-class stub with full table of all classes, including
  production vs. experimental status
* fix KeyframePointCloudMap::boundingBox() using TOrientedBox for correct AABB
  The previous implementation called TBoundingBox::compose(pose), which only
  transforms the two diagonal corners (min/max), giving an incorrect result
  under rotation. Replace with TOrientedBoxf::getAxisAlignedBox(), which
  transforms all 8 corners before taking the envelope.
* Contributors: Jose Luis Blanco-Claraco

2.9.0 (2026-05-11)
------------------
* fix: KeyframePointCloudMap viz should honor pointSize
* Merge pull request `#143 <https://github.com/MOLAorg/mola/issues/143>`_ from MOLAorg/bump-cmake-version
  bump min req cmake version to 3.22
* bump min req cmake version to 3.22
* Merge pull request `#142 <https://github.com/MOLAorg/mola/issues/142>`_ from MOLAorg/feat/add-keyframe-map-capable-api
  feat: Add keyframe map capable API
* feat: Add keyframe map capable API
* Contributors: Jose Luis Blanco-Claraco

2.8.0 (2026-04-29)
------------------
* Merge pull request `#139 <https://github.com/MOLAorg/mola/issues/139>`_ from MOLAorg/fix/monothonic-kf-ids
  Fix: ensure monothonic KF ids
* Fix: ensure monothonic KF ids
* Merge pull request `#138 <https://github.com/MOLAorg/mola/issues/138>`_ from MOLAorg/fix/kf-map
  fix: robustify edge cases from last API changes
* fix: robustify edge cases from last API changes
* Merge pull request `#137 <https://github.com/MOLAorg/mola/issues/137>`_ from MOLAorg/feat/metric-map-changes-for-lo-grav-align
  mola_metric_maps: per-KF pose plumbing for online gravity rebake
* feat(mola_metric_maps): per-KF pose plumbing for online gravity rebake
  Add the KeyframePointCloudMap APIs needed by mola_lidar_odometry's
  online gravity-rebake feature:
  - cloneKFPoses: snapshot of all KF poses keyed by id
  - setKeyframePose: per-KF pose overwrite with cache invalidation
  - lastInsertedKeyFrameID / nextFreeKeyFrameID_public: id introspection
  - drainEvictedKeyFrameIDs: pull-then-clear list of KFs dropped during
  remove_frames_farther_than-driven evictions inside insertObservation
  Also exports the MOLA_METRIC_MAPS_HAS_KFM_POSE_PLUMBING feature macro
  so downstream packages in separate repos (mola_lidar_odometry) can
  guard usage with __has_include + this macro and stay buildable
  against older mola_metric_maps checkouts.
  Adds unit-test coverage for the new APIs in
  test-mola_metric_maps_keyframemap.
  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
* Merge pull request `#134 <https://github.com/MOLAorg/mola/issues/134>`_ from Zeal-Robotics/perf/keyframe-prewarm-global-submap
  perf(mola_metric_maps): pre-warm global-frame submap data in icp_get_prepared_as_global
* perf(mola_metric_maps): pre-warm global-frame submap data in icp_get_prepared_as_global
  icp_get_prepared_as_global() was building, on the search submap, only the
  local-frame KD-tree and per-point covariances (via KeyFrame::buildCache):
  - bbox
  - kdTreeEnsureIndexBuilt3D() on pointcloud\_     (local frame)
  - computeCovariancesAndDensity()                 (local frame)
  The very first call to nn_search_cov2cov() then lazily materialized the
  global-frame counterparts on the caller thread:
  - pointcloud_global()      (deep copy of pointcloud\_ rotated by pose())
  - kdTreeEnsureIndexBuilt3D() on the global cloud
  - covariancesGlobal()      (rotated covariances)
  For a non-trivial preloaded local map this stalled the first ICP align()
  by several seconds (e.g. ~6 s on a typical localization map), defeating
  the purpose of having a separate "prepare global" hook. Front-ends that
  explicitly pre-warm via icp_get_prepared_as_global() at startup were
  hit hardest, since the remaining lazy work then showed up on the very
  first scan instead of being amortized across startup.
  Pre-trigger the same three calls at the end of
  icp_get_prepared_as_global() so the search submap is fully ready by the
  time ICP::align() / nn_search_cov2cov() runs.
* Contributors: Jose Luis Blanco-Claraco, Robin Van Cauwenbergh

2.7.0 (2026-04-22)
------------------
* Reorganize website
* Merge pull request `#121 <https://github.com/MOLAorg/mola/issues/121>`_ from MOLAorg/fix/clean-up-old-mrpt-version-checks
  Clean up: remove old mrpt version fallback code sections
* Contributors: Jose Luis Blanco-Claraco

2.6.1 (2026-04-02)
------------------
* Merge pull request `#119 <https://github.com/MOLAorg/mola/issues/119>`_ from MOLAorg/fix/thread-safety
  Fix/thread safety
* mola_metric_maps: FIX potential race conditions
  They didn't happen in practice but to be safe
  code clean up
  wip
* Fix potential race in KeyframePointCloudMap::icp_get_prepared_as_global()
* Fix MRPT version required for updated insertAnotherMap() API
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

2.6.0 (2026-03-12)
------------------
* Merge pull request `#113 <https://github.com/MOLAorg/mola/issues/113>`_ from MOLAorg/feat/kf-map-view-vectors
  Feat:kf map view vectors filter for NN search
* Honor the documented disable semantics for max_view_angle_deg
* NDT tests: fix clang-tidy warnings
* view-direction NN filter; add KF map unit tests
* Fix: possible invalid deref
* Better criterion to select active frames
* KeyFramePointCloudMap: Add two debug env vars to control visualization
* Merge pull request `#107 <https://github.com/MOLAorg/mola/issues/107>`_ from MOLAorg/fix/viz-decay-clouds
  Fix/viz-decay-clouds
* BUG FIX: creationOptions were not loaded from ini file in KeyframePointCloudMap
* fix clang-tidy warning: avoid std::endl
* Update coyright notes
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

2.5.0 (2026-02-14)
------------------
* Merge pull request `#100 <https://github.com/MOLAorg/mola/issues/100>`_ from MOLAorg/fix/remove-mrpt-deprecated-maps
  Remove use of mrpt deprecated maps
* Avoid use of deprecated mrpt::maps classes
* Merge pull request `#99 <https://github.com/MOLAorg/mola/issues/99>`_ from MOLAorg/feat/ros2-bridge-pub-geographic
  ROS2 bridge: publish geographic poses too
* ros2 bridge: use geographic_msgs, store the last georeference info internally, and publish georef poses
  merge of these commits:
  - Enable many more clang-tidy checks
  - lint clean
  - implement publishing georeferenced poses
  - mola-viz: fix potential crash on edge case with all points having NaN value
  - FIX: potential crash if no MapServer is present and map services are called
* Contributors: Jose Luis Blanco-Claraco

2.4.0 (2025-12-28)
------------------
* KeyframePointCloudMap: viz now reuses mrpt function for better generalized per-field coloring and avoid code duplication
* Contributors: Jose Luis Blanco-Claraco

2.3.0 (2025-12-15)
------------------
* KF metric map: change heuristic to select nearby KFs for NN matching taking into account the orientation
* KeyFramePointCloudMap: new rendering option to show XYZ axes
* Contributors: Jose Luis Blanco-Claraco

2.2.1 (2025-11-08)
------------------
* Metric maps: implement missing loading 'color.A' from config files
* Contributors: Jose Luis Blanco-Claraco

2.2.0 (2025-10-28)
------------------
* format
* Upgrade to use the upcoming MRPT 2.15 API for CGenericsPointsMap
* KeyFrames metric map: new option to visualize (via ROS publish) with a maximum number of points, downsampling for better performance
* Contributors: Jose Luis Blanco-Claraco

2.1.0 (2025-10-20)
------------------
* Fix formatting
* Implement getAsSimplePointsMap()
* KeyframePointCloudMap: Fix class must be copy-constructible
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2025-10-13)
------------------
* Merge pull request `#93 <https://github.com/MOLAorg/mola/issues/93>`_ from MOLAorg/feature/better-lio
  Changes for new LIO
* add optional debug viz files; fix race conditions
* cov2cov pairings now saves the sqrt(cov_inv)
* Move to new mp2p_icp cov2cov matcher API
* Update missing copyright notices
* New KeyframePointCloudMap map
* Fix typos and clang-tidy hints
* Fix clang-tidy formatting tips
* Contributors: Jose Luis Blanco-Claraco

1.9.1 (2025-07-07)
------------------

1.9.0 (2025-06-06)
------------------

1.8.1 (2025-05-28)
------------------
* Fix: Do not use the deprecated ament_target_dependencies()
* Contributors: Jose Luis Blanco-Claraco

1.8.0 (2025-05-25)
------------------
* Update copyright year
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-05-06)
------------------
* fix clang-format
* Metric maps can now be rendered as semitransparent pointclouds
* Contributors: Jose Luis Blanco-Claraco

1.6.4 (2025-04-23)
------------------
* robin-map: Update to v1.4.0
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
* NDT maps: more render options (enable colormaps,etc.)
* mola_metric_maps: robin-maps upgraded to latest version
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2024-09-29)
------------------

1.2.0 (2024-09-16)
------------------
* gcc warning fix
* Avoid gcc warning
* Merge pull request `#69 <https://github.com/MOLAorg/mola/issues/69>`_ from MOLAorg/new-map-ndt
  New NDT-3D metric map
* Add NDT-3D map class
* Remove leftover dead .cpp file from MOLA package template
* FIX BUG: missing cmake dependency on robin_map in exported targets
* Contributors: Jose Luis Blanco-Claraco

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
* Update robin-map to latest version (Fix cmake < 3.5 compatibility warning)
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
* Metric maps: load insertion options from field 'insertOpts' instead of 'insertionOptions' for compatibility with all other MRPT maps
* disable clang-format in 3rdparty submodules
* Fix usage of const_cast<> with proper value() method
* bump cmake_minimum_required to 3.5
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* Add macro HASHED_VOXEL_POINT_CLOUD_WITH_CACHED_ACCESS
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------

1.0.0 (2024-03-19)
------------------
* implement cached conversion to pointcloud
* make cfg file section optional
* FIX: error on rendering empty voxel maps
* HashedVoxelPointCloud: add missing reserve()
* copyright update
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
