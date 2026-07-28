# Running these tests under a sanitizer or valgrind

The map classes here are used from several threads (a mapping thread, an ICP
thread, TBB workers and, for `mola::IncrementalPointCloud`, a background k-d
tree rebuild thread), so the failures worth hunting are memory errors and data
races that a plain test run does not surface. The instrumented builds live
outside the colcon workspace, so they never disturb `install/`.

`run-sanitizers.sh` does the whole thing:

```bash
tests/run-sanitizers.sh asan          # AddressSanitizer + UBSan
tests/run-sanitizers.sh tsan          # ThreadSanitizer
tests/run-sanitizers.sh valgrind      # memcheck, on a short run
tests/run-sanitizers.sh asan 400 7    # <frames> <seed> for the stress test
```

It builds this package standalone into `/tmp/mola_metric_maps-<mode>` and runs
the whole test suite, the stress test last.

## The stress test

`test-mola_metric_maps_incrementalpointcloud_stress` reproduces the usage
pattern of `mola_lidar_odometry` rather than checking results: full-size scans,
a sliding window that evicts and recycles storage slots, background rebuilds in
flight, cov2cov queries on the TBB paths, map copy-outs for publishing, and a
mapping thread concurrent with an ICP thread. It takes `[<frames>] [<seed>]`;
150 frames is a couple of minutes under ASan, and a longer run with several
seeds is the way to chase a non-deterministic failure:

```bash
for s in 1 2 3 4; do
  ASAN_OPTIONS=detect_leaks=0 ./bin/test-mola_metric_maps_incrementalpointcloud_stress 400 $s &
done; wait
```

## Caveats that cost time if you rediscover them

- **TSan needs ASLR reduced.** Ubuntu's default `vm.mmap_rnd_bits=32` makes it
  abort with `unexpected memory mapping`. Run it under `setarch $(uname -m) -R`
  (what the script does) or `sudo sysctl vm.mmap_rnd_bits=28`.
- **oneTBB is not instrumented**, so TSan cannot see the happens-before edges
  its task scheduler establishes, and reports every container that
  `parallel_for` workers fill and the caller merges afterwards as a race.
  `tsan-suppressions.txt` filters the ones whose stack frames name TBB, but not
  the ones whose frames are all plain `std::vector` and library code. Building
  this package with `-DMOLA_METRIC_MAPS_USE_TBB=OFF` does *not* silence them:
  mp2p_icp and MRPT use TBB too. **As of 2026-07-28 the residue is 69 reports,
  all from `test-mola_metric_maps_keyframemap` and all of that shape; the
  `IncrementalPointCloud` tests are clean.** Triage against that baseline
  rather than adding suppressions.
- **MRPT and TBB are not instrumented either.** ASan still checks every
  allocation in the process (its interceptors are global), but only accesses
  compiled into this package are checked against the shadow map.

## Running the real pipeline against the instrumented library

The most faithful test bed is an actual dataset run. There is no need to rebuild
the world: this library keeps its SONAME, so an instrumented copy can be put in
front of the installed one and the ASan runtime preloaded.

```bash
LD_PRELOAD=$(gcc -print-file-name=libasan.so) \
LD_LIBRARY_PATH=/tmp/mola_metric_maps-asan/lib:$LD_LIBRARY_PATH \
ASAN_OPTIONS=detect_leaks=0 \
MOLA_LOCALMAP_CLASS=mola::IncrementalPointCloud \
  <the usual mola-lo / icp_bench command line>
```
