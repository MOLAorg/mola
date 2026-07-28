#!/bin/bash
# Builds this package standalone with the requested instrumentation and runs its
# tests. See README-sanitizers.md.
#
# Usage: run-sanitizers.sh {asan|tsan|valgrind} [<stress frames>] [<stress seed>]
set -euo pipefail

MODE="${1:-asan}"
FRAMES="${2:-150}"
SEED="${3:-1}"

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="/tmp/mola_metric_maps-${MODE}"

# nanoflann >= 1.10 is usually not the distro one; point CMake at wherever it is:
NANOFLANN_PREFIX="${NANOFLANN_PREFIX:-$HOME/code/nanoflann/install-prefix}"

EXTRA_CMAKE_ARGS=()

case "$MODE" in
  asan)
    FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -g"
    RUNNER=(env ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=print_stacktrace=1)
    ;;
  tsan)
    FLAGS="-fsanitize=thread -fno-omit-frame-pointer -g"
    # setarch -R: TSan aborts under Ubuntu's default ASLR entropy.
    RUNNER=(setarch "$(uname -m)" -R env
            "TSAN_OPTIONS=history_size=7:suppressions=${SRC_DIR}/tests/tsan-suppressions.txt")
    # Expect noise from the TBB task scheduler, which TSan cannot see into; see
    # README-sanitizers.md before believing any report.
    ;;
  valgrind)
    FLAGS="-O1 -g -fno-omit-frame-pointer"
    RUNNER=(valgrind --error-exitcode=42 --errors-for-leak-kinds=none --leak-check=no)
    # memcheck is ~30x slower, so the stress run has to be much shorter:
    FRAMES="${2:-8}"
    ;;
  *)
    echo "unknown mode '$MODE' (expected asan | tsan | valgrind)" >&2
    exit 1
    ;;
esac

cmake -S "$SRC_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH="$NANOFLANN_PREFIX" \
  -DCMAKE_CXX_FLAGS="$FLAGS" \
  -DCMAKE_EXE_LINKER_FLAGS="$FLAGS" \
  -DCMAKE_SHARED_LINKER_FLAGS="$FLAGS" \
  "${EXTRA_CMAKE_ARGS[@]+"${EXTRA_CMAKE_ARGS[@]}"}"

cmake --build "$BUILD_DIR" -j"$(nproc)"

STRESS="test-mola_metric_maps_incrementalpointcloud_stress"

for t in "$BUILD_DIR"/bin/test-*; do
  [ "$(basename "$t")" = "$STRESS" ] && continue
  echo "=== $(basename "$t")"
  "${RUNNER[@]}" "$t"
done

if [ -x "$BUILD_DIR/bin/$STRESS" ]; then
  echo "=== $STRESS  (frames=$FRAMES seed=$SEED)"
  "${RUNNER[@]}" "$BUILD_DIR/bin/$STRESS" "$FRAMES" "$SEED"
fi

echo "All instrumented tests passed ($MODE)."
