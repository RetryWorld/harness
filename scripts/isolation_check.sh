#!/usr/bin/env bash
# Prove `edge/` is extractable and self-contained: copy it alone to a temp dir,
# with no network, and build + test.
#
# The property this defends: `git filter-repo --subdirectory-filter edge` must
# remain a complete extraction, and a customer or auditor must be able to build
# the enforcement path from source with nothing but a C++ compiler and CMake.
#
# It breaks silently — one convenient `../` include, one FetchContent for a JSON
# library — and nothing fails until the day someone asks for source escrow or an
# independent audit.
set -euo pipefail
SRC="$(cd "$(dirname "$0")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

cp -r "$SRC" "$TMP/edge"
rm -rf "$TMP/edge/build" "$TMP/edge/build-san" "$TMP/edge/build-release" "$TMP/edge/dist" "$TMP/edge/.git"

echo "== no escapes above edge/"
if grep -rn '#include *"\.\./\.\.' "$TMP/edge" --include='*.cpp' --include='*.hpp' --include='*.h'; then
  echo "FAIL: source includes reach outside edge/" >&2; exit 1
fi
if grep -rniE 'FetchContent|find_package|ExternalProject|CPMAddPackage' "$TMP/edge/CMakeLists.txt"; then
  echo "FAIL: CMakeLists.txt pulls an external dependency" >&2; exit 1
fi

echo "== float flags present"
# These are load-bearing: without them the compiler may fuse a*b+c into an FMA,
# changing the last bits of `margin` and silently breaking bit-identical
# agreement with the simulation rig's shadow gate.
for flag in -ffp-contract=off -fno-fast-math; do
  grep -q -- "$flag" "$TMP/edge/CMakeLists.txt" || { echo "FAIL: $flag missing" >&2; exit 1; }
done

echo "== standalone build + test"
GEN=""; command -v ninja >/dev/null 2>&1 && GEN="-G Ninja"
# shellcheck disable=SC2086
cmake -S "$TMP/edge" -B "$TMP/build" $GEN -DCMAKE_BUILD_TYPE=Release > /dev/null
cmake --build "$TMP/build" --parallel > /dev/null
(cd "$TMP/build" && ctest --output-on-failure)

echo "== exported symbol surface"
# Only the extern "C" ABI may be exported. A leaked C++ symbol is a mangled name
# that ties consumers to our compiler and standard library.
LEAKED=$(nm -D --defined-only "$TMP/build/libharness_kernel.so" \
  | awk '$2 ~ /[TDB]/ {print $3}' | grep -v '^harness_' | grep -v '^_' || true)
if [ -n "$LEAKED" ]; then
  echo "FAIL: non-C-ABI symbols exported:" >&2; echo "$LEAKED" >&2; exit 1
fi

echo "PASS: edge/ builds standalone with zero dependencies."
echo "      Extraction is 'git filter-repo --subdirectory-filter edge'."
