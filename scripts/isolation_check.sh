#!/usr/bin/env bash
# Prove `schemas/` + `edge/` together are extractable and self-contained: copy
# that pair alone to a temp dir, with no network beyond installing the pinned
# system libraries CI already has, and build + test.
#
# The property this defends: a customer or auditor must be able to build the
# enforcement path from source with nothing but a C++ compiler, CMake,
# protobuf/yaml-cpp, nlohmann-json, SQLite3, OpenSSL, and buf installed. buf joined that list when schemas/gen/
# stopped being committed (see schemas/.gitignore): the extracted pair now
# carries .proto files and generates its own gencode rather than shipping a
# generated tree, which costs one more build tool and buys gencode that always
# matches the extracting machine's protobuf runtime.
# schemas/ became a real, required dependency of
# edge/ once profile.proto replaced the hand-rolled JSON profile (build brief
# §3) — edge/ alone stopped being a complete extraction the day that happened,
# so the unit this script defends grew to match. See build brief §2 "On a
# separate repo": schemas/ and edge/ are what get shipped to a customer or a
# ROS 2 working group together, without backend/ or db/.
#
# It breaks silently — one convenient `../../` include, one FetchContent for a
# library — and nothing fails until the day someone asks for source escrow or
# an independent audit.
set -euo pipefail
EDGE_SRC="$(cd "$(dirname "$0")/.." && pwd)"
ROOT_SRC="$(cd "$EDGE_SRC/.." && pwd)"
SCHEMAS_SRC="$ROOT_SRC/schemas"
if [ -d "$EDGE_SRC/schemas" ]; then
  # Downstream release layout: edge sources are the repository root and the
  # matching schemas snapshot is vendored beneath it.
  SCHEMAS_SRC="$EDGE_SRC/schemas"
fi
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

mkdir -p "$TMP/edge" "$TMP/schemas"
cp -r "$EDGE_SRC/." "$TMP/edge/"
cp -r "$SCHEMAS_SRC/." "$TMP/schemas/"
rm -rf "$TMP/edge/build" "$TMP/edge"/build-* "$TMP/edge/dist" "$TMP/edge/.git"
rm -rf "$TMP/edge/schemas"
rm -rf "$TMP/schemas/node_modules" "$TMP/schemas/.git"

echo "== no escapes above the {schemas, edge} pair"
if grep -rn '#include *"\.\./\.\./\.\.' "$TMP/edge" --include='*.cpp' --include='*.hpp' --include='*.h'; then
  echo "FAIL: source includes reach outside the schemas/+edge/ pair" >&2; exit 1
fi
if grep -rniE 'FetchContent|ExternalProject|CPMAddPackage' "$TMP/edge/CMakeLists.txt"; then
  echo "FAIL: CMakeLists.txt fetches a dependency over the network" >&2; exit 1
fi

echo "== float flags present"
# These are load-bearing: without them the compiler may fuse a*b+c into an FMA,
# changing the last bits of `margin` and silently breaking bit-identical
# agreement with the simulation rig's shadow gate.
for flag in -ffp-contract=off -fno-fast-math; do
  grep -q -- "$flag" "$TMP/edge/CMakeLists.txt" || { echo "FAIL: $flag missing" >&2; exit 1; }
done

echo "== schemas/ regenerates its own gencode from .proto"
# gen/ is gitignored machine-specific output (see schemas/.gitignore), so the
# extracted pair must be able to produce it from the .proto files it carries —
# that, not a diff against a committed tree, is the property worth checking:
# an extraction that only builds because someone's stale gen/ came along in
# the copy is not a self-contained extraction.
#
# cpp only, via an inline --template: gen/ts's plugin is protoc-gen-es, a node
# binary from schemas/node_modules, which this copy deliberately dropped. The
# standalone build below consumes gen/cpp and nothing else.
rm -rf "$TMP/schemas/gen"
if command -v buf >/dev/null 2>&1 && command -v protoc >/dev/null 2>&1; then
  (cd "$TMP/schemas" && buf generate \
     --template '{"version":"v2","plugins":[{"protoc_builtin":"cpp","out":"gen/cpp"}]}')
  if [ ! -f "$TMP/schemas/gen/cpp/harness/v1/profile.pb.cc" ]; then
    echo "FAIL: buf generate produced no gen/cpp — the extracted pair cannot build itself" >&2
    exit 1
  fi
else
  echo "FAIL: buf and protoc are required — gen/ is generated, not committed," >&2
  echo "      so without them there is no gencode to build the extraction from." >&2
  exit 1
fi

echo "== standalone build + test"
GEN=""; command -v ninja >/dev/null 2>&1 && GEN="-G Ninja"
# shellcheck disable=SC2086
cmake -S "$TMP/edge" -B "$TMP/build" $GEN -DCMAKE_BUILD_TYPE=Release \
  -DHARNESS_SCHEMAS_GEN_CPP="$TMP/schemas/gen/cpp" > /dev/null
cmake --build "$TMP/build" --parallel > /dev/null
(cd "$TMP/build" && ctest --output-on-failure)

echo "== exported symbol surface"
# Only the hk_/hkc_ C ABI may be exported. A leaked C++ symbol is a mangled
# name that ties consumers to our compiler, our standard library and our exact
# protobuf version -- the coupling the C ABI exists to prevent.
#
# This runs on Linux AND macOS. It used to be Linux-only, which is how a
# regression exporting 555 harness::v1::* and protobuf symbols went unnoticed
# on a macOS dev machine.
#
# The filter used to be `grep -v -E '^(hk_|hkc_|_)'`. Every mangled C++ name
# begins with `_Z`, so that trailing `_` alternative whitelisted precisely what
# the check exists to catch and it passed vacuously. Leading-underscore
# symbols are now enumerated instead: the platform's own linker-generated
# entries, and nothing else.
case "$(uname -s)" in
  Linux)
    SO="$TMP/build/libharness_kernel.so"
    # GNU nm: -D reads the dynamic symbol table; T/D/B/R are defined+exported.
    EXPORTED=$(nm -D --defined-only "$SO" | awk '$2 ~ /[TDBR]/ {print $3}')
    # Symbols the linker itself puts in every ELF shared object.
    ALLOWED_UNDERSCORE='^(_init|_fini|_edata|_end|__bss_start)$'
    ;;
  Darwin)
    SO="$TMP/build/libharness_kernel.dylib"
    # Mach-O prefixes every C symbol with '_', so strip it before matching and
    # the same prefix rules apply to both platforms.
    EXPORTED=$(nm -gU "$SO" | awk '{print $3}' | sed 's/^_//')
    ALLOWED_UNDERSCORE='^$'
    ;;
  *)
    SO=""
    ;;
esac

if [ -n "$SO" ]; then
  LEAKED=$(printf '%s\n' "$EXPORTED" \
    | grep -v -E '^(hk_|hkc_)' \
    | grep -v -E "$ALLOWED_UNDERSCORE" \
    | grep -v '^$' || true)
  if [ -n "$LEAKED" ]; then
    COUNT=$(printf '%s\n' "$LEAKED" | grep -c .)
    echo "FAIL: $COUNT non-C-ABI symbol(s) exported from $(basename "$SO"):" >&2
    printf '%s\n' "$LEAKED" | head -20 >&2
    if [ "$COUNT" -gt 20 ]; then
      echo "  ... and $((COUNT - 20)) more" >&2
    fi
    exit 1
  fi
  # A surface that exports nothing means the ABI itself vanished -- a linker
  # or visibility change can produce that, and it must not read as "clean".
  KEPT=$(printf '%s\n' "$EXPORTED" | grep -c -E '^hk_' || true)
  if [ "$KEPT" -lt 7 ]; then
    echo "FAIL: expected the 7 hk_ ABI functions to be exported, found $KEPT" >&2
    exit 1
  fi
  echo "  clean ($KEPT hk_ functions, no C++ leakage)"
else
  echo "  (skipped: unsupported platform $(uname -s))"
fi

echo "PASS: schemas/ + edge/ build standalone as a pair."
