#!/usr/bin/env bash
# ABI stability gate (build brief §6, edge/docs/ABI.md). Diffs both headers
# against the base ref and the exported symbol table against the committed
# baseline. Run from edge/, after a Release build exists at $1 (the build dir).
#
#   ./scripts/check_abi.sh build main
#
# Exit 0: no ABI-relevant drift, or drift accompanied by an HK_ABI_VERSION /
#         HKC_ABI_VERSION bump (informational note only, not auto-verified —
#         the changelog row in edge/docs/ABI.md is what a reviewer checks).
# Exit 1: symbol table drifted from the committed baseline.
set -euo pipefail
cd "$(dirname "$0")/.."

BUILD_DIR="${1:?usage: check_abi.sh <build-dir> <base-ref>}"
BASE_REF="${2:?usage: check_abi.sh <build-dir> <base-ref>}"
BASELINE="abi/symbols.txt"

echo "== header diff vs $BASE_REF"
for hdr in include/harness/harness_kernel.h include/harness/harness_critic.h; do
  if git diff --quiet "$BASE_REF" -- "$hdr" 2>/dev/null; then
    echo "  $hdr: unchanged"
  else
    echo "  $hdr: CHANGED — review for field reordering/removal, signature changes, enum renumbering"
    git diff "$BASE_REF" -- "$hdr" || true
  fi
done

echo "== exported symbol table"
if [ "$(uname -s)" != "Linux" ]; then
  echo "  skipped: nm -D is Linux-only (see edge/docs/ABI.md)"
  exit 0
fi

LIB=""
for candidate in "$BUILD_DIR"/libharness_kernel.so "$BUILD_DIR"/*.so; do
  [ -f "$candidate" ] && LIB="$candidate" && break
done
if [ -z "$LIB" ]; then
  echo "ERROR: no libharness_kernel.so found under $BUILD_DIR" >&2
  exit 1
fi

GENERATED="$(mktemp)"
nm -D --defined-only "$LIB" | awk '$2 ~ /[TDB]/ {print $3}' | grep -E '^(hk_|hkc_)' | sort > "$GENERATED"

if [ ! -f "$BASELINE" ]; then
  echo "  no committed baseline at $BASELINE yet — bootstrapping (see edge/docs/ABI.md)"
  mkdir -p abi
  cp "$GENERATED" "$BASELINE"
  echo "  wrote $BASELINE ($(wc -l < "$BASELINE") symbols) — commit this file to establish the v1 baseline"
  cat "$BASELINE"
  exit 0
fi

if diff -u "$BASELINE" "$GENERATED"; then
  echo "  symbol table matches committed baseline"
else
  echo "ERROR: exported symbol table drifted from $BASELINE" >&2
  echo "  If this is a deliberate ABI change: bump HK_ABI_VERSION / HKC_ABI_VERSION," >&2
  echo "  add a row to edge/docs/ABI.md's changelog, and update $BASELINE." >&2
  exit 1
fi
