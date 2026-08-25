#!/usr/bin/env bash
# Build THE artifact. One tarball, consumed identically by the customer's
# runtime and by our simulation rig.
#
# If the rig loaded a different build than the customer installs, "the thresholds
# were derived with the code that enforces them" would be an assertion nobody can
# check. Here it is a property of the build: same tree, same compiler, same
# install rules, one manifest.
#
#   ./scripts/build_artifact.sh 0.1.0
#
# Outputs to dist/:
#   harness-kernel-<ver>-<target>.tar.gz   bin/ lib/ include/ share/
#   release-manifest.json                   version, commit, compiler, ABI, sha256
set -euo pipefail
cd "$(dirname "$0")/.."

VERSION="${1:?usage: build_artifact.sh <version>}"
TARGET="${TARGET:-$(uname -m)-$(uname -s | tr '[:upper:]' '[:lower:]')-gnu}"
OUT=dist
BUILD=build-release
rm -rf "$OUT" "$BUILD" && mkdir -p "$OUT"

echo "== provenance"
COMMIT="$(git rev-parse HEAD 2>/dev/null || echo unknown)"
if [ -n "$(git status --porcelain 2>/dev/null | head -1)" ]; then
  # A release built from a dirty tree cannot be reproduced from its commit, so
  # the provenance claim on every threshold derived with it is unfalsifiable.
  echo "ERROR: working tree is dirty; refusing to build a release artifact" >&2
  exit 1
fi
CXX_VERSION="$(${CXX:-c++} --version | head -1)"
echo "  commit  $COMMIT"
echo "  cxx     $CXX_VERSION"

STAGE="$OUT/harness-kernel-$VERSION"

echo "== configure + build"
GEN=""; command -v ninja >/dev/null 2>&1 && GEN="-G Ninja"
# shellcheck disable=SC2086
cmake -S . -B "$BUILD" $GEN \
  -DCMAKE_BUILD_TYPE=Release \
  -DHARNESS_BUILD_COMMIT="$COMMIT" \
  -DCMAKE_INSTALL_PREFIX="$PWD/$STAGE"
cmake --build "$BUILD" --parallel

echo "== test"
# A kernel that fails its own tests must never be published. The sanitizer pass
# is a separate configure because ASan is never shipped in a release binary.
(cd "$BUILD" && ctest --output-on-failure)

echo "== sanitizers"
cmake -S . -B "$BUILD-san" $GEN -DCMAKE_BUILD_TYPE=Debug -DHARNESS_SANITIZE=ON \
  -DHARNESS_BUILD_COMMIT="$COMMIT" > /dev/null
cmake --build "$BUILD-san" --parallel > /dev/null
(cd "$BUILD-san" && ctest --output-on-failure)
rm -rf "$BUILD-san"

echo "== abi check"
# The header and the library must agree. A header change without an ABI bump is
# how a consumer silently misreads the envelope.
HDR_ABI=$(grep -oP '#define HARNESS_ABI_VERSION \K[0-9]+' include/harness_kernel.h)
LIB_ABI=$("$BUILD/harness-kernel" --abi)
if [ "$HDR_ABI" != "$LIB_ABI" ]; then
  echo "ERROR: header ABI $HDR_ABI != library ABI $LIB_ABI" >&2
  exit 1
fi
echo "  ABI v$HDR_ABI"

echo "== package"
cmake --install "$BUILD" > /dev/null
mkdir -p "$STAGE/share"
cp ../schemas/proto/harness/v1/enforcement.proto "$STAGE/share/" 2>/dev/null || \
  echo "  (proto not copied — building outside the monorepo, expected post-extraction)"

TARBALL="$OUT/harness-kernel-$VERSION-$TARGET.tar.gz"
tar -C "$OUT" -czf "$TARBALL" "harness-kernel-$VERSION"
rm -rf "$STAGE"

SHA=$(sha256sum "$TARBALL" | cut -d' ' -f1)
cat > "$OUT/release-manifest.json" <<JSON
{
  "version": "$VERSION",
  "commit": "$COMMIT",
  "target": "$TARGET",
  "compiler": "$CXX_VERSION",
  "abi_version": $HDR_ABI,
  "schema_revision": "harness.v1",
  "artifacts": {
    "kernel_tarball": {"path": "$(basename "$TARBALL")", "sha256": "$SHA"}
  }
}
JSON
cat "$OUT/release-manifest.json"
