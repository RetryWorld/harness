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
#   harness-kernel-<ver>-<target>.tar.gz   bin/ lib/ include/harness/ share/proto/
#   release-manifest-<target>.json          version, commit, compiler, ABI, sha256
#
# schemas/ is a required build input as of the v2 (schema-driven) kernel —
# see edge/scripts/isolation_check.sh for why the extraction unit grew from
# edge/ alone to {schemas/, edge/} together.
set -euo pipefail
cd "$(dirname "$0")/.."

VERSION="${1:?usage: build_artifact.sh <version>}"
# Must match install.sh's TARGET computation exactly (see the comment there) —
# any drift here is a manifest/tarball 404 on install.
if [ -z "${TARGET:-}" ]; then
  ARCH="$(uname -m)"
  case "$(uname -s)" in
    Linux)  OS_TAG=linux-gnu ;;
    Darwin) OS_TAG=apple-darwin ;;
    *)      OS_TAG="$(uname -s | tr '[:upper:]' '[:lower:]')" ;;
  esac
  TARGET="${ARCH}-${OS_TAG}"
fi
OUT=dist
BUILD=build-release
SCHEMAS_DIR=../schemas
[ -d schemas ] && SCHEMAS_DIR=schemas
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
  -DCMAKE_INSTALL_PREFIX="$PWD/$STAGE"
cmake --build "$BUILD" --parallel

echo "== test"
# A kernel that fails its own tests must never be published. The sanitizer pass
# is a separate configure because ASan is never shipped in a release binary.
(cd "$BUILD" && ctest --output-on-failure)

echo "== sanitizers"
cmake -S . -B "$BUILD-san" $GEN -DCMAKE_BUILD_TYPE=Debug -DHARNESS_SANITIZE=ON > /dev/null
cmake --build "$BUILD-san" --parallel > /dev/null
(cd "$BUILD-san" && ASAN_OPTIONS=detect_leaks=0 ctest --output-on-failure)
rm -rf "$BUILD-san"

echo "== abi check"
# The header and the library must agree. A header change without an ABI bump is
# how a consumer silently misreads the envelope.
HDR_ABI=$(sed -n -E 's/#define HK_ABI_VERSION ([0-9]+)/\1/p' include/harness/harness_kernel.h)
LIB_ABI=$("$BUILD/rearguard" abi)
if [ "$HDR_ABI" != "$LIB_ABI" ]; then
  echo "ERROR: header ABI $HDR_ABI != library-reported ABI $LIB_ABI" >&2
  exit 1
fi
echo "  ABI v$HDR_ABI"

echo "== package"
cmake --install "$BUILD" > /dev/null
mkdir -p "$STAGE/share/proto/harness/v1"
cp "$SCHEMAS_DIR"/proto/harness/v1/*.proto "$STAGE/share/proto/harness/v1/" 2>/dev/null || \
  echo "  (proto not copied — building outside the monorepo, expected post-extraction)"

TARBALL="$OUT/harness-kernel-$VERSION-$TARGET.tar.gz"
tar -C "$OUT" -czf "$TARBALL" "harness-kernel-$VERSION"
rm -rf "$STAGE"

SHA=$(sha256sum "$TARBALL" 2>/dev/null | cut -d' ' -f1 || shasum -a 256 "$TARBALL" | cut -d' ' -f1)
MANIFEST="$OUT/release-manifest-$TARGET.json"
cat > "$MANIFEST" <<JSON
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
cat "$MANIFEST"
