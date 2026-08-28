#!/usr/bin/env sh
# One-line install of the harness kernel release artifact, mirroring the
# `curl | sh` UX of tools like rustup/deno/bun:
#
#   curl -fsSL https://raw.githubusercontent.com/<org>/<repo>/main/scripts/install.sh | sh
#
# This does NOT compile anything and does NOT touch a package manager. It
# downloads the same tarball `build_artifact.sh` produces and GitHub Actions
# publishes to a release, verifies it against `release-manifest.json`'s
# sha256, and unpacks it. That is the same artifact the simulation rig loads
# (see RELEASE.md) — there is no separate "install path" that could
# drift from what the tests actually exercised.
#
# Usage:
#   ./install.sh                 # latest release, current OS/arch
#   ./install.sh v0.2.0          # a specific git tag
#   HARNESS_INSTALL_DIR=~/tools ./install.sh
#
# Deliberately POSIX `sh`, not bash: this file is fetched and executed via a
# pipe, often through `sh` explicitly, before anyone can assume bash exists.
set -eu

# The org isn't final yet (see RELEASE.md — no release process exists as
# of this writing); override with HARNESS_REPO once the release repo is named.
REPO="${HARNESS_REPO:-RetryWorld/harness}"
VERSION="${1:-${HARNESS_VERSION:-latest}}"
INSTALL_DIR="${HARNESS_INSTALL_DIR:-$HOME/.harness}"

err() { echo "error: $*" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || err "'$1' is required but not found on PATH"; }

need curl
need tar

# Must match build_artifact.sh's TARGET computation exactly — that string is
# baked into the tarball and manifest filenames, so any drift here is a
# download 404, not a subtle bug.
ARCH="$(uname -m)"
case "$(uname -s)" in
  Linux)  OS_TAG=linux-gnu ;;
  Darwin) OS_TAG=apple-darwin ;;
  *)      OS_TAG="$(uname -s | tr '[:upper:]' '[:lower:]')" ;;
esac
TARGET="${ARCH}-${OS_TAG}"

echo "== target: $TARGET"

if [ "$VERSION" = "latest" ]; then
  BASE_URL="https://github.com/$REPO/releases/latest/download"
else
  BASE_URL="https://github.com/$REPO/releases/download/$VERSION"
fi

# One manifest per target, not one per release: a release can carry tarballs
# for several platforms built on different CI runners, and "release-
# manifest.json" alone can't say which sha256 belongs to which target.
MANIFEST_NAME="release-manifest-$TARGET.json"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "== fetching $MANIFEST_NAME ($VERSION)"
curl -fsSL "$BASE_URL/$MANIFEST_NAME" -o "$WORK/manifest.json" \
  || err "no release manifest for $TARGET at $BASE_URL/$MANIFEST_NAME (unsupported platform, or no release published yet)"

# No jq: this is a 10-line JSON object with unique flat keys, and a shell
# installer shouldn't require a dependency the target machine might not have.
json_field() {
  grep -o "\"$1\"[[:space:]]*:[[:space:]]*\"[^\"]*\"" "$2" | head -1 | sed -E 's/.*: *"([^"]*)"/\1/'
}

RELEASE_VERSION="$(json_field version "$WORK/manifest.json")"
TARBALL_PATH="$(json_field path "$WORK/manifest.json")"
TARBALL_SHA="$(json_field sha256 "$WORK/manifest.json")"
[ -n "$TARBALL_PATH" ] && [ -n "$TARBALL_SHA" ] || err "malformed manifest at $BASE_URL/$MANIFEST_NAME"

echo "== fetching $TARBALL_PATH"
curl -fsSL "$BASE_URL/$TARBALL_PATH" -o "$WORK/kernel.tar.gz" \
  || err "download failed: $BASE_URL/$TARBALL_PATH"

echo "== verifying sha256"
if command -v sha256sum >/dev/null 2>&1; then
  ACTUAL_SHA="$(sha256sum "$WORK/kernel.tar.gz" | cut -d' ' -f1)"
else
  need shasum
  ACTUAL_SHA="$(shasum -a 256 "$WORK/kernel.tar.gz" | cut -d' ' -f1)"
fi
[ "$ACTUAL_SHA" = "$TARBALL_SHA" ] \
  || err "sha256 mismatch: manifest says $TARBALL_SHA, downloaded file is $ACTUAL_SHA"

echo "== installing to $INSTALL_DIR"
rm -rf "$WORK/extracted"
mkdir -p "$WORK/extracted"
tar -C "$WORK/extracted" -xzf "$WORK/kernel.tar.gz"
# The tarball's one top-level dir is harness-kernel-<version>/{bin,lib,include,share}.
EXTRACTED_ROOT="$(find "$WORK/extracted" -mindepth 1 -maxdepth 1 -type d | head -1)"
[ -n "$EXTRACTED_ROOT" ] || err "unexpected tarball layout"

mkdir -p "$INSTALL_DIR"
rm -rf "$INSTALL_DIR/bin" "$INSTALL_DIR/lib" "$INSTALL_DIR/include" "$INSTALL_DIR/share"
cp -r "$EXTRACTED_ROOT/." "$INSTALL_DIR/"

BIN="$INSTALL_DIR/bin/harness-kernel"
[ -x "$BIN" ] || err "install completed but $BIN is missing or not executable"

echo "== sanity check"
"$BIN" --version
"$BIN" --abi

# Make it runnable without a fresh shell knowing $INSTALL_DIR: append a PATH
# line once, the same idempotent pattern rustup/deno use, rather than
# requiring the caller to edit their shell rc by hand.
PATH_LINE="export PATH=\"$INSTALL_DIR/bin:\$PATH\""
RC_FILE=""
case "${SHELL:-}" in
  */zsh) RC_FILE="$HOME/.zshrc" ;;
  */bash) RC_FILE="$HOME/.bashrc" ;;
esac

if [ -n "$RC_FILE" ] && [ -f "$RC_FILE" ] && ! grep -qF "$INSTALL_DIR/bin" "$RC_FILE" 2>/dev/null; then
  printf '\n# added by harness-kernel install.sh\n%s\n' "$PATH_LINE" >> "$RC_FILE"
  echo "== added $INSTALL_DIR/bin to PATH in $RC_FILE (restart your shell, or run: $PATH_LINE)"
else
  echo "== add this to your shell profile: $PATH_LINE"
fi

echo "== installed harness-kernel $RELEASE_VERSION ($TARGET) to $INSTALL_DIR"
