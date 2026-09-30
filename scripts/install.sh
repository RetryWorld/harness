#!/usr/bin/env sh
# One-line install of the harness kernel release artifact, mirroring the
# `curl | sh` UX of tools like rustup/deno/bun:
#
#   curl -fsSL https://rearguard.dev/install | sh && export PATH="$HOME/.harness/bin:$PATH"
#
# This does not compile anything. It installs the supported ROS 2 runtime and
# MCAP storage plugin when absent, then downloads the same tarball
# `build_artifact.sh` produces and GitHub Actions
# publishes to a release, verifies it against `release-manifest.json`'s
# sha256, and unpacks it. That is the same artifact the simulation rig loads
# (see README.md, "Releasing") — there is no separate "install path" that could
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

# Override the release source for mirrors or pre-production testing.
REPO="${HARNESS_REPO:-RetryWorld/harness}"
VERSION="${1:-${HARNESS_VERSION:-latest}}"
INSTALL_DIR="${HARNESS_INSTALL_DIR:-$HOME/.harness}"

err() { echo "error: $*" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || err "'$1' is required but not found on PATH"; }
if [ -t 1 ] && [ "${NO_COLOR:-}" = "" ] && [ "${TERM:-}" != "dumb" ]; then
  CYAN='\033[1;36m'; GREEN='\033[1;32m'; DIM='\033[2m'; RESET='\033[0m'
else
  CYAN=''; GREEN=''; DIM=''; RESET=''
fi
step() { printf '%b\n' "${CYAN}›${RESET} $*"; }
okay() { printf '%b\n' "${GREEN}✓${RESET} $*"; }

need curl
need tar

# Must match build_artifact.sh's TARGET computation exactly — that string is
# baked into the tarball and manifest filenames, so any drift here is a
# download 404, not a subtle bug.
ARCH="$(uname -m)"
case "$(uname -s)" in
  Linux)  OS_TAG=linux-gnu ;;
  *)      err "Rearguard currently supports Linux edge computers only" ;;
esac
TARGET="${ARCH}-${OS_TAG}"

printf '\n%b\n\n' "${CYAN}Rearguard${RESET} ${DIM}Harness CLI installer${RESET}"
step "Target: $TARGET"

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

step "Fetching release manifest ($VERSION)"
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

step "Downloading Rearguard CLI $RELEASE_VERSION"
curl -fsSL "$BASE_URL/$TARBALL_PATH" -o "$WORK/kernel.tar.gz" \
  || err "download failed: $BASE_URL/$TARBALL_PATH"

step "Verifying SHA-256"
if command -v sha256sum >/dev/null 2>&1; then
  ACTUAL_SHA="$(sha256sum "$WORK/kernel.tar.gz" | cut -d' ' -f1)"
else
  need shasum
  ACTUAL_SHA="$(shasum -a 256 "$WORK/kernel.tar.gz" | cut -d' ' -f1)"
fi
[ "$ACTUAL_SHA" = "$TARBALL_SHA" ] \
  || err "sha256 mismatch: manifest says $TARBALL_SHA, downloaded file is $ACTUAL_SHA"
okay "Release verified"

step "Installing to $INSTALL_DIR"
rm -rf "$WORK/extracted"
mkdir -p "$WORK/extracted"
tar -C "$WORK/extracted" -xzf "$WORK/kernel.tar.gz"
# The tarball's one top-level dir is harness-kernel-<version>/{bin,lib,include,share}.
EXTRACTED_ROOT="$(find "$WORK/extracted" -mindepth 1 -maxdepth 1 -type d | head -1)"
[ -n "$EXTRACTED_ROOT" ] || err "unexpected tarball layout"

EXTRACTED_RUNTIME_CONTRACT="$EXTRACTED_ROOT/share/harness/runtime/edge-runtime.json"
EXTRACTED_RUNTIME_INSTALLER="$EXTRACTED_ROOT/share/harness/install/install_runtime_deps.sh"
[ -r "$EXTRACTED_RUNTIME_CONTRACT" ] || err "release is missing its edge runtime contract"
[ -x "$EXTRACTED_RUNTIME_INSTALLER" ] || err "release is missing its ROS runtime installer"
step "Installing ROS 2 Jazzy recording runtime when needed"
"$EXTRACTED_RUNTIME_INSTALLER"
"$EXTRACTED_RUNTIME_INSTALLER" --check
okay "ROS 2 and MCAP recording are ready"

mkdir -p "$INSTALL_DIR"
rm -rf "$INSTALL_DIR/bin" "$INSTALL_DIR/lib" "$INSTALL_DIR/include" "$INSTALL_DIR/share"
cp -r "$EXTRACTED_ROOT/." "$INSTALL_DIR/"

RUNTIME_CONTRACT="$INSTALL_DIR/share/harness/runtime/edge-runtime.json"
[ -r "$RUNTIME_CONTRACT" ] || err "release is missing its edge runtime contract"

# rearguard, not "harness-kernel": the v1 daemon (src/main.cpp, --version/
# --abi) is gone, and the only executable the tarball carries now is the CLI
# (see CMakeLists.txt install(TARGETS ...) and README.md's "What you get").
BIN="$INSTALL_DIR/bin/rearguard"
[ -x "$BIN" ] || err "install completed but $BIN is missing or not executable"

step "Checking installed CLI"
# `abi` exercises the installed shared library through the C ABI, so this
# catches an unpacked tree whose binary and .so disagree -- not just a file
# that happens to exist.
"$BIN" abi
okay "CLI is ready"

CRITIC_BIN="$INSTALL_DIR/bin/harness_critic"
[ -x "$CRITIC_BIN" ] || err "install completed but the critic compiler/runtime is missing"
okay "Dynamic proposition critic compiler/runtime is installed"
grep -q '"critic_activation"[[:space:]]*:[[:space:]]*"after_profile_setup"' "$RUNTIME_CONTRACT" \
  || err "release permits critic activation before profile setup"
okay "Critic inference is gated on completed profile setup"

# Persist the path in login and interactive shell startup files. A script
# executed through `curl | sh` cannot mutate its parent shell; the public
# one-liner therefore ends with an export for the current terminal, while
# these edits make every later terminal work without another command.
PATH_LINE="export PATH=\"$INSTALL_DIR/bin:\$PATH\""
persist_path() {
  _rc_file="$1"
  if ! grep -qF "$INSTALL_DIR/bin" "$_rc_file" 2>/dev/null; then
    touch "$_rc_file"
    printf '\n# added by harness-kernel install.sh\n%s\n' "$PATH_LINE" >> "$_rc_file"
  fi
}

persist_path "$HOME/.profile"
case "${SHELL:-}" in
  */zsh)
    persist_path "$HOME/.zshrc"
    [ ! -f "$HOME/.zprofile" ] || persist_path "$HOME/.zprofile"
    ;;
  */bash)
    persist_path "$HOME/.bashrc"
    [ ! -f "$HOME/.bash_profile" ] || persist_path "$HOME/.bash_profile"
    ;;
esac

printf '\n%b\n' "${GREEN}✓ Installed Rearguard CLI ${RELEASE_VERSION}${RESET} ${DIM}($TARGET)${RESET}"
if ! command -v rearguard >/dev/null 2>&1; then
  printf '%b\n' "${DIM}  PATH saved for future terminals; the install one-liner exports it in this terminal.${RESET}"
fi
