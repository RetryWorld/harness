"""Fetch and cache a published release tarball.

The programmatic sibling of `scripts/install.sh` — same download, same
SHA256 verification against the same `release-manifest-<target>.json`, same
tarball layout. A consumer that builds from source (`build.py`) and one that
fetches a release are meant to converge on the identical `.so`; this is how a
long-running process (the simulation rig, a service) gets that artifact
without shelling out to a POSIX script.

# Why this exists alongside `build.py`

`build.py`'s `ensure_kernel()` compiles `edge/` from source — the right tool
while you're iterating on the kernel itself, or don't have a released version
yet. `RELEASE.md` is explicit that anything derived from a source build stays
non-authoritative and must never be what a customer-facing rig loads:

    # WRONG — the rig would exercise a build no customer ever receives
    add_subdirectory(../edge)

`ensure_release()` is the alternative for everyone else: pin a version,
download the same tarball a real client's `curl | sh` installs, verify it,
cache it by version+target so a second call is a stat().
"""

from __future__ import annotations

import hashlib
import json
import os
import platform
import shutil
import sys
import tarfile
import tempfile
import urllib.request
from dataclasses import dataclass
from pathlib import Path

DEFAULT_REPO = "RetryWorld/harness"


class ReleaseFetchError(RuntimeError):
    pass


@dataclass(frozen=True)
class ReleaseBuild:
    version: str
    target: str
    prefix: Path
    library: Path
    header: Path
    binary: Path
    fetched_now: bool

    def env(self) -> dict[str, str]:
        return {"HARNESS_KERNEL_LIB": str(self.library)}


def _target() -> str:
    """Must match build_artifact.sh's TARGET computation exactly — any drift
    here is a download 404, not a subtle bug."""
    arch = platform.machine()
    system = platform.system()
    os_tag = {"Linux": "linux-gnu", "Darwin": "apple-darwin"}.get(system, system.lower())
    return f"{arch}-{os_tag}"


def _library_name() -> str:
    return "libharness_kernel.dylib" if sys.platform == "darwin" else "libharness_kernel.so"


def ensure_release(
    version: str,
    cache_dir: str | Path,
    *,
    repo: str = DEFAULT_REPO,
    force: bool = False,
) -> ReleaseBuild:
    """Return a released kernel build, downloading it first if the cache misses.

    `version` is a bare version string ("0.1.4"), not a tag — the tag is
    `f"v{version}"`, matching `build_artifact.sh`'s `${GITHUB_REF_NAME#v}`.
    """
    cache_dir = Path(cache_dir)
    target = _target()
    prefix = cache_dir / f"{version}-{target}"
    lib = prefix / "lib" / _library_name()

    if lib.exists() and not force:
        return ReleaseBuild(
            version=version, target=target, prefix=prefix, library=lib,
            header=prefix / "include" / "harness_kernel.h",
            binary=prefix / "bin" / "harness-kernel",
            fetched_now=False,
        )

    base_url = f"https://github.com/{repo}/releases/download/v{version}"
    manifest_name = f"release-manifest-{target}.json"

    with tempfile.TemporaryDirectory() as tmp_str:
        tmp = Path(tmp_str)
        manifest_path = tmp / "manifest.json"
        try:
            urllib.request.urlretrieve(f"{base_url}/{manifest_name}", manifest_path)
        except OSError as e:
            raise ReleaseFetchError(
                f"no release manifest for {target} at {base_url}/{manifest_name} "
                f"(unsupported platform, or v{version} not published yet): {e}"
            ) from e

        manifest = json.loads(manifest_path.read_text())
        artifact = manifest["artifacts"]["kernel_tarball"]
        tarball_path = tmp / artifact["path"]
        urllib.request.urlretrieve(f"{base_url}/{artifact['path']}", tarball_path)

        actual_sha = hashlib.sha256(tarball_path.read_bytes()).hexdigest()
        expected_sha = artifact["sha256"]
        if actual_sha != expected_sha:
            raise ReleaseFetchError(
                f"sha256 mismatch: manifest says {expected_sha}, downloaded file is {actual_sha}"
            )

        extracted = tmp / "extracted"
        extracted.mkdir()
        with tarfile.open(tarball_path) as tf:
            tf.extractall(extracted, filter="data")  # path-traversal-safe (Python 3.12+)
        roots = [p for p in extracted.iterdir() if p.is_dir()]
        if len(roots) != 1:
            raise ReleaseFetchError(f"unexpected tarball layout: {roots}")

        cache_dir.mkdir(parents=True, exist_ok=True)
        staged = Path(tempfile.mkdtemp(prefix=f".tmp-harness-{version}-{target}-", dir=cache_dir))
        for item in roots[0].iterdir():
            shutil.move(str(item), str(staged / item.name))

        # Atomic publish, same reasoning as build.py's ensure_kernel: a
        # concurrent fetch of the same version+target is equally valid, so a
        # loser discards its own copy rather than racing a rename.
        prefix.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.rename(staged, prefix)
        except OSError:
            if not lib.exists():
                raise
            shutil.rmtree(staged, ignore_errors=True)

    if not lib.exists():
        raise ReleaseFetchError(f"release fetch completed but {lib} is missing")

    return ReleaseBuild(
        version=version, target=target, prefix=prefix, library=lib,
        header=prefix / "include" / "harness_kernel.h",
        binary=prefix / "bin" / "harness-kernel",
        fetched_now=True,
    )
