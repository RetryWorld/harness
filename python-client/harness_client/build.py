"""Build the harness kernel from source, on demand, if it isn't built yet.

# The shape

    ensure_kernel()  ->  check cache  ->  hit?  return the .so
                                      ->  miss? cmake build, test, install, cache

The kernel is not baked into the image and is not a pre-published artifact the
run depends on. The image carries the *source* and a C++ toolchain; the first
run that needs a given source revision builds it, every later run finds it and
moves on. That is the closest thing to `pip install` semantics for a compiled
safety artifact.

# Why cache on a source hash rather than a version string

A version string is a promise a human keeps. A source hash is a fact. If someone
edits `project_into` and forgets to bump a version, a version-keyed cache serves
the stale `.so` and the sweep derives thresholds with code that is not the code
in the tree — silently, and exactly in the place where silence is most expensive.

Hashing the source closure makes that impossible: different source, different
key, automatic rebuild. It also means no publish step, no registry, and no
manual pin to keep in sync while `edge/` is changing daily.

# What is hashed

Every file that can change the compiled behaviour: `CMakeLists.txt`, all `.cpp`
and `.hpp`, and the C header. Not READMEs, not scripts.

`CMakeLists.txt` matters more here than a build file usually would: it carries
`-ffp-contract=off` and `-fno-fast-math`. Drop either and the compiler may fuse
`a*b+c` into an FMA, changing the last bits of `margin` and breaking bit-identical
agreement with the shadow gate. A flags edit must invalidate the cache.

# Concurrency

Shards start together and will race to build. The build goes to a temp directory
and is moved into place atomically, so a loser sees a complete tree or nothing —
never a half-written `.so` that loads and misbehaves.
"""

from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

# Files whose contents can change what the kernel computes.
_HASHED_SUFFIXES = (".cpp", ".hpp", ".h", ".cmake")
_HASHED_NAMES = ("CMakeLists.txt",)
_SKIP_DIRS = {"build", "build-san", "dist", ".git"}


@dataclass(frozen=True)
class KernelBuild:
    source_hash: str
    prefix: Path
    library: Path
    header: Path
    binary: Path
    built_now: bool
    build_seconds: float

    def env(self) -> dict[str, str]:
        return {"HARNESS_KERNEL_LIB": str(self.library)}


def source_hash(edge_dir: Path) -> str:
    """Hash the compile-relevant closure of `edge/`.

    Sorted, path-prefixed, and content-addressed, so the result is independent of
    filesystem ordering and of where the tree happens to be checked out.
    """
    h = hashlib.blake2b(digest_size=32)
    files: list[Path] = []
    for p in sorted(edge_dir.rglob("*")):
        if not p.is_file():
            continue
        if _SKIP_DIRS & set(p.relative_to(edge_dir).parts):
            continue
        if p.name in _HASHED_NAMES or p.suffix in _HASHED_SUFFIXES:
            files.append(p)
    if not files:
        raise RuntimeError(f"no buildable sources under {edge_dir}")
    for p in sorted(files):
        h.update(str(p.relative_to(edge_dir)).encode())
        h.update(b"\0")
        h.update(p.read_bytes())
    return h.hexdigest()[:16]


def _library_name() -> str:
    import sys

    return "libharness_kernel.dylib" if sys.platform == "darwin" else "libharness_kernel.so"


def ensure_kernel(
    edge_dir: str | Path,
    cache_dir: str | Path,
    *,
    force: bool = False,
) -> KernelBuild:
    """Return a built kernel, building it first if the cache misses."""
    edge_dir = Path(edge_dir).resolve()
    cache_dir = Path(cache_dir)
    digest = source_hash(edge_dir)
    prefix = cache_dir / digest
    lib = prefix / "lib" / _library_name()

    if lib.exists() and not force:
        return KernelBuild(
            source_hash=digest,
            prefix=prefix,
            library=lib,
            header=prefix / "include" / "harness_kernel.h",
            binary=prefix / "bin" / "harness-kernel",
            built_now=False,
            build_seconds=0.0,
        )

    started = time.time()
    cache_dir.mkdir(parents=True, exist_ok=True)
    staged = _build(edge_dir, digest, stage_dir=cache_dir)

    # Atomic publish. Another shard may have finished first; if so its tree is
    # equally valid (same source hash, same build) so we discard ours rather
    # than clobbering a tree something may already have dlopen'd.
    prefix.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.rename(staged, prefix)
    except OSError:
        if not lib.exists():
            raise
        shutil.rmtree(staged, ignore_errors=True)

    if not lib.exists():
        raise RuntimeError(f"build completed but {lib} is missing")

    return KernelBuild(
        source_hash=digest,
        prefix=prefix,
        library=lib,
        header=prefix / "include" / "harness_kernel.h",
        binary=prefix / "bin" / "harness-kernel",
        built_now=True,
        build_seconds=time.time() - started,
    )


def _build(edge_dir: Path, digest: str, *, stage_dir: Path, run_tests: bool = True) -> Path:
    """cmake configure + build + install into the customer's layout.

    `cmake --install` rather than hand-copying: the install rules in
    CMakeLists.txt are what a release tarball is packaged from, so the tree the
    sweep loads and the tree a customer unpacks come from the same code path
    instead of being two things that resemble each other.

    The unit tests run before the artifact is published to the cache. A kernel
    that fails its own tests must never become the thing a sweep derives
    thresholds with, and the tests take under a second.

    The staged tree is created *inside* `stage_dir` (the cache's own directory,
    e.g. a Modal Volume), not the container's local /tmp: `ensure_kernel`
    publishes it with `os.rename`, which is only atomic — and only possible at
    all — within one filesystem. Staging on a different device raises EXDEV.
    Only the throwaway cmake build directory uses local /tmp; it's discarded
    either way and putting it on the (usually slower, network) volume would
    just slow the compile down for nothing.
    """
    staged = Path(tempfile.mkdtemp(prefix=f".tmp-harness-{digest}-", dir=stage_dir))
    build_dir = Path(tempfile.mkdtemp(prefix=f"harness-build-{digest}-"))

    env = dict(os.environ)
    # Recorded in the binary and reported by harness_build_commit(). No git in
    # this path, so the source hash IS the provenance handle.
    commit = env.get("HARNESS_BUILD_COMMIT", f"srchash:{digest}")

    generator = ["-G", "Ninja"] if shutil.which("ninja") else []
    subprocess.run(
        ["cmake", "-S", str(edge_dir), "-B", str(build_dir), *generator,
         "-DCMAKE_BUILD_TYPE=Release",
         f"-DHARNESS_BUILD_COMMIT={commit}",
         f"-DCMAKE_INSTALL_PREFIX={staged}"],
        check=True, env=env,
    )
    subprocess.run(
        ["cmake", "--build", str(build_dir), "--parallel", str(os.cpu_count() or 4)],
        check=True, env=env,
    )

    if run_tests:
        subprocess.run(["ctest", "--output-on-failure"], cwd=build_dir, check=True, env=env)

    subprocess.run(["cmake", "--install", str(build_dir)], check=True, env=env)
    shutil.rmtree(build_dir, ignore_errors=True)

    lib = staged / "lib" / _library_name()
    if not lib.exists():
        # CMake versions the SONAME; resolve to the real file so ctypes has a
        # concrete path rather than a dangling link into a temp dir.
        for cand in sorted((staged / "lib").glob(f"{_library_name()}*")):
            if cand.is_file():
                shutil.copy2(cand, lib)
                break

    (staged / "build-manifest.json").write_text(
        f'{{"source_hash":"{digest}","built_at":"{time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}",'
        f'"commit":"{commit}","tests":"{"passed" if run_tests else "skipped"}"}}\n'
    )
    return staged
