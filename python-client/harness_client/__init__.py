"""Python client for the harness kernel.

Wraps `include/harness_kernel.h` the same way any consumer would: `ctypes`
against the C ABI (`kernel.py`), plus two ways to get the `.so` onto disk —
`build.py`'s build-from-source cache (for iterating on the kernel itself, or
before a version has been released) and `release.py`'s fetch-a-published-
release cache (the production path — see RELEASE.md). Nothing in this
package is specific to any one consumer; `sim/` depends on it the way any
other integration would, by installing it and importing it.
"""

from __future__ import annotations

from .build import KernelBuild, ensure_kernel, source_hash
from .kernel import Kernel, KernelError, KernelInfo, Profile, ProjectionResult
from .release import ReleaseBuild, ReleaseFetchError, ensure_release

__all__ = [
    "Kernel",
    "KernelError",
    "KernelInfo",
    "Profile",
    "ProjectionResult",
    "KernelBuild",
    "ensure_kernel",
    "source_hash",
    "ReleaseBuild",
    "ReleaseFetchError",
    "ensure_release",
]
