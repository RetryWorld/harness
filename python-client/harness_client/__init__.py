"""Python client for the harness kernel.

Wraps `edge/include/harness_kernel.h` the same way any consumer would: `ctypes`
against the C ABI (`kernel.py`), plus a build-from-source cache for the common
case where a consumer has the `edge/` source but no pre-built artifact yet
(`build.py`). Nothing in this package is sim-specific — `sim/` depends on it
the way any other integration would, by installing it and importing it.
"""

from __future__ import annotations

from .build import KernelBuild, ensure_kernel, source_hash
from .kernel import Kernel, KernelError, KernelInfo, Profile, ProjectionResult

__all__ = [
    "Kernel",
    "KernelError",
    "KernelInfo",
    "Profile",
    "ProjectionResult",
    "KernelBuild",
    "ensure_kernel",
    "source_hash",
]
