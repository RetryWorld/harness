"""Load the edge kernel through its C ABI — the same way a customer does.

`ctypes` against the C ABI, not a Python extension module. The kernel exposes one
interface — `edge/include/harness_kernel.h` — and our rig is not a privileged
consumer of it. A Python-specific binding would mean the sweep exercised an
interface no customer has, which breaks the claim the provenance story rests on:
that the gate was evaluated by the code the robot runs.

`build.ensure_kernel()` produces the `.so`; this module only loads it.
"""

from __future__ import annotations

import ctypes
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Must match include/harness_kernel.h. Checked at load; a mismatch means this
# Python was written against a different contract than the .so implements, and
# proceeding would misread the envelope.
EXPECTED_ABI_VERSION = 1

HARNESS_OK = 0
VERDICT_ADMIT, VERDICT_DROP, VERDICT_REPLACE = 1, 2, 3

PRED_TYPE_COMPATIBILITY = 1 << 0
PRED_VALUE_RANGE = 1 << 2


class KernelError(RuntimeError):
    pass


class ProjectionResult(ctypes.Structure):
    """Mirrors harness_projection_result_t. Field order is part of the ABI."""

    _fields_ = [
        ("verdict", ctypes.c_int32),
        ("failed", ctypes.c_uint32),
        ("margin", ctypes.c_double),
        ("binding_index", ctypes.c_int32),
    ]


@dataclass(frozen=True)
class KernelInfo:
    kernel_version: str
    schema_revision: str
    build_commit: str
    abi_version: int
    library_path: str


def _default_library_path() -> str:
    # No search-path guessing. Loading whatever libharness_kernel.so happens to
    # be on the system would be a silent provenance failure, so the caller must
    # say which one — normally the path returned by harness_build.ensure_kernel.
    p = os.environ.get("HARNESS_KERNEL_LIB")
    if not p:
        raise KernelError(
            "no kernel path given and HARNESS_KERNEL_LIB is unset; "
            "pass the path from build.ensure_kernel()"
        )
    return p


class Kernel:
    """Thin, allocation-conscious wrapper over the C ABI."""

    def __init__(self, library_path: str | None = None):
        self.library_path = library_path or _default_library_path()
        if not Path(self.library_path).exists():
            raise KernelError(f"kernel library not found: {self.library_path}")
        self._lib = ctypes.CDLL(self.library_path)
        self._bind()

        abi = self._lib.harness_abi_version()
        if abi != EXPECTED_ABI_VERSION:
            raise KernelError(
                f"ABI mismatch: library reports {abi}, this module expects "
                f"{EXPECTED_ABI_VERSION}. Regenerate or re-pin; do not proceed."
            )

    def _bind(self) -> None:
        L = self._lib
        L.harness_abi_version.restype = ctypes.c_uint32
        for name in ("harness_kernel_version", "harness_schema_revision", "harness_build_commit"):
            getattr(L, name).restype = ctypes.c_char_p
        L.harness_status_str.argtypes = [ctypes.c_int32]
        L.harness_status_str.restype = ctypes.c_char_p

        L.harness_profile_load.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.POINTER(ctypes.c_void_p)]
        L.harness_profile_load.restype = ctypes.c_int32
        L.harness_profile_free.argtypes = [ctypes.c_void_p]
        L.harness_profile_arity.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_size_t)]
        L.harness_profile_arity.restype = ctypes.c_int32

        L.harness_project_batch.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_size_t,
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ProjectionResult),
        ]
        L.harness_project_batch.restype = ctypes.c_int32

    def info(self) -> KernelInfo:
        return KernelInfo(
            kernel_version=self._lib.harness_kernel_version().decode(),
            schema_revision=self._lib.harness_schema_revision().decode(),
            build_commit=self._lib.harness_build_commit().decode(),
            abi_version=int(self._lib.harness_abi_version()),
            library_path=self.library_path,
        )

    def _check(self, status: int) -> None:
        if status != HARNESS_OK:
            raise KernelError(self._lib.harness_status_str(status).decode())

    def load_profile(self, profile_json: str) -> "Profile":
        handle = ctypes.c_void_p()
        raw = profile_json.encode()
        self._check(self._lib.harness_profile_load(raw, len(raw), ctypes.byref(handle)))
        arity = ctypes.c_size_t()
        self._check(self._lib.harness_profile_arity(handle, ctypes.byref(arity)))
        return Profile(self, handle, int(arity.value))


class Profile:
    def __init__(self, kernel: Kernel, handle: ctypes.c_void_p, arity: int):
        self._k = kernel
        self._h = handle
        self.arity = arity

    def project_batch(self, candidates: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Evaluate N candidate actions through the real gate.

        Returns `(admitted, emitted, margin)` — the same triple shape the JAX
        shadow gate produces, so the conformance comparison is direct with no
        adapter layer that could paper over a disagreement.
        """
        c = np.ascontiguousarray(candidates, dtype=np.float64)
        if c.ndim != 2 or c.shape[1] != self.arity:
            raise KernelError(f"expected (N, {self.arity}) candidates, got {c.shape}")
        n = c.shape[0]

        emitted = np.zeros_like(c)
        results = (ProjectionResult * n)()
        self._k._check(
            self._k._lib.harness_project_batch(
                self._h,
                c.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                n,
                self.arity,
                emitted.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                results,
            )
        )
        admitted = np.fromiter((r.verdict == VERDICT_ADMIT for r in results), dtype=bool, count=n)
        margin = np.fromiter((r.margin for r in results), dtype=np.float64, count=n)
        return admitted, emitted, margin

    def close(self) -> None:
        if self._h:
            self._k._lib.harness_profile_free(self._h)
            self._h = None

    def __enter__(self) -> "Profile":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
