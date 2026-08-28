# Releasing the edge artifact

**One artifact. Not one for the customer and another for us.**

```
harness-kernel-<ver>-<target>.tar.gz
├── bin/harness-kernel          standalone daemon
├── lib/libharness_kernel.so    dlopen or link from any language
├── lib/libharness_kernel.a     static link into a customer binary
├── include/harness_kernel.h    the contract
└── share/enforcement.proto     wire format of the logs it emits
```

The simulation rig unpacks this same tarball, verifies the same SHA256 against
the same `release-manifest-<target>.json`, and `dlopen`s the same `.so` through the same
C ABI. There is no wheel, no Python extension module, and no privileged install
path — which is what makes *"the thresholds were derived with the code that
enforces them"* a property of the build rather than a sentence in a README.

An earlier design shipped a Python extension module alongside the tarball. Two
artifacts with two ABIs and two build tools meant our rig exercised an interface
no customer installs, and a packaging break would have surfaced on a customer's
robot rather than in our own sweep.

The C ABI is also what made the Rust→C++ port cheap: `include/harness_kernel.h`
did not change, so the sim's ctypes loader, the conformance check and every
customer integration were unaffected. Only the build command moved.

## Release

```bash
cd edge
./scripts/isolation_check.sh          # must pass first
./scripts/build_artifact.sh 0.1.0
```

The release build runs the unit tests AND a separate ASan+UBSan configure before
packaging. A kernel that fails its own tests must never become the thing a sweep
derives thresholds with.

The build refuses to run on a dirty tree: a release that cannot be reproduced
from its commit makes every threshold derived with it unfalsifiable.

Publish `dist/` to `HARNESS_ARTIFACT_BASE_URL/<version>/`, then set:

```
sim/sim_rig/config.py::EDGE_ARTIFACT_VERSION = "0.1.0"
```

and re-run `make gate-conformance`. A kernel change can move a gate verdict,
which moves a rejection rate, which moves a threshold. **The pin bump and the
conformance re-run are one action, never two.**

## What the sim must never do

```cmake
# WRONG — the rig would exercise a build no customer ever receives
add_subdirectory(../edge)
```

Path-referencing hides packaging bugs until deployment day and breaks the
provenance claim, because the tested bytes and the shipped bytes diverge.

For local iteration against an unreleased build use `make kernel-dev`, which
builds `0.0.0-dev` and fetches it with `--local`. That path skips SHA
verification, so anything derived with it must stay non-authoritative.

## Versioning

The library, static library and daemon all come from one `project(VERSION)` in
`CMakeLists.txt`, so they cannot diverge.

Two other numbers move independently and more slowly:

- **`HARNESS_ABI_VERSION`** in `include/harness_kernel.h`. Bumped on any header
  change. A kernel patch release must not force every consumer to recompile, so
  this is not tied to the release version. `build_artifact.sh` fails if the
  header and the library disagree.
- **`SCHEMA_REVISION`**. A customer's ingest must decode episode files written by
  *older* kernels than the one it runs, so protobuf field numbers are permanent.

## Consuming it, per language

| Consumer | Mechanism |
|---|---|
| our sim | `ctypes.CDLL` → `harness_client.kernel` (`edge/python-client/`), installed by `sim/` like any other consumer |
| ROS 2 | thin `rclcpp` `ros2_control` controller plugin, `dlopen` |
| C / C++ | `#include <harness_kernel.h>`, link `-lharness_kernel` |
| Rust | `bindgen` or hand-written `extern "C"` |
| C++ | `#include <harness_kernel.h>` — the header is `extern "C"`-guarded |
| Go / Python / anything | FFI against the header |

Check `harness_abi_version()` before any other call. A mismatch means the caller
was compiled against a different contract, and proceeding would misread the
envelope.
