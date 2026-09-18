# harness kernel

A runtime safety gate for physical AI. Every command a model produces passes
through it before it reaches an actuator, and every decision it makes is
recorded as evidence.

It ships as a **C ABI shared library** plus a CLI. The entire contract is two
headers, so any language that can call C can integrate it — there is no
privileged consumer and no preferred binding.

```
model / policy ──► harness kernel ──► actuator
                        │
                        └──► enforcement events (MCAP episode)
```

The kernel does four things, in this order, every control cycle:

| Mechanism | What it does |
|---|---|
| **Projection** | Clamps a command to its declared bound. Runs every cycle, allocation-free. |
| **Isolation** | Accounts inference and critic time against declared budgets; flags overruns. |
| **Transfer** | Moves authority between bands: nominal → Band A (task-preserving recovery) → Band B (safety fallback, preempts A unconditionally). |
| **Critics** | Independently compiled plugins that supply advisory evidence. They never gate directly. |

## What you get

```
harness-kernel-<version>-<target>.tar.gz
├── bin/rearguard                    validate | show | replay | verify | hash | diff | abi
├── lib/libharness_kernel.so           dlopen or link from any language
├── lib/libharness_kernel.a            static link into your own binary
├── lib/libthreshold_critic.so         reference critic
├── lib/libstall_critic.so             reference critic
├── include/harness/harness_kernel.h   the kernel contract
├── include/harness/harness_critic.h   the critic contract
└── share/proto/harness/v1/*.proto     wire format: profile, enforcement events, provenance, critics
```

**One artifact — not one for you and another for us.** Our own simulation rig
unpacks this same tarball, verifies the same SHA256 against the same
`release-manifest-<target>.json`, and `dlopen`s the same `.so` through the same
C ABI. That is what makes *"the thresholds were derived with the code that
enforces them"* a property of the build rather than a claim in a document.

## Install

```bash
curl -fsSL https://rearguard.dev/install | sh
```

Downloads the release tarball, verifies its SHA256 against the published
manifest, unpacks to `~/.harness`, and adds `~/.harness/bin` to your `PATH`.
Override with `HARNESS_INSTALL_DIR`, pin a version with `./install.sh v0.2.0`.

It compiles nothing and touches no package manager.

### Uninstall

```bash
rearguard uninstall            # asks first; --yes to skip the prompt
rearguard uninstall --dry-run  # show exactly what would go, remove nothing
```

Removes the four installed directories and the `PATH` line the installer added
to your shell rc, and nothing else. It works out where it was installed from
its own location, so it removes the installation you are actually running.
If the directory does not look like a harness install, it refuses rather than
deleting anything.

## Quick start

```bash
# 1. Validate a Harness Profile — and watch a constraint fire
rearguard validate profile.yaml
rearguard validate bad_profile.yaml
#   → error[C1]: profile_eligible=true requires derivation.engine in
#                {MUJOCO, GAZEBO, HARDWARE} (got ENGINE_OTHER)

# 2. See what it actually enforces — bounds, budgets, bands, critics
rearguard show profile.yaml

# 3. Replay a trace through the gate and record the episode
rearguard replay --profile profile.yaml --trace trace.jsonl --out episode.mcap

# 4. Read the episode back and check it
rearguard verify episode.mcap
```

## CLI

The [edge profile workflow](observation/WORKFLOW.md) adds persistent candidates,
failure acceptance and text guidance, MCAP generation requests, artifact review
and approved deployment through `rearguard workflow`. Database sync, trained
critic inference and the recovery controller are explicit placeholders; shipped
runtime activation remains blocked until real adapters exist.

For live ROS 2 recording and operator promotion of sensor/action windows to
failures, use [`rearguard observe`](observation/README.md). It starts with
empty critic/recovery lists and presents pending review drafts. This is an
observation-only stage; actuator enforcement and recovery execution remain
separate work.

`rearguard` is the operator surface: validate profiles, replay traces, and
inspect or compare the recorded episodes.

```
rearguard validate <profile.yaml> [--json]
rearguard show     <profile.yaml> [--json]
rearguard replay   --profile <p.yaml> --trace <t.jsonl>
                     [--out <episode.mcap>] [--critic-lib-dir <dir>] [--json]
rearguard verify   <episode.mcap> [--expect-hash <hex>] [--json]
rearguard hash     <profile.yaml> | --joints <j0,j1,j2> [--json]
rearguard diff     <a.mcap> <b.mcap> [--json]
rearguard abi
rearguard uninstall [--prefix <dir>] [--dry-run] [--yes] [--json]
```

Every command takes `--json` for CI and scripting. Full reference, per-command
behaviour and recipes: [`tools/harness_cli/README.md`](tools/harness_cli/README.md).

## Integrating

| Consumer | Mechanism |
|---|---|
| C / C++ | `#include <harness/harness_kernel.h>`, link `-lharness_kernel` |
| Rust | `bindgen`, or a hand-written `extern "C"` block |
| Go / Python / anything | FFI against the header |
| ROS 2 | `ros2_control` controller plugin — see [`ros2/harness_controller/`](ros2/harness_controller/) |
| a critic | your own shared library implementing `harness/harness_critic.h`, loaded independently of the kernel |

**Check `hk_abi_version()` before any other call.** A mismatch means you were
compiled against a different contract, and proceeding would misread the
envelope. The exported surface is exactly seven functions:

```c
hk_abi_version  hk_create  hk_destroy  hk_gate
hk_submit_proposal  hk_report_budget  hk_last_error
```

## Compatibility

Three version numbers move independently, on purpose:

- **Release version** — the library, static library and CLI all come from one
  `project(VERSION)`, so they cannot diverge from each other.
- **`HK_ABI_VERSION`** (and `HKC_ABI_VERSION` for critics) — bumped on any
  header change. A kernel patch release must not force you to recompile, so
  this is deliberately not tied to the release version. The release build fails
  if the header and the library-reported version (`rearguard abi`) disagree.
- **`schema_version`** on `HarnessProfile` — protobuf field numbers are
  **permanent**, never reused and never renumbered. Your ingest must be able to
  decode episode files written by kernels *older* than the one you run. `buf
  breaking` enforces this mechanically in CI.

## Guarantees

- **No allocation on the decision path.** `hk_gate` writes through
  caller-owned buffers. At 1 kHz an allocator is exactly the unbounded-latency
  component Isolation exists to exclude. A test replaces global `operator new`
  and asserts the count is zero.
- **No exceptions across the ABI.** Every entry point is `noexcept`; the
  `extern "C"` layer catches everything at the boundary. An exception escaping
  into your process is undefined behaviour.
- **Bit-identical replay.** Compiled with `-ffp-contract=off` and
  `-fno-fast-math`, both load-bearing rather than stylistic: GCC otherwise
  contracts `a*b+c` into an FMA, changing the last bits of `margin` and
  silently breaking agreement with the rig that derived your thresholds.
  `isolation_check.sh` asserts both flags are present, and they are inside the
  build cache key. `rearguard diff` and `verify --expect-hash` are how you
  check that property yourself.
- **Only the C ABI is exported.** The shared library's dynamic symbol table
  contains exactly the seven `hk_` functions and nothing else — no C++ symbols,
  no protobuf symbols. A leaked C++ symbol would tie you to our compiler, our
  standard library and our exact protobuf version, which is the coupling the C
  ABI exists to prevent. `isolation_check.sh` asserts the surface on every
  build, on both Linux and macOS.
- **Fail closed.** A profile that violates any of the eleven structural
  constraints (C1–C11) is rejected, not accepted with a warning. `hk_create`
  re-validates defensively even though the CLI already did.
- **Dropping evidence never blocks the gate.** When the event sink is full the
  kernel counts the loss and carries on. `rearguard verify` reconstructs how
  much was lost from sequence-number gaps, independently of the file's own
  accounting.
- **Auditable in isolation.** `schemas/` + `edge/` build as a standalone pair
  with no network and no `FetchContent` — the property that keeps this
  extractable for source escrow or third-party audit. protobuf and yaml-cpp
  are the only external dependencies, and must be provided by the system.

## Why C++

Chosen so a customer's safety case can use a qualified toolchain — MISRA C++ or
AUTOSAR C++14 — without a licensing or version-lag question, and so `edge/`
is one language rather than Rust plus a C++ `rclcpp` shim.

The cost is named rather than argued away: the language provides no memory or
data-race guarantees. The compensating controls are `-Werror` with a wide
warning set, ASan+UBSan in CI, a second compiler, and a decision path with no
allocation and no exceptions.

## Status

| Piece | State |
|---|---|
| Projection — value range, clamping | done, allocation-free, tested |
| Harness Profile: parse + fail-closed validate (C1–C11) | done |
| Isolation — budget accounting, overrun detection | done |
| Transfer — two-band, Band A recovery, Band B fallback | done |
| Critic ABI + dynamic loading + two reference critics | done |
| Evidence — MCAP writer/reader, crash-safe | done — a truncated file parses up to its last committed record |
| C ABI, header, ABI version check | done |
| `rearguard` | done — validate, show, replay, verify, hash, diff, abi |
| Standalone build, sanitizers, determinism gates | done |
| ROS 2 `ros2_control` controller plugin | builds on Jazzy / Ubuntu 24.04; plugin-load and gate-loop tests pass, including a Band B transfer under sustained violation. Mock bring-up (`mock_components/GenericSystem`) runs. Not yet wired into the Gazebo rig. |
| **Projection — freshness, stability, schedulability, delivery** | not implemented; only value range is evaluated |
| **Finiteness predicate** | not implemented — see Known gaps |
| **`python-client/`, sim rig bindings** | not yet ported to the `hk_`/`hkc_` ABI — treat as broken |

Verification on every build: 7 test suites (unit, golden-trace, no-alloc,
property, MCAP, ABI consumer) with 0 failures, the same suites again under
ASan+UBSan, plus `isolation_check.sh` and the dependency-rule gate.

### Known gaps

- **NaN is admitted ungated.** `hk_gate` evaluates only value range. For a NaN
  candidate, `margin` is NaN, `NaN < 0.0` is false, so no violation is
  recorded and the NaN propagates to the actuator — and poisons the cached
  last-admitted value. Infinities, being ordered against the bounds, *are*
  clamped correctly. The asymmetry is pinned by a characterization test that
  will fail the moment a finiteness predicate lands, rather than living in a
  comment. Do not rely on the gate to reject non-finite commands.

## Build from source

Requires a C++20 toolchain, CMake ≥ 3.22, protobuf, yaml-cpp, nlohmann-json,
SQLite3 and OpenSSL Crypto. On Ubuntu the added packages are
`nlohmann-json3-dev libsqlite3-dev libssl-dev`; on Homebrew they are
`nlohmann-json sqlite openssl@3`.

Observation and profile workflow commands are native C++. For live ROS 2
observation, source Jazzy before configuring and add `-DHARNESS_WITH_ROS2=ON`.
Install `ros-jazzy-rosbag2-storage-mcap` for MCAP recording/export. The default
build keeps offline commands available without ROS. See [SimFlow](../SimFlow.md)
for the full simulation command sequence.

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure

./scripts/isolation_check.sh         # standalone + offline + determinism flags
./scripts/check_dependency_rules.sh  # layering rules
```

## Repository layout

```
edge/
├── include/harness/
│   ├── harness_kernel.h      THE contract — every consumer sees exactly this
│   └── harness_critic.h      the critic contract
├── src/
│   ├── kernel/               hk_gate: the decision path (no alloc, no exceptions)
│   ├── profile/              Harness Profile load + fail-closed validate (C1–C11)
│   ├── critic/               the two reference critics
│   ├── mcap/                 episode writer/reader, crash-safe
│   ├── replay/               trace replay rig + event-stream hashing
│   └── format/               the evidence table, rendered in exactly one place
├── tools/harness_cli/      the Rearguard CLI implementation (see its README)
├── ros2/                     ros2_control controller plugin + mock bring-up
├── tests/                    no framework; hand-rolled, runs in ~2s
├── scripts/                  build_artifact.sh, install.sh, isolation_check.sh
└── python-client/            ctypes wrapper — NOT yet ported to the hk_ ABI
```

---

# Releasing

*Maintainers only — everything above this line is what a consumer of the
artifact needs.*

## Cut a release

```bash
cd edge
./scripts/isolation_check.sh          # must pass first
./scripts/build_artifact.sh 0.1.0
```

The release build runs the unit tests **and** a separate ASan+UBSan configure
before packaging. A kernel that fails its own tests must never become the thing
a sweep derives thresholds with.

It refuses to run on a dirty tree: a release that cannot be reproduced from its
commit makes every threshold derived with it unfalsifiable. It also fails if
`HK_ABI_VERSION` in the header and the library-reported version
(`rearguard abi`) disagree.

## Publish, then re-pin

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
provenance claim, because the tested bytes and the shipped bytes diverge. The
rig unpacks the same tarball a customer does, verifies the same SHA256 against
the same `release-manifest-<target>.json`, and `dlopen`s the same `.so` through
the same C ABI. There is no wheel, no Python extension module, and no
privileged install path — which is what makes *"the thresholds were derived
with the code that enforces them"* a property of the build rather than a
sentence in a document.

An earlier design shipped a Python extension module alongside the tarball. Two
artifacts with two ABIs and two build tools meant our rig exercised an
interface no customer installs, and a packaging break would have surfaced on a
customer's robot rather than in our own sweep.

For local iteration against an unreleased build use `make kernel-dev`, which
builds `0.0.0-dev` and fetches it with `--local`. **That path skips SHA
verification, so anything derived with it must stay non-authoritative.**

## Re-integration after the v1→v2 ABI change

The C ABI is what made the Rust→C++ port cheap: the header being the contract,
not the implementation, meant a language rewrite didn't force every consumer to
change with it. The v1→v2 kernel rewrite (schema-driven profiles, two-band
Transfer, the `hk_`/`hkc_` ABI replacing the old `harness_`-prefixed one — see
`harness-invariants-build-brief-v2.md`) broke that continuity deliberately: the
ABI itself changed shape, so every consumer does need to re-integrate.

- **`ros2/harness_controller/`** — ported. Holds an `hk_handle`, calls `hk_gate`
  in `update_and_write_commands`, drains the event ring same-cycle via a
  realtime-safe raw-bytes republish. Not yet wired into `sim/gazebo_rig`'s
  colcon workspace.
- **`python-client/`** and the sim rig's kernel bindings (`sim/sim_rig/`) —
  **not ported.** Still call the v1 `harness_project_batch` surface. Treat as
  broken until that work lands.

## Tiny learned critic

The new [native critic tooling](docs/tiny-critic.md) provides C++ inference and
MCAP replay for the smallest initial model. Python training/export is separate
under [`models/critic`](../models/critic/README.md). This experimental path is
not yet connected to live recovery enforcement.
