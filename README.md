# edge — harness kernel (C++20)

A **C ABI shared library** plus a standalone daemon. Language-agnostic by
construction: the entire contract is `include/harness_kernel.h`, and every
consumer sees exactly that surface.

```
edge/
├── CMakeLists.txt            zero third-party dependencies, offline build
├── include/harness_kernel.h  THE contract (unchanged across the Rust→C++ port)
├── src/
│   ├── kernel.cpp/.hpp       project_into — the decision path
│   ├── profile.cpp/.hpp      Harness Profile parse + fail-closed validate
│   ├── json.cpp/.hpp         purpose-built strict parser (see below)
│   ├── cabi.cpp              extern "C" surface
│   └── main.cpp              daemon: --version, --abi, validate
├── tests/                    no framework; hand-rolled, runs in <1s
├── scripts/                  build_artifact.sh, isolation_check.sh
└── python-client/            harness_client — ctypes ABI wrapper + build-from-
                              source cache. A Python package, not C++; excluded
                              from the CMake build and from isolation_check.sh's
                              standalone-build claim about the kernel itself.
```

## Status: verified

| Check | Result |
|---|---|
| `cmake --build` with `-Werror` and a wide warning set | clean |
| `ctest` | **92 checks, 0 failures** |
| ASan + UBSan | 92 checks, 0 failures |
| `isolation_check.sh` (standalone, offline, no deps) | pass |
| Exported symbols | exactly the 10 `harness_*` C ABI functions |
| Loaded from Python via `ctypes` | works, incl. fail-closed profile rejection |

## Why C++ and not Rust

A deliberate reversal. The costs are named rather than argued away:

- **No memory or data-race guarantees from the language.** The compensating
  controls are `-Werror` with a wide warning set, ASan+UBSan in CI, a second
  compiler (clang) in CI, and a decision path with no allocation and no
  exceptions. These are practices, and practices decay; the Rust versions were
  compiler-enforced and did not.
- **`-ffp-contract=off` and `-fno-fast-math` are load-bearing.** GCC defaults to
  contracting `a*b+c` into an FMA, which changes the last bits of `margin`.
  Rust does not do this. Drop either flag and bit-identical agreement with the
  simulation rig's shadow gate silently breaks — the conformance check would
  start failing for a reason that looks like a logic bug. `isolation_check.sh`
  asserts both flags are present, and they are inside the build cache key.

What C++ buys: one language in `edge/` instead of Rust + a C++ `rclcpp` shim;
abundant qualified toolchains if a customer's safety case demands one (MISRA
C++, AUTOSAR C++14) without the Ferrocene licensing and version-lag question;
and a much larger robotics hiring pool.

## Why the JSON parser is ours

Not invented-here. `edge/` must build standalone and offline — that property is
what keeps it extractable for source escrow or a third-party audit, and a
`FetchContent` dependency breaks it. Beyond that, this artifact will be audited:
300 lines a reviewer reads beats 25,000 they wave through. The profile schema is
small, fixed, and ours.

The parser is strict on purpose — duplicate keys, trailing data, `\u` escapes and
malformed numbers are all errors. A profile is a safety artifact; silently
accepting a nearly-valid one is the wrong failure mode.

## Rules

- **No allocation on the decision path.** `project_into` writes through a
  caller-owned `std::span`. At 1 kHz an allocator is exactly the
  unbounded-latency component Isolation exists to exclude. A test replaces
  global `operator new` and asserts the count is zero.
- **No exceptions across the ABI.** `project_into` is `noexcept`; the `extern "C"`
  layer catches everything at the boundary. An exception escaping into a
  customer's process is undefined behaviour.
- **Hidden visibility.** Only the `extern "C"` surface is exported; a leaked C++
  symbol would tie consumers to our compiler and standard library.
- **`edge/` builds standalone.** No includes above `edge/`, no package manager,
  no network. Enforced by `scripts/isolation_check.sh` in CI.

## Status — NOT yet minimally usable

| Piece | State |
|---|---|
| Projection: value-range, type, finiteness | done, allocation-free, tested |
| Harness Profile parse + fail-closed validate | done, incl. joint wcet/rate admission |
| C ABI + header + ABI version check | done |
| Standalone build, sanitizers, symbol hygiene | done |
| **Projection: stability, freshness, schedulability, delivery** | missing |
| **Isolation** | missing |
| **Transfer** | missing |
| **ROS 2 controller plugin** | missing — what makes it "a ROS 2 extension" |
| **Unchunked MCAP logger** (§9.2, open action #8) | missing |
| **Plugin loading** | missing |
| **Event loop** | `--version`, `--abi`, `validate` only |

Today the kernel is a library the sim can call, not something that gates a robot.
The highest-value next piece is the `ros2_control` controller plugin that
`dlopen`s it, plus a measured hook cost inside a real control loop (open action
#2) — that retires the largest architectural risk while it is still cheap to
rethink.

## Build

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure

./scripts/isolation_check.sh      # standalone + offline + symbol hygiene
./scripts/build_artifact.sh 0.1.0 # release tarball + manifest
```

The simulation rig does not run any of this by hand — `harness_client.build`
(installed from `edge/python-client/`, the same package any consumer of `edge/`
would use) runs the same CMake commands inside the Modal container and caches
the result by source hash.
