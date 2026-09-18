# Build Brief — Harness Invariants (v2)

**Supersedes:** v1 (assumed a Rust workspace in a separate repo)
**Targets:** the `retry.world` monorepo, `schemas/` and `edge/`
**Owner:** Alex (architecture) → Claude Code (implementation)
**Language:** C++20. Rust is out.

---

## 0. What this is

Four things do not change across simulators, a bench arm, and a G1: the kernel ABI, the Harness
Profile schema, the enforcement event schema and replay tuple, and the critic interface. This brief
promotes them to a first-class layer with one source of truth and mechanical drift detection.

This is **not a new repo**. It is work inside the existing monorepo:

- `schemas/` becomes the single source of truth for all four, generated to C++, Python, TypeScript
- `edge/` gains the kernel core, the critic ABI, the replay rig, and the CLI

It is also **not greenfield**. `schemas/proto/harness/v1/enforcement.proto` exists and is sound.
Extend it additively. Do not rewrite it, do not renumber it, do not create a parallel definition.

---

## 1. Scope fence

### In scope

- Extending `enforcement.proto` (additive only) and adding `profile.proto`, `critic.proto`,
  `provenance.proto`
- `buf`-based codegen to C++, Python, TypeScript, committed and CI-diffed
- A profile validator enforcing the structural constraints in §4
- Kernel core: Projection, Isolation accounting, two-band Transfer, alloc-free hot path
- Critic C ABI plus two deliberately trivial reference critics
- A replay rig: trace in, enforcement events out, no ROS, no simulator
- Unchunked MCAP writer sufficient for one episode file, plus a minimal reader
- Four golden traces and a runnable end-to-end demo

### Out of scope

No ROS 2 dependency in `schemas/` or in the kernel core. The ROS binding stays a separate layer in
`edge/` and is not part of this work. No simulator. No physics. No VLA inference. No frontend, no
annotation UI, no database. No cloud. No real critic logic beyond the two references.

`LEGACY_BEFORE_HARNESS` is out of the build graph and off the include path. If anything in this
work compiles against it, that is a failure.

---

## 2. Layout

```
schemas/
├── buf.yaml
├── buf.gen.yaml
├── buf.lock
├── proto/harness/v1/
│   ├── enforcement.proto        # EXISTS — extend additively
│   ├── provenance.proto         # new: ProvenanceBlock, DerivationTuple
│   ├── profile.proto            # new: HarnessProfile
│   └── critic.proto             # new: CriticDecl, EvidenceClass
├── gen/                         # GENERATED, COMMITTED, CI-diffed
│   ├── cpp/    python/    ts/
├── examples/
│   ├── profiles/bench3.valid.yaml
│   ├── profiles/invalid/         # one fixture per constraint C1–C10
│   └── traces/*.jsonl
└── tests/golden/                 # expected event-stream hashes

edge/
├── include/harness/
│   ├── harness_kernel.h          # C ABI, hand-written, stability-gated
│   └── harness_critic.h          # C ABI
├── src/
│   ├── kernel/                   # gate logic, alloc-free
│   ├── critic/                   # ABI shim + two reference critics
│   ├── profile/                  # YAML→JSON→proto loader + validator
│   ├── mcap/                     # unchunked writer, minimal reader
│   └── replay/                   # trace driver, event sink
├── tools/harness_cli/          # Rearguard CLI implementation
├── tests/{unit,golden,property,abi,alloc}
└── python-client/                # consumes schemas/gen/python — no local copies
```

### Dependency rules, enforced in CI

1. `schemas/` depends on nothing else in the monorepo.
2. No package anywhere compiles against a hand-written copy of a generated type. Grep for the
   message names outside `schemas/gen/` and fail on a match in a non-generated file.
3. The kernel core links no ROS symbol. Check the symbol table.

### On a separate repo

The only legitimate case is publication: shipping the profile schema and kernel ABI to customers or
to a ROS 2 working group without exposing `backend/` and `db/`. Handle it as a one-way mirror
(`git subtree split` on `schemas/` pushed to a public read-only repo on tag). Monorepo → public,
never the reverse. A repo that pulls from the production branch makes `schemas/` a copy with a sync
step, which reintroduces exactly the drift this layer exists to eliminate.

---

## 3. Toolchain

`buf` for lint, breaking-change detection, and generation. Three CI gates:

- `buf lint` — style and naming
- `buf breaking --against '.git#branch=production,subdir=schemas'` — **this is the gate that
  mechanically enforces the compatibility rule already written at the top of `enforcement.proto`.**
  Field numbers are permanent; this makes that a property rather than a promise.
- `buf generate` then `git diff --exit-code schemas/gen/` — generated code is committed, so drift
  fails the PR that caused it

Generation targets: `protoc-gen-cpp` for the kernel, `protoc-gen-python` for backend/sim/python-client,
and a TypeScript generator (`protobuf-es` or `ts-proto`, pick one and pin it) for `web/`.

### The proto3 presence trap

In proto3 a scalar at its default is indistinguishable from unset. An omitted torque bound
deserialises as `0.0`; an omitted `wcet_ms` reads as a zero budget. For a safety artifact this is
the wrong default in both directions.

**Rule: every scalar in `profile.proto` that must be explicitly authored is declared `optional`,
and the validator rejects absent presence rather than trusting the value.** This applies to all
bounds, budgets, thresholds, and timeouts. It does not apply to enforcement events, where the writer
always sets every field it emits.

### Profile authoring

Author in YAML. Load via YAML → JSON → `google::protobuf::util::JsonStringToMessage` with
`ignore_unknown_fields = false`, so a misspelled key is a load error. The kernel consumes the
serialised binary form; nothing in the hot path parses YAML.

Structural constraints (§4) cannot be expressed in protobuf and are implemented as a hand-written
validator in `edge/src/profile/`. `protovalidate` with CEL expressions embedded in the `.proto` is
worth evaluating as an alternative, but check its C++ maturity before committing; the hand-written
table is the safe default and is about two hundred lines.

---

## 4. `profile.proto` — the Harness Profile

### 4.1 Shape

```
HarnessProfile
  schema_version            uint32
  profile_id                string
  profile_version           string   (semver)
  parent_profile            optional ProfileRef

  Embodiment embodiment
    class_id                string   e.g. "unitree_g1_29dof"
    dof                     uint32
    repeated joint_names    string   ordered
    joint_order_hash        string   hex over canonicalised joint_names

  ModelBinding model_binding
    model_node              string
    output_topic            string
    output_interface        enum { EFFORT, POSITION, VELOCITY }

  Projection projection
    repeated Bound output_region     { optional double min; optional double max; }
    optional double max_staleness_ms
    on_reject               enum { HOLD_LAST, ZERO, REJECT }

  Isolation isolation
    optional double inference_wcet_ms
    optional double inference_rate_hz
    optional uint32 max_payload_bytes
    optional double transport_deadline_ms
    optional uint64 critic_budget_total_us

  repeated CriticDecl critics
    critic_id               string
    evidence_class          string
    optional double window_ms
    optional uint64 wcet_us
    impl_hash               string
    ProvenanceBlock provenance

  Transfer transfer
    BandA band_a
      repeated Recovery recoveries
        evidence_class          string
        action_ref              string
        optional uint32 retry_budget
        optional double timeout_ms
        Predicate reentry_predicate      # REQUIRED
    BandB band_b
      optional double reject_rate_window_ms
      optional double reject_rate_max
      optional uint32 overrun_count_max
      optional string ood_signal
      optional double ood_max
      string fallback_node
      bool latch
    # precedence is NOT a field. Band B preempts Band A, in code.

  Timing timing
    clock_domain            enum { CLOCK_SIM, CLOCK_WALL }
    bool timing_authoritative

  Eligibility eligibility
    bool profile_eligible

  ProvenanceBlock provenance
```

### 4.2 `provenance.proto`

Engine-neutral. The previous version of this brief hardcoded MuJoCo, which reads a stack decision
into the schema. It should not.

```
DerivationTuple
  engine                enum { ENGINE_UNSPECIFIED, MUJOCO, GAZEBO, HARDWARE, OTHER }
  engine_version        string
  model_hash            string
  engine_config_hash    string   # solver opts, timestep, substeps — engine-specific blob
  optional uint64 seed
  initial_state_hash    string
  action_trace_hash     string
  profile_version       string

ProvenanceBlock
  DerivationTuple derivation
  source                enum { SIMULATION, HARDWARE, HAND_AUTHORED }
  optional uint32 geometry_provenance_tier
  confidence            enum { SEED, VALIDATED, DEPLOYED }
  derived_by            string
  notes                 string
```

`engine_config_hash` does the same job the tuple did before, without the schema needing to know what
the engine's configuration looks like. The structural guarantee survives a change of simulator,
which is the invariance claim this layer makes about itself.

### 4.3 Two-band Transfer, normatively

Band A is task-preserving recovery, triggered by critic evidence. Band A commands pass through
Projection like any other command source. Band A has a retry budget and a timeout.

Band B is safety fallback, triggered by projection rejection rate, isolation overrun, staleness, or
the OOD signal. Band B preempts Band A unconditionally, and with `latch` set does not release
without an explicit reset.

Re-entry from Band A to nominal requires the recovery's `reentry_predicate` to hold. There is no
implicit re-entry. A recovery that completes without a passing predicate emits `ReentryDenied` and
escalates to Band B.

Precedence is not configurable. It is a safety property, not a policy, and someone will ask.

### 4.4 Structural constraints

The point of the schema. Each gets a named fixture in `schemas/examples/profiles/invalid/` that must
fail with that constraint's error code. A constraint without a failing fixture is not implemented.

| # | Constraint | Rationale |
|---|---|---|
| C1 | `profile_eligible == true` ⟹ `derivation.engine ∈ ELIGIBLE_ENGINES` and `engine_version` non-empty | An integration-rehearsal rig cannot promote a profile, by construction. `ELIGIBLE_ENGINES` is a compile-time constant in the validator, not a profile field, so it cannot be widened by configuration. |
| C2 | `timing_authoritative == true` ⟹ `clock_domain == CLOCK_WALL` | Simulated timing is never authoritative |
| C3 | Every `recoveries[].evidence_class` matches a declared `critics[].evidence_class` | No recovery for evidence nothing produces |
| C4 | `sum(critics[].wcet_us) <= critic_budget_total_us` | Critics are a model load under Isolation |
| C5 | Band B has at least one trigger set and a non-empty `fallback_node` | No profile without a fallback |
| C6 | Every recovery declares a `reentry_predicate` | The open question in Lee et al., closed by schema |
| C7 | `len(output_region) == dof == len(joint_names)` | Ordering bugs are the most common integration failure in this class |
| C8 | `joint_order_hash` matches the hash of `joint_names` | Detects hand-edited profiles |
| C9 | `parent_profile.embodiment.class_id == embodiment.class_id` when parent is set | Thresholds are embodiment-scoped and non-inheriting |
| C10 | `confidence == DEPLOYED` ⟹ `source == HARDWARE` | Simulation provenance cannot claim deployed confidence |
| C11 | Every `optional` field listed in §4.1 has presence | The proto3 default trap |

---

## 5. Extending `enforcement.proto`

**Additive only. Reserve, never reuse.** `EnforcementEvent` currently uses 1–7 and 10–12; free
numbers are 8, 9, and 13+.

### 5.1 `EnforcementEvent` additions

```proto
message EnforcementEvent {
  // ... existing 1-7 unchanged ...

  uint64 seq = 8;                  // monotone per session, no gaps
  uint32 dropped_since_last = 9;   // ring-buffer losses since previous event

  oneof body {
    ProjectionEvent projection = 10;
    IsolationEvent  isolation  = 11;
    TransferEvent   transfer   = 12;
    CriticEvent     critic     = 13;   // new
    ReentryEvent    reentry    = 14;   // new
  }

  ClockDomain clock_domain = 15;
  optional uint64 wall_time_ns = 16;   // populated when clock_domain == CLOCK_WALL
}
```

`seq` and `dropped_since_last` are the most important additions. The kernel's event sink drops under
pressure by design, because dropping evidence must never block the gate. Without these two fields a
quiet episode and a lossy one are indistinguishable, which silently corrupts every threshold derived
from the corpus.

`sim_time_s` (field 7) stays and becomes meaningful only when `clock_domain == CLOCK_SIM`.

### 5.2 New messages

```proto
enum ClockDomain {
  CLOCK_UNSPECIFIED = 0;
  CLOCK_SIM = 1;
  CLOCK_WALL = 2;
}

message CriticEvent {
  Stamp   decided_at = 1;
  string  critic_id = 2;
  string  evidence_class = 3;
  double  confidence = 4;
  uint32  suggested_mode = 5;      // advisory only, never authoritative
  bytes   evidence_payload = 6;
  uint64  eval_elapsed_us = 7;
  uint64  eval_declared_wcet_us = 8;
}

message ReentryEvent {
  Stamp   decided_at = 1;
  string  recovery_ref = 2;
  bool    predicate_result = 3;
  string  denied_reason = 4;
  uint32  retry_index = 5;
}
```

Add `CRITIC = 4` and `REENTRY = 5` to `Mechanism`. Enum additions are wire-safe.

### 5.3 `TransferEvent` additions

The existing message is single-band. `to_baseline` as a bool cannot express Band A. Extend rather
than replace; existing fields 1–10 stay.

```proto
message TransferEvent {
  // ... existing 1-10 unchanged; to_baseline retained for compatibility ...

  Band     from_band = 11;
  Band     to_band = 12;
  string   recovery_ref = 13;    // set when to_band == BAND_A
  uint32   retry_index = 14;
  bool     preempted_band_a = 15; // Band B fired while Band A was in flight
}

enum Band {
  BAND_UNSPECIFIED = 0;
  BAND_NOMINAL = 1;
  BAND_A = 2;
  BAND_B = 3;
}
```

### 5.4 `EpisodeHeader` — generalise the engine fields

Fields 6 (`mjcf_hash`) and 7 (`mujoco_version`) hardcode one engine. Deprecate, do not renumber.

```proto
message EpisodeHeader {
  // ... 1-5, 8-10 unchanged ...
  string mjcf_hash = 6      [deprecated = true];
  string mujoco_version = 7 [deprecated = true];

  DerivationTuple derivation = 11;   // replaces 6 and 7

  // Preflight identity — what actually loaded, recorded once per episode.
  repeated string joint_names = 12;
  string joint_order_hash = 13;
  ModelBinding.OutputInterface output_interface = 14;
  uint32 kernel_abi_version = 15;
  string kernel_build_hash = 16;
  string profile_hash = 17;
  ClockDomain clock_domain = 18;
}
```

Fields 12–14 close a real hole: `ProjectionEvent.candidate` and `.emitted` are positional
`repeated double` with nothing in the file anchoring the position or the unit. An episode whose
joint ordering cannot be reconstructed is not usable for threshold derivation.

Emit the header as `seq == 0` of every session, always.

---

## 6. Kernel C ABI

`edge/include/harness/harness_kernel.h`. Hand-written now (no cbindgen), so stability is enforced by
diffing rather than by generation.

```c
#define HK_ABI_VERSION 1

typedef struct hk_handle hk_handle;

typedef enum {
    HK_OK = 0, HK_ERR_INVALID_PROFILE = 1, HK_ERR_JOINT_MISMATCH = 2,
    HK_ERR_BUFFER_FULL = 3, HK_ERR_NULL_ARG = 4
} hk_status;

typedef enum { HK_ADMIT = 0, HK_CLAMP = 1, HK_HOLD_LAST = 2, HK_REJECT = 3 } hk_decision;
typedef enum { HK_BAND_NOMINAL = 0, HK_BAND_A = 1, HK_BAND_B = 2 } hk_band;

typedef struct {
    uint64_t t_ns; uint32_t n_joints; const double* effort;
} hk_command;

typedef struct {
    hk_decision decision; hk_band band; uint32_t n_joints; double* effort;
} hk_gated_command;

/* Caller-owned ring buffer. The kernel never allocates. */
typedef struct {
    uint8_t* buf; size_t cap; size_t* len; uint32_t* dropped;
} hk_event_sink;

uint32_t    hk_abi_version(void);
hk_status   hk_create(const uint8_t* profile_pb, size_t len, hk_handle** out);
void        hk_destroy(hk_handle*);
hk_status   hk_gate(hk_handle*, const hk_command*, hk_gated_command*, hk_event_sink*);
hk_status   hk_submit_proposal(hk_handle*, const hkc_proposal*);
hk_status   hk_report_budget(hk_handle*, uint32_t source_id, uint64_t elapsed_ns);
const char* hk_last_error(hk_handle*);
```

`hk_create` takes serialised `HarnessProfile` bytes, not a path and not YAML.

### Hard constraints on `hk_gate`

1. No heap allocation. No `std::string`, no `std::vector`, no `std::function`, no container growth.
2. No locks, no I/O, no syscalls, no exceptions escaping. Errors return `hk_status`.
3. Bounded work, proportional only to `n_joints`.
4. Events serialise into the caller's ring buffer. A full buffer increments `dropped` and returns
   `HK_OK`. **Dropping evidence must never block the gate.**
5. Compiled with `-ffp-contract=off`, no fast-math, no reassociation. Determinism over speed.
6. Protobuf serialisation on the hot path must write into the caller's buffer with no allocation.
   If the generated C++ API cannot do this, hand-roll the wire encoding for the event messages;
   they are small and the field numbers are fixed. Do not relax constraint 1 to avoid this.

Constraint 6 is the one that will be argued about. It is the price of the real-time claim.

### Stability policy (`edge/docs/ABI.md`)

`harness_kernel.h` and a committed symbol list (`nm -D --defined-only`, filtered to `hk_`/`hkc_`)
are both diffed in CI. Either diff fails the build unless the PR bumps `HK_ABI_VERSION` and adds a
changelog row. A C11 translation unit that includes the header and calls every function is built in
CI, so the header stays C-clean regardless of what the implementation does.

---

## 7. Critic ABI

Critics observe and propose. They never actuate. This is the seam that lets a reasoning
adjudicator be replaced by the deterministic kernel, and it is enforced by the type system:
**no function on this interface emits a command.**

```c
typedef struct hkc_handle hkc_handle;

typedef struct {
    uint64_t t_ns; uint32_t n_joints; uint32_t n_samples;
    const double* effort;    /* n_samples * n_joints, row-major */
    const double* position;
    const double* velocity;
    const double* aux; uint32_t n_aux;
} hkc_window;

typedef struct {
    uint32_t evidence_class; float confidence; uint32_t suggested_mode;
    uint8_t payload[64]; uint32_t payload_len;
} hkc_proposal;

uint32_t   hkc_abi_version(void);
hk_status  hkc_create(const uint8_t* params_pb, size_t len, hkc_handle** out);
void       hkc_destroy(hkc_handle*);
hk_status  hkc_evaluate(hkc_handle*, const hkc_window*, hkc_proposal* out);
uint64_t   hkc_declared_wcet_us(hkc_handle*);
```

All allocation happens in `hkc_create`. `hkc_evaluate` is alloc-free. The caller times it and reports
via `hk_report_budget`.

Two reference critics, deliberately trivial:

- `threshold_critic` — an aux channel crosses a bound for N consecutive samples. Proves windowing.
- `stall_critic` — position delta below epsilon while commanded effort is above a bound. Proves
  multi-channel reasoning.

Nothing smarter here. Real critics arrive from the offline track as compiled artifacts with an
`impl_hash`.

---

## 8. Replay rig and traces

Trace format, JSONL, one line per control cycle. Human-readable and hand-editable on purpose:
provoking a new failure mode should cost editing a file, not standing up a simulator.

```json
{"t_ns": 1000000, "effort": [0.0, 1.2, -0.4], "position": [...], "velocity": [...], "aux": {"retention": 1.0}}
```

Four golden traces with committed event-stream hashes in `schemas/tests/golden/`:

| Trace | Provokes | Expected |
|---|---|---|
| `a_nominal` | nothing | all `ADMIT`, one `EpisodeHeader` at `seq==0`, no transitions |
| `b_torque_excursion` | joint 1 ramps past `output_region.max` | `CLAMP` then sustained rejects → transition to `BAND_B` on rate trigger |
| `c_evidence_recovery` | `retention` aux drops | `CriticEvent` → `BAND_NOMINAL→BAND_A` → recovery → `ReentryEvent{pass}` → back to nominal |
| `d_critic_overrun` | critic elapsed exceeds declared WCET | `IsolationEvent{compute_overrun}` → `BAND_B` with `preempted_band_a == true` |

Trace `d` is the important one. It is the exact failure Lee et al. open Section 5 with, and it is
what proves Band B preemption works.

---

## 9. Testing

**Codegen round-trip.** `buf generate` produces byte-identical output to `schemas/gen/`. Generated
TypeScript compiles under `tsc --strict`. Generated Python imports cleanly.

**Wire compatibility.** `buf breaking` against `production`. Plus a fixture test: a serialised event
written by the current schema decodes under the previous release's generated code with known fields
intact.

**Negative schema.** One fixture per constraint C1–C11, each asserting its specific error code, not
merely that validation failed.

**ABI stability.** Header diff, symbol-list diff, and the C11 consumer TU built in CI.

**Determinism.** Same profile plus same trace produces a byte-identical event stream. Hash the
serialised stream with timestamps taken from the trace rather than the wall clock. Run twice in one
process and once in a fresh process, so both iteration-order and address-dependent behaviour are
caught.

**No-allocation.** Override `operator new` / `operator new[]` / `operator delete` to `std::abort()`
when a thread-local guard is set; set the guard around `hk_gate` and `hkc_evaluate`. This test
protects the entire real-time argument. Make it impossible to skip, and run it under both debug and
release.

**Property tests.** Explicit loops over generated profiles and traces (no `proptest` equivalent
needed):
- emitted effort is within `output_region` for every decision except `REJECT`
- `band == BAND_B` with `latch` set implies no subsequent `BAND_A` without a reset
- `seq` strictly increasing with no gaps; `dropped_since_last` accounts for every missing event
- no re-entry to nominal without a passing `ReentryEvent`
- a full ring buffer increments `dropped` and never aborts

**Sanitisers.** ASan and UBSan builds of the kernel and the C consumer as a CI job.

**Timing.** Google Benchmark on `hk_gate`, reporting p50/p99/p99.9 for 7, 29, and 58 joints. CI
**records** and fails only on a large regression against a committed baseline. Absolute WCET is not
decidable on shared CI runners and must not be gated there; the real number comes off dedicated
hardware.

**MCAP.** Write an episode, read it back, assert the `EpisodeHeader` and replay tuple survive. Then
`SIGKILL` the writer mid-stream in a subprocess and assert the partial file parses up to the last
committed record. Crash-safety is the reason for the unchunked writer; test it here rather than
discovering it on the robot.

---

## 10. The runnable demo

Must work from a clean clone with a C++ toolchain, CMake, and `buf`, in under two minutes, with no
ROS, no simulator, and no network.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build

# 1. Validate, and watch a constraint fire
./build/edge/rearguard validate schemas/examples/profiles/bench3.valid.yaml
./build/edge/rearguard validate schemas/examples/profiles/invalid/c1_ineligible_engine.yaml
#   → error[C1]: profile_eligible=true requires derivation.engine in {MUJOCO, HARDWARE}
#                (got GAZEBO)

# 2. Nominal
./build/edge/rearguard replay \
    --profile schemas/examples/profiles/bench3.valid.yaml \
    --trace   schemas/examples/traces/a_nominal.jsonl

# 3. The interesting one
./build/edge/rearguard replay \
    --profile schemas/examples/profiles/bench3.valid.yaml \
    --trace   schemas/examples/traces/c_evidence_recovery.jsonl \
    --out     /tmp/episode.mcap

./build/edge/rearguard verify /tmp/episode.mcap
```

Expected shape of step 3:

```
seq    t_ms     event                 detail
0         0.0   EpisodeHeader         joints=3 order=8f2a… abi=1 profile=bench3@0.1.0 clock=SIM
1       412.0   CriticEvent           threshold_critic  grasp_retention_lost  conf=0.93
2       412.0   TransferEvent         BAND_NOMINAL → BAND_A  recovery=regrasp
3       412.0   ProjectionEvent       REPLACE  j1 candidate=14.20 emitted=12.00  VALUE_RANGE
…
41      688.0   ReentryEvent          regrasp  predicate=PASS  retry=0
42      688.0   TransferEvent         BAND_A → BAND_NOMINAL  trigger=reentry_pass

3000 cycles · 42 events · 0 dropped
gate p50 1.9 µs · p99 4.4 µs · p99.9 7.1 µs   [informational — shared hardware]
wrote /tmp/episode.mcap (41 KiB, replay tuple attached)
```

Lands when steps 1–3 plus `verify` run clean and the stream hash matches
`schemas/tests/golden/c_evidence_recovery.hash`.

---

## 11. Definition of done

1. `ctest` green including sanitiser, no-alloc, and ABI jobs.
2. All four golden traces reproduce committed hashes, in-process and fresh-process.
3. All eleven constraints have a failing fixture asserting a specific error code.
4. `buf lint`, `buf breaking`, and `buf generate` diff all clean in CI.
5. C11 consumer TU builds and runs; symbol list unchanged or version bumped.
6. Generated TypeScript compiles under `tsc --strict`; generated Python imports.
7. `edge/python-client/` imports from `schemas/gen/python/` with no local copies.
8. An MCAP file from the rig opens in the `mcap` CLI and survives `SIGKILL`.
9. The §10 demo runs from a clean clone in under two minutes.
10. `schemas/` links nothing else in the monorepo; kernel core has no ROS symbol.

---

## 12. Traps to refuse

**Renumbering or rewriting `enforcement.proto`.** It is sound and its compatibility rule is already
stated in the file. Extend additively. `buf breaking` will catch violations, but do not fight it.

**Scope creep into ROS.** The moment a `ros2_control` type appears in `schemas/` or the kernel core,
the layer stops being invariant. The binding lives elsewhere in `edge/`.

**A second definition.** A hand-written struct that mirrors a generated message will appear
somewhere, usually in the CLI or a test helper, and it will drift. Grep for it in CI.

**Making Band B precedence configurable.** No. Safety property, not policy.

**Relaxing the no-alloc rule to accommodate protobuf serialisation.** See §6 constraint 6. Hand-roll
the encoder instead.

**Convenience allocation in error paths.** A single `std::string` in an error branch defeats the
no-alloc test precisely where it matters most.

**Treating the CI timing number as a WCET.** It is a regression signal.

**Building a viewer.** The CLI prints a table.

---

## 13. Order of work

1. `buf` setup, lint and breaking gates, generation to all three targets, `schemas/gen/` committed
2. `provenance.proto`, then the additive `enforcement.proto` extensions (§5)
3. `profile.proto` and the validator with all eleven constraints and their fixtures
4. Kernel `hk_create` plus Projection only; `a_nominal` and `b_torque_excursion` passing
5. No-alloc and determinism tests, **before** critics exist
6. Critic ABI and the two reference critics
7. Two-band Transfer; `c_evidence_recovery` and `d_critic_overrun` passing
8. MCAP writer, reader, crash test
9. CLI and the §10 demo

Steps 1–3 pay compounding interest. If time runs short, cut 8 and 9 before cutting 5.

---

## 14. Open questions for Alex

1. **`ELIGIBLE_ENGINES` membership.** C1 needs a compile-time list. `{MUJOCO, HARDWARE}` is the
   assumption; confirm against the current stack decision.
2. **TypeScript generator.** `protobuf-es` or `ts-proto`. Pick one; `web/` may already have a
   preference.
3. **Does `schemas/` currently generate anything?** The tree shows only the `.proto`. If `backend/`
   or `web/` presently hand-writes types matching these messages, that is a migration to sequence
   before step 1, not after.
4. **`Predicate` enum reuse.** The existing enum spans all three PIT mechanisms
   (`SCHEDULABILITY`, `LIFECYCLE_STATE`), which is good. Confirm the re-entry predicate in
   `profile.proto` should reuse it rather than introduce a parallel type.
