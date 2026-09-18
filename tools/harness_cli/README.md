# `rearguard`

The operator-facing surface of the harness kernel: validate a Harness Profile,
replay a trace through the gate, and inspect or compare the MCAP episodes that
come out.

Live ROS 2 observation is available through `rearguard observe`: start a
recorder, mark a sensor/action window, promote it with an operator failure
description and inspect pending critic/recovery drafts. See the
[SO-101 walkthrough and ROS contract](../../observation/README.md).
This observation mode does not activate actuator enforcement.

`observe` and `workflow` are implemented in C++ and called directly by the CLI.
Build with `-DHARNESS_WITH_ROS2=ON` after sourcing Jazzy for live commands and
MCAP export. Offline builds support saved-session inspection and profile review.
The CLI no longer launches or installs a Python observation adapter.

`rearguard workflow` implements the persistent profile lifecycle through
approved artifact installation. See [the workflow contract and commands](../../observation/WORKFLOW.md).

It is deliberately the only "viewer". Build brief §12 lists building one as a
non-goal because **this table is it** — so anything you need to see about an
episode, you see here.

```
rearguard validate <profile.yaml> [--json]
rearguard show     <profile.yaml> [--json]
rearguard replay   --profile <p.yaml> --trace <t.jsonl>
                     [--out <episode.mcap>] [--critic-lib-dir <dir>] [--json]
rearguard verify   <episode.mcap> [--expect-hash <hex>] [--json]
rearguard hash     <profile.yaml> [--json]
rearguard hash     --joints <j0,j1,j2> [--json]
rearguard diff     <a.mcap> <b.mcap> [--json]
rearguard abi
rearguard uninstall [--prefix <dir>] [--dry-run] [--yes] [--json]
```

## Where the binary comes from

| How | Path |
|---|---|
| Local build | `edge/build/rearguard` |
| Release tarball | `bin/rearguard`, alongside `harness-kernel` |
| `scripts/install.sh` | `~/.harness/bin/rearguard` (on `PATH`) |

It is installed by the same `install(TARGETS ...)` rule as the kernel itself,
so the CLI you run is always from the same build as the kernel you're running.

## Quick start

The build brief §10 demo, end to end, with no ROS, no simulator and no network:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build
CLI=./build/rearguard

# 1. Validate, and watch a constraint fire
$CLI validate ../schemas/examples/profiles/bench3.valid.yaml
$CLI validate ../schemas/examples/profiles/invalid/c1_ineligible_engine.yaml

# 2. See what that profile actually enforces
$CLI show ../schemas/examples/profiles/bench3.valid.yaml

# 3. Replay a trace and record the episode
$CLI replay --profile ../schemas/examples/profiles/bench3.valid.yaml \
            --trace   ../schemas/examples/traces/c_evidence_recovery.jsonl \
            --out     /tmp/episode.mcap

# 4. Read it back and check it
$CLI verify /tmp/episode.mcap
```

---

## Commands

### `validate <profile.yaml>`

Parses a Harness Profile and runs the eleven structural constraints (C1–C11)
that protobuf cannot express. **Fails closed**: it reports *every* violation it
finds, not just the first, and any non-empty result means do not load this
profile.

```
$ rearguard validate bench3.valid.yaml
bench3.valid.yaml: valid

$ rearguard validate c1_ineligible_engine.yaml
error[C1]: profile_eligible=true requires derivation.engine in {MUJOCO, GAZEBO, HARDWARE} (got ENGINE_OTHER)
```

The `C1`–`C11` codes are stable and machine-checkable — match on those, not on
the message text.

`validate` tells you whether a profile is *legal*. It does not tell you what it
*says* — for that, see `show`.

---

### `show <profile.yaml>`

Prints the profile the kernel would be handed, grouped by enforcement
mechanism. Use it to answer "what limits is this thing actually enforcing?"

```
$ rearguard show bench3.valid.yaml
profile      bench3@0.1.0  schema_version=1
embodiment   bench3_arm  dof=3
joints       3  [j0, j1, j2]
             order_hash 33dbfb07113ef0bc (declared)  ok
binding      vla_policy -> /bench3/cmd_effort as EFFORT
timing       clock=CLOCK_SIM  authoritative=false
eligibility  profile_eligible=true

projection   on_reject=ON_REJECT_HOLD_LAST  max_staleness=50 ms
               j0  [-10, 10]
               j1  [-10, 10]
               j2  [-10, 10]
isolation    inference_wcet=8 ms  rate=100 Hz
             max_payload=4096 B  transport_deadline=5 ms
             critic_budget_total=2000 us
critics      1 declared
               threshold_critic  evidence=grasp_retention_lost  window=200 ms  wcet=500 us  impl=deadbeef
transfer
             band_a  1 recovery(ies)
               on grasp_retention_lost  -> regrasp  retry_budget=2  timeout=500 ms  reentry=LIFECYCLE_STATE
             band_b  fallback=safe_hold  latch=true
               reject_rate > 0.2 over 1000 ms
               overrun_count > 3
               ood retention > 0.5
provenance   source=PROVENANCE_SOURCE_SIMULATION  confidence=CONFIDENCE_VALIDATED  by=alex
             engine=ENGINE_MUJOCO 3.1.0  seed=42  geometry_tier=1
             model=abc123  engine_config=cfg123
             notes: bench golden profile

validate     ok -- C1-C11 clean
```

Three things it does on purpose:

**It shows the loaded profile, not the file.** What it prints is the
`HarnessProfile` message after parsing — the same object the kernel is handed.
It never re-reads the YAML, because a second reading of the authoring format
would be a second definition of the profile.

**It prints presence, not defaults.** Every bound, budget, threshold and
timeout in the schema is `optional` so that an omitted value cannot
deserialise as `0.0`. An absent field prints `<unset>` (and is `null` under
`--json`), never `0` — which is what makes a C11 failure legible:

```
isolation    inference_wcet=<unset> ms  rate=100 Hz
...
validate     1 constraint failure(s)
             error[C11]: missing required presence for 'isolation.inference_wcet_ms'
```

**It does not refuse an invalid profile.** The profile you most need to look at
is usually the one that just failed validation. `show` renders it anyway and
reports the verdict at the end, so it is never mistaken for a clean bill of
health. A C8 mismatch is shown in place:

```
             order_hash deadbeefdeadbeef (declared)
                        33dbfb07113ef0bc (computed)  MISMATCH -- this is what C8 rejects
```

Exit code is `0` only if the profile also validates clean, so
`show` is a drop-in for `validate` in CI when you want the failure output to
carry context.

Where the bound count and the joint count disagree (**C7**), bounds are labelled
by index rather than joint name — inventing a name for a bound with no joint
would hide the mismatch:

```
projection   on_reject=ON_REJECT_HOLD_LAST  max_staleness=50 ms
               [0]  [-10, 10]
               [1]  [-10, 10]
```

---

### `hash <profile.yaml>` / `hash --joints <j0,j1,j2>`

Computes the canonical joint-order hash. This exists to debug a **C8**
(joint-order hash mismatch) rejection.

```
$ rearguard hash bench3.valid.yaml
profile           bench3@0.1.0
dof               3
joints            3  [j0, j1, j2]
joint_order_hash  33dbfb07113ef0bc  (computed)
declared          33dbfb07113ef0bc  (embodiment.joint_order_hash)

  ok    computed matches declared
```

**It does not run `validate` first, on purpose.** A profile that fails C8 fails
validation, so validating first would refuse the exact case you need this for.

`--joints` hashes an arbitrary ordering with no profile involved, which is how
you check a live rig's joint order against what a profile declares:

```
$ rearguard hash --joints joint_1,joint_2,joint_3
joints            3  [joint_1, joint_2, joint_3]
joint_order_hash  78c7c412c710d288
```

Order matters completely — `j1,j0,j2` hashes to something entirely unrelated to
`j0,j1,j2`. That is the point: a silently reordered episode derives a bound for
the wrong actuator.

---

### `replay --profile <p.yaml> --trace <t.jsonl>`

Drives a trace through the kernel and its critics, prints the event table, and
reports the `event_stream_hash`.

**Refuses to run on a profile that fails validation** — you cannot accidentally
produce evidence under a profile that was never admissible.

| Flag | Meaning |
|---|---|
| `--profile` | Harness Profile (required) |
| `--trace` | JSONL trace to drive through the gate (required) |
| `--out` | Write the episode to this MCAP. Omit and nothing is recorded. |
| `--critic-lib-dir` | Where to find `lib<critic_id>.{so,dylib}`. Defaults to the build dir. |

```
$ rearguard replay --profile bench3.valid.yaml --trace c_evidence_recovery.jsonl --out ep.mcap
seq        t_ms   event                 detail
0           0.0   EpisodeHeader         joints=3 order=33dbfb07 abi=1 profile=bench3@0.1.0 clock=CLOCK_SIM
0           0.0   ProjectionEvent       ADMIT margin=10.000000
10         40.0   CriticEvent           threshold_critic  grasp_retention_lost  conf=1.000000
11         40.0   TransferEvent         BAND_NOMINAL -> BAND_A  recovery=regrasp  trigger=critic_evidence
16         60.0   ReentryEvent          regrasp  predicate=PASS  retry=0
17         60.0   TransferEvent         BAND_A -> BAND_NOMINAL  recovery=regrasp  trigger=reentry_pass
...
8 cycles · 20 events · 0 dropped
gate p50 0.4 µs · p99 2.0 µs · p99.9 2.0 µs   [informational -- shared hardware]
event_stream_hash: 84a868ea6cd0573c
```

The timing numbers vary run to run and are a **regression signal, not a WCET**
(§12). The `event_stream_hash` does not vary: identical inputs produce an
identical stream, which is what `diff` and `verify --expect-hash` rely on.

---

### `verify <episode.mcap>`

Reads an episode back and checks it. This renders the same table `replay` does —
from the same code (`src/format/event_table.cpp`) — so the two cannot drift.

It then reports named checks rather than one pass/fail, because "this episode is
unusable" and "this episode lost 3 events to a full buffer" need different
responses:

```
$ rearguard verify ep.mcap
...table...

ep.mcap
event_stream_hash: 84a868ea6cd0573c

  ok    MCAP magic
  ok    footer present (file is complete)
  ok    exactly one EpisodeHeader (found 1)
  ok    20 event(s) decoded, 0 undecodable
  ok    seq contiguous, no gaps
  ok    log_time monotonic
  ok    kernel ABI 1 matches this binary
```

| Check | Severity | Why |
|---|---|---|
| MCAP magic | fail | Not an episode file at all |
| Footer present | **fail** | Missing footer = truncated (e.g. `SIGKILL`). Events shown are still valid; the episode did not finish. |
| Exactly one `EpisodeHeader` | fail | An episode has exactly one identity |
| All records decodable | fail | Corrupt evidence |
| `seq` regressions | fail | Events out of order or duplicated |
| `log_time` monotonic | fail | Timeline is not reconstructable |
| `seq` gaps | **warn** | Evidence was dropped. Expected: the kernel drops rather than blocks when the sink is full. |
| Kernel ABI mismatch | **warn** | Still valid evidence of what *that* kernel did — it just isn't comparable to this binary. |

Two details worth knowing:

- **Gap counting doesn't trust the file.** The kernel assigns a `seq` to every
  event it *attempts* to emit, including dropped ones, so gaps in `seq` are an
  independent record of loss. `verify` derives the count that way and
  cross-checks it against the `dropped_since_last` the events carry; if the two
  accounts disagree, it says so.
- **`--expect-hash` proves the file is the event stream.** `verify` recomputes
  `event_stream_hash` from the bytes on disk using the same code path `replay`
  hashes with (`src/replay/event_hash.hpp`).

```bash
HASH=$(rearguard replay --profile p.yaml --trace t.jsonl --out ep.mcap --json | jq -r .event_stream_hash)
rearguard verify ep.mcap --expect-hash "$HASH"
```

---

### `diff <a.mcap> <b.mcap>`

The determinism regression check. Two runs that should be identical must share
an `event_stream_hash`; when they don't, this finds the first event whose bytes
differ and renders both rows.

```
$ rearguard diff run1.mcap other.mcap
a  run1.mcap
   hash=84a868ea6cd0573c  20 event(s)
b  other.mcap
   hash=b023b3210c5a1e8d  17 event(s)

  ok    EpisodeHeaders describe the same run
  FAIL  event streams differ

first divergence at event index 2:

     seq        t_ms   event                 detail
  a  2          10.0   ProjectionEvent       ADMIT margin=9.500000
  b  2          10.0   ProjectionEvent       ADMIT margin=5.000000
```

It compares only the `EpisodeHeader` fields that decide whether the two files
are runs of *the same thing* — `profile_hash`, `joint_order_hash`, ABI, clock
domain, output interface. Fields that differ legitimately between runs, like
`scenario_id`, are ignored. A header difference is reported first because it
*explains* a divergence rather than being one.

---

### `abi`

Prints the kernel ABI version this binary was built against. Per ABI rule 4,
check this before anything else consumes the kernel.

```
$ rearguard abi
1
```

---

### `connect`

Authenticates a headless edge computer with a Rearguard web account. The CLI
uses outbound HTTPS only, so the browser and edge computer do not need to share
a network.

```
$ rearguard connect
Rearguard device pairing

Open https://rearguard.dev/device
and enter this code:

  RG7K-M2QF

Waiting for approval...
```

The code expires after ten minutes and works once. After browser approval, the
CLI writes the device ID and its random credential to
`$XDG_CONFIG_HOME/rearguard/device.json` (or
`~/.config/rearguard/device.json`) with mode `0600`. The database stores only a
SHA-256 digest of that credential. `--name <device-name>` overrides the Linux
hostname and `--json` produces machine-readable progress records.

---

### `uninstall`

Removes an installation created by `scripts/install.sh`.

It lives here rather than in a `scripts/uninstall.sh` because `scripts/` is not
part of the release — a user who installed with `curl | sh` has `rearguard`
on their `PATH` and no shell script to run. The uninstaller has to be the thing
they already have.

```
$ rearguard uninstall
harness installation at /Users/you/.harness
  (derived from /Users/you/.harness/bin/rearguard)

would remove 9 file(s) in:
  /Users/you/.harness/bin
  /Users/you/.harness/lib
  /Users/you/.harness/include
  /Users/you/.zshrc  (PATH entry: export PATH="/Users/you/.harness/bin:$PATH")

Remove these? [y/N] y
removed harness installation from /Users/you/.harness
restart your shell (or open a new one) so PATH no longer points there
```

| Flag | Meaning |
|---|---|
| `--prefix <dir>` | Uninstall from here instead of the location this binary is running from |
| `--dry-run` | Print exactly what would be removed, remove nothing |
| `--yes` | Skip the confirmation prompt |
| `--json` | Machine-readable result |

**It undoes exactly what the installer did.** The four unpacked directories
(`bin`, `lib`, `include`, `share`) and the `PATH` block appended to your shell
rc — nothing else. The prefix directory itself is removed only if it is empty
afterwards, since it may be a directory you use for other things.

Safety properties worth knowing, because this command deletes files:

- **It refuses anything that isn't a harness install.** The target must contain
  both `bin/rearguard` and `lib/libharness_kernel.*`. A typo'd `--prefix`, or
  running it from a build tree, removes nothing and exits 1.
- **It never `rm -rf`s the prefix.** Only the four known subdirectories are
  removed, by name.
- **Non-interactive input is not consent.** With no TTY and no `--yes` it
  aborts rather than reading EOF as "yes".
- **It only removes a `PATH` line it can prove is its own** — the installer's
  marker comment followed by a line naming *this* prefix's `bin`. A second
  harness install's entry, or anything else in your rc, is left alone. The rc
  file is rewritten via a temp file and rename, so an interrupted uninstall
  cannot truncate it.

For development, `--dry-run` and `--yes` are the useful pair:

```bash
cmake --install build --prefix /tmp/harness-test
/tmp/harness-test/bin/rearguard uninstall --dry-run     # inspect
/tmp/harness-test/bin/rearguard uninstall --yes         # tear down
```

Deleting the binary that is currently running is fine on Unix — the inode
survives until the process exits.

---

## Exit codes

| Code | Meaning |
|---|---|
| `0` | Success — valid, verified, or identical |
| `1` | Everything else: a failed check, a usage error, or an unreadable file |

Warnings never change the exit code. `verify` exits 0 on an episode with
dropped events or a foreign ABI, because both are real, expected conditions.

## `--json`

Every command except `abi` takes `--json` and emits exactly one object on
stdout, for CI and scripting:

```bash
$ rearguard validate p.yaml --json
{"path":"p.yaml","valid":true,"errors":[]}

$ rearguard show p.yaml --json
{"profile_id":"bench3","profile_version":"0.1.0","schema_version":1,"parent_profile":null,
 "embodiment":{...},"projection":{...},"isolation":{"inference_wcet_ms":null, ...},
 "critics":[...],"transfer":{...},"provenance":{...},
 "path":"p.yaml","valid":true,"errors":[]}

$ rearguard verify ep.mcap --json
{"path":"ep.mcap","ok":true,"event_stream_hash":"84a868ea6cd0573c","header_count":1,
 "event_count":20,"undecodable":0,"footer_present":true,"dropped_reported":0,
 "missing_events":0,"seq_regressions":0,"time_regressions":0,"kernel_abi_file":1,
 "kernel_abi_binary":1,"checks":[{"result":"ok","detail":"MCAP magic"}, ...]}
```

`verify --json` carries the verdict and counters but **no per-event array**, on
purpose: the event table has exactly one definition, and a second JSON-shaped
rendering of the same records is precisely the drift §12 warns about. Read the
MCAP directly if you need per-event data.

## Argument parsing is strict

An unknown flag, a repeated flag, a flag missing its value, or a stray
positional is an error — never a silent no-op:

```
$ rearguard replay --profile p.yaml --trace t.jsonl --ouput ep.mcap
error: unknown flag '--ouput'
```

Without this, that typo would run the replay, write no MCAP, and exit `0`. Same
reasoning as the profile JSON parser (see `edge/README.md`): a nearly-valid
invocation of a tool that produces safety evidence is not something to accept
quietly.

## Recipes

**Gate determinism in CI.** Fails if a change altered the event stream:

```bash
rearguard replay --profile p.yaml --trace t.jsonl --out new.mcap >/dev/null
rearguard diff golden.mcap new.mcap || exit 1
```

**Check a rig's joint order against its profile.** The ROS bring-up keeps its
golden order in `bringup/config/golden_joint_order.yaml`; hash that list and
compare it to what the profile declares, before anything runs:

```bash
RIG=$(rearguard hash --joints joint_1,joint_2,joint_3 --json | jq -r .joint_order_hash)
PROFILE=$(rearguard hash profile.yaml --json | jq -r .joint_order_hash_declared)
[ "$RIG" = "$PROFILE" ] || echo "joint order mismatch — this is what C8 rejects"
```

**Triage an episode off a robot.** Was it complete, did it lose evidence, was it
even produced by this kernel?

```bash
rearguard verify field_episode.mcap
```

**Prove an MCAP wasn't altered** since the run that produced it:

```bash
rearguard verify field_episode.mcap --expect-hash "$RECORDED_HASH"
```
