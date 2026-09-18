# Mock bring-up

Development plan **Step 4**: run the harness controller against
`mock_components/GenericSystem` — real `controller_manager`, real executor,
real DDS, real lifecycle, real update loop, **no simulator**.

This separation is the point. It isolates *"does the gate run correctly
inside ros2_control"* from *"does the robot behave"*, which are different
failures with different fixes. Gazebo comes after this passes, not before.

```
ros2 launch harness_controller harness_mock.launch.py     # terminal 1
ros2 run harness_controller harness_flow.py               # terminal 2
```

## Topology

```
/upstream_effort_controller/commands   (Float64MultiArray — you publish here)
         |
         v
upstream_effort_controller             ForwardCommandController
         |   writes into the harness's exported reference interfaces,
         |   "harness_controller/joint_N/effort"
         v
harness_controller                     hk_gate runs here
         |   writes the GATED effort
         v
joint_N/effort                         mock_components/GenericSystem
```

Nothing reaches the hardware effort interface except through the gate:
`harness_controller` claims those interfaces exclusively, and
`controller_manager`'s resource manager enforces that. It is not a matter of
convention or of everyone remembering to publish to the right topic.

The chaining itself is a string match — `ForwardCommandController` claims
`"<joints[i]>/<interface_name>"`, which must resolve to the reference
interface name the harness exports. Get it wrong and activation fails loudly
rather than quietly bypassing the gate.

## What you'll see

`harness_flow.py` publishes where a policy would publish and decodes what a
customer's ingest would decode, then narrates it:

| Command | Decision | What it demonstrates |
|---|---|---|
| 12.5 Nm | `ADMIT` | The gate is transparent when nothing is wrong. `emitted == candidate`, bit for bit. |
| 39.0 Nm | `ADMIT` | `margin` collapsing toward zero — the quantity a threshold is later derived from, logged on admitted cycles too. |
| 250 Nm | `CLAMP` → `TRANSFER` | Projection clamps mechanically on *every* violating cycle; separately, the violation *rate* trips Band B. |
| 5.0 Nm after that | still `0.0` | Band B is terminal. Good behaviour does not win authority back. |

Two topics, deliberately different in kind:

- **`/harness_controller/decision`** — a cheap per-cycle mirror of
  `hk_gated_command`. Says *what* happened.
- **`/harness_controller/enforcement_events`** — varint-length-prefixed
  `harness.v1.EnforcementEvent` protobufs, drained straight from the kernel's
  event ring and republished as raw bytes. Says *why*: which predicate
  failed, the signed margin, sequence numbers, and how many events were
  dropped for want of ring space. This is what lands in an MCAP episode.

The controller republishes those bytes **without parsing them** — protobuf
decoding never touches the control loop. Decoding happens in the demo script,
which is exactly the split the design intends.

## Things worth knowing before you're surprised by them

**Band B trips after ~250 ms of sustained violation** (measured: 50 clamped
cycles). The window looks back 500 ms — ~100 samples at 200 Hz — and the
trigger needs the violating fraction of that whole window to exceed
`band_b_reject_rate_max`, so more than half the window must be violations.
`kRejectRateMinSamples` (= 4) is the minimum needed to *evaluate* the rate, not
to trip it; conflating the two predicts a transfer roughly 12x sooner than
happens. Set `band_b_reject_rate_max: 1.0` to make the trigger unreachable and
watch clamping alone.

**In Band B, `ProjectionEvent.emitted` does not match what the actuator got.**
`kernel_gate.cpp` emits the PROJECTION event before the Output block applies
the band fallback, so `emitted` carries the clamped projection (e.g. 40.0)
while the actuator receives the `on_reject` output (0.0). `enforcement.proto`
defines that field as "what actually reached the actuator", so this is a
provenance defect, not a display quirk: a threshold derived from an episode
that entered Band B would be derived from values that never reached the robot.
`harness_flow.py` cross-checks the two topics and prints a PROVENANCE MISMATCH
warning when it sees this.

**Band B is terminal in this ABI.** `kernel_gate.cpp` leaves Band B only via a
Band A reentry, and Band A needs a critic this rehearsal profile does not
declare. Once transferred, every cycle emits the `on_reject` output until the
controller is reconfigured:

```
ros2 control set_controller_state harness_controller inactive
ros2 control set_controller_state harness_controller active
```

That rebuilds a fresh `hk_handle` through `on_configure`. `harness_flow.py`
detects a controller already in Band B at startup and tells you this rather
than showing you a screen of zeros.

**NaN is not gated.** `hk_gate` has no finiteness predicate — `margin =
min(NaN, NaN)` is NaN and `NaN < 0.0` is false — so a NaN command is
`ADMIT`ted unclamped, cached as `last_admitted`, and logged with
`margin=+inf`. `±inf` *is* handled (it clamps). Run `harness_flow.py --repl`
and enter `nan` to watch it. Pinned by
`test_gate_loop.cpp::nan_reference_is_admitted_ungated`.

**Nothing here is derived from anything.** `test_torque_limit_nm: 40.0` and
the Band B thresholds are rehearsal constants chosen to make behaviour
visible. A bound without a derivation tuple is precisely what constraint C1
exists to refuse — none of these values may be copied into a real Harness
Profile.

## Files

| Path | What it is |
|---|---|
| `urdf/harness_mock.urdf` | 3-joint arm on `mock_components/GenericSystem`. |
| `config/harness_controllers.yaml` | `controller_manager` config and both controllers. |
| `config/golden_joint_order.yaml` | The §8 golden order. A mismatch aborts `on_configure` — no remap, no warning-only path. |
| `launch/harness_mock.launch.py` | Brings it all up, in the required order. |
| `scripts/harness_flow.py` | The narrated walkthrough / REPL. |

The launch file injects `golden_joint_order_file` at launch time rather than
committing a machine-specific absolute path, and spawns the harness
controller *before* the upstream one — the reference interfaces must exist
before anything can claim them, and spawning concurrently is a race that
fails roughly one run in five with an error that reads like a typo.
