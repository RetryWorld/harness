# Live observation and pending proposals

For the persistent candidate → failure/guidance → demonstration → approval →
deployment flow, see [the edge profile workflow](WORKFLOW.md). It adds a local
transactional profile store, MCAP window export and explicit integration ports.
The original session-only commands below remain available.

`rearguard observe` runs a ROS 2 observer alongside the policy. It records
camera images, measured joints and issued actions continuously into rosbag2.
An operator marks a window, reviews its evidence, then promotes it to a failure.
Promotion creates two pending review drafts: one for a critic and one for a
recovery. Neither draft contains executable code or changes the active profile.

This is an operator-assisted workflow. An empty critic list does not detect
semantic failures automatically. A perturbation alone is not proof that a task
failed. The capture trigger is explicitly `operator_mark`; promotion records
the operator's identity and description. Draft generation uses a fixed review
template, not a VLM or an inferred recovery policy.

## Build

On the Linux ROS 2 Jazzy machine, after copying the changed source:

```bash
cd ~/retry.world
source /opt/ros/jazzy/setup.bash
sudo apt-get install -y nlohmann-json3-dev libsqlite3-dev libssl-dev ros-jazzy-rosbag2-storage-mcap
cmake -S edge -B edge/build -DHARNESS_WITH_ROS2=ON -DHARNESS_BUILD_TESTS=ON
cmake --build edge/build -j 2
ctest --test-dir edge/build --output-on-failure
```

Use `~/retry.world/edge/build/rearguard` below. An older installed
`~/.harness/bin/rearguard` may contain an older implementation. Observation
and workflow are native C++ under [native/](native/), called directly by the
CLI; no Python interpreter or Python adapter is launched or installed.
`HARNESS_WITH_ROS2=ON` compiles the `rclcpp`/`rosbag2_cpp` integration, and its
binary requires the sourced ROS environment. The default `OFF` build supports
offline profile review and saved session inspection without ROS; live commands
and MCAP export explain how to enable the ROS build. CMake installs only the
example JSON configuration under `share/harness/observation`.

Existing SQLite stores, artifact references, request receipts and session JSON
remain readable. Use a fresh session directory for new recordings. All
observation commands print JSON; ROS may also print its own log messages.

The native implementation is split into `common` (JSON/hash/config handling),
`evidence` (window timing and coverage), `store` (SQLite review transactions),
`runtime` (adapter interfaces and supervision), `recording` (ROS bag segments
and MCAP export), `ros` (subscriptions and request transport), and `commands`
(CLI dispatch). ROS libraries link to the observation layer, never the kernel.

CTest runs the C++ evidence/workflow tests. A ROS-enabled build also runs a real
MCAP round-trip test. To exercise a local DDS observer with synthetic messages,
run `edge/build/harness_observation_ros_tests --live` after sourcing ROS. It uses
domain 97 and temporary files; it does not drive a robot or replace the SO-101
simulation check.

## SO-101 walkthrough

In **each Linux terminal**, use the same ROS environment as the bench:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=71
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd ~/retry.world
CLI="$PWD/edge/build/rearguard"
SESSION=/tmp/so101-observation-01
```

Choose a new `SESSION` directory for each attempt. Existing evidence is never
overwritten. The observer does not need the controller workspace overlay and
may start before the simulator. If you use a nondefault RMW implementation,
export the same `RMW_IMPLEMENTATION` in each terminal.

**Terminal 1: inspect the empty observation profile and activate the observer.**

```bash
"$CLI" observe profile --config edge/observation/so101.json
"$CLI" observe start --config edge/observation/so101.json \
  --session "$SESSION" --domain-id 71 --wall-timeout 1800
```

The profile shows `mode: observation_only`, `critics: []`, `recoveries: []`,
the joint order and the actual topic bindings. This is an observation
configuration, **not** a kernel `HarnessProfile` and cannot be passed to
`rearguard validate` or used as an actuator enforcement profile.

**Terminal 2: run the policy and simulator.**

```bash
./sim/robot_bench/run.sh --robot so101 --task pick_place \
  --domain-id 71 --headless --record
```

Use `--gui` from the Ubuntu desktop to see Gazebo. `--observer-view side`
adds `/observer_camera/image_raw` for human viewing. `--validate-policy`
additionally runs the benchmark evaluator; its private object-pose topic
is never consumed by this observer. The bench's existing completion/timeout
behavior still applies. `--sim-only` is useful for viewing the scene, but
cannot produce a promotable action/sensor window without a policy publishing
commands. The observer remains alive when the bench exits.

**Terminal 3: check coverage before disturbing the live run.**

```bash
"$CLI" observe status --session "$SESSION"
```

Wait until all four topics have publishers and increasing message counts.
`heartbeat_age_wall_s` should be small. Then, while the policy is attempting
the pick:

```bash
./sim/robot_bench/perturb.sh push_object
"$CLI" observe capture --session "$SESSION" --before 5 --after 3 \
  --label "Inspect the pick attempt around the observed cube displacement"
"$CLI" observe windows --session "$SESSION"
```

`perturb.sh` resolves the latest bench run and its Gazebo partition as before;
use its `--run` option for concurrent runs. The harness never reads that
partition, the perturbation log, `/robot_bench/events`, or simulator poses.

Capture returns a `w-...` ID with `status: collecting`. Five seconds of
pre-context and three seconds of post-context are requested in **ROS time**.
On a slow simulator, collecting three simulated seconds may take much longer
than three wall seconds. Wait for `window_ready` in Terminal 1, or run
`windows` again until its status is `ready`.

Review the actual camera view and the window's topic coverage. If it depicts
a failure, promote that specific ID, describing what you observed:

```bash
"$CLI" observe promote --session "$SESSION" --window w-REPLACE_WITH_ID \
  --operator alex --description "REPLACE with the failure observed in this window"
"$CLI" observe proposals --session "$SESSION"
"$CLI" observe profile --session "$SESSION"
```

The observer acknowledges the failure ID over ROS 2. `proposals` shows two
`pending` drafts linked to that failure and window; `executable` and
`acceptance_enabled` are false. The profile still contains zero critics and
recoveries. Stop here for this stage. Ctrl-C Terminal 1 closes the rosbag.

## Evidence and ROS contract

The session contains `session.json` (bindings/domain/RMW/config), `state.json`
(heartbeat, coverage, windows, failures, proposals), `events.jsonl` (audit
events), and `rosbag2/` (closed recording segments containing received original
messages, clock and audit events). `recording.json` indexes closed segments by
clock epoch and timestamp. New runs default to MCAP (`--storage sqlite3` remains
available); ROS needs its MCAP storage plugin for MCAP recording/export.
Recording is continuous for this session; windows reference its timestamps.
Retain the entire directory. This first implementation has no automatic
retention rotation: choose a bounded `--wall-timeout` and manage saved sessions.
It checks free space and stops below 250 MB rather than intentionally filling
the disk. Unclosed/crashed sessions must be treated as incomplete; check the
heartbeat, observer status and rosbag integrity before relying on them.

Bag timestamps are ROS receipt time; original message headers preserve source
time. Window coverage also records monotonic receipt times. Headerless action
arrays have `source_ns: null`: they cannot establish VLA inference latency.
The configured joint order supplies their otherwise unnamed joint binding.
For a VLA publishing named, stamped trajectories, configure
`trajectory_msgs/msg/JointTrajectory` instead; this adapter expects full joint
position targets in configured order. Inference-start/finish telemetry and
the precise image a VLA consumed require instrumentation at that policy's
boundary; observing its output and camera topics alone cannot establish them.

Subscriptions use the ROS sensor-data QoS (best effort, volatile) to receive
ordinary sensor and reliable action publishers without requiring those
publishers to change. This may drop messages. Coverage is the evidence
actually received, not a lossless-capture guarantee. At least two valid
messages from every required topic must occur within a window for promotion.
Inspect `observed_span_ns`, first/last stamps and counts: this minimum check
does not guarantee uninterrupted coverage or certify the failure description.
A backwards ROS clock jump starts a new epoch and invalidates windows still
collecting. Shutdown marks unfinished windows `incomplete`.
Capture now checks clock/source/receipt freshness, and a clock stall marks
open windows incomplete after fifteen wall seconds. An empty session argument
is rejected before any files are created.

Capture/promotion requests and acknowledgements use `std_msgs/msg/String`
with a version-1 JSON envelope on session-specific
`/harness/observation/s_<session-id>/requests` and `/responses`. Requests carry
a unique `request_id`; the client retries that ID while waiting up to 15 wall
seconds. Recent requests are deduplicated; repeated promotion of a window
returns the original failure. Audit events publish on the matching `/events`
topic and are also recorded. Local `status`, `windows` and `proposals` are
artifact queries and still work after the observer stops.

## Physical robot and local VLA

Copy `so101.json`, set `clock` to `ros_system`, and configure the robot's camera,
joint and policy-output topic names, message types and joint order. Run in the
robot's actual ROS domain and middleware environment. No Gazebo library,
policy source import, action publisher, controller-manager service or actuator
interface is used by this component. Task semantics are supplied by the
operator at promotion; switching from the bench controller to a local VLA
does not silently change that rule.

Implemented locally with core unit tests and a C++ CLI build. Live Jazzy
recording and SO-101 perturbation acceptance remain to be run on Linux.
