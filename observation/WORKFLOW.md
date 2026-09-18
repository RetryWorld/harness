# Edge profile lifecycle

This implements the edge half of the product flow:

```text
local critic adapter → candidate window → pending profile review
    → user accepts failure + text guidance
    → MCAP + guidance generation request
    → external demonstration + critic/recovery artifacts + validation report
    → user approves the exact bundle → installed runtime configuration
    → adapter readiness check → local recovery supervision
```

Database connectivity, trained critic inference and the recovery controller
are explicit C++ placeholders in [native/runtime.hpp](native/runtime.hpp). They do not report readiness or
silently acknowledge actuator operations. The demonstration generator is an
external service: the edge exports its input package and ingests its output.
No model API is called and no behavior-cloning training is performed here.

The current CLI can complete review and local installation with externally
provided artifacts. Installed configurations report `installed_not_enforcing`.
Activation is refused while the inference/controller placeholders remain.
The existing effort-output kernel controller is not connected to SO-101's
position interface by this workflow. `Runtime` is a supervisory coordinator;
the eventual controller adapter must implement exclusive command ownership,
realtime gating, fallback and policy reentry for the associated controller.

## Start a persistent profile and a recording session

Build the updated local source on Linux after transferring it yourself:

```bash
cd ~/retry.world
source /opt/ros/jazzy/setup.bash
sudo apt-get install -y nlohmann-json3-dev libsqlite3-dev libssl-dev ros-jazzy-rosbag2-storage-mcap
cmake -S edge -B edge/build -DHARNESS_WITH_ROS2=ON -DHARNESS_BUILD_TESTS=ON
cmake --build edge/build -j 2
ctest --test-dir edge/build --output-on-failure
export ROS_DOMAIN_ID=71
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
CLI="$PWD/edge/build/rearguard"
STORE="$PWD/edge-profiles/so101"
SESSION=/tmp/so101-workflow-01

"$CLI" workflow init --store "$STORE" --config edge/observation/so101.json
"$CLI" observe start --config edge/observation/so101.json \
  --store "$STORE" --session "$SESSION" --domain-id 71 --storage mcap
```

Run `init` only once for a profile. The store survives simulator and observer
restarts; each observer session still needs a new directory. Other terminals
must set their own `CLI`, `STORE`, `SESSION` and ROS environment. The default
recording format is now MCAP and needs the ROS 2 MCAP storage plugin. If it is
unavailable, `--storage sqlite3` permits recording, but exporting a window to
MCAP still requires that plugin. No format fallback happens silently.

Run Foxglove, SO-101 and perturbations as in the existing walkthrough. No
Gazebo poses or perturbation events are passed to the critic adapter. It
receives topic/type/CDR samples with ROS receipt time and epoch, plus the robot
binding. The in-memory inference context is bounded to 20 ROS seconds and
16 MiB (whichever limit is reached first); it may be shorter than the disk
recording. Full evidence is recorded to disk independently.

The `CriticInferencePlaceholder.evaluate()` method returns no detections.
To exercise its downstream interface while a policy and sensors are live:

```bash
"$CLI" observe candidate --session "$SESSION" \
  --evidence-class target_moved --confidence 0.9 --before 10 --after 2
```

This explicitly records `source: test_injection`; it is not a model result or
proof of failure. A real adapter returns `Detection(detector_id,
evidence_class, confidence, before_s, after_s, source)` instead. Candidate
marks from the same detector/evidence class have a ten-second ROS cooldown.
The ordinary `observe capture` path also registers completed windows for
review when a store is attached, labeled as `operator_mark`.

Capture is refused if required streams are missing/invalid, their receipt
ages exceed ten wall seconds, source stamps are more than five ROS seconds
old (or one second in the future), or the clock has not advanced for ten
wall seconds. If the clock stops during collection, the window becomes
`incomplete` after fifteen wall seconds. A reset invalidates open windows.

## Candidate → confirmed failure and guidance

After `candidate_pending` appears in the observer terminal:

```bash
"$CLI" workflow show --store "$STORE"
```

Find the `c-...` ID in `candidates`. Its status is `pending_review`, with the
window, topic coverage, detector identity and confidence. Export the exact
window's recorded ROS messages and attach their content hash:

```bash
"$CLI" workflow export-window --store "$STORE" \
  --session "$SESSION" --candidate c-REPLACE
```

Recording is split into closed five-ROS-second segments, and a segment closes
when a window becomes ready. Clock epochs never share a segment. Export reads
closed segments for that epoch, selects the requested ROS timestamps, checks
required topic counts and writes a standalone MCAP. This works while recording
continues. It preserves source header timestamps inside the original CDR.

Review that MCAP in Foxglove before classifying the candidate. Save a JSON
payload file, for example `accept.json`:

```json
{
  "candidate_id": "c-REPLACE",
  "description": "Describe the failure actually observed",
  "guidance": "Describe the desired behavior in this edge case"
}
```

Then submit the review. Use the current `revision` from `workflow show`:

```bash
"$CLI" workflow apply --store "$STORE" --operation accept_failure \
  --payload accept.json --actor alex --expected-revision 2
```

This returns the failure ID. An unconfirmed or incomplete window cannot pass
this step. `reject_candidate` accepts `candidate_id` and `reason`. The legacy
`observe promote` command still belongs to the earlier session-only template
demo; use `workflow apply --operation accept_failure` for this persistent flow.

## Package MCAP + text for the external model

Create a request payload:

```json
{"failure_id": "f-REPLACE", "requested_model": "your-external-model-id"}
```

```bash
"$CLI" workflow apply --store "$STORE" --operation request_demonstration \
  --payload request.json --actor alex --expected-revision 3
"$CLI" workflow generation-request --store "$STORE" \
  --job job-REPLACE --out /tmp/so101-generation-01
```

The output directory contains `evidence.mcap` and `request.json`, with the
failure description, exact text guidance/revision, window, robot/topic
binding, requested model and immutable `input_hash`. The database/service
connector can eventually deliver this package; currently the command only
writes local files. The webapp or external generation/training service must
produce observation/action demonstrations and trained artifacts. A text plan
or narration alone is not a behavior-cloning demonstration.

## Ingest, review and install a returned behavior

Import each local file. `artifact` copies bytes into a content-addressed store
and returns `{sha256, size, kind}`. Use the returned references in the bundle.

```bash
"$CLI" workflow artifact --store "$STORE" --kind demonstration_mcap --file demonstration.mcap
"$CLI" workflow artifact --store "$STORE" --kind critic_model --file critic.model
"$CLI" workflow artifact --store "$STORE" --kind recovery_model --file recovery.model
"$CLI" workflow artifact --store "$STORE" --kind validation_report --file validation.json
```

The `submit_bundle` payload is:

```json
{
  "job_id": "job-REPLACE",
  "bundle": {
    "job_input_hash": "FROM_JOB",
    "binding_hash": "FROM_PROFILE",
    "generator": "service/model/version",
    "demonstration": {"sha256": "HASH", "size": 123, "kind": "demonstration_mcap"},
    "critic": {"sha256": "HASH", "size": 123, "kind": "critic_model"},
    "recovery": {"sha256": "HASH", "size": 123, "kind": "recovery_model"},
    "validation": {"sha256": "HASH", "size": 123, "kind": "validation_report"},
    "contract": {
      "controller_id": "so101",
      "evidence_class": "target_moved",
      "entry_predicate": "validated_target_displacement",
      "completion_predicate": "validated_target_reacquired",
      "fallback": "hold",
      "confidence_min": 0.9,
      "timeout_ms": 3000,
      "retry_budget": 1
    }
  }
}
```

These contract values illustrate the format; they are not validated SO-101
settings. Predicate names are identifiers the controller adapter must support,
not executable expressions. Opaque model files are never deserialized here.

The validation report must contain `passed: true`, a nonempty `validator`,
`binding_hash`, `job_input_hash`, `demonstration_sha256`, `critic_sha256`,
`recovery_sha256`, and `contract_hash`. Each must match the submitted bundle.
`contract_hash` is SHA256 of UTF-8 JSON with sorted keys, separators `,` and `:`,
and no NaN values (the `workflow.digest` helper). This verifies provenance
consistency, not the truth of an external validation claim. The edge does not
run generated demonstrations in a simulator or certify task safety. MCAP
imports check their file envelope and identity; review/validation must also
check their contents and whether observations/actions form usable examples.

```bash
"$CLI" workflow apply --store "$STORE" --operation submit_bundle \
  --payload bundle.json --actor generation-service --expected-revision 4
```

The returned bundle is `pending_approval`. After reviewing the demonstration
and validation results, approve its exact `content_hash`:

```json
{"bundle_id": "b-REPLACE", "content_hash": "EXACT_REVIEWED_HASH"}
```

```bash
"$CLI" workflow apply --store "$STORE" --operation approve_bundle \
  --payload approval.json --actor alex --expected-revision 5
```

Finally, `deploy` takes `{"bundle_id": "b-REPLACE"}`. It checks all artifact
hashes again and atomically changes the installed deployment pointer:

```bash
"$CLI" workflow apply --store "$STORE" --operation deploy \
  --payload deployment.json --actor alex --expected-revision 6
"$CLI" observe activate --session "$SESSION"
```

The last command currently returns an explicit error because the critic and
controller adapters are placeholders. Installation is real; actuator control
is unavailable. Approval cannot enable placeholders. Old deployments remain
in history; redeploying a still-approved bundle is an explicit rollback.
`deactivate` takes a `reason` and clears the installed pointer. Revising guidance
requires deactivation of that failure's installed behavior first and marks its
older jobs/bundles `superseded`.

## Runtime adapter contract

The critic adapter must load the approved model and return detections for the
configured observation binding. The controller adapter's `prepare` must
verify controller identity, command interface, exclusive authority and all
predicate/fallback identifiers. `begin` must suspend the nominal policy and
acquire recovery authority; `poll` returns `running`, `failed`, or `succeeded`
only after the approved completion predicate passes. `resume` acknowledges
the handoff back to nominal control. `hold` acknowledges applied fallback.

`Runtime` handles rising-edge critic evidence, confidence thresholds, bounded
retries, monotonic-time deadlines, completion/reentry, stale inputs, clock
reset and installed-profile changes. It latches fallback and never reports
that hold succeeded without an acknowledgement. Tests use explicit fake
adapters. Production adapter readiness must remain false until the real
controller path is installed and verified. The observer's supervisory loop
runs approximately once per wall second; hard timing guarantees and command
gating belong in the controller's realtime loop, not in this C++ recorder.

## Future database/webapp connection

`workflow.sqlite3` is local durable state. A transaction commits the profile
revision, request receipt and outbox event together. Retrying the same exact
`request_id` returns the original result, even after process restart. Reusing
that ID for different content fails; a stale `expected_revision` fails without
changing the profile. Supply `--request-id` when retrying a CLI mutation.

```bash
"$CLI" workflow outbox --store "$STORE" --after 0
```

The output includes sequence numbers and up to 100 events with full profile
snapshots; use the last sequence as the next cursor. No delivery is claimed,
no events are deleted, and no remote acknowledgement is fabricated. The
`DatabaseSyncPlaceholder` names the missing connector explicitly.

The observer publishes profile revisions as JSON `std_msgs/msg/String` on
`/harness/observation/s_<session-id>/profile`, reliable/transient-local depth 1,
and records them in its bag. Incoming review requests can use the existing
session `/requests` and `/responses` topics with `operation: workflow` and a
`workflow_request` containing the same versioned request envelope used by
`Store.apply`. Local CLI mutations use the same store; the observer picks up
new revisions. The profile's `enforcement_status` remains `unavailable` with
the shipped placeholders; observer `status.runtime` is the live coordinator
status. Actor names are audit labels on this trusted local interface, not
authentication. The future remote connector must authenticate reviewers and
bind their decisions to these exact request/revision/hash fields.

Local tests cover persistence, idempotency, conflicts, provenance, approval,
stale capture, clock epochs and runtime transitions with fake adapters. This
new segmented recording/export path still needs the Linux ROS 2 acceptance
run; no files were transferred to Linux by the agent.
