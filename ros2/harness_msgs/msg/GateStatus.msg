# One gate decision. Published every control cycle, best-effort.
#
# This is the OPERATIONAL view: what the gate did, in a form a system
# integrator can wire into their own logic without decoding protobuf. It is
# deliberately not the evidence record — that is EnforcementEvents, and the
# system of record is the MCAP episode, not either topic.
#
# `candidate` and `emitted` are positional and are indexed by the
# joint_names in HarnessInfo on this controller's latched ~/info topic.
# Reading them without that ordering is a mistake the wire format cannot
# stop you from making.

builtin_interfaces/Time stamp

# What the kernel decided this cycle. Mirrors hk_decision in
# edge/include/harness/harness_kernel.h.
#
# NOTE: in BAND_B, DECISION_CLAMP does NOT mean "clamped to a bound" — the
# kernel reports it for an on_reject=ZERO fallback. Interpret `decision`
# together with `band`, never alone.
uint8 DECISION_ADMIT=0
uint8 DECISION_CLAMP=1
uint8 DECISION_HOLD_LAST=2
uint8 DECISION_REJECT=3
uint8 decision

# Which band holds authority. Mirrors hk_band. Band B preempts Band A
# unconditionally, in kernel code — it is not configurable.
uint8 BAND_NOMINAL=0
uint8 BAND_A=1
uint8 BAND_B=2
uint8 band

# What the model asked for, before the gate.
float64[] candidate

# What the gate actually wrote to the command interface. This is the
# authoritative "what reached the actuator" on this topic — unlike
# harness.v1.ProjectionEvent.emitted, which is recorded before the band
# fallback is applied and therefore does not reflect Band B output.
float64[] emitted

# Enforcement events the kernel could not fit in its ring this cycle.
# Non-zero means evidence was lost, never that enforcement was skipped:
# dropping evidence must never block the gate.
uint32 dropped_events
