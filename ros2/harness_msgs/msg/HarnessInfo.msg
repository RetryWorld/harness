# Identity of a running harness gate. Published ONCE per activation on a
# latched (transient-local, depth 1) topic, so anything that subscribes later
# still receives it.
#
# This exists so GateStatus can stay lean. The positional float64[] arrays in
# GateStatus are uninterpretable without a joint ordering, and republishing a
# string[] of joint names at the controller update rate would be wasteful and
# still wouldn't prove the ordering was the one the profile was validated
# against. The ordering lives here, once, next to the hash that pins it.
#
# Same reasoning as harness.v1.EpisodeHeader: an episode whose joint ordering
# cannot be reconstructed is not usable for threshold derivation.

# Which profile is loaded, and which embodiment it was scoped to. Thresholds
# are embodiment-scoped and non-inheriting (constraint C9), so profile
# identity alone is not enough to interpret a bound.
string profile_id
string profile_version
string embodiment_class_id

# Ordered joint names for every gated joint, and the FNV-1a hash over them
# that the profile validator checks (constraint C8). Every positional array
# in GateStatus is indexed by this ordering.
string[] joint_names
string joint_order_hash

# The declared output region, per joint, in the units of the output
# interface. Same length and ordering as joint_names.
float64[] output_region_min
float64[] output_region_max

# What the gate writes. Mirrors harness.v1.OutputInterface.
uint8 OUTPUT_INTERFACE_UNSPECIFIED=0
uint8 OUTPUT_INTERFACE_EFFORT=1
uint8 OUTPUT_INTERFACE_POSITION=2
uint8 OUTPUT_INTERFACE_VELOCITY=3
uint8 output_interface

# Kernel C ABI version reported by hk_abi_version() at load. A consumer that
# does not recognise this must not interpret the bounds above.
uint32 kernel_abi_version

# Where Band B hands authority, if it fires.
string fallback_node
