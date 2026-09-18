# A live TAP on the kernel's enforcement-event ring. Not the system of record.
#
# The record is the MCAP episode written in-process by the edge logger. This
# topic is best-effort and will drop under load; treating it as the
# provenance trail would quietly make DDS the custodian of evidence that
# threshold derivations depend on.
#
# The payload is deliberately opaque to ROS. Enforcement events are Protobuf,
# not CDR (enforcement.proto §9.4): they must be decodable by an ingest
# pipeline that has no ROS installation, and must not be pinned to any ROS
# distribution's IDL. The controller republishes the ring's bytes without
# parsing them, so protobuf decoding never touches the control loop.
#
# The envelope exists so the blob is self-describing. A bare byte array with
# its format documented in a comment somewhere is exactly how a decoder ends
# up silently misreading a stream after a schema change.

builtin_interfaces/Time stamp

# Fully-qualified name of the message type inside `data`.
# Currently always "harness.v1.EnforcementEvent".
string schema

# How records are framed inside `data`. Currently always
# "protobuf-varint-delimited": each record is a base-128 varint byte length
# followed by that many bytes of the serialised message, repeated until the
# buffer is consumed.
string encoding

# Events the kernel dropped for want of ring space since the previous
# publish. Also carried per-event as EnforcementEvent.dropped_since_last, so
# a consumer reading only the MCAP record still sees the loss.
uint32 dropped_since_last

# One or more framed records, exactly as drained from the kernel's ring.
uint8[] data
