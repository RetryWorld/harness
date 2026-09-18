# schemas — single source of truth

One definition, four consumers, one direction of codegen. Nothing here imports
from any package; every package generates from here.

```
proto/harness/v1/enforcement.proto
  ├─→ sim/sim_rig/schemas/enforcement_pb2.py    protobuf-python   [wired]
  ├─→ backend/                                       shares the _pb2   [wired]
  ├─→ edge/src/generated/                            protoc --cpp_out  [stub]
  └─→ app/src/lib/generated/                        ts-proto          [stub]
```

Regenerate with `../scripts/gen_proto.sh` from the repo root.

## Rules

**Generated code is committed.** A build-time regeneration could produce a
decoder that disagrees with the schema already embedded in MCAP files shipped to
customers. The schema record inside a `.mcap` is a serialised FileDescriptorSet;
once a file is out, its wire format is frozen whether we like it or not.

**Field numbers are permanent.** Never reuse, never renumber. A threshold shipped
to a customer is read back by a decoder that may be older than the writer.

**Nothing edits its own bindings.** If a consumer needs a field, it goes in the
`.proto` and everyone regenerates.

## Not yet here

The Harness Profile itself, and the replay tuple. Both are currently Python
dataclasses in `sim/sim_rig/`, which is fine while `sim/` is the only writer
and wrong the moment the edge kernel or the registry needs them. The natural home is a
JSON Schema definition here, generating C++ (or a hand-written reader, as the
profile parser already is), Pydantic and TypeScript.

Do that move when the second consumer appears, not before — but do it before the
registry stores its first profile, because migrating a populated registry to a
new schema source is the expensive version of this.
