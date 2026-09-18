# Native tiny critic

All new edge critic code is C++. Python model definition, training, Modal jobs,
and export live in [`models/critic`](../../models/critic/README.md).

The native library loads separate vision and temporal-head engines, validates
model hashes and exact tensor shapes, caches a fixed-size feature history, and
emits timestamped experimental scores. A bounded worker utility replaces queued
work with the latest request and invalidates results across resets or expiry.
Inference is not part of the realtime kernel ABI.

The current executable supports completed MCAP replay and canonical-tensor
diagnostics. **It is not yet connected to the live observer's candidate adapter
or approved recovery supervision.** Existing activation remains blocked. Model
scores are uncalibrated and cannot establish controller readiness.

## Build on Jetson

Use the TensorRT version provided by the installed, supported JetPack stack.
The backend targets the TensorRT 10 named-tensor/enqueueV3 API. Generate the
repository's protobuf code and install the existing edge dependencies first,
as described in the edge README.

```bash
source /opt/ros/jazzy/setup.bash
cmake -S edge -B edge/build \
  -DHARNESS_WITH_ROS2=ON -DHARNESS_WITH_TENSORRT=ON \
  -DHARNESS_BUILD_TESTS=ON
cmake --build edge/build -j 2
ctest --test-dir edge/build --output-on-failure
```

Build engines from the exported bundle **outside active policy operation**.
The following assumes `trtexec` is available on PATH from the target SDK:

```bash
trtexec --onnx=/models/critic/vision.onnx \
  --saveEngine=/models/critic/vision.engine --fp16 \
  --memPoolSize=workspace:32 --skipInference
trtexec --onnx=/models/critic/head.onnx \
  --saveEngine=/models/critic/head.engine --fp16 \
  --memPoolSize=workspace:32 --skipInference
edge/build/harness_critic seal-engines /models/critic
```

Sealing records the engine hashes against the exported manifest. It establishes
identity, not numerical parity or physical validation. Engine deserialization
and IO checks fail for incompatible engines; engines are specific to the target
hardware/software stack. Verify TensorRT/PyTorch accuracy after conversion.
The graph IO must remain linear float32; FP16 is permitted internally.

The backend preallocates device IO buffers and binds tensors during loading.
It uses a low-priority nonblocking CUDA stream and synchronizes only its own
stream. This is a scheduling hint, not isolation from the VLA. TensorRT runtime
workspace, context memory, CPU decoding, and shared memory bandwidth must all
be measured alongside the policy. No Jetson timing or memory result is claimed
from the Mac CPU tests.

## Replay a recorded window

```bash
edge/build/harness_critic replay /models/critic tensorrt \
  /data/evidence.mcap edge/observation/so101.json
```

Each output line contains `t_ns`, `epoch`, validity status, evidence scores,
and `source: experimental_critic`. No controller commands are generated.
Missing/stale required streams emit `insufficient_evidence`; warmup emits
`warming_up`. Scores only appear for valid windows. The evidence scores use a
sigmoid and are explicitly not calibrated confidence.

The native MCAP reader uses ROS2 `rosbag2_cpp` and CDR decoding. Use completed
single-epoch Harness exports, not a recording mixing reset clocks. It accepts
raw `sensor_msgs/Image`, `JointState`, `Float64MultiArray` commands, `Imu`, and
`WrenchStamped`. Arbitrary sensor schemas need explicit additional decoders.
Images are encoded at the configured sampling rate; reused frames reuse cached
features. Required-topic freshness checks apply to receipt and source stamps.

## Native CPU development backend

Install an ONNX Runtime C++ SDK separately, then point CMake to its headers and
library. This does not embed or invoke Python:

```bash
cmake -S edge -B edge/build \
  -DHARNESS_WITH_ONNX=ON \
  -DONNXRUNTIME_INCLUDE_DIR=/opt/onnxruntime/include \
  -DONNXRUNTIME_LIBRARY=/opt/onnxruntime/lib/libonnxruntime.so
cmake --build edge/build -j 2
```

The same replay command can use `onnx` instead of `tensorrt` when ROS2 was also
enabled. Without ROS2, the native `score` and `encode` diagnostics still work:

```bash
edge/build/harness_critic inspect /models/critic
edge/build/harness_critic score /models/critic onnx /data/window.f32
edge/build/harness_critic encode /models/critic onnx /data/image.rgb 640 480 /data/features.f32
```

`window.f32` is a little-endian float32 concatenation of the exported window's
`vision`, `camera_mask`, `sensors`, `joint_mask`, and `step_mask` tensors, each
in row-major order without a batch dimension. Dimensions come from the hashed
bundle contract. It is a diagnostic interchange, not a replacement for MCAP.
`encode` accepts tightly packed RGB8 and writes 5×256 float features.

Both inference backends are opt-in. The default edge build still compiles and
tests the new tensor, preprocessing, and worker code without either SDK.

## Remaining acceptance work

The native CPU path and Python export path are tested locally. The ROS2 MCAP
and TensorRT implementations require Linux/Jetson compilation and acceptance
testing. The Modal entry point has not been executed. Before connecting this
critic to live candidate generation, finish the worker/observer wiring, train
on reviewed real failure windows, validate cross-embodiment behavior, and measure
policy latency and memory under concurrent load. Runtime activation additionally
requires the existing separate controller and approval contracts.
