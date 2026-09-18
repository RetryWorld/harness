# harness_msgs — the ROS-facing interface contract for the harness gate.
#
# Deliberately a separate package from harness_controller, and deliberately
# NOT part of edge/'s extractable unit in spirit even though it sits under
# edge/ros2/:
#
#   - The kernel (edge/) has no ROS dependency at all and must keep none;
#     its contract is the C ABI header. Nothing here is included by it.
#   - These messages are versioned on their own. A customer integrating
#     against GateStatus needs it to be stable across kernel patch releases,
#     which is a different cadence from HK_ABI_VERSION.
#   - rosidl output is per-distribution and not binary-compatible across
#     them. That coupling is acceptable for an operational interface a system
#     integrator wires into their own stack, and is exactly why the EVIDENCE
#     stream stays Protobuf instead (see EnforcementEvents.msg).
cmake_minimum_required(VERSION 3.22)
project(harness_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HarnessInfo.msg"
  "msg/GateStatus.msg"
  "msg/EnforcementEvents.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
