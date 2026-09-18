# harness_controller — ament_cmake package.
#
# The harness kernel itself is NOT vendored or rebuilt by this CMakeLists: per
# the build brief's non-negotiable #5, the kernel is built inside the
# container from sim/gazebo_rig's caller (which calls
# harness_client.build.ensure_kernel(), the existing source-hash cache), and
# this package is pointed at that build's output via two cache variables:
#
#   -DHARNESS_KERNEL_INCLUDE_DIR=<KernelBuild.header's parent dir>
#   -DHARNESS_KERNEL_LIB=<KernelBuild.library>
#
# This package never does `add_subdirectory(edge)` or similar path reference
# (edge/README.md, "What the sim must never do": "the rig would exercise a build no customer ever
# receives") — it links the same .so ensure_kernel() just built and tested.
cmake_minimum_required(VERSION 3.22)
project(harness_controller LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED HARNESS_KERNEL_INCLUDE_DIR OR NOT DEFINED HARNESS_KERNEL_LIB)
  message(FATAL_ERROR
    "HARNESS_KERNEL_INCLUDE_DIR and HARNESS_KERNEL_LIB must be set — this "
    "package links a kernel build produced by "
    "harness_client.build.ensure_kernel(), never its own copy.")
endif()

# The kernel artifact ships only include/harness/ + share/proto/ (see
# edge/scripts/build_artifact.sh) — no compiled protobuf library. This
# package builds the HarnessProfile it hands to hk_create() itself, so it
# compiles the same committed, buf-generated sources edge/CMakeLists.txt
# does (schemas/gen/cpp, never regenerated here) against its own protobuf.
# Default assumes an in-monorepo checkout (edge/ros2/harness_controller ->
# ../../../schemas/gen/cpp); override HARNESS_SCHEMAS_GEN_CPP once this
# package is extracted standalone.
set(HARNESS_SCHEMAS_GEN_CPP "${CMAKE_CURRENT_SOURCE_DIR}/../../../schemas/gen/cpp" CACHE PATH
    "Path to schemas/gen/cpp, buf-generated (gitignored output, not committed).")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(harness_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# See edge/CMakeLists.txt for why this isn't a plain CONFIG find_package:
# apt's libprotobuf-dev (Ubuntu 24.04, the ROS 2 Jazzy reference environment)
# ships no protobufConfig.cmake, only pkg-config files.
find_package(protobuf CONFIG QUIET)
if(NOT protobuf_FOUND)
  find_package(Protobuf REQUIRED)
endif()

add_library(harness_proto OBJECT
  ${HARNESS_SCHEMAS_GEN_CPP}/harness/v1/profile.pb.cc
  ${HARNESS_SCHEMAS_GEN_CPP}/harness/v1/critic.pb.cc
  ${HARNESS_SCHEMAS_GEN_CPP}/harness/v1/enforcement.pb.cc
  ${HARNESS_SCHEMAS_GEN_CPP}/harness/v1/provenance.pb.cc
)
target_include_directories(harness_proto PUBLIC ${HARNESS_SCHEMAS_GEN_CPP})
target_link_libraries(harness_proto PUBLIC protobuf::libprotobuf)
target_compile_options(harness_proto PRIVATE -w)
# harness_controller (below) is a SHARED library linking harness_proto's
# objects — unlike edge/CMakeLists.txt's own harness_proto, which inherits
# PIC from that project's global CMAKE_POSITION_INDEPENDENT_CODE ON, this is
# a separate CMake project and needs it declared explicitly, or the link
# below fails with "recompile with -fPIC".
set_target_properties(harness_proto PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(harness_controller SHARED
  src/harness_controller.cpp
)
# The installed .so has no default RPATH to libharness_kernel.so — that lib
# lives in ensure_kernel()'s content-addressed cache dir, not any standard
# linker search path. Without this, the BUILD-tree binary (and this
# package's own gtest, run via `colcon test` straight from the build dir)
# finds it fine via CMake's automatic build-tree RPATH, but the INSTALLED
# library controller_manager dlopen()s at runtime does not — verified: it
# fails with "libharness_kernel.so.1: cannot open shared object file" only
# after `colcon build`'s install step, never during the build/test step.
get_filename_component(_harness_kernel_lib_dir "${HARNESS_KERNEL_LIB}" DIRECTORY)
set_target_properties(harness_controller PROPERTIES
  INSTALL_RPATH "${_harness_kernel_lib_dir}"
  BUILD_WITH_INSTALL_RPATH FALSE
)
target_include_directories(harness_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${HARNESS_KERNEL_INCLUDE_DIR}
)
if(TARGET yaml-cpp::yaml-cpp)
  set(_yaml_cpp_target yaml-cpp::yaml-cpp)
else()
  set(_yaml_cpp_target ${YAML_CPP_LIBRARIES})
  target_include_directories(harness_controller PRIVATE ${YAML_CPP_INCLUDE_DIR})
endif()

# One keyword-style target_link_libraries call for everything. CMake forbids
# mixing keyword- and plain-signature target_link_libraries calls on the same
# target, and ament_target_dependencies() (used elsewhere in the ROS 2
# ecosystem) issues a PLAIN-signature call internally — combining it with
# ${HARNESS_KERNEL_LIB}/yaml-cpp's keyword-style calls above errors with
# "keyword signature ... has already been used". Modern rclcpp/
# controller_interface/hardware_interface/pluginlib/realtime_tools/std_msgs
# all export proper `<pkg>::<pkg>` imported targets, so this needs no
# ament_target_dependencies() call at all.
target_link_libraries(harness_controller PRIVATE
  ${HARNESS_KERNEL_LIB}
  ${_yaml_cpp_target}
  harness_proto
  protobuf::libprotobuf
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
  ${std_msgs_TARGETS}
  ${harness_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface harness_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS harness_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Mock bring-up: URDF, controller config, launch file and the flow demo.
# Development plan Step 4 runs the controller against
# mock_components/GenericSystem before any simulator — real controller
# manager, real executor, real DDS, no physics.
install(
  DIRECTORY bringup/urdf bringup/config bringup/launch
  DESTINATION share/${PROJECT_NAME}/bringup
)
install(
  PROGRAMS bringup/scripts/harness_flow.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_plugin_load test/test_plugin_load.cpp)
  target_include_directories(test_plugin_load PRIVATE
    include ${HARNESS_KERNEL_INCLUDE_DIR}
  )
  find_package(lifecycle_msgs REQUIRED)
  # PLAIN signature here, not keyword: ament_add_gtest() above already calls
  # target_link_libraries(test_plugin_load gtest ...) in plain form
  # internally, and CMake forbids mixing plain and keyword calls on the same
  # target (the same rule that made the library target above use one merged
  # keyword call instead of ament_target_dependencies()).
  target_link_libraries(test_plugin_load
    ${HARNESS_KERNEL_LIB}
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    ${lifecycle_msgs_TARGETS}
  )
  # Ensures the freshly-built plugin (not a stale installed one) is what the
  # test's pluginlib::ClassLoader resolves — §12 step 1 must run against the
  # binary this CMakeLists just produced.
  add_dependencies(test_plugin_load harness_controller)

  # Gate behaviour. Unlike test_plugin_load (which goes through pluginlib and
  # stops at on_configure), this one links the controller library directly so
  # it can construct the controller, hand it command interfaces and drive
  # update_and_write_commands — i.e. actually execute hk_gate.
  ament_add_gtest(test_gate_loop test/test_gate_loop.cpp)
  target_include_directories(test_gate_loop PRIVATE
    include ${HARNESS_KERNEL_INCLUDE_DIR}
  )
  # std_msgs and realtime_tools are linked PRIVATE on harness_controller, so
  # their include directories do not propagate to anything that links it —
  # but this test includes harness_controller.hpp, which includes both.
  target_link_libraries(test_gate_loop
    harness_controller
    ${HARNESS_KERNEL_LIB}
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    realtime_tools::realtime_tools
    ${std_msgs_TARGETS}
    ${harness_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${lifecycle_msgs_TARGETS}
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle controller_interface hardware_interface pluginlib realtime_tools
  std_msgs harness_msgs diagnostic_msgs
)
ament_package()