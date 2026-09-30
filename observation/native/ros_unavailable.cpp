#include "common.hpp"
#include "critic_features.hpp"
namespace harness::observation {
Json compile_snapshot_embedding(const fs::path &, const Json &, const Json &,
                                std::uint32_t) {
  return {{"status", "runtime_unavailable"},
          {"error", "ROS 2/MCAP support is not present in this build"}};
}
bool snapshot_compiler_available() { return false; }
namespace {
[[noreturn]] void unavailable() {
  throw std::runtime_error(
      "live observation and MCAP export require a native ROS build: source ROS "
      "2 Jazzy, configure with -DHARNESS_WITH_ROS2=ON, then rebuild");
}
} // namespace
Json ros_start(const Options &) { unavailable(); }
Json ros_discover(const Options &) { unavailable(); }
Json ros_request(const Options &, const Json &) { unavailable(); }
fs::path export_window(const fs::path &, const Json &, const fs::path &) {
  unavailable();
}
} // namespace harness::observation
