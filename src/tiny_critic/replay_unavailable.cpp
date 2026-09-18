#include "critic.hpp"
namespace harness::tiny {
Json replay_mcap(Critic &, const fs::path &, const Json &, std::ostream &) {
  throw std::runtime_error("MCAP decoding requires HARNESS_WITH_ROS2=ON (native rosbag2_cpp/CDR)");
}
}
