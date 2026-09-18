// §12 step 1: confirm the harness kernel compiles and loads as an rclcpp
// controller plugin via pluginlib, and that the C ABI boundary is intact.
// No simulator involved — this test never touches Gazebo or hardware
// interfaces, only the plugin's own lifecycle up through on_configure().
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "gtest/gtest.h"
#include "harness/harness_kernel.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

std::string write_golden_file(const std::vector<std::string> & joints)
{
  const std::string path = "/tmp/harness_controller_test_golden_order.yaml";
  std::ofstream out(path, std::ios::trunc);
  out << "joints:\n";
  for (const auto & j : joints) {
    out << "  - " << j << "\n";
  }
  return path;
}

std::shared_ptr<controller_interface::ChainableControllerInterface> load_controller(
  pluginlib::ClassLoader<controller_interface::ChainableControllerInterface> & loader,
  const std::vector<std::string> & joints,
  const std::vector<std::string> & command_joints,
  const std::string & golden_file)
{
  auto controller = loader.createSharedInstance("harness_controller/HarnessController");

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("joints", joints),
     rclcpp::Parameter("command_joints", command_joints),
     rclcpp::Parameter("golden_joint_order_file", golden_file),
     rclcpp::Parameter("index_map_output_path", "/tmp/harness_controller_test_index_map.json"),
     rclcpp::Parameter("test_torque_limit_nm", 40.0)});

  const auto init_result =
    controller->init("test_harness_controller", "", 500, "", options);
  EXPECT_EQ(init_result, controller_interface::return_type::OK);
  return controller;
}

}  // namespace

TEST(HarnessControllerPluginLoad, abi_version_matches_compiled_header)
{
  // The controller's own on_init() checks this and errors out if it doesn't
  // match; this assertion documents that the same check the plugin makes at
  // runtime is also true in this test's own process.
  EXPECT_EQ(hk_abi_version(), static_cast<uint32_t>(HK_ABI_VERSION));
}

TEST(HarnessControllerPluginLoad, pluginlib_loads_and_configures_with_correct_order)
{
  pluginlib::ClassLoader<controller_interface::ChainableControllerInterface> loader(
    "controller_interface", "controller_interface::ChainableControllerInterface");

  ASSERT_TRUE(loader.isClassAvailable("harness_controller/HarnessController"))
    << "pluginlib could not find harness_controller/HarnessController — check "
       "harness_controller_plugin.xml export";

  const std::vector<std::string> joints = {"left_shoulder_pitch", "right_shoulder_pitch"};
  const std::vector<std::string> command_joints = joints;
  const std::string golden_file = write_golden_file(joints);

  auto controller = load_controller(loader, joints, command_joints, golden_file);
  ASSERT_NE(controller, nullptr);

  const auto configure_result = controller->get_node()->configure();
  EXPECT_EQ(
    configure_result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    << "on_configure() did not reach INACTIVE — joint-ordering check or "
       "harness_profile_load likely failed even though the golden order matches";
}

TEST(HarnessControllerPluginLoad, aborts_loudly_on_joint_order_mismatch)
{
  pluginlib::ClassLoader<controller_interface::ChainableControllerInterface> loader(
    "controller_interface", "controller_interface::ChainableControllerInterface");

  const std::vector<std::string> configured_joints = {"left_shoulder_pitch", "right_shoulder_pitch"};
  const std::vector<std::string> golden_joints = {"right_shoulder_pitch", "left_shoulder_pitch"};
  const std::string golden_file = write_golden_file(golden_joints);

  auto controller = load_controller(loader, configured_joints, configured_joints, golden_file);
  ASSERT_NE(controller, nullptr);

  const auto configure_result = controller->get_node()->configure();
  // §8: wrong order must abort, never silently reorder or warn-only.
  EXPECT_NE(configure_result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}