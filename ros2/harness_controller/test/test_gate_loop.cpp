// Functional tests for the gate itself: hk_gate driven through the real
// controller, in a real update loop, with real command interfaces.
//
// test_plugin_load.cpp covers lifecycle only — it never calls
// update_and_write_commands, so it proves the plugin loads and the profile
// validates but says nothing about what the gate DOES. This file is the
// other half: for each class of input, assert what actually reaches the
// effort command interface.
//
// No simulator, no controller_manager, no DDS on the assertion path. The
// controller is constructed directly and handed command interfaces backed by
// plain doubles, so `hw_effort[i]` below IS what the hardware would have
// been commanded — the same observable the safety claim is about.

#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "harness_controller/harness_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr double kTorqueLimitNm = 40.0;
const std::vector<std::string> kJoints = {"joint_1", "joint_2", "joint_3"};

// reference_interfaces_ is protected on ChainableControllerInterface, and
// on_export_reference_interfaces() (which sizes it) is protected on the
// controller. A test needs both: the first to inject a candidate command,
// the second because without it reference_interfaces_ is still empty.
class TestableHarnessController : public harness_controller::HarnessController
{
public:
  using controller_interface::ChainableControllerInterface::reference_interfaces_;
  using harness_controller::HarnessController::on_export_reference_interfaces;
};

std::string write_golden_file()
{
  const std::string path = "/tmp/harness_controller_gate_loop_golden.yaml";
  std::ofstream out(path, std::ios::trunc);
  out << "joints:\n";
  for (const auto & j : kJoints) {
    out << "  - " << j << "\n";
  }
  return path;
}

// One configured, interface-assigned controller plus the storage its command
// interfaces write through. `hw_effort` is the observable under test.
struct Rig
{
  std::shared_ptr<TestableHarnessController> controller;
  std::vector<double> hw_effort;
  // Held for lifetime: LoanedCommandInterface refers to these, and the
  // controller writes through them into hw_effort.
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  void update(std::uint64_t t_ns)
  {
    const rclcpp::Time time(static_cast<std::int64_t>(t_ns), RCL_ROS_TIME);
    const auto period = rclcpp::Duration::from_seconds(0.005);
    ASSERT_EQ(
      controller->update_and_write_commands(time, period),
      controller_interface::return_type::OK);
  }

  void set_reference(double value)
  {
    for (auto & r : controller->reference_interfaces_) {
      r = value;
    }
  }
};

// `band_b_reject_rate_max` defaults to 0.5; a test that wants Band B out of
// the way passes 1.0 (a rate strictly greater than 1.0 is impossible).
std::unique_ptr<Rig> make_rig(double band_b_reject_rate_max = 1.0)
{
  auto rig = std::make_unique<Rig>();
  rig->controller = std::make_shared<TestableHarnessController>();

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("joints", kJoints),
     rclcpp::Parameter("command_joints", kJoints),
     rclcpp::Parameter("golden_joint_order_file", write_golden_file()),
     rclcpp::Parameter("index_map_output_path", ""),
     rclcpp::Parameter("test_torque_limit_nm", kTorqueLimitNm),
     rclcpp::Parameter("band_b_reject_rate_window_ms", 500.0),
     rclcpp::Parameter("band_b_reject_rate_max", band_b_reject_rate_max)});

  if (rig->controller->init("harness_controller", "", 200, "", options) !=
    controller_interface::return_type::OK)
  {
    ADD_FAILURE() << "controller init() failed";
    return rig;
  }
  if (rig->controller->get_node()->configure().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    ADD_FAILURE() << "on_configure() did not reach INACTIVE — hk_create likely rejected the "
                     "profile the controller built";
    return rig;
  }

  // Sizes reference_interfaces_ to the command-joint count. The returned
  // handles are the controller_manager's business; the test only needs the
  // side effect.
  (void)rig->controller->on_export_reference_interfaces();

  rig->hw_effort.assign(kJoints.size(), 0.0);
  rig->command_interfaces.reserve(kJoints.size());
  std::vector<hardware_interface::LoanedCommandInterface> loaned;
  loaned.reserve(kJoints.size());
  for (std::size_t i = 0; i < kJoints.size(); ++i) {
    rig->command_interfaces.emplace_back(
      kJoints[i], hardware_interface::HW_IF_EFFORT, &rig->hw_effort[i]);
    loaned.emplace_back(rig->command_interfaces[i]);
  }

  // State interfaces are deliberately empty: update_and_write_commands reads
  // only reference_interfaces_ and writes only command_interfaces_. Assigning
  // none keeps the test honest about what the decision path actually touches.
  rig->controller->assign_interfaces(std::move(loaned), {});
  return rig;
}

}  // namespace

// An in-bounds command must reach the actuator bit-for-bit. A gate that
// perturbs admitted commands is worse than no gate.
TEST(HarnessControllerGateLoop, admits_in_bounds_command_unchanged)
{
  auto rig = make_rig();
  ASSERT_EQ(rig->controller->reference_interfaces_.size(), kJoints.size());

  rig->set_reference(12.5);
  rig->update(1'000'000);

  for (std::size_t i = 0; i < kJoints.size(); ++i) {
    EXPECT_DOUBLE_EQ(rig->hw_effort[i], 12.5) << "joint index " << i;
  }
}

// Projection clamps every violating cycle, independent of Transfer — see the
// header comment in edge/src/kernel/kernel_gate.cpp. The command that reaches
// the actuator is the declared bound, not the requested value and not zero.
TEST(HarnessControllerGateLoop, clamps_out_of_bounds_command_to_declared_bound)
{
  auto rig = make_rig();

  rig->set_reference(250.0);
  rig->update(1'000'000);
  for (std::size_t i = 0; i < kJoints.size(); ++i) {
    EXPECT_DOUBLE_EQ(rig->hw_effort[i], kTorqueLimitNm) << "joint index " << i;
  }

  rig->set_reference(-250.0);
  rig->update(2'000'000);
  for (std::size_t i = 0; i < kJoints.size(); ++i) {
    EXPECT_DOUBLE_EQ(rig->hw_effort[i], -kTorqueLimitNm) << "joint index " << i;
  }
}

// Sustained violation crosses the Band B reject-rate trigger. The observable
// transition is unambiguous because the command is positive and out of range:
// while NOMINAL it clamps to +limit, and once Band B holds authority the
// profile's ON_REJECT_ZERO output takes over and it becomes exactly 0.
TEST(HarnessControllerGateLoop, sustained_violation_transfers_to_band_b_and_zeroes_output)
{
  auto rig = make_rig(/*band_b_reject_rate_max=*/0.5);

  rig->set_reference(250.0);

  // kRejectRateMinSamples = 4 (edge/src/kernel/kernel_state.hpp): the rate is
  // not evaluated at all until the window holds at least four samples, so the
  // first cycles clamp rather than transfer.
  rig->update(1'000'000);
  EXPECT_DOUBLE_EQ(rig->hw_effort[0], kTorqueLimitNm)
    << "first violating cycle must clamp, not transfer — the reject-rate window is not full yet";

  bool reached_band_b = false;
  for (int cycle = 2; cycle <= 20 && !reached_band_b; ++cycle) {
    rig->update(static_cast<std::uint64_t>(cycle) * 1'000'000ULL);
    reached_band_b = (rig->hw_effort[0] == 0.0);
  }
  ASSERT_TRUE(reached_band_b)
    << "20 consecutive 100%-violating cycles inside a 500 ms window did not trip the Band B "
       "reject-rate trigger";

  // Band B is terminal in this ABI: kernel_gate.cpp leaves it only through a
  // Band A reentry, and Band A needs a critic this profile does not declare.
  // An in-bounds command must NOT restore authority to the model.
  rig->set_reference(1.0);
  for (int cycle = 21; cycle <= 30; ++cycle) {
    rig->update(static_cast<std::uint64_t>(cycle) * 1'000'000ULL);
  }
  EXPECT_DOUBLE_EQ(rig->hw_effort[0], 0.0)
    << "Band B released itself without a reentry path — that would contradict "
       "kernel_gate.cpp's output block";
}

// CHARACTERIZATION TEST — pins a known gap, does not endorse it.
//
// hk_gate evaluates only VALUE_RANGE. For a NaN candidate,
// `margin = min(NaN, NaN)` is NaN and `NaN < 0.0` is false, so no violation
// is recorded, std::min/std::max propagate the NaN through the clamp, and it
// is ADMITted to the actuator — and cached as last_admitted, poisoning any
// future HOLD_LAST output.
//
// When a finiteness predicate lands in Projection this test WILL fail. That
// is the point: the failure is the signal to update it, and until then the
// gap is asserted out loud rather than living in a comment nobody reads.
TEST(HarnessControllerGateLoop, nan_reference_is_admitted_ungated)
{
  auto rig = make_rig();

  rig->set_reference(std::numeric_limits<double>::quiet_NaN());
  rig->update(1'000'000);

  EXPECT_TRUE(std::isnan(rig->hw_effort[0]))
    << "a finiteness predicate appears to have landed in Projection — good. Update this test "
       "to assert the new (safe) behaviour and drop the characterization comment.";
}

// Infinities, unlike NaN, ARE ordered against the bounds, so Projection
// handles them correctly. Kept alongside the NaN test so the asymmetry is
// documented by assertion rather than by prose.
TEST(HarnessControllerGateLoop, infinite_reference_is_clamped)
{
  auto rig = make_rig();

  rig->set_reference(std::numeric_limits<double>::infinity());
  rig->update(1'000'000);
  EXPECT_DOUBLE_EQ(rig->hw_effort[0], kTorqueLimitNm);

  rig->set_reference(-std::numeric_limits<double>::infinity());
  rig->update(2'000'000);
  EXPECT_DOUBLE_EQ(rig->hw_effort[0], -kTorqueLimitNm);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
