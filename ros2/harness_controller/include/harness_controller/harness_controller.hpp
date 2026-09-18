// harness_controller — thin rclcpp ChainableControllerInterface over the
// harness kernel's C ABI (edge/include/harness/harness_kernel.h, v2 — the
// hk_/hkc_ ABI, not the retired harness_-prefixed one).
//
// Stage 1 rehearsal rig only (sim/gazebo_rig). This controller owns no safety
// logic: every decision happens inside hk_gate(), so the binary this
// package's plugin-load test exercises is the binary that ships. See
// edge/README.md's "Integrating" table: "ROS 2 | thin rclcpp ros2_control
// controller plugin, dlopen".
#ifndef HARNESS_CONTROLLER__HARNESS_CONTROLLER_HPP_
#define HARNESS_CONTROLLER__HARNESS_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "harness/harness_kernel.h"
#include "harness_msgs/msg/enforcement_events.hpp"
#include "harness_msgs/msg/gate_status.hpp"
#include "harness_msgs/msg/harness_info.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace harness_controller
{

class HarnessController : public controller_interface::ChainableControllerInterface
{
public:
  HarnessController();
  ~HarnessController() override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;

private:
  // Parameters, resolved in on_configure.
  std::vector<std::string> joints_;           // all 29, golden-checked order
  std::vector<std::string> command_joints_;    // arm-only subset, effort-gated
  std::string golden_joint_order_file_;
  std::string index_map_output_path_;
  double test_torque_limit_nm_ = 0.0;
  std::size_t event_buffer_bytes_ = 8192;
  // Band B's projection-reject-rate trigger. Exposed as parameters because
  // without one reachable trigger the rehearsal profile can never leave
  // BAND_NOMINAL: no critics are declared (so no Band A proposal exists) and
  // this controller never calls hk_report_budget (so the isolation-overrun
  // trigger cannot fire either). Sustained clamping is the one condition a
  // ROS-side demo can actually drive.
  double band_b_reject_rate_window_ms_ = 500.0;
  double band_b_reject_rate_max_ = 0.5;

  // Golden joint order check (§8). No remap, no warning-only path: a mismatch
  // is a configure-time abort.
  [[nodiscard]] bool check_golden_joint_order();
  void write_index_map() const;

  // Builds and serialises the Stage 1 rehearsal HarnessProfile (harness.v1,
  // schemas/proto/harness/v1/profile.proto) for hk_create. Off the decision
  // path — called once from on_configure.
  [[nodiscard]] std::string build_profile_pb() const;

  // The loaded kernel handle, owned off the decision path (on_configure/
  // on_activate only). Destroyed in on_deactivate/destructor.
  hk_handle * handle_ = nullptr;
  std::size_t arity_ = 0;

  // Per-cycle scratch buffers, sized once in on_configure so the decision
  // path (update_and_write_commands) never allocates.
  std::vector<double> candidate_;
  std::vector<double> emitted_;

  // Caller-owned append buffer for hk_gate's event sink (harness/harness_kernel.h
  // hk_event_sink). Drained and reset every cycle, right after hk_gate — see
  // update_and_write_commands. Never parsed here: the framed
  // harness.v1.EnforcementEvent records are republished raw and decoded
  // downstream (sim/gazebo_rig/mcap_ext.py), so this controller links no
  // protobuf parsing into its hot path.
  std::vector<std::uint8_t> event_buf_;
  std::size_t event_len_ = 0;
  std::uint32_t event_dropped_ = 0;

  // --- observability, read by the non-RT diagnostics timer ----------------
  //
  // The control path only ever stores into these. Reporting — logging a band
  // transition, publishing /diagnostics — happens in publish_diagnostics(),
  // which the controller_manager executor serves on a normal thread. That
  // keeps allocation and I/O off the decision path, which a plain
  // RCLCPP_WARN in update() would not.
  std::atomic<std::uint8_t> band_now_{HK_BAND_NOMINAL};
  std::atomic<std::uint64_t> cycles_total_{0};
  std::atomic<std::uint64_t> clamped_cycles_{0};
  std::atomic<std::uint64_t> band_b_cycles_{0};
  std::atomic<std::uint64_t> dropped_events_total_{0};
  std::atomic<std::uint64_t> write_failures_total_{0};

  // Owned by the diagnostics timer alone — never touched from update().
  std::uint8_t last_reported_band_ = HK_BAND_NOMINAL;
  std::uint64_t last_cycles_total_ = 0;
  std::uint64_t last_clamped_cycles_ = 0;

  void publish_diagnostics();
  void publish_info();

  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::shared_ptr<rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>> diagnostics_publisher_;

  // Latched: transient-local, depth 1, published once per activation so a
  // late subscriber can still interpret GateStatus's positional arrays.
  std::shared_ptr<rclcpp::Publisher<harness_msgs::msg::HarnessInfo>> info_publisher_;

  std::shared_ptr<rclcpp::Publisher<harness_msgs::msg::GateStatus>> status_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<harness_msgs::msg::GateStatus>>
    rt_status_publisher_;

  std::shared_ptr<rclcpp::Publisher<harness_msgs::msg::EnforcementEvents>> enforcement_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<harness_msgs::msg::EnforcementEvents>>
    rt_enforcement_publisher_;
};

}  // namespace harness_controller

#endif  // HARNESS_CONTROLLER__HARNESS_CONTROLLER_HPP_
