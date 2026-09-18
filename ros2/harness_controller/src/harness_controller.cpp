#include "harness_controller/harness_controller.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "harness/v1/profile.pb.h"
#include "pluginlib/class_list_macros.hpp"

namespace harness_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using hardware_interface::CommandInterface;

namespace
{

// Mirrors edge/src/profile/validator.cpp's canonical_joint_order_hash (C8)
// byte for byte: FNV-1a 64-bit over joint_names joined with '\n'. Duplicated
// rather than linked because the exported kernel artifact ships only
// include/harness/ + share/proto/ (edge/scripts/build_artifact.sh) — the
// internal validator is not part of that surface. Keep this in sync if the
// validator's algorithm ever changes.
std::string canonical_joint_order_hash(const std::vector<std::string> & joint_names)
{
  constexpr std::uint64_t kOffsetBasis = 0xcbf29ce484222325ULL;
  constexpr std::uint64_t kPrime = 0x100000001b3ULL;
  std::uint64_t hash = kOffsetBasis;
  bool first = true;
  for (const auto & name : joint_names) {
    if (!first) {
      hash ^= static_cast<unsigned char>('\n');
      hash *= kPrime;
    }
    first = false;
    for (char raw_c : name) {
      hash ^= static_cast<unsigned char>(raw_c);
      hash *= kPrime;
    }
  }
  char buf[17];
  std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(hash));
  return std::string(buf);
}

// Profile identity, shared by the profile the controller builds and the
// HarnessInfo it advertises. Declared once so the two can never disagree —
// a consumer correlating a GateStatus against a profile by id must not be
// reading two different string literals that happened to match.
constexpr const char * kProfileId = "gazebo_stage1_rehearsal";
constexpr const char * kProfileVersion = "0.0.0";
constexpr const char * kEmbodimentClassId = "gazebo_stage1_pinned";

const char * band_name(std::uint8_t band)
{
  switch (band) {
    case HK_BAND_NOMINAL:
      return "NOMINAL";
    case HK_BAND_A:
      return "BAND_A";
    case HK_BAND_B:
      return "BAND_B";
  }
  return "UNKNOWN";
}

}  // namespace

HarnessController::HarnessController() = default;

HarnessController::~HarnessController()
{
  if (handle_ != nullptr) {
    hk_destroy(handle_);
    handle_ = nullptr;
  }
}

CallbackReturn HarnessController::on_init()
{
  // hk_abi_version() must be checked before any other kernel call
  // (edge/include/harness/harness_kernel.h, ABI rule 4).
  if (hk_abi_version() != HK_ABI_VERSION) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "harness kernel ABI mismatch: controller built against %u, loaded kernel reports %u",
      HK_ABI_VERSION, hk_abi_version());
    return CallbackReturn::ERROR;
  }

  try {
    auto node = get_node();
    node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
    node->declare_parameter<std::vector<std::string>>(
      "command_joints", std::vector<std::string>{});
    node->declare_parameter<std::string>("golden_joint_order_file", "");
    node->declare_parameter<std::string>("index_map_output_path", "");
    node->declare_parameter<double>("test_torque_limit_nm", 0.0);
    node->declare_parameter<int>("event_buffer_bytes", 8192);
    node->declare_parameter<double>("band_b_reject_rate_window_ms", 500.0);
    node->declare_parameter<double>("band_b_reject_rate_max", 0.5);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_node()->get_logger(), "on_init parameter declaration failed: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

bool HarnessController::check_golden_joint_order()
{
  if (golden_joint_order_file_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "golden_joint_order_file parameter is empty");
    return false;
  }

  std::vector<std::string> golden;
  try {
    YAML::Node root = YAML::LoadFile(golden_joint_order_file_);
    YAML::Node list = root["joints"] ? root["joints"] : root;
    for (const auto & n : list) {
      golden.push_back(n.as<std::string>());
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_node()->get_logger(), "failed to read golden_joint_order_file '%s': %s",
      golden_joint_order_file_.c_str(), e.what());
    return false;
  }

  if (golden.size() != joints_.size()) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "joint-ordering ABORT: golden list has %zu joints, configured 'joints' param has %zu",
      golden.size(), joints_.size());
    return false;
  }

  for (std::size_t i = 0; i < golden.size(); ++i) {
    if (golden[i] != joints_[i]) {
      RCLCPP_FATAL(
        get_node()->get_logger(),
        "joint-ordering ABORT at index %zu: golden='%s' configured='%s'. "
        "No remap, no best-effort reorder — fix the configuration and restart.",
        i, golden[i].c_str(), joints_[i].c_str());
      return false;
    }
  }

  RCLCPP_INFO(
    get_node()->get_logger(), "joint-ordering assertion passed: %zu joints, golden order intact",
    golden.size());
  return true;
}

void HarnessController::write_index_map() const
{
  if (index_map_output_path_.empty()) {
    return;
  }
  std::ofstream out(index_map_output_path_, std::ios::trunc);
  if (!out.is_open()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "could not open index_map_output_path '%s' for writing",
      index_map_output_path_.c_str());
    return;
  }
  out << "{\n  \"joint_index_map\": [\n";
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    out << "    {\"index\": " << i << ", \"name\": \"" << joints_[i] << "\"}";
    out << (i + 1 < joints_.size() ? ",\n" : "\n");
  }
  out << "  ]\n}\n";
}

std::string HarnessController::build_profile_pb() const
{
  // test_torque_limit_nm_ is an arbitrary test constant (§9) — it has no
  // derivation and must never be copied into a real Harness Profile. The
  // domain is torque, so an embodiment is required; "gazebo_stage1_pinned" is
  // not a registered embodiment, it is a rehearsal-rig label, deliberately
  // not reused anywhere else.
  //
  // Every optional scalar the validator's C11 requires presence for is set
  // explicitly below (never left at proto3's implicit zero default) — see
  // edge/src/profile/validator.cpp check_c11.
  harness::v1::HarnessProfile profile;
  profile.set_schema_version(1);
  profile.set_profile_id(kProfileId);
  profile.set_profile_version(kProfileVersion);

  auto * embodiment = profile.mutable_embodiment();
  embodiment->set_class_id(kEmbodimentClassId);
  embodiment->set_dof(static_cast<std::uint32_t>(arity_));
  for (const auto & j : command_joints_) {
    embodiment->add_joint_names(j);
  }
  embodiment->set_joint_order_hash(canonical_joint_order_hash(command_joints_));

  auto * binding = profile.mutable_model_binding();
  binding->set_model_node("/upstream_effort_controller");
  binding->set_output_topic(std::string("/") + get_node()->get_name() + "/commands");
  binding->set_output_interface(harness::v1::EFFORT);

  auto * projection = profile.mutable_projection();
  for (std::size_t i = 0; i < arity_; ++i) {
    auto * bound = projection->add_output_region();
    bound->set_min(-test_torque_limit_nm_);
    bound->set_max(test_torque_limit_nm_);
  }
  projection->set_max_staleness_ms(50.0);
  projection->set_on_reject(harness::v1::Projection::ON_REJECT_ZERO);

  auto * isolation = profile.mutable_isolation();
  isolation->set_inference_wcet_ms(2.0);
  isolation->set_inference_rate_hz(500.0);
  isolation->set_max_payload_bytes(4096);
  isolation->set_transport_deadline_ms(2.0);
  // No critics declared in this rehearsal profile (Stage 1 exercises
  // Projection only, per edge/README.md's status table) — 0 is the honest
  // sum of a critic list with zero entries, and still satisfies C11's
  // presence requirement.
  isolation->set_critic_budget_total_us(0);

  // Band B. Two triggers declared (C5 needs at least one):
  //
  //   - projection reject rate: the only one reachable from ROS in this rig,
  //     since sustained out-of-bound commands are something a demo or a
  //     misbehaving upstream controller can actually produce. The trigger
  //     needs the violating fraction of the WHOLE lookback window to exceed
  //     the max, so at 200 Hz with a 500 ms window (~100 samples) that is
  //     ~50 sustained violating cycles, not 4 — kRejectRateMinSamples = 4 is
  //     only the minimum needed to evaluate the rate at all.
  //   - isolation overrun: declared for completeness but unreachable here,
  //     because this controller never calls hk_report_budget (there is no
  //     model inference or critic to time yet).
  //
  // Band B is TERMINAL in this ABI regardless of `latch`: kernel_gate.cpp
  // only leaves Band B via a Band A reentry path, and Band A is unreachable
  // without critics. Once entered, every cycle emits the on_reject output
  // until the controller is reconfigured (which builds a fresh hk_handle).
  //
  // fallback_node "/none" is a rehearsal-rig placeholder — never a real
  // deployment target, same status as test_torque_limit_nm_ above.
  auto * band_b = profile.mutable_transfer()->mutable_band_b();
  band_b->set_reject_rate_window_ms(band_b_reject_rate_window_ms_);
  band_b->set_reject_rate_max(band_b_reject_rate_max_);
  band_b->set_overrun_count_max(3);
  band_b->set_fallback_node("/none");
  band_b->set_latch(false);
  // No Band A recoveries declared: this rehearsal profile has no critics to
  // ground a recovery's evidence_class against (C3).

  auto * timing = profile.mutable_timing();
  // The real ROS 2 control loop's own wall clock — honest, so C2's
  // timing_authoritative requirement is met directly rather than sidestepped.
  timing->set_clock_domain(harness::v1::CLOCK_WALL);
  timing->set_timing_authoritative(true);

  // eligibility.profile_eligible left false (default): this profile is a
  // rehearsal-rig fixture, never a deployable artifact, so C1's
  // engine/engine_version requirement does not apply.

  auto * provenance = profile.mutable_provenance();
  provenance->set_source(harness::v1::PROVENANCE_SOURCE_HAND_AUTHORED);
  provenance->set_derived_by("harness_controller stage1 rehearsal rig");
  provenance->set_notes(
    "no coverage, no derivation — Stage 1 plugin-load and gating rehearsal only");

  return profile.SerializeAsString();
}

CallbackReturn HarnessController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  command_joints_ = node->get_parameter("command_joints").as_string_array();
  golden_joint_order_file_ = node->get_parameter("golden_joint_order_file").as_string();
  index_map_output_path_ = node->get_parameter("index_map_output_path").as_string();
  test_torque_limit_nm_ = node->get_parameter("test_torque_limit_nm").as_double();
  event_buffer_bytes_ = static_cast<std::size_t>(
    std::max<int>(0, static_cast<int>(node->get_parameter("event_buffer_bytes").as_int())));
  band_b_reject_rate_window_ms_ =
    node->get_parameter("band_b_reject_rate_window_ms").as_double();
  band_b_reject_rate_max_ = node->get_parameter("band_b_reject_rate_max").as_double();

  if (joints_.empty() || command_joints_.empty()) {
    RCLCPP_FATAL(node->get_logger(), "'joints' and 'command_joints' must both be non-empty");
    return CallbackReturn::ERROR;
  }

  // §8 pre-flight assertion. Abort loudly; no simulator involved yet.
  if (!check_golden_joint_order()) {
    return CallbackReturn::ERROR;
  }
  write_index_map();

  arity_ = command_joints_.size();
  candidate_.assign(arity_, 0.0);
  emitted_.assign(arity_, 0.0);

  const std::string profile_pb = build_profile_pb();

  if (handle_ != nullptr) {
    hk_destroy(handle_);
    handle_ = nullptr;
  }
  const hk_status rc = hk_create(
    reinterpret_cast<const std::uint8_t *>(profile_pb.data()), profile_pb.size(), &handle_);
  if (rc != HK_OK) {
    RCLCPP_FATAL(
      node->get_logger(), "hk_create failed (status %d): %s", static_cast<int>(rc),
      handle_ != nullptr ? hk_last_error(handle_) : "handle not created");
    return CallbackReturn::ERROR;
  }

  event_buf_.assign(event_buffer_bytes_, 0);
  event_len_ = 0;
  event_dropped_ = 0;
  // hk_create above built a fresh handle, which starts in BAND_NOMINAL. This
  // is what makes deactivate/activate the documented way out of a terminal
  // Band B.
  band_now_.store(HK_BAND_NOMINAL, std::memory_order_relaxed);
  last_reported_band_ = HK_BAND_NOMINAL;

  // --- operational plane --------------------------------------------------
  //
  // Best-effort, not reliable. These topics describe enforcement; they never
  // carry it. A reliable QoS here would let a slow subscriber apply
  // backpressure to a publisher called from the control loop, which is the
  // one thing a gate must never be exposed to. Loss is already accounted for
  // in the protocol (dropped_events / dropped_since_last).
  const auto status_qos = rclcpp::SystemDefaultsQoS().keep_last(16).best_effort();

  status_publisher_ =
    node->create_publisher<harness_msgs::msg::GateStatus>("~/gate_status", status_qos);
  rt_status_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<harness_msgs::msg::GateStatus>>(
      status_publisher_);
  rt_status_publisher_->msg_.candidate.resize(arity_, 0.0);
  rt_status_publisher_->msg_.emitted.resize(arity_, 0.0);

  enforcement_publisher_ =
    node->create_publisher<harness_msgs::msg::EnforcementEvents>("~/enforcement_events", status_qos);
  rt_enforcement_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<harness_msgs::msg::EnforcementEvents>>(
      enforcement_publisher_);
  // Constant per-publish fields, set once: the control path only ever fills
  // in `data`, `stamp` and the drop count.
  rt_enforcement_publisher_->msg_.schema = "harness.v1.EnforcementEvent";
  rt_enforcement_publisher_->msg_.encoding = "protobuf-varint-delimited";
  // Reserve once so every subsequent per-cycle resize(len), len <= cap, is a
  // no-op on capacity — the decision path below never reallocates.
  rt_enforcement_publisher_->msg_.data.reserve(event_buffer_bytes_);

  // Latched identity. Transient-local so a tool that starts after the
  // controller still learns the joint ordering its positional arrays are
  // indexed by, instead of guessing.
  info_publisher_ = node->create_publisher<harness_msgs::msg::HarnessInfo>(
    "~/info", rclcpp::QoS(1).transient_local().reliable());

  // /diagnostics is global by convention — that is the point. It is where a
  // customer's existing rqt_robot_monitor, diagnostic aggregator and safety
  // supervisor already look, so a band transition reaches their monitoring
  // with no integration work on their side.
  diagnostics_publisher_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());

  return CallbackReturn::SUCCESS;
}

void HarnessController::publish_info()
{
  harness_msgs::msg::HarnessInfo info;
  info.profile_id = kProfileId;
  info.profile_version = kProfileVersion;
  info.embodiment_class_id = kEmbodimentClassId;
  info.joint_names = command_joints_;
  info.joint_order_hash = canonical_joint_order_hash(command_joints_);
  info.output_region_min.assign(arity_, -test_torque_limit_nm_);
  info.output_region_max.assign(arity_, test_torque_limit_nm_);
  info.output_interface = harness_msgs::msg::HarnessInfo::OUTPUT_INTERFACE_EFFORT;
  info.kernel_abi_version = hk_abi_version();
  info.fallback_node = "/none";
  info_publisher_->publish(info);
}

void HarnessController::publish_diagnostics()
{
  // Runs on the controller_manager executor, NOT the control loop. Allocation
  // and logging are fine here and must not happen on the other side.
  const auto band = band_now_.load(std::memory_order_relaxed);
  const auto cycles = cycles_total_.load(std::memory_order_relaxed);
  const auto clamped = clamped_cycles_.load(std::memory_order_relaxed);
  const auto band_b = band_b_cycles_.load(std::memory_order_relaxed);
  const auto dropped = dropped_events_total_.load(std::memory_order_relaxed);
  const auto write_failures = write_failures_total_.load(std::memory_order_relaxed);

  // Edge-triggered logging: authority changes are the events an operator has
  // to see, and they are rare enough that a log line per transition costs
  // nothing. Steady state stays silent.
  if (band != last_reported_band_) {
    if (band == HK_BAND_B) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "HARNESS BAND TRANSITION %s -> %s: authority has left the model. Band B is terminal "
        "in this ABI — it is exited only via a Band A reentry, and this profile declares no "
        "critics. Every cycle from here emits the profile's on_reject output regardless of "
        "what is commanded. Deactivate and reactivate this controller to rebuild the kernel "
        "handle.",
        band_name(last_reported_band_), band_name(band));
    } else {
      RCLCPP_WARN(
        get_node()->get_logger(), "HARNESS BAND TRANSITION %s -> %s",
        band_name(last_reported_band_), band_name(band));
    }
    last_reported_band_ = band;
  }

  const std::uint64_t d_cycles = cycles - last_cycles_total_;
  const std::uint64_t d_clamped = clamped - last_clamped_cycles_;
  last_cycles_total_ = cycles;
  last_clamped_cycles_ = clamped;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string(get_node()->get_name()) + ": harness gate";
  // Identifies WHICH gate and WHICH bounds, so several harness controllers on
  // one robot (arm, gripper, base) stay distinguishable in an aggregator.
  status.hardware_id =
    std::string(kProfileId) + "/" + kProfileVersion + " @ " + kEmbodimentClassId;

  if (band == HK_BAND_B) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Band B: authority withdrawn from the model (terminal until reconfigure)";
  } else if (band == HK_BAND_A) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Band A: recovery in progress";
  } else if (d_clamped > 0) {
    // Clamping is enforcement working, not a fault — but it means the model
    // is asking for something the profile forbids, which an operator wants to
    // know before the reject rate trips Band B.
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Nominal, but clamping: model output is exceeding the declared bound";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Nominal: all commands inside the declared output region";
  }

  auto kv = [&status](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = key;
    pair.value = value;
    status.values.push_back(pair);
  };

  kv("band", band_name(band));
  kv("gate_cycles_total", std::to_string(cycles));
  kv("clamped_cycles_total", std::to_string(clamped));
  kv("clamp_rate_recent", d_cycles > 0
      ? std::to_string(static_cast<double>(d_clamped) / static_cast<double>(d_cycles))
      : "n/a");
  kv("band_b_cycles_total", std::to_string(band_b));
  // Non-zero means evidence was lost, never that enforcement was skipped.
  kv("enforcement_events_dropped", std::to_string(dropped));
  // Non-zero means a gated command did not reach the actuator at all.
  kv("command_write_failures", std::to_string(write_failures));
  kv("profile_id", kProfileId);
  kv("profile_version", kProfileVersion);
  kv("embodiment_class_id", kEmbodimentClassId);
  kv("joint_order_hash", canonical_joint_order_hash(command_joints_));
  kv("kernel_abi_version", std::to_string(hk_abi_version()));
  kv("output_region_nm", "+/-" + std::to_string(test_torque_limit_nm_));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_node()->now();
  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
}

CallbackReturn HarnessController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Republished per activation, not per configure: a deactivate/activate
  // cycle rebuilds the kernel handle, and a subscriber that joined in between
  // needs the identity that goes with the handle now running.
  publish_info();

  // 1 Hz. Diagnostics describe a trend, not a cycle — the per-cycle view is
  // ~/gate_status. Created here rather than in on_configure so an inactive
  // controller does not report on a gate that is not running.
  diagnostics_timer_ = get_node()->create_wall_timer(
    std::chrono::seconds(1), [this]() { publish_diagnostics(); });

  return CallbackReturn::SUCCESS;
}

CallbackReturn HarnessController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  diagnostics_timer_.reset();
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration HarnessController::command_interface_configuration() const
{
  InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL};
  for (const auto & j : command_joints_) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return cfg;
}

InterfaceConfiguration HarnessController::state_interface_configuration() const
{
  InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL};
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return cfg;
}

std::vector<hardware_interface::CommandInterface> HarnessController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> refs;
  // Zero, not NaN. Under the v1 kernel a NaN reference failed the finiteness
  // check and logged a DROP every cycle until the upstream controller chained
  // in — ~2000 spurious events at the head of every recording.
  //
  // The v2 kernel makes zero MORE important, not less: hk_gate has no
  // finiteness predicate at all (kernel_gate.cpp evaluates only VALUE_RANGE,
  // and `NaN < 0.0` is false), so a NaN reference would now be ADMITted and
  // written straight to the effort interface — and cached as last_admitted.
  // Zero is a physically safe effort default and a valid in-bounds candidate,
  // so the gate stays transparent until real traffic arrives. See
  // test_gate_loop.cpp's nan_reference_is_admitted_ungated, which pins this
  // gap deliberately.
  reference_interfaces_.resize(arity_, 0.0);
  for (std::size_t i = 0; i < command_joints_.size(); ++i) {
    refs.emplace_back(
      get_node()->get_name(),
      command_joints_[i] + "/" + hardware_interface::HW_IF_EFFORT,
      &reference_interfaces_[i]);
  }
  return refs;
}

bool HarnessController::on_set_chained_mode(bool /*chained_mode*/)
{
  // Stage 1 only exercises the chained topology described in the build
  // brief §7: an unchained standalone mode is out of scope.
  return true;
}

controller_interface::return_type HarnessController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // No standalone subscriber path in Stage 1 — reference values arrive by the
  // upstream JointGroupEffortController writing directly into
  // reference_interfaces_ while this controller runs chained (§7).
  return controller_interface::return_type::OK;
}

controller_interface::return_type HarnessController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // --- decision path: no allocation, no exceptions, bounded work only ------
  for (std::size_t i = 0; i < arity_; ++i) {
    candidate_[i] = reference_interfaces_[i];
  }

  hk_command cmd{};
  cmd.t_ns = static_cast<std::uint64_t>(time.nanoseconds());
  cmd.n_joints = static_cast<std::uint32_t>(arity_);
  cmd.effort = candidate_.data();

  hk_gated_command gated{};
  gated.n_joints = static_cast<std::uint32_t>(arity_);
  gated.effort = emitted_.data();

  event_len_ = 0;
  event_dropped_ = 0;
  hk_event_sink sink{};
  sink.buf = event_buf_.data();
  sink.cap = event_buf_.size();
  sink.len = &event_len_;
  sink.dropped = &event_dropped_;

  const hk_status rc = hk_gate(handle_, &cmd, &gated, &sink);

  // set_value() is [[nodiscard]] on Jazzy and can genuinely fail (it retries
  // acquiring the handle, then gives up). A dropped write is not cosmetic
  // here: ProjectionEvent.emitted is defined as "what actually reached the
  // actuator", so a silent failure would make the episode log assert
  // something untrue about the robot — precisely the claim this controller
  // exists to keep honest. Accumulate, then act on it below the decision path.
  bool writes_ok = true;
  if (rc == HK_OK) {
    for (std::size_t i = 0; i < arity_; ++i) {
      writes_ok = command_interfaces_[i].set_value(emitted_[i]) && writes_ok;
    }
  } else {
    // hk_gate itself failed (not a gate REJECT — an ABI-level error, e.g.
    // joint-count mismatch). Hold zero-risk zeros rather than propagate a
    // stale or garbage command downstream.
    for (std::size_t i = 0; i < arity_; ++i) {
      writes_ok = command_interfaces_[i].set_value(0.0) && writes_ok;
      emitted_[i] = 0.0;
    }
  }
  // Counters only — no logging, no allocation. publish_diagnostics() reads
  // these on the executor thread and does the reporting.
  cycles_total_.fetch_add(1, std::memory_order_relaxed);
  band_now_.store(static_cast<std::uint8_t>(gated.band), std::memory_order_relaxed);
  if (gated.decision != HK_ADMIT) {
    clamped_cycles_.fetch_add(1, std::memory_order_relaxed);
  }
  if (gated.band == HK_BAND_B) {
    band_b_cycles_.fetch_add(1, std::memory_order_relaxed);
  }
  if (event_dropped_ > 0) {
    dropped_events_total_.fetch_add(event_dropped_, std::memory_order_relaxed);
  }
  if (!writes_ok) {
    write_failures_total_.fetch_add(1, std::memory_order_relaxed);
  }
  // --- end decision path ----------------------------------------------------

  // Draining outside hk_gate itself: this is a bounded memcpy plus a
  // lock-free realtime publish, never a parse — the framed
  // harness.v1.EnforcementEvent records go out raw and are decoded by
  // whatever consumes the tap, so no protobuf deserialisation sits on this
  // hot path.
  if (event_len_ > 0 && rt_enforcement_publisher_ && rt_enforcement_publisher_->trylock()) {
    auto & msg = rt_enforcement_publisher_->msg_;
    msg.stamp = time;
    msg.dropped_since_last = event_dropped_;
    msg.data.resize(event_len_);  // no-op on capacity: reserved to cap in on_configure
    std::memcpy(msg.data.data(), event_buf_.data(), event_len_);
    rt_enforcement_publisher_->unlockAndPublish();
  }
  if (rt_status_publisher_ && rt_status_publisher_->trylock()) {
    auto & msg = rt_status_publisher_->msg_;
    msg.stamp = time;
    msg.decision = static_cast<std::uint8_t>(gated.decision);
    msg.band = static_cast<std::uint8_t>(gated.band);
    msg.dropped_events = event_dropped_;
    for (std::size_t i = 0; i < arity_; ++i) {
      // Pre-sized in on_configure; assignment, never resize.
      msg.candidate[i] = candidate_[i];
      msg.emitted[i] = emitted_[i];
    }
    rt_status_publisher_->unlockAndPublish();
  }

  if (!writes_ok) {
    // ERROR here deactivates this controller via controller_manager. That is
    // the intended response, not an overreaction: a harness that cannot write
    // the command it just gated is not enforcing anything, and the honest
    // failure is to stop claiming that it is.
    //
    // This is the one log call left on the control path, deliberately: the
    // controller is about to be deactivated, so the diagnostics timer may
    // never run again to report it. A terminal path is allowed to allocate.
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "failed to write a gated effort command interface — the gate's decision did not reach "
      "the actuator; deactivating rather than logging an emitted value that never took effect");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

}  // namespace harness_controller

PLUGINLIB_EXPORT_CLASS(
  harness_controller::HarnessController, controller_interface::ChainableControllerInterface)
