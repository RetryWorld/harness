#include "common.hpp"
#include "evidence.hpp"
#include "recording.hpp"
#include "runtime.hpp"
#include "events.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <future>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <set>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/file.h>
#include <sys/utsname.h>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <unistd.h>

namespace harness::observation {
namespace {
using String = std_msgs::msg::String;
struct RosContext {
  rclcpp::Context::SharedPtr value = std::make_shared<rclcpp::Context>();
  explicit RosContext(Ns domain, bool initialize_logging = true) {
    require(domain >= 0 && domain <= 232, "domain-id must be within 0..232");
    rclcpp::InitOptions init;
    init.set_domain_id(static_cast<std::size_t>(domain));
    init.auto_initialize_logging(initialize_logging);
    value->init(0, nullptr, init);
  }
  ~RosContext() {
    if (rclcpp::ok(value))
      value->shutdown("scan/session complete");
  }
};
struct SessionLock {
  int fd;
  explicit SessionLock(const fs::path &session)
      : fd(::open((session / "observer.lock").c_str(), O_CREAT | O_WRONLY,
                  0600)) {
    require(fd >= 0, "cannot open observer lock");
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
      ::close(fd);
      throw std::runtime_error("observer already running in this session");
    }
  }
  ~SessionLock() { ::close(fd); }
};
template <class T> T decode(const rclcpp::SerializedMessage &data) {
  T message;
  rclcpp::Serialization<T> codec;
  codec.deserialize_message(&data, &message);
  return message;
}
std::shared_ptr<rclcpp::SerializedMessage> serialize(const String &message) {
  auto data = std::make_shared<rclcpp::SerializedMessage>();
  rclcpp::Serialization<String> codec;
  codec.serialize_message(&message, data.get());
  return data;
}
String string_message(const Json &value) {
  String message;
  message.data = value.dump();
  return message;
}
Ns stamp(const builtin_interfaces::msg::Time &value) {
  return static_cast<Ns>(value.sec) * 1000000000LL + value.nanosec;
}
bool finite(const std::vector<double> &values) {
  return std::all_of(values.begin(), values.end(),
                     [](double v) { return std::isfinite(v); });
}
template <class T> T decode_bytes(const std::vector<std::uint8_t> &bytes) {
  rclcpp::SerializedMessage serialized(bytes.size());
  auto &raw = serialized.get_rcl_serialized_message();
  require(raw.buffer_capacity >= bytes.size(), "serialized message allocation failed");
  std::memcpy(raw.buffer, bytes.data(), bytes.size());
  raw.buffer_length = bytes.size();
  return decode<T>(serialized);
}

// The install-time bootstrap critic is a deliberately tiny, fixed-shape MLP
// over the latest joint/action summary. Its network scoring allocates nothing. Its weights are
// deterministic per profile but untrained, so it is shadow-only and can only
// create review candidates.  It gives every installation a real inference
// path while the TensorRT vision critic is trained and validated.
class BootstrapCritic final : public CriticAdapter {
  static constexpr std::size_t features = 12, hidden = 16;
  Json deployment_;
  std::array<double, features * hidden> first_{};
  std::array<double, hidden> first_bias_{}, second_{}, feature_{};
  double second_bias_ = 0.0;
  std::size_t samples_ = 0;
  Ns epoch_ = -1;

  static double squash(double value) {
    return std::copysign(std::log1p(std::abs(value)), value) / 8.0;
  }

public:
  explicit BootstrapCritic(Json deployment) : deployment_(std::move(deployment)) {
    require(deployment_.value("runtime", "") == "bootstrap_mlp_v1" &&
                deployment_.value("mode", "") == "shadow" &&
                deployment_.value("trained", true) == false,
            "invalid bootstrap critic contract");
    std::uint64_t state = std::stoull(deployment_.at("seed").get<std::string>(), nullptr, 16);
    auto weight = [&]() {
      state ^= state << 13; state ^= state >> 7; state ^= state << 17;
      return (static_cast<double>(state & 0xffffU) / 32767.5 - 1.0) * 0.35;
    };
    for (auto &value : first_) value = weight();
    for (auto &value : first_bias_) value = weight() * 0.1;
    for (auto &value : second_) value = weight();
  }
  bool ready() const override { return true; }
  bool load(const fs::path &, const Json &, const Json &) override { return false; }
  std::optional<Detection> evaluate(const Observations &observations) override {
    if (epoch_ != observations.epoch) { epoch_ = observations.epoch; samples_ = 0; }
    const ObservationMessage *joint_packet = nullptr, *command_packet = nullptr;
    for (auto it = observations.messages.rbegin(); it != observations.messages.rend(); ++it) {
      if (!joint_packet && it->type == "sensor_msgs/msg/JointState") joint_packet = &*it;
      if (!command_packet && it->type == "std_msgs/msg/Float64MultiArray") command_packet = &*it;
      if (joint_packet && command_packet) break;
    }
    if (!joint_packet) return std::nullopt;
    const auto joint = decode_bytes<sensor_msgs::msg::JointState>(joint_packet->cdr);
    if (joint.position.empty() || !finite(joint.position)) return std::nullopt;
    feature_.fill(0);
    auto summarize = [&](const std::vector<double> &values, std::size_t offset) {
      if (values.empty() || !finite(values)) return;
      double sum = 0, maximum = 0, squared = 0;
      for (const auto value : values) {
        sum += std::abs(value); maximum = std::max(maximum, std::abs(value));
        squared += value * value;
      }
      feature_[offset] = squash(sum / static_cast<double>(values.size()));
      feature_[offset + 1] = squash(maximum);
      feature_[offset + 2] = squash(std::sqrt(squared / static_cast<double>(values.size())));
    };
    summarize(joint.position, 0); summarize(joint.velocity, 3); summarize(joint.effort, 6);
    if (command_packet) {
      const auto command = decode_bytes<std_msgs::msg::Float64MultiArray>(command_packet->cdr);
      if (finite(command.data) && command.data.size() == joint.position.size()) {
        double sum = 0, maximum = 0, squared = 0;
        for (std::size_t index = 0; index < command.data.size(); ++index) {
          const double error = command.data[index] - joint.position[index];
          sum += std::abs(error); maximum = std::max(maximum, std::abs(error));
          squared += error * error;
        }
        feature_[9] = squash(sum / static_cast<double>(command.data.size()));
        feature_[10] = squash(maximum);
        feature_[11] = squash(std::sqrt(squared / static_cast<double>(command.data.size())));
      }
    }
    ++samples_;
    if (samples_ < deployment_.value("minimum_samples", std::size_t{8}))
      return std::nullopt;
    std::array<double, hidden> layer{};
    for (std::size_t h = 0; h < hidden; ++h) {
      layer[h] = first_bias_[h];
      for (std::size_t f = 0; f < features; ++f)
        layer[h] += first_[h * features + f] * feature_[f];
      layer[h] = std::tanh(layer[h]);
    }
    double logit = second_bias_;
    for (std::size_t h = 0; h < hidden; ++h) logit += second_[h] * layer[h];
    const double score = 1.0 / (1.0 + std::exp(-logit));
    if (score < deployment_.value("candidate_threshold", 0.9)) return std::nullopt;
    const auto deployment_id = deployment_.at("id").get<std::string>();
    return Detection{deployment_id, deployment_.at("classes")[0].get<std::string>(), score,
                     5, 2, "bootstrap_untrained_critic", deployment_id, false};
  }
};
std::pair<bool, Json> validate(const Json &topic, const Json &config,
                               const rclcpp::SerializedMessage &data) {
  const auto type = topic["type"].get<std::string>();
  const auto joints = config["joint_order"].get<std::vector<std::string>>();
  if (type == "sensor_msgs/msg/JointState") {
    const auto m = decode<sensor_msgs::msg::JointState>(data);
    bool valid = m.name.size() == m.position.size();
    std::map<std::string, double> positions;
    for (std::size_t i = 0; i < std::min(m.name.size(), m.position.size());
         ++i) {
      if (positions.contains(m.name[i]))
        valid = false;
      positions[m.name[i]] = m.position[i];
    }
    for (const auto &joint : joints)
      valid =
          valid && positions.contains(joint) && std::isfinite(positions[joint]);
    return {valid, stamp(m.header.stamp)};
  }
  if (type == "std_msgs/msg/Float64MultiArray") {
    const auto m = decode<std_msgs::msg::Float64MultiArray>(data);
    return {m.data.size() == joints.size() && finite(m.data), nullptr};
  }
  if (type == "trajectory_msgs/msg/JointTrajectory") {
    const auto m = decode<trajectory_msgs::msg::JointTrajectory>(data);
    bool valid = m.joint_names == joints && !m.points.empty();
    for (const auto &point : m.points)
      valid = valid && point.positions.size() == joints.size() &&
              finite(point.positions) && finite(point.velocities) &&
              finite(point.accelerations) && finite(point.effort);
    return {valid, stamp(m.header.stamp)};
  }
  const auto m = decode<sensor_msgs::msg::Image>(data);
  return {m.width > 0 && m.height > 0 &&
              m.data.size() == static_cast<std::size_t>(m.height) * m.step,
          stamp(m.header.stamp)};
}
class Observer {
  Options options_;
  fs::path session_;
  Json config_;
  SessionLock lock_;
  std::string id_ = unique_id(), prefix_ = "/harness/observation/s_" + id_;
  Evidence evidence_;
  std::unique_ptr<Store> store_;
  CriticInferencePlaceholder no_inference_, runtime_critic_;
  CriticAdapter *inference_ = &no_inference_;
  std::unique_ptr<BootstrapCritic> bootstrap_critic_;
  RecoveryControllerPlaceholder controller_;
  std::unique_ptr<Runtime> runtime_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Context::SharedPtr context_;
  std::unique_ptr<Recording> writer_;
  rclcpp::Publisher<String>::SharedPtr events_, replies_, profiles_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  rclcpp::Subscription<String>::SharedPtr requests_;
  std::map<std::string, Ns> counts_, last_wall_, detection_marks_;
  std::map<std::string, std::pair<std::string, Json>> responses_;
  std::deque<std::string> response_order_;
  std::set<std::string> announced_;
  std::deque<ObservationMessage> history_;
  std::size_t raw_bytes_ = 0;
  Json published_revision_ = nullptr;
  std::ofstream journal_;

  void event(const std::string &kind, Json value) {
    value.update({{"kind", kind},
                  {"receipt_ros_ns", node_->now().nanoseconds()},
                  {"wall_ns", wall_ns()}});
    journal_ << value.dump() << '\n';
    journal_.flush();
    require(journal_.good(), "event journal write failed");
    const int journal_fd =
        ::open((session_ / "events.jsonl").c_str(), O_RDONLY);
    require(journal_fd >= 0, "cannot synchronize event journal");
    const int sync_result = ::fsync(journal_fd);
    ::close(journal_fd);
    require(sync_result == 0, "event journal fsync failed");
    auto message = string_message(value);
    writer_->write(prefix_ + "/events", "std_msgs/msg/String",
                   serialize(message), node_->now().nanoseconds(),
                   evidence_.state["epoch"].get<Ns>());
    events_->publish(message);
    const bool failed = kind.find("failed") != std::string::npos ||
                        kind.find("skipped") != std::string::npos ||
                        (value.contains("ok") && value["ok"] == false);
    record_event("observer", kind, failed ? "failure" : "success", value);
    std::cout << value.dump(2) << std::endl;
  }
  void tick() {
    const auto old_epoch = evidence_.state["epoch"];
    evidence_.tick(node_->now().nanoseconds());
    if (old_epoch != evidence_.state["epoch"]) {
      history_.clear();
      raw_bytes_ = 0;
      detection_marks_.clear();
      if (runtime_ &&
          (runtime_->status == "nominal" || runtime_->status == "recovering"))
        event("runtime",
              {{"transition", runtime_->fallback("ROS clock reset")}});
    }
  }
  void receive(const Json &topic,
               const std::shared_ptr<rclcpp::SerializedMessage> &data) {
    tick();
    const auto now = *evidence_.now;
    const auto name = topic["name"].get<std::string>();
    const auto type = topic["type"].get<std::string>();
    if (type != "rosgraph_msgs/msg/Clock") {
      const auto &bytes = data->get_rcl_serialized_message();
      history_.push_back({name, type,
                          std::vector<std::uint8_t>(
                              bytes.buffer, bytes.buffer + bytes.buffer_length),
                          now});
      raw_bytes_ += bytes.buffer_length;
      while (!history_.empty() &&
             (raw_bytes_ > 16 * 1024 * 1024 ||
              now - history_.front().receipt_ros_ns > 20000000000LL)) {
        raw_bytes_ -= history_.front().cdr.size();
        history_.pop_front();
      }
      bool valid = false;
      Json source = nullptr;
      try {
        auto result = validate(topic, config_, *data);
        valid = result.first;
        source = result.second;
      } catch (const std::exception &) {
      }
      evidence_.sample(name, now, source, monotonic_ns(), valid);
      ++counts_[name];
      last_wall_[name] = monotonic_ns();
    }
    writer_->write(name, type, data, now, evidence_.state["epoch"].get<Ns>());
  }
  Json candidate(const Json &detection) {
    require(store_ != nullptr, "candidate workflow requires --store");
    evidence_.check_live();
    const auto confidence = detection.at("confidence").get<double>();
    require(std::isfinite(confidence) && confidence >= 0 && confidence <= 1,
            "confidence must be between zero and one");
    for (const auto *key : {"detector_id", "evidence_class", "source"})
      required_text(detection.at(key), key);
    const auto key = encoded(
        Json::array({detection["detector_id"], detection["evidence_class"]}));
    require(!detection_marks_.contains(key) ||
                *evidence_.now - detection_marks_[key] >= 10000000000LL,
            "candidate detector is in its ten-second ROS cooldown");
    auto &w = evidence_.capture(detection.at("before_s").get<double>(),
                                detection.at("after_s").get<double>(),
                                detection.at("evidence_class"));
    w.update({{"trigger", "critic_candidate"}, {"detection", detection}});
    detection_marks_[key] = *evidence_.now;
    return w;
  }
  void request(const String &message) {
    Json id = nullptr, response;
    std::string hash;
    try {
      require(message.data.size() <= 16384, "request too large");
      const auto data = Json::parse(message.data);
      require(data.is_object() && data.value("schema_version", 0) == 1,
              "request requires schema_version 1");
      id = data.at("request_id");
      require(id.is_string() && !id.get<std::string>().empty() &&
                  id.get<std::string>().size() <= 64,
              "invalid request_id");
      hash = digest(data);
      const auto key = id.get<std::string>();
      if (responses_.contains(key)) {
        require(responses_[key].first == hash,
                "request_id already used for different content");
        replies_->publish(string_message(responses_[key].second));
        return;
      }
      tick();
      Json result;
      const auto op = data.at("operation").get<std::string>();
      if (op == "capture") {
        evidence_.check_live();
        result = evidence_.capture(
            data.at("before").get<double>(), data.at("after").get<double>(),
            required_text(data.at("label"), "label").substr(0, 2000));
      } else if (op == "candidate")
        result = candidate({{"detector_id", data.at("detector_id")},
                            {"evidence_class", data.at("evidence_class")},
                            {"confidence", data.at("confidence")},
                            {"before_s", data.at("before")},
                            {"after_s", data.at("after")},
                            {"source", "test_injection"}});
      else if (op == "activate") {
        require(runtime_ != nullptr, "activation requires --store");
        result = runtime_->activate();
      } else if (op == "workflow") {
        require(store_ != nullptr, "workflow requests require --store");
        result = store_->apply(data.at("workflow_request"));
      } else if (op == "promote")
        result = evidence_.promote(data.at("window_id"), data.at("description"),
                                   data.at("operator"));
      else
        throw std::runtime_error("unknown operation");
      atomic_json(session_ / "state.json", evidence_.state);
      event(op, {{"request_id", id}, {"result", result}});
      response = {{"schema_version", 1},
                  {"request_id", id},
                  {"ok", true},
                  {"result", result}};
    } catch (const std::exception &error) {
      response = {{"schema_version", 1},
                  {"request_id", id},
                  {"ok", false},
                  {"error", error.what()}};
    }
    if (id.is_string() && !hash.empty() &&
        !responses_.contains(id.get<std::string>())) {
      const auto key = id.get<std::string>();
      responses_[key] = {hash, response};
      response_order_.push_back(key);
      if (response_order_.size() > 1000) {
        responses_.erase(response_order_.front());
        response_order_.pop_front();
      }
    }
    replies_->publish(string_message(response));
  }
  void status() {
    tick();
    evidence_.expire();
    bool fresh = true;
    try {
      evidence_.check_live();
    } catch (const std::exception &) {
      fresh = false;
    }
    Observations observations;
    observations.config = config_;
    observations.epoch = evidence_.state["epoch"].get<Ns>();
    observations.receipt_ros_ns = *evidence_.now;
    observations.messages.assign(history_.begin(), history_.end());
    if (fresh && store_)
      if (const auto detection = inference_->evaluate(observations)) {
        try {
          event("candidate_marked", {{"window", candidate(detection->json())}});
        } catch (const std::exception &e) {
          event("candidate_skipped", {{"reason", e.what()}});
        }
      }
    if (runtime_) {
      const auto result = runtime_->step(
          observations, static_cast<double>(monotonic_ns()) / 1e9, fresh);
      if (!result.is_null())
        event("runtime", {{"transition", result}});
      evidence_.state["runtime"] = runtime_->snapshot();
    }
    for (const auto &window : evidence_.state["windows"])
      if (window["status"] == "ready" &&
          !announced_.contains(window["id"].get<std::string>())) {
        writer_->close();
        event("window_ready", {{"window_id", window["id"]}});
        if (store_) {
          const auto detection = window.value(
              "detection", Json{{"detector_id", "operator"},
                                {"evidence_class", window["label"]},
                                {"confidence", 1.0},
                                {"source", "operator_mark"}});
          try {
            const auto result = store_->apply(
                {{"schema_version", 1},
                 {"request_id",
                  "candidate-" + id_ + "-" + window["id"].get<std::string>()},
                 {"actor", "edge_observer"},
                 {"expected_revision", store_->snapshot()["revision"]},
                 {"operation", "candidate"},
                 {"payload",
                  {{"window", window},
                   {"detection", detection},
                   {"session_id", id_},
                   {"binding_hash", digest(config_)}}}});
            event("candidate_pending", result);
          } catch (const std::exception &e) {
            event("candidate_registration_failed",
                  {{"window_id", window["id"]}, {"error", e.what()}});
            continue;
          }
        }
        announced_.insert(window["id"].get<std::string>());
      }
    if (store_) {
      const auto profile = store_->snapshot();
      if (profile["revision"] != published_revision_) {
        const auto message = string_message(profile);
        profiles_->publish(message);
        writer_->write(prefix_ + "/profile", "std_msgs/msg/String",
                       serialize(message), *evidence_.now,
                       evidence_.state["epoch"].get<Ns>());
        published_revision_ = profile["revision"];
      }
    }
    Json topics = Json::object();
    for (const auto &[name, count] : counts_)
      topics[name] = {
          {"count", count},
          {"last_receipt_age_wall_s",
           last_wall_.contains(name)
               ? Json(static_cast<double>(monotonic_ns() - last_wall_[name]) /
                      1e9)
               : Json(nullptr)},
          {"publishers", node_->count_publishers(name)}};
    evidence_.state.update({{"heartbeat_wall_ns", wall_ns()},
                            {"receipt_ros_ns", *evidence_.now},
                            {"topics", topics}});
    atomic_json(session_ / "state.json", evidence_.state);
    require(fs::space(session_).available >= 250000000,
            "recording stopped: less than 250 MB free space");
  }

public:
  Observer(const Options &options, fs::path session, Json config,
           rclcpp::Context::SharedPtr context)
      : options_(options), session_(std::move(session)),
        config_(std::move(config)), lock_(session_), evidence_(config_, id_),
        context_(std::move(context)) {
    require(!fs::exists(session_ / "session.json"),
            "session already exists; use a new directory (evidence is never "
            "overwritten)");
    require(fs::space(session_).available >= 1000000000,
            "at least 1 GB free space required");
    if (options_.values.contains("--store")) {
      store_ = std::make_unique<Store>(options_.path("--store"));
      require(store_->snapshot()["binding_hash"] == digest(config_),
              "observer config differs from workflow binding");
      const auto deployment = store_->snapshot().value("critic_deployment", Json(nullptr));
      if (deployment.is_object() && deployment.value("runtime", "") == "bootstrap_mlp_v1") {
        bootstrap_critic_ = std::make_unique<BootstrapCritic>(deployment);
        inference_ = bootstrap_critic_.get();
      }
      runtime_ =
          std::make_unique<Runtime>(*store_, runtime_critic_, controller_);
    }
    rclcpp::NodeOptions node_options;
    node_options.context(context_);
    node_options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", config_["clock"] == "ros_sim")});
    node_ = std::make_shared<rclcpp::Node>(
        "harness_observer_" + id_.substr(0, 8), node_options);
    const auto *rmw = std::getenv("RMW_IMPLEMENTATION");
    Json meta = {{"schema_version", 1},
                 {"session_id", id_},
                 {"domain_id", options_.integer("--domain-id", 71)},
                 {"prefix", prefix_},
                 {"config", config_},
                 {"started_wall_ns", wall_ns()},
                 {"rmw", rmw ? rmw : "default"},
                 {"timestamp_contract", "bag timestamps are receipt ROS time; "
                                        "message headers retain source time"},
                 {"workflow_store",
                  store_ ? Json(store_->root.string()) : Json(nullptr)}};
    atomic_json(session_ / "session.json", meta);
    atomic_json(session_ / "state.json", evidence_.state);
    auto topics = config_["topics"];
    topics.push_back(
        {{"name", prefix_ + "/events"}, {"type", "std_msgs/msg/String"}});
    if (store_)
      topics.push_back(
          {{"name", prefix_ + "/profile"}, {"type", "std_msgs/msg/String"}});
    if (config_["clock"] == "ros_sim")
      topics.push_back(
          {{"name", "/clock"}, {"type", "rosgraph_msgs/msg/Clock"}});
    fs::create_directories(session_ / "rosbag2");
    writer_ = std::make_unique<Recording>(session_, topics,
                                          options_.get("--storage", "mcap"));
    journal_.open(session_ / "events.jsonl", std::ios::app);
    require(journal_.good(), "cannot open events journal");
    events_ = node_->create_publisher<String>(prefix_ + "/events", 10);
    replies_ = node_->create_publisher<String>(prefix_ + "/responses", 10);
    if (store_)
      profiles_ = node_->create_publisher<String>(
          prefix_ + "/profile", rclcpp::QoS(1).transient_local().reliable());
    for (const auto &topic : topics) {
      const auto name = topic["name"].get<std::string>();
      if (name.starts_with(prefix_))
        continue;
      if (name != "/clock")
        counts_[name] = 0;
      subscriptions_.push_back(node_->create_generic_subscription(
          name, topic["type"], rclcpp::SensorDataQoS(),
          [this, topic](std::shared_ptr<rclcpp::SerializedMessage> data) {
            receive(topic, data);
          }));
    }
    requests_ = node_->create_subscription<String>(
        prefix_ + "/requests", 10, [this](const String &m) { request(m); });
  }
  void run() {
    const auto started = monotonic_ns();
    Ns next_status = started;
    std::string termination = "wall_timeout";
    evidence_.state["status"] = "observing";
    try {
      Json critics = Json::array();
      if (bootstrap_critic_)
        critics.push_back(store_->snapshot()["critic_deployment"]);
      event("observer_started", {{"session", session_.string()},
                                 {"critics", critics},
                                 {"recoveries", Json::array()}});
      rclcpp::ExecutorOptions executor_options;
      executor_options.context = context_;
      rclcpp::executors::SingleThreadedExecutor executor(executor_options);
      executor.add_node(node_);
      while (rclcpp::ok(context_) &&
             static_cast<double>(monotonic_ns() - started) / 1e9 <
                 options_.number("--wall-timeout", 1800)) {
        executor.spin_once(std::chrono::milliseconds(100));
        if (monotonic_ns() >= next_status) {
          status();
          next_status = monotonic_ns() + 1000000000LL;
        }
      }
      if (!rclcpp::ok(context_))
        termination = "interrupted";
    } catch (...) {
      finish("error");
      throw;
    }
    finish(termination);
  }
  void finish(const std::string &reason) {
    if (runtime_)
      runtime_->deactivate();
    evidence_.state.update(
        {{"status", reason == "error" ? "failed" : "stopped"},
         {"termination_reason", reason}});
    for (auto &window : evidence_.state["windows"])
      if (window["status"] == "collecting")
        window["status"] = "incomplete";
    atomic_json(session_ / "state.json", evidence_.state);
    writer_->close();
    journal_.close();
  }
};
} // namespace
Json ros_start(const Options &options) {
  const auto config = load_config(options.need("--config"));
  const auto session = options.path("--session");
  require(options.number("--wall-timeout", 1800) > 0,
          "wall-timeout must be positive");
  const auto storage = options.get("--storage", "mcap");
  require(storage == "mcap" || storage == "sqlite3",
          "storage must be mcap or sqlite3");
  RosContext context(options.integer("--domain-id", 71));
  fs::create_directories(session);
  Observer observer(options, session, config, context.value);
  observer.run();
  return nullptr;
}
Json discover_domain(Ns domain, Ns settle_ms) {
  // Setup discovery creates many short-lived contexts in parallel. ROS logging
  // is process-global, so initializing it from each context produces a warning
  // for every probed domain. The scanner does not emit ROS log records.
  RosContext context(domain, false);
  const auto scanner_name = "harness_setup_scan_" + unique_id().substr(0, 8);
  rclcpp::NodeOptions node_options;
  node_options.context(context.value);
  auto node = std::make_shared<rclcpp::Node>(scanner_name, node_options);

  // DDS discovery is asynchronous. This is a single bounded wait, never a
  // ros2 CLI subprocess per artifact. The graph is then copied in one pass.
  const auto deadline = monotonic_ns() + settle_ms * 1000000LL;
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context.value;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(node);
  while (rclcpp::ok(context.value) && monotonic_ns() < deadline)
    executor.spin_once(std::chrono::milliseconds(25));

  auto fq_name = [](const std::string &name, const std::string &space) {
    if (space.empty() || space == "/")
      return "/" + name;
    return space + (space.ends_with('/') ? "" : "/") + name;
  };
  auto endpoint = [&](const rclcpp::TopicEndpointInfo &info) {
    return Json{{"node", fq_name(info.node_name(), info.node_namespace())},
                {"type", info.topic_type()}};
  };

  Json nodes = Json::array();
  std::set<std::string> controller_nodes;
  std::map<std::string, std::set<std::string>> service_servers;
  const auto graph_nodes =
      node->get_node_graph_interface()->get_node_names_and_namespaces();
  for (const auto &[name, space] : graph_nodes) {
    if (name == scanner_name)
      continue;
    const auto full = fq_name(name, space);
    nodes.push_back({{"name", name}, {"namespace", space}, {"fq_name", full}});
    if (name.find("controller") != std::string::npos ||
        space.find("controller") != std::string::npos)
      controller_nodes.insert(full);
    for (const auto &[service, types] :
         node->get_service_names_and_types_by_node(name, space)) {
      (void)types;
      service_servers[service].insert(full);
    }
  }
  std::sort(nodes.begin(), nodes.end(), [](const Json &a, const Json &b) {
    return a.at("fq_name") < b.at("fq_name");
  });

  Json topics = Json::array();
  for (const auto &[name, types] : node->get_topic_names_and_types()) {
    Json publishers = Json::array(), subscribers = Json::array();
    for (const auto &info : node->get_publishers_info_by_topic(name))
      if (info.node_name() != scanner_name)
        publishers.push_back(endpoint(info));
    for (const auto &info : node->get_subscriptions_info_by_topic(name))
      if (info.node_name() != scanner_name)
        subscribers.push_back(endpoint(info));
    if (publishers.empty() && subscribers.empty())
      continue;
    const auto by_endpoint = [](const Json &a, const Json &b) {
      const auto &a_node = a.at("node").get_ref<const std::string &>();
      const auto &b_node = b.at("node").get_ref<const std::string &>();
      if (a_node != b_node)
        return a_node < b_node;
      return a.at("type").get_ref<const std::string &>() <
             b.at("type").get_ref<const std::string &>();
    };
    std::sort(publishers.begin(), publishers.end(), by_endpoint);
    std::sort(subscribers.begin(), subscribers.end(), by_endpoint);
    auto sorted_types = types;
    std::sort(sorted_types.begin(), sorted_types.end());
    topics.push_back({{"name", name},
                      {"types", sorted_types},
                      {"publisher_count", publishers.size()},
                      {"subscription_count", subscribers.size()},
                      {"publishers", publishers},
                      {"subscribers", subscribers}});
  }
  std::sort(topics.begin(), topics.end(), [](const Json &a, const Json &b) {
    return a.at("name") < b.at("name");
  });

  // One bounded, passive capture per topic. Never call services or send goals.
  // Keep previews small enough for the setup inventory transport.
  std::vector<rclcpp::GenericSubscription::SharedPtr> sample_subscriptions;
  std::size_t sample_budget = 2 * 1024 * 1024;
  std::size_t camera_sample_budget = 16 * 1024 * 1024;
  const auto hex = [](const std::uint8_t *bytes, std::size_t length) {
    constexpr char digits[] = "0123456789abcdef";
    std::string result;
    result.reserve(length * 2);
    for (std::size_t i = 0; i < length; ++i) {
      result.push_back(digits[bytes[i] >> 4]);
      result.push_back(digits[bytes[i] & 15]);
    }
    return result;
  };
  for (std::size_t index = 0; index < topics.size(); ++index) {
    auto &topic = topics[index];
    topic["sample"] = {{"status", "timeout"}};
    if (topic["types"].size() != 1 || topic["publishers"].empty()) {
      topic["sample"] = {{"status", "unavailable"},
                         {"error", "Requires one unambiguous type and a publisher."}};
      continue;
    }
    const auto type = topic["types"][0].get<std::string>();
    try {
      sample_subscriptions.push_back(node->create_generic_subscription(
          topic["name"].get<std::string>(), type,
          rclcpp::QoS(1).best_effort().durability_volatile(),
          [&, index, type](std::shared_ptr<rclcpp::SerializedMessage> message) {
            auto &sample = topics[index]["sample"];
            if (sample["status"] != "timeout") return;
            const auto &bytes = message->get_rcl_serialized_message();
            auto &budget = type == "sensor_msgs/msg/Image" ||
                                   type == "sensor_msgs/msg/CompressedImage"
                               ? camera_sample_budget
                               : sample_budget;
            if (budget < 1024 || bytes.buffer_length > 32 * 1024 * 1024) {
              sample = {{"status", "unavailable"}, {"error", "Setup sample size limit reached."}};
              return;
            }
            sample = {{"status", "captured"}, {"captured_wall_ns", wall_ns()},
                      {"byte_length", bytes.buffer_length}};
            try {
              if (type == "sensor_msgs/msg/Image") {
                const auto frame = decode<sensor_msgs::msg::Image>(*message);
                sample["data"] = {{"width", frame.width}, {"height", frame.height},
                                  {"encoding", frame.encoding}, {"step", frame.step},
                                  {"frame_id", frame.header.frame_id}};
                const bool mono = frame.encoding == "mono8";
                const bool bgr = frame.encoding == "bgr8" || frame.encoding == "bgra8";
                const std::size_t channels = mono ? 1 :
                    (frame.encoding == "rgba8" || frame.encoding == "bgra8" ? 4 : 3);
                const bool supported = mono || bgr || frame.encoding == "rgb8" || frame.encoding == "rgba8";
                if (supported && frame.width && frame.height &&
                    frame.step >= static_cast<std::size_t>(frame.width) * channels &&
                    frame.data.size() >= static_cast<std::size_t>(frame.step) * frame.height &&
                    budget >= 320 * 240 * 6 + 1024) {
                  const auto scale = std::max({1U, (frame.width + 319) / 320, (frame.height + 239) / 240});
                  const auto width = (frame.width + scale - 1) / scale;
                  const auto height = (frame.height + scale - 1) / scale;
                  std::vector<std::uint8_t> rgb;
                  rgb.reserve(width * height * 3);
                  for (std::uint32_t y = 0; y < height; ++y)
                    for (std::uint32_t x = 0; x < width; ++x) {
                      const auto offset = static_cast<std::size_t>(y * scale) * frame.step + x * scale * channels;
                      rgb.push_back(frame.data[offset + (mono ? 0 : bgr ? 2 : 0)]);
                      rgb.push_back(frame.data[offset + (mono ? 0 : 1)]);
                      rgb.push_back(frame.data[offset + (mono ? 0 : bgr ? 0 : 2)]);
                    }
                  sample["image"] = {{"width", width}, {"height", height},
                                     {"rgb_hex", hex(rgb.data(), rgb.size())}};
                }
              } else if (type == "sensor_msgs/msg/CompressedImage") {
                const auto frame = decode<sensor_msgs::msg::CompressedImage>(*message);
                sample["data"] = {{"format", frame.format},
                                  {"frame_id", frame.header.frame_id}};
                if (frame.data.size() <= 2 * 1024 * 1024 &&
                    budget >= frame.data.size() * 2 + 1024)
                  sample["compressed_image"] = {
                      {"format", frame.format},
                      {"data_hex", hex(frame.data.data(), frame.data.size())}};
              } else if (type == "sensor_msgs/msg/JointState") {
                const auto state = decode<sensor_msgs::msg::JointState>(*message);
                sample["data"] = {{"name", state.name}, {"position", state.position},
                                  {"velocity", state.velocity}, {"effort", state.effort}};
              } else if (type == "std_msgs/msg/String") {
                const auto value = decode<String>(*message).data;
                sample["data"] = value.substr(0, 4096);
                sample["truncated"] = value.size() > 4096;
              }
            } catch (const std::exception &error) {
              sample["error"] = error.what();
            }
            if (!sample.contains("data") || type == "sensor_msgs/msg/Image") {
              const auto length = std::min<std::size_t>(bytes.buffer_length, 4096);
              sample["cdr_hex"] = hex(bytes.buffer, length);
              sample["truncated"] = length < bytes.buffer_length;
            }
            const auto size = sample.dump().size();
            if (size > budget) {
              sample = {{"status", "unavailable"}, {"error", "Setup sample size limit reached."}};
            } else {
              budget -= size;
            }
          }));
    } catch (const std::exception &error) {
      topic["sample"] = {{"status", "unavailable"}, {"error", error.what()}};
    }
  }
  const auto sample_deadline = monotonic_ns() + 1500000000LL;
  while (!sample_subscriptions.empty() && rclcpp::ok(context.value) &&
         monotonic_ns() < sample_deadline) {
    executor.spin_once(std::chrono::milliseconds(25));
    if (std::none_of(topics.begin(), topics.end(), [](const Json &topic) {
          return topic["sample"]["status"] == "timeout";
        })) break;
  }
  sample_subscriptions.clear();

  Json services = Json::array();
  std::map<std::string, Json> action_services;
  std::set<std::string> controller_managers;
  const std::array<std::string, 5> action_suffixes = {
      "/_action/send_goal", "/_action/get_result", "/_action/cancel_goal",
      "/_action/feedback", "/_action/status"};
  for (const auto &[name, types] : node->get_service_names_and_types()) {
    if (!service_servers.contains(name) || service_servers[name].empty())
      continue;
    auto sorted_types = types;
    std::sort(sorted_types.begin(), sorted_types.end());
    Json servers = Json::array();
    for (const auto &server : service_servers[name])
      servers.push_back(server);
    services.push_back({{"name", name},
                        {"types", sorted_types},
                        {"server_count", servers.size()},
                        {"servers", servers}});
    for (const auto &suffix : action_suffixes) {
      if (name.ends_with(suffix)) {
        const auto base = name.substr(0, name.size() - suffix.size());
        auto &entry = action_services[base];
        if (entry.is_null())
          entry = {{"name", base}, {"endpoints", Json::array()}};
        entry["endpoints"].push_back(name.substr(base.size() + 1));
      }
    }
    const auto marker = name.find("/controller_manager/");
    if (marker != std::string::npos)
      controller_managers.insert(name.substr(0, marker + 19));
  }
  std::sort(services.begin(), services.end(), [](const Json &a, const Json &b) {
    return a.at("name") < b.at("name");
  });
  Json actions = Json::array();
  for (auto &[name, action] : action_services) {
    (void)name;
    auto &endpoints = action["endpoints"];
    std::sort(endpoints.begin(), endpoints.end());
    actions.push_back(std::move(action));
  }

  struct utsname system {};
  const bool have_uname = ::uname(&system) == 0;
  char hostname[256]{};
  const bool have_hostname = ::gethostname(hostname, sizeof(hostname) - 1) == 0;
  const auto env = [](const char *name) -> Json {
    const auto *value = std::getenv(name);
    return value && *value ? Json(value) : Json(nullptr);
  };
  const long pages = ::sysconf(_SC_PHYS_PAGES);
  const long page_size = ::sysconf(_SC_PAGE_SIZE);
  Json device = {
      {"hostname", have_hostname ? Json(hostname) : Json(nullptr)},
      {"os", have_uname ? Json(system.sysname) : Json(nullptr)},
      {"kernel", have_uname ? Json(system.release) : Json(nullptr)},
      {"architecture", have_uname ? Json(system.machine) : Json(nullptr)},
      {"cpu_threads", std::thread::hardware_concurrency()},
      {"memory_bytes", pages > 0 && page_size > 0
                           ? Json(static_cast<std::uint64_t>(pages) *
                                  static_cast<std::uint64_t>(page_size))
                           : Json(nullptr)}};
  Json managers = Json::array();
  for (const auto &manager : controller_managers)
    managers.push_back(manager);
  Json controller_node_list = Json::array();
  for (const auto &controller : controller_nodes)
    controller_node_list.push_back(controller);

  Json result = {
      {"schema", "rearguard.ros_setup_inventory"},
      {"schema_version", 1},
      {"captured_wall_ns", wall_ns()},
      {"domain_id", domain},
      {"settle_ms", settle_ms},
      {"ros", {{"distro", env("ROS_DISTRO")},
               {"rmw", env("RMW_IMPLEMENTATION")},
               {"localhost_only", env("ROS_LOCALHOST_ONLY")}}},
      {"device", device},
      {"nodes", nodes},
      {"topics", topics},
      {"services", services},
      {"actions", actions},
      {"controllers", {{"manager_surfaces", managers},
                        {"controller_named_nodes", controller_node_list},
                        {"note", "Controller identities and states require a "
                                 "controller_manager service query; this graph "
                                 "snapshot records the discovered surfaces."}}}};
  result["inventory_hash"] = digest(result);
  return result;
}
Json ros_discover(const Options &options) {
  const auto first = options.integer("--domain-min", 0);
  const auto last = options.integer("--domain-max", 232);
  const auto settle_ms = options.integer("--settle-ms", 750);
  const auto parallelism = options.integer("--parallelism", 16);
  require(first >= 0 && last <= 232 && first <= last,
          "domain range must be within 0..232 and nonempty");
  require(parallelism >= 1 && parallelism <= 32,
          "parallelism must be between 1 and 32");

  Json domains = Json::array(), ids = Json::array(), first_snapshot = nullptr;
  for (Ns batch = first; batch <= last; batch += parallelism) {
    std::vector<std::future<Json>> pending;
    const auto end = std::min(last + 1, batch + parallelism);
    for (Ns domain = batch; domain < end; ++domain)
      pending.push_back(std::async(std::launch::async, [domain, settle_ms] {
        return discover_domain(domain, settle_ms);
      }));
    for (auto &future : pending) {
      auto snapshot = future.get();
      if (first_snapshot.is_null())
        first_snapshot = snapshot;
      if (snapshot["nodes"].empty() && snapshot["topics"].empty() &&
          snapshot["services"].empty())
        continue;
      ids.push_back(snapshot["domain_id"]);
      snapshot.erase("schema");
      snapshot.erase("schema_version");
      snapshot.erase("ros");
      snapshot.erase("device");
      snapshot.erase("inventory_hash");
      domains.push_back(std::move(snapshot));
    }
  }
  Json result = {
      {"schema", "rearguard.ros_setup_inventory"},
      {"schema_version", 2},
      {"captured_wall_ns", wall_ns()},
      {"scan", {{"domain_min", first},
                {"domain_max", last},
                {"settle_ms", settle_ms},
                {"parallelism", parallelism}}},
      {"available_domain_ids", ids},
      {"domains", domains},
      {"ros", first_snapshot["ros"]},
      {"device", first_snapshot["device"]}};
  result["inventory_hash"] = digest(result);
  return result;
}
Json ros_request(const Options &options, const Json &meta) {
  Json data = {{"schema_version", 1},
               {"request_id", unique_id()},
               {"operation", options.command}};
  if (options.command == "capture" || options.command == "candidate") {
    data.update({{"before", options.number("--before", 5)},
                 {"after", options.number("--after", 3)},
                 {"label", options.get("--label", "critic candidate")}});
    if (options.command == "candidate")
      data.update(
          {{"detector_id", options.get("--detector-id", "manual-adapter-test")},
           {"evidence_class", options.need("--evidence-class")},
           {"confidence", options.number("--confidence", -1)}});
  } else if (options.command == "promote")
    data.update({{"window_id", options.need("--window")},
                 {"description", options.need("--description")},
                 {"operator", options.need("--operator")}});
  RosContext context(meta.at("domain_id").get<Ns>());
  rclcpp::NodeOptions node_options;
  node_options.context(context.value);
  auto node = std::make_shared<rclcpp::Node>(
      "harness_observation_cli_" + unique_id().substr(0, 8), node_options);
  const auto prefix = meta.at("prefix").get<std::string>();
  Json response = nullptr;
  auto subscription = node->create_subscription<String>(
      prefix + "/responses", 10, [&](const String &message) {
        try {
          auto value = Json::parse(message.data);
          if (value.is_object() && value.value("schema_version", 0) == 1 &&
              value.value("request_id", Json()) == data["request_id"])
            response = value;
        } catch (const std::exception &) {
        }
      });
  auto publisher = node->create_publisher<String>(prefix + "/requests", 10);
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context.value;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(node);
  const auto deadline = monotonic_ns() + 15000000000LL;
  Ns next_send = 0;
  while (rclcpp::ok(context.value) && response.is_null() &&
         monotonic_ns() < deadline) {
    executor.spin_once(std::chrono::milliseconds(100));
    if (publisher->get_subscription_count() && monotonic_ns() >= next_send) {
      publisher->publish(string_message(data));
      next_send = monotonic_ns() + 1000000000LL;
    }
  }
  require(!response.is_null(), "no observer acknowledgement in 15 seconds; "
                               "inspect state before retrying");
  return response;
}
} // namespace harness::observation
