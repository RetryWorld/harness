#include "common.hpp"
#include "evidence.hpp"
#include "recording.hpp"
#include "runtime.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <future>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/image.hpp>
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
  explicit RosContext(Ns domain) {
    require(domain >= 0 && domain <= 232, "domain-id must be within 0..232");
    rclcpp::InitOptions init;
    init.set_domain_id(static_cast<std::size_t>(domain));
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
  CriticInferencePlaceholder inference_, runtime_critic_;
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
      if (const auto detection = inference_.evaluate(observations)) {
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
      event("observer_started", {{"session", session_.string()},
                                 {"critics", Json::array()},
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
  RosContext context(domain);
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
  const auto graph_nodes = node->get_node_names_and_namespaces();
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
