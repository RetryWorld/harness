#include "common.hpp"
#include "critic_features.hpp"
#include "evidence.hpp"
#include "recording.hpp"
#include "runtime.hpp"
#include "events.hpp"
#include "service.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <fcntl.h>
#include <fstream>
#include <future>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rcutils/error_handling.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <set>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/file.h>
#include <sys/wait.h>
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
Json json_numbers(const std::vector<double> &values) {
  Json result = Json::array();
  for (const auto value : values)
    result.push_back(std::isfinite(value) ? Json(value) : Json(nullptr));
  return result;
}
template <class T> T decode_bytes(const std::vector<std::uint8_t> &bytes) {
  rclcpp::SerializedMessage serialized(bytes.size());
  auto &raw = serialized.get_rcl_serialized_message();
  require(raw.buffer_capacity >= bytes.size(), "serialized message allocation failed");
  std::memcpy(raw.buffer, bytes.data(), bytes.size());
  raw.buffer_length = bytes.size();
  return decode<T>(serialized);
}
std::vector<double> ordered(const std::vector<std::string> &names,
                            const std::vector<double> &values,
                            const std::vector<std::string> &order) {
  if (values.empty()) return {};
  require(names.size() == values.size(), "JointState field size mismatch");
  std::map<std::string, std::size_t> indexes;
  for (std::size_t i = 0; i < names.size(); ++i)
    require(indexes.emplace(names[i], i).second, "duplicate JointState name");
  std::vector<double> result;
  result.reserve(order.size());
  for (const auto &name : order) {
    require(indexes.contains(name), "JointState is missing bound joint: " + name);
    result.push_back(values[indexes.at(name)]);
  }
  return result;
}
void add_joint(CriticFeatureAccumulator &features,
               const sensor_msgs::msg::JointState &message,
               const std::vector<std::string> &joints) {
  features.joint(ordered(message.name, message.position, joints),
                 ordered(message.name, message.velocity, joints),
                 ordered(message.name, message.effort, joints));
}
void add_image(CriticFeatureAccumulator &features,
               const sensor_msgs::msg::Image &message) {
  const auto channels = message.encoding == "mono8" ? 1U :
      (message.encoding == "rgb8" || message.encoding == "bgr8") ? 3U :
      (message.encoding == "rgba8" || message.encoding == "bgra8") ? 4U : 0U;
  require(channels != 0, "unsupported critic image encoding: " + message.encoding);
  features.image(message.data, message.width, message.height, message.step,
                 channels, message.encoding == "bgr8" ||
                               message.encoding == "bgra8");
}

class ProfileCritic final : public CriticAdapter {
  Json deployment_;
  std::vector<Ns> last_evaluation_;
  std::vector<std::uint64_t> missed_deadlines_;
  Ns epoch_ = -1;
  Json telemetry_ = {{"score_kind", "uncalibrated_cosine_similarity"},
                     {"probability_calibrated", false},
                     {"scores", Json::object()}};

public:
  explicit ProfileCritic(Json deployment) : deployment_(std::move(deployment)) {
    require(deployment_.value("runtime", "") == "proposition_bank_v1" &&
                deployment_.value("encoder", "") == critic_feature_encoder &&
                deployment_.value("executable", false) &&
                deployment_.contains("heads") && deployment_["heads"].is_array(),
            "invalid executable proposition deployment");
    last_evaluation_.assign(deployment_["heads"].size(), -1);
    missed_deadlines_.assign(deployment_["heads"].size(), 0);
  }
  bool ready() const override { return true; }
  bool load(const fs::path &, const Json &, const Json &) override { return false; }
  Json telemetry() const override { return telemetry_; }
  std::optional<Detection> evaluate(const Observations &observations) override {
    if (observations.epoch != epoch_) {
      epoch_ = observations.epoch;
      std::fill(last_evaluation_.begin(), last_evaluation_.end(), -1);
    }
    std::vector<bool> due(deployment_["heads"].size(), false);
    std::uint32_t due_roles = 0;
    const auto evaluation_started = monotonic_ns();
    for (std::size_t i = 0; i < deployment_["heads"].size(); ++i) {
      const auto &head = deployment_["heads"][i];
      const auto period = static_cast<Ns>(std::llround(
          1e9 / head["scope"]["required_hz"].get<double>()));
      if (last_evaluation_[i] < 0 ||
          observations.receipt_ros_ns - last_evaluation_[i] >= period) {
        if (last_evaluation_[i] >= 0 &&
            observations.receipt_ros_ns - last_evaluation_[i] >
                period + period / 2)
          ++missed_deadlines_[i];
        due[i] = true;
        due_roles |= head["required_roles_mask"].get<std::uint32_t>();
        last_evaluation_[i] = observations.receipt_ros_ns;
      }
    }
    if (due_roles == 0) return std::nullopt;
    CriticFeatureAccumulator features;
    std::map<std::string, Json> topics;
    for (const auto &topic : observations.config["topics"])
      topics[topic["name"].get<std::string>()] = topic;
    std::map<std::string, const ObservationMessage *> latest_images;
    std::array<Ns, 5> role_times{};
    role_times.fill(-1);
    const auto joints =
        observations.config["joint_order"].get<std::vector<std::string>>();
    const auto start = std::max<Ns>(0, observations.receipt_ros_ns - 1000000000LL);
    for (const auto &packet : observations.messages) {
      if (packet.receipt_ros_ns < start || !topics.contains(packet.topic)) continue;
      const auto role = topics.at(packet.topic).value("role", "");
      const auto bit = critic_role_bit(role);
      if ((due_roles & bit) == 0) continue;
      if (bit != 0)
        role_times[static_cast<std::size_t>(std::countr_zero(bit))] =
            packet.receipt_ros_ns;
      if (role == "joint_state")
        add_joint(features,
                  decode_bytes<sensor_msgs::msg::JointState>(*packet.cdr), joints);
      else if (role == "action")
        features.action(
            decode_bytes<std_msgs::msg::Float64MultiArray>(*packet.cdr).data);
      else if (role == "camera")
        latest_images[packet.topic] = &packet;
    }
    for (const auto &[name, packet] : latest_images) {
      (void)name;
      add_image(features, decode_bytes<sensor_msgs::msg::Image>(*packet->cdr));
    }
    std::optional<Detection> best;
    std::map<std::uint32_t,
             std::array<float, critic_embedding_dimensions>> embeddings;
    for (std::size_t i = 0; i < deployment_["heads"].size(); ++i) {
      if (!due[i]) continue;
      const auto &head = deployment_["heads"][i];
      const auto mask = head["required_roles_mask"].get<std::uint32_t>();
      const auto age = static_cast<Ns>(std::llround(
          head["scope"]["max_input_age_ms"].get<double>() * 1e6));
      bool fresh = true;
      for (std::size_t role = 0; role < role_times.size(); ++role)
        if ((mask & (1U << role)) != 0)
          fresh = fresh && role_times[role] >= 0 &&
                  observations.receipt_ros_ns - role_times[role] <= age;
      if (!fresh) {
        telemetry_["scores"][head["proposition_id"].get<std::string>()] =
            {{"status", "insufficient_evidence"}, {"probability", nullptr},
             {"missed_deadlines", missed_deadlines_[i]}};
        continue;
      }
      if (!embeddings.contains(mask)) embeddings[mask] = features.embedding(mask);
      const auto &embedding = embeddings.at(mask);
      float similarity = -1;
      for (const auto &center : head["centers"]) {
        require(center.is_array() && center.size() == embedding.size(),
                "invalid proposition center");
        float dot = 0;
        for (std::size_t d = 0; d < embedding.size(); ++d)
          dot += embedding[d] * center[d].get<float>();
        similarity = std::max(similarity, std::clamp(dot, -1.F, 1.F));
      }
      const auto confidence = (static_cast<double>(similarity) + 1.0) / 2.0;
      telemetry_["scores"][head["proposition_id"].get<std::string>()] =
          {{"status", "score"}, {"similarity", similarity},
           {"confidence_proxy", confidence}, {"probability", nullptr},
           {"threshold", head.value("candidate_threshold", 0.95)},
           {"missed_deadlines", missed_deadlines_[i]},
           {"receipt_ros_ns", observations.receipt_ros_ns}};
      if (confidence < head.value("candidate_threshold", 0.9)) continue;
      if (!best || confidence > best->confidence)
        best = Detection{deployment_["id"], head["proposition_id"], confidence,
                         5, 2, "profile_proposition_bank_v1",
                         deployment_["id"], false};
    }
    telemetry_["last_evaluation_us"] =
        static_cast<double>(monotonic_ns() - evaluation_started) / 1000.0;
    telemetry_["deployment_id"] = deployment_["id"];
    return best;
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
  std::unique_ptr<ProfileCritic> profile_critic_;
  RecoveryControllerPlaceholder controller_;
  std::unique_ptr<Runtime> runtime_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Context::SharedPtr context_;
  std::unique_ptr<Recording> writer_;
  rclcpp::Publisher<String>::SharedPtr events_, replies_, profiles_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  rclcpp::Subscription<String>::SharedPtr requests_;
  rclcpp::TimerBase::SharedPtr critic_timer_;
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
                          std::make_shared<const std::vector<std::uint8_t>>(
                              bytes.buffer, bytes.buffer + bytes.buffer_length),
                          now});
      raw_bytes_ += bytes.buffer_length;
      while (!history_.empty() &&
             (raw_bytes_ > 16 * 1024 * 1024 ||
              now - history_.front().receipt_ros_ns > 20000000000LL)) {
        raw_bytes_ -= history_.front().cdr->size();
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
  void critic_tick() {
    tick();
    if (!evidence_.now || !store_) return;
    bool fresh = true;
    try { evidence_.check_live(); }
    catch (const std::exception &) { fresh = false; }
    Observations observations;
    observations.config = config_;
    observations.epoch = evidence_.state["epoch"].get<Ns>();
    observations.receipt_ros_ns = *evidence_.now;
    observations.messages.assign(history_.begin(), history_.end());
    if (fresh)
      if (const auto detection = inference_->evaluate(observations)) {
        try { event("candidate_marked", {{"window", candidate(detection->json())}}); }
        catch (const std::exception &error) {
          event("candidate_skipped", {{"reason", error.what()}});
        }
      }
    if (runtime_) {
      const auto result = runtime_->step(
          observations, static_cast<double>(monotonic_ns()) / 1e9, fresh);
      if (!result.is_null()) event("runtime", {{"transition", result}});
      evidence_.state["runtime"] = runtime_->snapshot();
    }
    evidence_.state["critic"] = inference_->telemetry();
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
      const auto profile = store_->snapshot();
      require(profile["binding_hash"] == digest(config_),
              "observer config differs from workflow binding");
      const auto deployment = profile.value("critic_deployment", Json(nullptr));
      if (deployment.is_object() && deployment.value("executable", false)) {
        profile_critic_ = std::make_unique<ProfileCritic>(deployment);
        inference_ = profile_critic_.get();
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
    if (profile_critic_) {
      double rate = 1;
      for (const auto &head : store_->snapshot()["critic_deployment"]["heads"])
        rate = std::max(rate, head["scope"]["required_hz"].get<double>());
      const auto period = std::chrono::nanoseconds(
          static_cast<Ns>(std::llround(1e9 / std::min(rate, 2000.0))));
      critic_timer_ = node_->create_wall_timer(period, [this] { critic_tick(); });
    }
  }
  void run() {
    const auto started = monotonic_ns();
    Ns next_status = started;
    std::string termination = "wall_timeout";
    evidence_.state["status"] = "observing";
    try {
      Json critics = Json::array();
      if (profile_critic_)
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
        executor.spin_once(std::chrono::milliseconds(1));
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
Json compile_snapshot_embedding(const fs::path &artifact, const Json &binding,
                                const Json &window,
                                std::uint32_t required_roles) {
  constexpr auto supported = critic_joint_state | critic_action | critic_camera;
  if ((required_roles & ~supported) != 0)
    return {{"status", "unsupported_modalities"},
            {"error", "first-deploy encoder supports joint_state, action and camera roles"}};
  require(fs::is_regular_file(artifact), "accepted MCAP artifact is missing");
  std::map<std::string, Json> topics;
  for (const auto &topic : binding["topics"])
    topics[topic["name"].get<std::string>()] = topic;
  const auto joints = binding["joint_order"].get<std::vector<std::string>>();
  const auto end = window.contains("mark_ns")
                       ? window["mark_ns"].get<Ns>()
                       : window.at("end_ns").get<Ns>();
  const auto begin =
      std::max(window.at("start_ns").get<Ns>(), end - 1000000000LL);
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage;
  storage.uri = artifact.string();
  storage.storage_id = "mcap";
  reader.open(storage, {"cdr", "cdr"});
  std::map<std::string, std::string> types;
  for (const auto &topic : reader.get_all_topics_and_types())
    types[topic.name] = topic.type;
  CriticFeatureAccumulator features;
  std::map<std::string, std::vector<std::uint8_t>> latest_images;
  while (reader.has_next()) {
    const auto message = reader.read_next();
    if (message->recv_timestamp < begin || message->recv_timestamp > end ||
        !topics.contains(message->topic_name))
      continue;
    const auto &topic = topics.at(message->topic_name);
    const auto role = topic.value("role", "");
    if ((required_roles & critic_role_bit(role)) == 0) continue;
    require(types.at(message->topic_name) == topic.at("type"),
            "accepted MCAP schema differs from profile binding");
    const auto &raw = *message->serialized_data;
    require(raw.buffer_length <= 64 * 1024 * 1024,
            "accepted MCAP packet exceeds size limit");
    std::vector<std::uint8_t> bytes(raw.buffer, raw.buffer + raw.buffer_length);
    if (role == "joint_state")
      add_joint(features, decode_bytes<sensor_msgs::msg::JointState>(bytes), joints);
    else if (role == "action")
      features.action(
          decode_bytes<std_msgs::msg::Float64MultiArray>(bytes).data);
    else if (role == "camera")
      latest_images[message->topic_name] = std::move(bytes);
  }
  for (const auto &[name, bytes] : latest_images) {
    (void)name;
    add_image(features, decode_bytes<sensor_msgs::msg::Image>(bytes));
  }
  if ((features.roles() & required_roles) != required_roles)
    return {{"status", "insufficient_evidence"},
            {"available_roles_mask", features.roles()},
            {"required_roles_mask", required_roles}};
  const auto vector = features.embedding(required_roles);
  Json values = Json::array();
  for (const auto value : vector) values.push_back(value);
  return {{"status", "ready"},
          {"encoder", critic_feature_encoder},
          {"roles_mask", features.roles()},
          {"window", {{"start_ns", begin}, {"end_ns", end}}},
          {"vector", std::move(values)}};
}
bool snapshot_compiler_available() { return true; }

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
  // Each setup domain runs in its own worker process. ROS logging and dynamic
  // typesupport own process-global state, so domain contexts must not initialize
  // them concurrently in one process.
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
            if (sample["status"] != "timeout") {
              // Preserve a short numeric history for setup charts. The first
              // message remains the canonical sample and the bounded series
              // adds temporal evidence without turning discovery into the
              // long-lived MCAP data plane.
              if (sample["status"] == "captured" && sample.contains("series") &&
                  sample["series"].size() < 64) {
                try {
                  if (type == "sensor_msgs/msg/JointState") {
                    const auto state = decode<sensor_msgs::msg::JointState>(*message);
                    sample["series"].push_back({{"captured_wall_ns", wall_ns()},
                                                 {"name", state.name},
                                                 {"value", json_numbers(state.position)}});
                  } else if (type == "std_msgs/msg/Float64MultiArray") {
                    const auto values = decode<std_msgs::msg::Float64MultiArray>(*message);
                    sample["series"].push_back({{"captured_wall_ns", wall_ns()},
                                                 {"value", json_numbers(values.data)}});
                  }
                } catch (const std::exception &error) {
                  sample["series_error"] = error.what();
                }
              }
              return;
            }
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
                sample["data"] = {{"name", state.name},
                                  {"position", json_numbers(state.position)},
                                  {"velocity", json_numbers(state.velocity)},
                                  {"effort", json_numbers(state.effort)}};
                sample["series"] = Json::array();
                sample["series"].push_back({{"captured_wall_ns", sample["captured_wall_ns"]},
                                              {"name", state.name},
                                              {"value", json_numbers(state.position)}});
              } else if (type == "std_msgs/msg/Float64MultiArray") {
                const auto values = decode<std_msgs::msg::Float64MultiArray>(*message);
                sample["data"] = {{"value", json_numbers(values.data)}};
                sample["series"] = Json::array();
                sample["series"].push_back({{"captured_wall_ns", sample["captured_wall_ns"]},
                                              {"value", json_numbers(values.data)}});
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
      const std::string message = error.what();
      if (rcutils_error_is_set())
        rcutils_reset_error();
      topic["sample"] = {{"status", "unavailable"}, {"error", message}};
    }
  }
  const auto sample_deadline = monotonic_ns() + 1500000000LL;
  while (!sample_subscriptions.empty() && rclcpp::ok(context.value) &&
         monotonic_ns() < sample_deadline) {
    executor.spin_once(std::chrono::milliseconds(25));
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

struct DiscoveryWorker {
  Ns domain;
  pid_t pid;
  int output;
};

DiscoveryWorker start_discovery_worker(Ns domain, Ns settle_ms) {
  const auto &executable = service_executable();
  require(!executable.empty(), "rearguard executable path is unavailable");

  int output[2] = {-1, -1};
  require(::pipe(output) == 0, "could not create ROS discovery worker pipe");
  const pid_t pid = ::fork();
  if (pid < 0) {
    ::close(output[0]);
    ::close(output[1]);
    throw std::runtime_error("could not start ROS discovery worker");
  }
  if (pid == 0) {
    ::close(output[0]);
    if (::dup2(output[1], STDOUT_FILENO) < 0)
      _exit(126);
    ::close(output[1]);
    if (::setenv("REARGUARD_DISCOVERY_WORKER", "1", 1) != 0)
      _exit(126);
    std::vector<std::string> args{
        executable.string(), "scan", "setup",
        "--domain-min", std::to_string(domain),
        "--domain-max", std::to_string(domain),
        "--settle-ms", std::to_string(settle_ms),
        "--parallelism", "1"};
    std::vector<char *> pointers;
    pointers.reserve(args.size() + 1);
    for (auto &arg : args)
      pointers.push_back(arg.data());
    pointers.push_back(nullptr);
    ::execv(executable.c_str(), pointers.data());
    _exit(127);
  }

  ::close(output[1]);
  return {domain, pid, output[0]};
}

Json finish_discovery_worker(const DiscoveryWorker &worker) {
  std::string body;
  std::array<char, 65536> buffer{};
  for (;;) {
    const auto count = ::read(worker.output, buffer.data(), buffer.size());
    if (count > 0) {
      body.append(buffer.data(), static_cast<std::size_t>(count));
      continue;
    }
    if (count < 0 && errno == EINTR)
      continue;
    require(count == 0, "could not read ROS discovery worker output");
    break;
  }
  ::close(worker.output);

  int status = 0;
  while (::waitpid(worker.pid, &status, 0) < 0) {
    if (errno == EINTR)
      continue;
    throw std::runtime_error("could not wait for ROS discovery worker");
  }
  const auto domain = std::to_string(worker.domain);
  if (WIFSIGNALED(status))
    throw std::runtime_error("ROS discovery worker crashed with signal " +
                             std::to_string(WTERMSIG(status)) +
                             " while scanning domain " + domain);
  require(WIFEXITED(status) && WEXITSTATUS(status) == 0,
          "ROS discovery worker failed while scanning domain " + domain);
  try {
    return Json::parse(body);
  } catch (const Json::exception &error) {
    throw std::runtime_error("invalid ROS discovery worker output for domain " +
                             domain + ": " + error.what());
  }
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
  const auto *worker = std::getenv("REARGUARD_DISCOVERY_WORKER");
  const bool isolate_batches = !service_executable().empty() &&
                               !(worker && std::string(worker) == "1");
  for (Ns batch = first; batch <= last; batch += parallelism) {
    const auto end = std::min(last + 1, batch + parallelism);
    if (isolate_batches) {
      std::vector<DiscoveryWorker> workers;
      workers.reserve(static_cast<std::size_t>(end - batch));
      for (Ns domain = batch; domain < end; ++domain)
        workers.push_back(start_discovery_worker(domain, settle_ms));
      std::exception_ptr worker_failure;
      for (const auto &worker_process : workers) {
        try {
          const auto result = finish_discovery_worker(worker_process);
          if (first_snapshot.is_null())
            first_snapshot = result;
          for (const auto &id : result.at("available_domain_ids"))
            ids.push_back(id);
          for (const auto &domain : result.at("domains"))
            domains.push_back(domain);
        } catch (...) {
          if (!worker_failure)
            worker_failure = std::current_exception();
        }
      }
      if (worker_failure)
        std::rethrow_exception(worker_failure);
      continue;
    }

    std::vector<std::future<Json>> pending;
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
