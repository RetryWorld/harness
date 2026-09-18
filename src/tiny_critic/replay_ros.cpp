#include "critic.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <iostream>
#include <map>
#include <optional>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <set>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace harness::tiny {
namespace {
using observation::require;
struct Packet {
  Ns time = 0;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message;
};
template<class T> T decode(const Packet &packet) {
  T output;
  const auto &raw = *packet.message->serialized_data;
  rclcpp::SerializedMessage bytes(raw.buffer_length);
  auto &target = bytes.get_rcl_serialized_message();
  std::memcpy(target.buffer, raw.buffer, raw.buffer_length);
  target.buffer_length = raw.buffer_length;
  rclcpp::Serialization<T> codec;
  codec.deserialize_message(&bytes, &output);
  return output;
}
Ns stamp(const builtin_interfaces::msg::Time &value) {
  return static_cast<Ns>(value.sec) * 1000000000LL + static_cast<Ns>(value.nanosec);
}
float normalize(double value, const Json &meta, const char *scale, float &known) {
  if (meta.contains(scale)) {
    const double denominator = meta.at(scale);
    require(std::isfinite(denominator) && denominator > 0, "invalid sensor scale");
    known = 1;
    return static_cast<float>(std::clamp(value / denominator, -10.0, 10.0));
  }
  known = 0;
  return static_cast<float>(std::clamp(std::copysign(std::log1p(std::abs(value)), value) / 10, -1.0, 1.0));
}
class Canonical {
  Critic &critic_;
  Json binding_;
  std::map<std::string, Json> topics_;
  std::vector<std::string> camera_names_, joint_names_, topic_order_;
  std::map<std::string, Packet> latest_;
  std::map<std::string, std::pair<Ns,std::vector<float>>> features_;
  bool fresh(Ns source, Ns receipt, Ns now) const {
    const auto limit = static_cast<Ns>(critic_.spec().max_age_s * 1e9);
    return receipt <= now && source <= now && now - receipt <= limit && now - source <= limit;
  }
public:
  Canonical(Critic &critic, Json binding) : critic_(critic), binding_(std::move(binding)) {
    joint_names_ = binding_.at("joint_order").get<std::vector<std::string>>();
    require(!joint_names_.empty() && std::set<std::string>(joint_names_.begin(), joint_names_.end()).size() == joint_names_.size(), "invalid joint order");
    std::size_t states = 0, actions = 0, auxiliary = 0;
    for (const auto &topic : binding_.at("topics")) {
      const auto name = topic.at("name").get<std::string>(), role = topic.at("role").get<std::string>(), type = topic.at("type").get<std::string>();
      const bool supported = (role == "camera" && type == "sensor_msgs/msg/Image") ||
          (role == "joint_state" && type == "sensor_msgs/msg/JointState") ||
          (role == "action" && type == "std_msgs/msg/Float64MultiArray") ||
          (role == "imu" && type == "sensor_msgs/msg/Imu") ||
          (role == "wrench" && type == "geometry_msgs/msg/WrenchStamped");
      require(supported || !topic.value("required", true), "unsupported required critic topic: " + name);
      if (!supported) continue;
      require(!topics_.contains(name), "duplicate critic topic");
      topics_[name] = topic; topic_order_.push_back(name);
      if (role == "camera") camera_names_.push_back(name);
      if (role == "joint_state") ++states;
      if (role == "action") ++actions;
      if (role == "imu") auxiliary += 3;
      if (role == "wrench") auxiliary += 2;
    }
    require(states == 1 && actions <= 1 && !camera_names_.empty() && camera_names_.size() <= critic_.spec().cameras &&
            joint_names_.size() + auxiliary <= critic_.spec().joints, "binding exceeds critic capacity");
  }
  bool uses(const std::string &name) const { return topics_.contains(name); }
  void reset() { latest_.clear(); features_.clear(); critic_.reset(); }
  void receive(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &message, const std::string &type) {
    const auto &name = message->topic_name;
    require(topics_.at(name).at("type") == type, "MCAP schema differs from binding");
    require(message->serialized_data->buffer_length <= 64 * 1024 * 1024, "critic packet too large");
    latest_[name] = {message->recv_timestamp, message};
  }
  Json sample(Ns now, Ns epoch) {
    Frame f(critic_.spec()); f.t_ns = now; f.epoch = epoch; f.valid = true;
    Json reasons = Json::array();
    std::optional<sensor_msgs::msg::JointState> state;
    std::optional<std_msgs::msg::Float64MultiArray> command;
    Ns state_time = 0, command_time = 0;
    std::size_t aux_index = joint_names_.size();
    for (const auto &name : topic_order_) {
      const auto &topic = topics_.at(name);
      const auto role = topic.at("role").get<std::string>();
      bool valid = false;
      const auto found = latest_.find(name);
      if (found != latest_.end()) {
        const auto &p = found->second;
        if (fresh(p.time, p.time, now)) {
          if (role == "camera") {
            const auto image = decode<sensor_msgs::msg::Image>(p);
            valid = fresh(stamp(image.header.stamp), p.time, now);
            if (valid) {
              const auto camera = static_cast<std::size_t>(std::find(camera_names_.begin(), camera_names_.end(), name) - camera_names_.begin());
              auto cached = features_.find(name);
              if (cached == features_.end() || cached->second.first != p.time) {
                const auto w = static_cast<std::size_t>(image.width), h = static_cast<std::size_t>(image.height);
                const auto channels = image.encoding == "mono8" ? 1U :
                    (image.encoding == "rgb8" || image.encoding == "bgr8") ? 3U :
                    (image.encoding == "rgba8" || image.encoding == "bgra8") ? 4U : 0U;
                require(w > 0 && h > 0 && w <= 16384 && h <= 16384 && w * h <= 16777216 && channels != 0 &&
                    image.step >= w * channels && image.data.size() == h * image.step, "invalid camera bytes/encoding");
                std::vector<std::uint8_t> rgb(w * h * 3);
                const bool bgr = image.encoding == "bgr8" || image.encoding == "bgra8";
                for (std::size_t y = 0; y < h; ++y)
                  for (std::size_t x = 0; x < w; ++x)
                    for (std::size_t c = 0; c < 3; ++c)
                      rgb[(y * w + x) * 3 + c] = image.data[y * image.step + x * channels + (channels == 1 ? 0 : bgr ? 2 - c : c)];
                std::vector<float> values(feature_size);
                critic_.encode(rgb, w, h, w * 3, values);
                features_[name] = {p.time, std::move(values)};
              }
              const auto &values = features_.at(name).second;
              std::copy(values.begin(), values.end(), f.vision.begin() + static_cast<std::ptrdiff_t>(camera * feature_size));
              f.camera_mask[camera] = 1;
            }
          } else if (role == "joint_state") {
            auto m = decode<sensor_msgs::msg::JointState>(p);
            valid = fresh(stamp(m.header.stamp), p.time, now);
            if (valid) { state = std::move(m); state_time = p.time; }
          } else if (role == "action") {
            command = decode<std_msgs::msg::Float64MultiArray>(p); command_time = p.time; valid = true;
          } else {
            auto vector_token = [&](std::size_t offset, std::initializer_list<double> values, int kind, bool available) {
              if (!available || !std::all_of(values.begin(), values.end(), [](double v){ return std::isfinite(v); })) return;
              auto *row = f.sensors.data() + (aux_index + offset) * sensor_size;
              std::size_t j = 0;
              for (const auto value : values) {
                row[j] = normalize(value, kind == 3 ? Json{{"scale", 1}} : Json::object(), "scale", row[10 + j]);
                row[5 + j] = 1; ++j;
              }
              row[16] = static_cast<float>(kind) / 5;
              row[17] = static_cast<float>(static_cast<double>(now - p.time) / (critic_.spec().max_age_s * 1e9));
              f.joint_mask[aux_index + offset] = 1;
            };
            if (role == "imu") {
              const auto m = decode<sensor_msgs::msg::Imu>(p);
              valid = fresh(stamp(m.header.stamp), p.time, now);
              if (valid) {
                vector_token(0, {m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z}, 1, m.angular_velocity_covariance[0] != -1);
                vector_token(1, {m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z}, 2, m.linear_acceleration_covariance[0] != -1);
                vector_token(2, {m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w}, 3, m.orientation_covariance[0] != -1);
              }
            } else {
              const auto m = decode<geometry_msgs::msg::WrenchStamped>(p);
              valid = fresh(stamp(m.header.stamp), p.time, now);
              if (valid) {
                vector_token(0, {m.wrench.force.x,m.wrench.force.y,m.wrench.force.z}, 4, true);
                vector_token(1, {m.wrench.torque.x,m.wrench.torque.y,m.wrench.torque.z}, 5, true);
              }
            }
          }
        }
      }
      if (!valid && topic.value("required", true)) { f.valid = false; reasons.push_back("missing_or_stale:" + name); }
      if (role == "imu") aux_index += 3;
      if (role == "wrench") aux_index += 2;
    }
    if (state) {
      const auto &m = *state;
      std::map<std::string,std::size_t> order;
      for (std::size_t i = 0; i < m.name.size(); ++i) require(order.emplace(m.name[i],i).second, "duplicate JointState names");
      for (const auto *values : {&m.position, &m.velocity, &m.effort})
        require(values->empty() || values->size() == m.name.size(), "JointState array size mismatch");
      if (command) require(command->data.size() == joint_names_.size(), "command joint order mismatch");
      const auto all_meta = binding_.value("critic_joint_metadata", Json::object());
      const auto interface = binding_.value("command_interface", "");
      for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        if (!order.contains(joint_names_[i])) continue;
        const auto k = order.at(joint_names_[i]);
        auto *row = f.sensors.data() + i * sensor_size;
        const auto meta = all_meta.value(joint_names_[i], Json::object());
        const auto type = meta.value("type", "");
        const std::array<const std::vector<double> *,3> arrays{&m.position, &m.velocity, &m.effort};
        constexpr std::array<const char *,3> scales{"position_scale", "velocity_scale", "effort_scale"};
        for (std::size_t c = 0; c < 3; ++c) {
          if (arrays[c]->empty() || !std::isfinite((*arrays[c])[k])) continue;
          const double value = (*arrays[c])[k];
          if (c == 0 && type == "continuous") {
            row[0] = static_cast<float>(std::sin(value)); row[18] = static_cast<float>(std::cos(value)); row[10] = 1;
          } else if (c == 0 && meta.contains("lower")) {
            const double low = meta.at("lower"), high = meta.at("upper");
            require(std::isfinite(low) && std::isfinite(high) && high > low, "invalid joint limits");
            row[0] = static_cast<float>(std::clamp(2 * (value - low) / (high - low) - 1, -10.0, 10.0)); row[10] = 1;
          } else row[c] = normalize(value, meta, scales[c], row[10 + c]);
          row[5 + c] = 1;
        }
        if (command && (interface == "position" || interface == "velocity" || interface == "effort") && std::isfinite(command->data[i])) {
          row[3] = normalize(command->data[i], meta, (interface + "_scale").c_str(), row[13]); row[8] = 1;
          row[19] = static_cast<float>(static_cast<double>(now - command_time) / (critic_.spec().max_age_s * 1e9));
        }
        row[14] = type == "revolute" ? 1.F / 3 : type == "prismatic" ? 2.F / 3 : type == "continuous" ? 1.F : 0.F;
        row[15] = interface == "position" ? 1.F / 3 : interface == "velocity" ? 2.F / 3 : interface == "effort" ? 1.F : 0.F;
        row[17] = static_cast<float>(static_cast<double>(now - state_time) / (critic_.spec().max_age_s * 1e9));
        f.joint_mask[i] = std::any_of(row + 5, row + 9, [](float v){ return v == 1; }) ? 1.F : 0.F;
      }
    }
    auto result = critic_.push(f); result["reasons"] = reasons; return result;
  }
};
}
Json replay_mcap(Critic &critic, const fs::path &path, const Json &binding, std::ostream &out) {
  Canonical canonical(critic, binding);
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions options; options.uri = path.string(); options.storage_id = "mcap";
  reader.open(options, {"cdr", "cdr"});
  std::map<std::string,std::string> types;
  for (const auto &topic : reader.get_all_topics_and_types()) types[topic.name] = topic.type;
  // Exported Harness windows contain one epoch. Reject a reset instead of
  // sorting disjoint epochs together. Native rosbag reader orders by receipt.
  Ns last = -1, next = -1;
  const auto period = static_cast<Ns>(std::llround(1e9 / critic.spec().sample_hz));
  std::size_t samples = 0;
  while (reader.has_next()) {
    auto message = reader.read_next();
    if (!canonical.uses(message->topic_name)) continue;
    const Ns now = message->recv_timestamp;
    require(now >= 0 && now >= last, "MCAP must be a single monotonic clock epoch");
    if (next < 0) next = now;
    if (now - next > period * static_cast<Ns>(critic.spec().steps)) {
      out << canonical.sample(next, 0).dump() << '\n'; ++samples;
      canonical.reset(); next = now;
    }
    while (next < now) { out << canonical.sample(next, 0).dump() << '\n'; ++samples; next += period; }
    canonical.receive(message, types.at(message->topic_name)); last = now;
  }
  if (last >= 0 && next == last) { out << canonical.sample(next, 0).dump() << '\n'; ++samples; }
  require(samples > 0, "MCAP contains no bound observations");
  return {{"samples", samples}};
}
}
