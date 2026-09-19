// Real rosbag2/MCAP round trip, plus an opt-in DDS observer smoke test.
// The messages are synthetic test inputs, not a simulator or learned policy.
#include "recording.hpp"
#include "store.hpp"
#include <csignal>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <spawn.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;
using namespace harness::observation;
namespace {
template <class T>
std::shared_ptr<rclcpp::SerializedMessage> serialize(const T &m) {
  auto bytes = std::make_shared<rclcpp::SerializedMessage>();
  rclcpp::Serialization<T> codec;
  codec.serialize_message(&m, bytes.get());
  return bytes;
}
void recording_test(const fs::path &root) {
  fs::create_directories(root / "rosbag2");
  const auto config = load_config(HARNESS_OBSERVATION_CONFIG);
  atomic_json(root / "session.json", {{"config", config}});
  Recording recording(root, config["topics"]);
  for (Ns epoch : {0, 1})
    for (Ns time : {1, 2, 3})
      for (const auto &topic : config["topics"]) {
        const auto name = topic["name"].get<std::string>();
        const auto type = topic["type"].get<std::string>();
        // CDR contents identify the epoch. Export must never include epoch
        // zero.
        if (type == "sensor_msgs/msg/JointState") {
          sensor_msgs::msg::JointState m;
          m.position = {static_cast<double>(epoch)};
          recording.write(name, type, serialize(m), time, epoch);
        } else if (type == "sensor_msgs/msg/Image") {
          sensor_msgs::msg::Image m;
          m.data = {static_cast<std::uint8_t>(epoch)};
          recording.write(name, type, serialize(m), time, epoch);
        } else {
          std_msgs::msg::Float64MultiArray m;
          m.data = {static_cast<double>(epoch)};
          recording.write(name, type, serialize(m), time, epoch);
        }
      }
  recording.close();
  const auto path = export_window(
      root, {{"status", "ready"}, {"epoch", 1}, {"start_ns", 1}, {"end_ns", 2}},
      root / "export");
  require(fs::file_size(path) > 16, "MCAP not written");
  rosbag2_cpp::Reader reader;
  reader.open((root / "export").string());
  unsigned count = 0;
  while (reader.has_next()) {
    const auto m = reader.read_next();
    require(m->recv_timestamp >= 1 && m->recv_timestamp <= 2,
            "export timestamp out of bounds");
    rclcpp::SerializedMessage bytes(*m->serialized_data);
    if (m->topic_name == "/joint_states") {
      sensor_msgs::msg::JointState joint;
      rclcpp::Serialization<sensor_msgs::msg::JointState> codec;
      codec.deserialize_message(&bytes, &joint);
      require(joint.position == std::vector<double>{1.0},
              "export mixed clock epochs");
    }
    ++count;
  }
  require(count == 8, "export message count mismatch");
}
struct Child {
  pid_t pid = -1;
  explicit Child(std::vector<std::string> args) {
    std::vector<char *> pointers;
    for (auto &a : args)
      pointers.push_back(a.data());
    pointers.push_back(nullptr);
    require(posix_spawn(&pid, pointers[0], nullptr, nullptr, pointers.data(),
                        environ) == 0,
            "cannot launch native observer");
  }
  ~Child() {
    if (pid > 0) {
      ::kill(pid, SIGINT);
      int status = 0;
      ::waitpid(pid, &status, 0);
    }
  }
};
void live_test(const fs::path &root) {
  auto config = load_config(HARNESS_OBSERVATION_CONFIG);
  config["clock"] = "ros_system";
  atomic_json(root / "config.json", config);
  auto store = Store::create(root / "store", root / "config.json");
  const auto session = root / "live";
  const Ns domain = 97;
  Child observer({HARNESS_CLI_BIN, "observe", "start", "--config",
                  (root / "config.json").string(), "--session",
                  session.string(), "--store", store.root.string(),
                  "--domain-id", std::to_string(domain), "--wall-timeout",
                  "30"});
  rclcpp::InitOptions init;
  init.set_domain_id(static_cast<std::size_t>(domain));
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr, init);
  rclcpp::NodeOptions node_options;
  node_options.context(context);
  auto node = std::make_shared<rclcpp::Node>("native_observation_fixture", node_options);
  auto joints =
      node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  auto actions = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_controller/commands", 10);
  auto camera = node->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS());
  auto wrist = node->create_publisher<sensor_msgs::msg::Image>(
      "/wrist_camera/image_raw", rclcpp::SensorDataQoS());
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(node);
  auto publish = [&] {
    sensor_msgs::msg::JointState j;
    j.header.stamp = node->now();
    j.name = config["joint_order"].get<std::vector<std::string>>();
    j.position.resize(j.name.size());
    joints->publish(j);
    std_msgs::msg::Float64MultiArray a;
    a.data.resize(j.name.size());
    actions->publish(a);
    sensor_msgs::msg::Image image;
    image.header.stamp = node->now();
    image.width = 1;
    image.height = 1;
    image.step = 3;
    image.encoding = "rgb8";
    image.data = {255, 0, 0};
    camera->publish(image);
    wrist->publish(image);
    executor.spin_once(std::chrono::milliseconds(50));
  };
  const auto deadline = monotonic_ns() + 25000000000LL;
  while (!fs::exists(session / "session.json") && monotonic_ns() < deadline)
    publish();
  require(fs::exists(session / "session.json"), "observer did not start");
  const auto prefix =
      read_json(session / "session.json")["prefix"].get<std::string>();
  auto requests =
      node->create_publisher<std_msgs::msg::String>(prefix + "/requests", 10);
  Json response = nullptr;
  auto replies = node->create_subscription<std_msgs::msg::String>(
      prefix + "/responses", 10,
      [&](const std_msgs::msg::String &m) { response = Json::parse(m.data); });
  const auto warmup = monotonic_ns() + 3000000000LL;
  while (monotonic_ns() < warmup)
    publish();
  std_msgs::msg::String request;
  request.data = Json{{"schema_version", 1},
                      {"request_id", "native-live-candidate"},
                      {"operation", "candidate"},
                      {"detector_id", "test"},
                      {"evidence_class", "fixture"},
                      {"confidence", 0.9},
                      {"before", 1},
                      {"after", 1}}
                     .dump();
  Ns next_send = 0;
  while (response.is_null() && monotonic_ns() < deadline) {
    publish();
    if (monotonic_ns() >= next_send) {
      requests->publish(request);
      next_send = monotonic_ns() + 1000000000LL;
    }
  }
  require(!response.is_null() && response.value("ok", false),
          "candidate request failed: " + response.dump());
  while (store.snapshot()["candidates"].empty() && monotonic_ns() < deadline)
    publish();
  const auto profile = store.snapshot();
  require(profile["candidates"].size() == 1, "candidate not registered");
  const auto c = profile["candidates"].begin().value();
  const auto clip = export_window(session, c["window"], root / "live-export");
  const auto artifact = store.import_artifact(clip, "evidence_mcap");
  store.apply(
      {{"schema_version", 1},
       {"request_id", "live-evidence"},
       {"actor", "test"},
       {"expected_revision", profile["revision"]},
       {"operation", "attach_evidence"},
       {"payload", {{"candidate_id", c["id"]}, {"artifact", artifact}}}});
  require(
      store.snapshot()["candidates"].begin().value()["evidence"]["sha256"] ==
          artifact["sha256"],
      "live evidence not attached");
  context->shutdown("test complete");
}
} // namespace
int main(int argc, char **) {
  const auto root =
      fs::temp_directory_path() / ("harness-ros-test-" + unique_id());
  fs::create_directories(root);
  try {
    recording_test(root);
    if (argc > 1)
      live_test(root);
    fs::remove_all(root);
    std::cout << "Native ROS recording/export checks passed\n";
    return 0;
  } catch (const std::exception &e) {
    if (rclcpp::ok())
      rclcpp::shutdown();
    std::cerr << e.what() << "\nEvidence retained at " << root << '\n';
    return 1;
  }
}
