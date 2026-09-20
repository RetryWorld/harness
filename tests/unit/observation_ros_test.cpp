// Real rosbag2/MCAP round trip, plus an opt-in DDS observer smoke test.
// The messages are synthetic test inputs, not a simulator or learned policy.
#include "recording.hpp"
#include "store.hpp"
#include <csignal>
#include <fstream>
#include <future>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
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
// Runs only with --live, alongside the existing DDS smoke test.
void setup_snapshot_test() {
  rclcpp::InitOptions init;
  init.set_domain_id(98);
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr, init);
  rclcpp::NodeOptions node_options;
  node_options.context(context);
  auto node = std::make_shared<rclcpp::Node>("setup_snapshot_fixture", node_options);
  auto camera = node->create_publisher<sensor_msgs::msg::Image>(
      "/setup_test/camera", rclcpp::SensorDataQoS());
  auto compressed = node->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/setup_test/camera/compressed", rclcpp::SensorDataQoS());
  auto silent = node->create_publisher<std_msgs::msg::String>("/setup_test/silent", 1);
  auto text = node->create_publisher<std_msgs::msg::String>("/setup_test/text", 1);
  std::vector<std::string> args = {"setup", "--domain-min", "98", "--domain-max", "98",
                                   "--settle-ms", "1000", "--parallelism", "1"};
  std::vector<char *> argv;
  for (auto &arg : args) argv.push_back(arg.data());
  Options options(static_cast<int>(argv.size()), argv.data());
  auto future = std::async(std::launch::async, [&] { return ros_discover(options); });
  bool sent_first = false;
  while (future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
    sensor_msgs::msg::Image frame;
    frame.width = 2;
    frame.height = 1;
    frame.step = 8; // Padded row, BGR source.
    frame.encoding = "bgr8";
    frame.data = {0, 0, 255, 0, 255, 0, 0, 0};
    camera->publish(frame);
    sensor_msgs::msg::CompressedImage encoded;
    encoded.format = "jpeg";
    encoded.data = {0xff, 0xd8, 0xff, 0xd9};
    compressed->publish(encoded);
    if (text->get_subscription_count()) {
      std_msgs::msg::String value;
      value.data = sent_first ? "later" : "first";
      text->publish(value);
      sent_first = true;
    }
  }
  const auto result = future.get();
  require(result["domains"].size() == 1, "setup scan missed fixture domain");
  const auto &domain = result["domains"][0];
  const auto find = [&](const std::string &name) -> const Json & {
    for (const auto &topic : domain["topics"])
      if (topic["name"] == name) return topic["sample"];
    throw std::runtime_error("missing setup topic: " + name);
  };
  const auto &frame = find("/setup_test/camera");
  require(frame["status"] == "captured", "camera snapshot not captured");
  require(frame["image"]["rgb_hex"] == "ff000000ff00", "BGR camera preview is incorrect");
  require(frame["image"]["width"] == 2 && frame["image"]["height"] == 1,
          "camera snapshot dimensions changed");
  const auto &encoded = find("/setup_test/camera/compressed");
  require(encoded["compressed_image"]["format"] == "jpeg" &&
              encoded["compressed_image"]["data_hex"] == "ffd8ffd9",
          "compressed camera snapshot is incorrect");
  require(find("/setup_test/silent")["status"] == "timeout", "silent topic must time out");
  require(find("/setup_test/text")["data"] == "first", "setup sample was overwritten");
  for (const auto &entry : domain["nodes"])
    require(!entry["fq_name"].get<std::string>().starts_with("/harness_setup_scan_"),
            "scanner leaked into setup graph");
  context->shutdown("setup snapshot test complete");
}
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
    if (argc > 1) {
      setup_snapshot_test();
      live_test(root);
    }
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
