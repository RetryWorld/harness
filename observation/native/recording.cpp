#include "recording.hpp"
#include <iomanip>
#include <rosbag2_cpp/reader.hpp>
#include <set>
#include <sstream>
namespace harness::observation {
namespace {
rosbag2_storage::StorageOptions storage_options(const fs::path &path,
                                                const std::string &storage) {
  rosbag2_storage::StorageOptions options;
  options.uri = path.string();
  options.storage_id = storage;
  return options;
}
} // namespace
Recording::Recording(fs::path session, Json topics, std::string storage)
    : session_(std::move(session)), topics_(std::move(topics)),
      storage_(std::move(storage)) {
  atomic_json(session_ / "recording.json",
              {{"schema_version", 1}, {"segments", segments_}});
}
void Recording::write(const std::string &topic, const std::string &type,
                      std::shared_ptr<const rclcpp::SerializedMessage> data,
                      Ns now, Ns epoch) {
  if (writer_ && (epoch != current_["epoch"].get<Ns>() ||
                  now - current_["start_ns"].get<Ns>() >= 5000000000LL))
    close();
  if (!writer_) {
    std::ostringstream name;
    name << "rosbag2/segment-" << std::setw(6) << std::setfill('0')
         << segments_.size();
    current_ = {{"epoch", epoch},
                {"start_ns", now},
                {"end_ns", now},
                {"path", name.str()},
                {"storage_id", storage_}};
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(storage_options(session_ / name.str(), storage_),
                  {"cdr", "cdr"});
    for (const auto &item : topics_) {
      rosbag2_storage::TopicMetadata metadata;
      metadata.name = item["name"];
      metadata.type = item["type"];
      metadata.serialization_format = "cdr";
      writer_->create_topic(metadata);
    }
  }
  writer_->write(data, topic, type, now, now);
  current_["end_ns"] = now;
}
void Recording::close() {
  if (!writer_)
    return;
  writer_->close();
  writer_.reset();
  segments_.push_back(current_);
  atomic_json(session_ / "recording.json",
              {{"schema_version", 1}, {"segments", segments_}});
}
fs::path export_window(const fs::path &session, const Json &window,
                       const fs::path &destination) {
  require(window.at("status") == "ready", "only ready windows can be exported");
  const auto manifest = read_json(session / "recording.json");
  const auto config = read_json(session / "session.json")["config"];
  std::vector<Json> segments;
  for (const auto &s : manifest["segments"])
    if (s["epoch"] == window["epoch"] &&
        s["end_ns"].get<Ns>() >= window["start_ns"].get<Ns>() &&
        s["start_ns"].get<Ns>() <= window["end_ns"].get<Ns>())
      segments.push_back(s);
  require(!segments.empty(), "no closed evidence segments for window");
  std::map<std::string, unsigned> counts;
  std::set<std::string> registered;
  rosbag2_cpp::Writer writer;
  writer.open(storage_options(destination, "mcap"), {"cdr", "cdr"});
  for (const auto &s : segments) {
    rosbag2_cpp::Reader reader;
    reader.open(storage_options(session / s["path"].get<std::string>(),
                                s["storage_id"]),
                {"cdr", "cdr"});
    for (const auto &topic : reader.get_all_topics_and_types())
      if (registered.insert(topic.name).second)
        writer.create_topic(topic);
    while (reader.has_next()) {
      const auto message = reader.read_next();
      if (message->recv_timestamp >= window["start_ns"].get<Ns>() &&
          message->recv_timestamp <= window["end_ns"].get<Ns>()) {
        writer.write(message);
        ++counts[message->topic_name];
      }
    }
  }
  writer.close();
  for (const auto &t : config["topics"])
    if (t.value("required", true))
      require(counts[t["name"].get<std::string>()] >= 2,
              "exported MCAP lacks required topic coverage");
  std::vector<fs::path> files;
  for (const auto &entry : fs::directory_iterator(destination))
    if (entry.path().extension() == ".mcap")
      files.push_back(entry.path());
  require(files.size() == 1, "expected exactly one exported MCAP");
  return files.front();
}
} // namespace harness::observation
