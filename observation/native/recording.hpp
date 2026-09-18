#pragma once
#include "common.hpp"
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/writer.hpp>
namespace harness::observation {
class Recording {
  fs::path session_;
  Json topics_, segments_ = Json::array(), current_;
  std::string storage_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;

public:
  Recording(fs::path session, Json topics, std::string storage = "mcap");
  void write(const std::string &topic, const std::string &type,
             std::shared_ptr<const rclcpp::SerializedMessage> data, Ns now,
             Ns epoch);
  void close();
};
} // namespace harness::observation
