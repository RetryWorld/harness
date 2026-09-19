#pragma once
#include <cstdint>
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

namespace harness::observation {
using Json = nlohmann::json;
namespace fs = std::filesystem;
using Ns = std::int64_t;
Ns wall_ns();
Ns monotonic_ns();
std::string unique_id();
std::string encoded(const Json &value);
std::string digest(const Json &value);
std::string file_hash(const fs::path &path);
Json read_json(const fs::path &path);
void atomic_json(const fs::path &path, const Json &value);
void require(bool condition, const std::string &message);
std::string required_text(const Json &value, const std::string &name);
Json load_config(const fs::path &path);
struct Options {
  std::string command;
  std::map<std::string, std::string> values;
  Options(int argc, char **argv);
  std::string get(const std::string &key,
                  const std::string &fallback = "") const;
  std::string need(const std::string &key) const;
  fs::path path(const std::string &key) const;
  double number(const std::string &key, double fallback) const;
  Ns integer(const std::string &key, Ns fallback) const;
  void allow(const std::string &keys) const;
};
int command_main(const std::string &group, int argc, char **argv);
Json ros_start(const Options &options);
Json ros_discover(const Options &options);
Json ros_request(const Options &options, const Json &meta);
fs::path export_window(const fs::path &session, const Json &window,
                       const fs::path &destination);
} // namespace harness::observation
