#include "common.hpp"
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace harness::observation {
void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}
Ns wall_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
Ns monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
namespace {
std::string hex(const unsigned char *data, std::size_t size) {
  std::ostringstream out;
  for (std::size_t i = 0; i < size; ++i)
    out << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<unsigned>(data[i]);
  return out.str();
}
class Hash {
  EVP_MD_CTX *context_ = EVP_MD_CTX_new();

public:
  Hash() {
    require(context_ && EVP_DigestInit_ex(context_, EVP_sha256(), nullptr) == 1,
            "SHA256 initialization failed");
  }
  ~Hash() { EVP_MD_CTX_free(context_); }
  void add(const void *data, std::size_t size) {
    require(EVP_DigestUpdate(context_, data, size) == 1, "SHA256 failed");
  }
  std::string finish() {
    std::array<unsigned char, EVP_MAX_MD_SIZE> bytes{};
    unsigned size = 0;
    require(EVP_DigestFinal_ex(context_, bytes.data(), &size) == 1,
            "SHA256 failed");
    return hex(bytes.data(), size);
  }
};
} // namespace
std::string unique_id() {
  std::array<unsigned char, 16> bytes{};
  require(RAND_bytes(bytes.data(), static_cast<int>(bytes.size())) == 1,
          "random ID generation failed");
  return hex(bytes.data(), bytes.size());
}
// Preserve the original store's sorted, ASCII-escaped JSON hash contract,
// including Python's float notation boundaries. Timestamps remain integers.
std::string encoded(const Json &value) {
  if (value.is_object() || value.is_array()) {
    const bool object = value.is_object();
    std::string result = object ? "{" : "[";
    bool first = true;
    for (auto it = value.begin(); it != value.end(); ++it) {
      if (!first)
        result += ',';
      first = false;
      if (object)
        result += Json(it.key()).dump(-1, ' ', true) + ":";
      result += encoded(it.value());
    }
    return result + (object ? "}" : "]");
  }
  if (!value.is_number_float())
    return value.dump(-1, ' ', true);
  const double number = value.get<double>();
  require(std::isfinite(number), "nonfinite JSON number");
  if (number == 0)
    return std::signbit(number) ? "-0.0" : "0.0";
  char buffer[64];
  const auto conversion =
      std::to_chars(buffer, buffer + sizeof(buffer), std::abs(number),
                    std::chars_format::scientific);
  require(conversion.ec == std::errc(), "float serialization failed");
  std::string text(buffer, conversion.ptr);
  const auto e = text.find('e');
  const int exponent = std::stoi(text.substr(e + 1));
  auto digits = text.substr(0, e);
  const auto dot = digits.find('.');
  if (dot != std::string::npos)
    digits.erase(dot, 1);
  const std::string sign = number < 0 ? "-" : "";
  if (exponent < -4 || exponent >= 16) {
    std::string result = sign + digits.substr(0, 1);
    if (digits.size() > 1)
      result += "." + digits.substr(1);
    result += exponent < 0 ? "e-" : "e+";
    auto power = std::to_string(std::abs(exponent));
    if (power.size() < 2)
      power = "0" + power;
    return result + power;
  }
  const int point = exponent + 1;
  if (point <= 0)
    return sign + "0." + std::string(static_cast<std::size_t>(-point), '0') +
           digits;
  const auto position = static_cast<std::size_t>(point);
  if (position >= digits.size())
    return sign + digits + std::string(position - digits.size(), '0') + ".0";
  return sign + digits.substr(0, position) + "." + digits.substr(position);
}
std::string digest(const Json &value) {
  Hash h;
  const auto s = encoded(value);
  h.add(s.data(), s.size());
  return h.finish();
}
std::string file_hash(const fs::path &path) {
  std::ifstream stream(path, std::ios::binary);
  require(stream.good(), "cannot read artifact: " + path.string());
  Hash hash;
  std::array<char, 65536> block{};
  while (stream) {
    stream.read(block.data(), static_cast<std::streamsize>(block.size()));
    hash.add(block.data(), static_cast<std::size_t>(stream.gcount()));
  }
  require(stream.eof(), "artifact read failed");
  return hash.finish();
}
Json read_json(const fs::path &path) {
  std::ifstream stream(path);
  require(stream.good(), "cannot read " + path.string());
  return Json::parse(stream);
}
void atomic_json(const fs::path &path, const Json &value) {
  const fs::path temporary = path.string() + "." + unique_id() + ".tmp";
  const int fd = ::open(temporary.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0600);
  require(fd >= 0, "cannot create " + temporary.string());
  try {
    const auto body = value.dump(2) + "\n";
    std::size_t offset = 0;
    while (offset < body.size()) {
      const auto written =
          ::write(fd, body.data() + offset, body.size() - offset);
      require(written > 0, "write failed: " + temporary.string());
      offset += static_cast<std::size_t>(written);
    }
    require(::fsync(fd) == 0, "fsync failed");
    ::close(fd);
  } catch (...) {
    ::close(fd);
    fs::remove(temporary);
    throw;
  }
  fs::rename(temporary, path);
}
std::string required_text(const Json &value, const std::string &name) {
  require(value.is_string(), name + " must be text");
  auto text = value.get<std::string>();
  const auto first = text.find_first_not_of(" \t\r\n");
  require(first != std::string::npos && text.size() <= 16000,
          name + " must be nonempty text (at most 16000 bytes)");
  return text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}
Json load_config(const fs::path &path) {
  auto c = read_json(path);
  require(c.value("schema_version", 0) == 1 &&
              c.value("mode", "") == "observation_only",
          "requires observation_only configuration, schema_version 1");
  require(c.at("critics") == Json::array() &&
              c.at("recoveries") == Json::array(),
          "requires empty critics and recoveries");
  require(c.at("clock") == "ros_sim" || c.at("clock") == "ros_system",
          "clock must be ros_sim or ros_system");
  std::map<std::string, int> roles;
  std::map<std::string, bool> names;
  require(c.at("topics").is_array(), "topics must be an array");
  for (const auto &t : c.at("topics")) {
    const auto name = required_text(t.at("name"), "topic");
    require(name.front() == '/' && !names.contains(name),
            "topics must be absolute and unique");
    names[name] = true;
    const auto role = t.at("role").get<std::string>();
    const auto type = t.at("type").get<std::string>();
    require((role == "joint_state" && type == "sensor_msgs/msg/JointState") ||
                (role == "action" &&
                 (type == "std_msgs/msg/Float64MultiArray" ||
                  type == "trajectory_msgs/msg/JointTrajectory")) ||
                (role == "camera" && type == "sensor_msgs/msg/Image"),
            "unsupported topic role/type");
    ++roles[role];
  }
  require(roles["joint_state"] == 1 && roles["action"] == 1 &&
              roles["camera"] >= 1,
          "configure one joint_state, one action and at least one camera");
  names.clear();
  require(c.at("joint_order").is_array() && !c.at("joint_order").empty(),
          "joint_order must be nonempty");
  for (const auto &j : c.at("joint_order")) {
    const auto name = required_text(j, "joint");
    require(!names.contains(name), "duplicate joint");
    names[name] = true;
  }
  return c;
}
Options::Options(int argc, char **argv) {
  require(argc >= 1, "subcommand required (use --help)");
  command = argv[0];
  for (int i = 1; i < argc; ++i) {
    std::string key = argv[i];
    require(key.starts_with("--"), "expected option: " + key);
    const auto eq = key.find('=');
    std::string value;
    if (eq != std::string::npos) {
      value = key.substr(eq + 1);
      key.resize(eq);
    } else {
      require(i + 1 < argc, "missing value: " + key);
      value = argv[++i];
    }
    require(!values.contains(key), "repeated option: " + key);
    values[key] = value;
  }
}
std::string Options::get(const std::string &key,
                         const std::string &fallback) const {
  auto it = values.find(key);
  return it == values.end() ? fallback : it->second;
}
std::string Options::need(const std::string &key) const {
  require(values.contains(key), "required option: " + key);
  return required_text(get(key), key);
}
fs::path Options::path(const std::string &key) const {
  auto p = need(key);
  require(p != ".", key + " must name a dedicated directory");
  return fs::absolute(p);
}
double Options::number(const std::string &key, double fallback) const {
  if (!values.contains(key))
    return fallback;
  std::size_t used = 0;
  const auto s = get(key);
  const auto v = std::stod(s, &used);
  require(used == s.size() && std::isfinite(v), "invalid number: " + key);
  return v;
}
Ns Options::integer(const std::string &key, Ns fallback) const {
  if (!values.contains(key))
    return fallback;
  std::size_t used = 0;
  const auto s = get(key);
  const auto v = std::stoll(s, &used);
  require(used == s.size(), "invalid integer: " + key);
  return v;
}
void Options::allow(const std::string &keys) const {
  for (const auto &[key, value] : values) {
    (void)value;
    require((" " + keys + " ").find(" " + key + " ") != std::string::npos,
            "unknown option: " + key);
  }
}
} // namespace harness::observation
