#pragma once

#include "common.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace harness::observation {

inline constexpr std::size_t critic_embedding_dimensions = 64;
inline constexpr const char *critic_feature_encoder =
    "role_window_stats64_v1";
enum CriticRole : std::uint32_t {
  critic_joint_state = 1U << 0,
  critic_action = 1U << 1,
  critic_camera = 1U << 2,
  critic_imu = 1U << 3,
  critic_wrench = 1U << 4,
};

inline std::uint32_t critic_role_bit(const std::string &role) {
  if (role == "joint_state") return critic_joint_state;
  if (role == "action") return critic_action;
  if (role == "camera") return critic_camera;
  if (role == "imu") return critic_imu;
  if (role == "wrench") return critic_wrench;
  return 0;
}

class CriticFeatureAccumulator {
  std::array<std::vector<double>, 5> signals_;
  std::vector<double> first_position_, last_position_;
  std::vector<double> first_command_, last_command_;
  std::array<double, 7> image_sum_{};
  std::size_t images_ = 0;
  std::uint32_t roles_ = 0;

  static double squash(double value) {
    return std::clamp(std::copysign(std::log1p(std::abs(value)) / 4.0, value),
                      -2.0, 2.0);
  }
  static void append(std::vector<double> &target,
                     const std::vector<double> &values) {
    for (const auto value : values)
      if (std::isfinite(value)) target.push_back(squash(value));
  }
  static void stats(const std::vector<double> &values,
                    std::span<float, 8> output) {
    if (values.empty()) return;
    double sum = 0, absolute = 0, squared = 0;
    double minimum = values.front(), maximum = values.front();
    for (const auto value : values) {
      sum += value;
      absolute += std::abs(value);
      squared += value * value;
      minimum = std::min(minimum, value);
      maximum = std::max(maximum, value);
    }
    const auto count = static_cast<double>(values.size());
    const auto mean = sum / count;
    double variance = 0;
    for (const auto value : values) variance += (value - mean) * (value - mean);
    const std::array<float, 8> summary{
        static_cast<float>(mean), static_cast<float>(absolute / count),
        static_cast<float>(std::sqrt(squared / count)),
        static_cast<float>(minimum), static_cast<float>(maximum),
        static_cast<float>(std::sqrt(variance / count)),
        static_cast<float>(maximum - minimum),
        static_cast<float>(std::min(1.0, count / 256.0))};
    std::copy(summary.begin(), summary.end(), output.begin());
  }
  static std::vector<double> difference(const std::vector<double> &first,
                                        const std::vector<double> &last) {
    std::vector<double> result;
    if (first.size() != last.size()) return result;
    result.reserve(first.size());
    for (std::size_t i = 0; i < first.size(); ++i)
      if (std::isfinite(first[i]) && std::isfinite(last[i]))
        result.push_back(squash(last[i] - first[i]));
    return result;
  }

public:
  void joint(const std::vector<double> &position,
             const std::vector<double> &velocity,
             const std::vector<double> &effort) {
    if (!position.empty()) {
      if (first_position_.empty()) first_position_ = position;
      last_position_ = position;
      append(signals_[0], position);
      roles_ |= critic_joint_state;
    }
    append(signals_[1], velocity);
    append(signals_[2], effort);
  }
  void action(const std::vector<double> &command) {
    if (command.empty()) return;
    if (first_command_.empty()) first_command_ = command;
    last_command_ = command;
    append(signals_[3], command);
    roles_ |= critic_action;
  }
  void image(std::span<const std::uint8_t> bytes, std::size_t width,
             std::size_t height, std::size_t step, std::size_t channels,
             bool bgr = false) {
    if (width == 0 || height == 0 || channels == 0 ||
        step < width * channels || bytes.size() != height * step)
      return;
    std::array<double, 3> sum{}, squared{};
    double edge = 0;
    std::size_t count = 0, edge_count = 0;
    const auto stride_x = std::max<std::size_t>(1, width / 24);
    const auto stride_y = std::max<std::size_t>(1, height / 24);
    auto channel = [&](std::size_t x, std::size_t y, std::size_t c) {
      const auto index = channels == 1 ? 0 : bgr ? 2 - c : c;
      return static_cast<double>(bytes[y * step + x * channels + index]) / 255.0;
    };
    for (std::size_t y = 0; y < height; y += stride_y)
      for (std::size_t x = 0; x < width; x += stride_x) {
        for (std::size_t c = 0; c < 3; ++c) {
          const auto value = channel(x, y, c);
          sum[c] += value;
          squared[c] += value * value;
        }
        if (x + stride_x < width) {
          double delta = 0;
          for (std::size_t c = 0; c < 3; ++c)
            delta += std::abs(channel(x, y, c) -
                              channel(x + stride_x, y, c));
          edge += delta / 3.0;
          ++edge_count;
        }
        ++count;
      }
    if (count == 0) return;
    for (std::size_t c = 0; c < 3; ++c) {
      const auto mean = sum[c] / static_cast<double>(count);
      image_sum_[c] += mean;
      image_sum_[3 + c] +=
          std::sqrt(std::max(0.0, squared[c] / static_cast<double>(count) -
                                      mean * mean));
    }
    image_sum_[6] += edge_count ? edge / static_cast<double>(edge_count) : 0;
    ++images_;
    roles_ |= critic_camera;
  }
  std::uint32_t roles() const { return roles_; }
  std::array<float, critic_embedding_dimensions>
  embedding(std::uint32_t required_roles) const {
    std::array<float, critic_embedding_dimensions> result{};
    if (required_roles & critic_joint_state) {
      stats(signals_[0], std::span<float, 8>(result.data() + 0, 8));
      stats(signals_[1], std::span<float, 8>(result.data() + 8, 8));
      stats(signals_[2], std::span<float, 8>(result.data() + 16, 8));
      const auto delta = difference(first_position_, last_position_);
      stats(delta, std::span<float, 8>(result.data() + 40, 8));
    }
    if (required_roles & critic_action) {
      stats(signals_[3], std::span<float, 8>(result.data() + 24, 8));
      if (last_position_.size() == last_command_.size()) {
        std::vector<double> tracking;
        tracking.reserve(last_position_.size());
        for (std::size_t i = 0; i < last_position_.size(); ++i)
          tracking.push_back(last_command_[i] - last_position_[i]);
        stats(tracking, std::span<float, 8>(result.data() + 32, 8));
      }
      const auto delta = difference(first_command_, last_command_);
      stats(delta, std::span<float, 8>(result.data() + 48, 8));
    }
    if ((required_roles & critic_camera) && images_ > 0)
      for (std::size_t i = 0; i < image_sum_.size(); ++i)
        result[56 + i] = static_cast<float>(
            image_sum_[i] / static_cast<double>(images_));
    result[63] = 1.0F; // stable bias keeps an all-zero physical window valid
    double norm = 0;
    for (const auto value : result)
      norm += static_cast<double>(value) * static_cast<double>(value);
    norm = std::sqrt(norm);
    for (auto &value : result)
      value = static_cast<float>(static_cast<double>(value) / norm);
    return result;
  }
};

// ROS builds decode the accepted MCAP window. Non-ROS development builds
// return an explicit unavailable record and never mark a generation executable.
Json compile_snapshot_embedding(const fs::path &artifact, const Json &binding,
                                const Json &window,
                                std::uint32_t required_roles);
bool snapshot_compiler_available();

} // namespace harness::observation
