#pragma once
#include "common.hpp"
#include <array>
#include <memory>
#include <span>
#include <vector>

namespace harness::tiny {
using observation::Json;
using observation::Ns;
namespace fs = std::filesystem;
inline constexpr std::size_t feature_size = 5 * 256, sensor_size = 20;
struct Spec {
  std::size_t steps = 16, cameras = 2, joints = 32, min_steps = 4;
  double sample_hz = 2, max_age_s = 0.75;
  static Spec parse(const Json &);
};
struct Tensor {
  std::string name;
  std::vector<std::int64_t> shape;
  std::vector<float> values;
  Tensor(std::string, std::vector<std::int64_t>);
};
class Engine {
public:
  virtual ~Engine() = default;
  virtual void run(std::span<const Tensor> inputs, Tensor &output) = 0;
};
std::unique_ptr<Engine> make_onnx(const fs::path &, const std::vector<Tensor> &, const Tensor &);
std::unique_ptr<Engine> make_tensorrt(const fs::path &, const std::vector<Tensor> &, const Tensor &);
// Shared with models/critic/harness_critic/preprocess.py. Stride includes padding.
void preprocess(std::span<const std::uint8_t> rgb, std::size_t width,
                std::size_t height, std::size_t stride, std::span<float> output);
struct Frame {
  Ns t_ns = 0, epoch = 0;
  bool valid = false;
  std::vector<float> vision, camera_mask, sensors, joint_mask;
  explicit Frame(const Spec &);
};
class Critic {
  Json manifest_;
  Spec spec_;
  std::vector<Tensor> image_, inputs_;
  Tensor features_, logits_;
  std::unique_ptr<Engine> vision_engine_, head_engine_;
  Ns last_time_ = -1, epoch_ = -1;
public:
  Critic(const fs::path &bundle, const std::string &backend);
  const Spec &spec() const { return spec_; }
  const Json &manifest() const { return manifest_; }
  void encode(std::span<const std::uint8_t>, std::size_t width, std::size_t height,
              std::size_t stride, std::span<float> features);
  Json push(const Frame &);
  void reset();
};
Json replay_mcap(Critic &, const fs::path &mcap, const Json &binding,
                 std::ostream &output);
} // namespace harness::tiny
