#include "critic.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <numeric>

namespace harness::tiny {
using observation::require;
using observation::read_json;
using observation::file_hash;
Spec Spec::parse(const Json &j) {
  Spec s;
  s.steps = j.at("steps"); s.cameras = j.at("cameras"); s.joints = j.at("joints");
  s.min_steps = j.at("min_steps"); s.sample_hz = j.at("sample_hz"); s.max_age_s = j.at("max_age_s");
  require(s.steps >= 4 && s.steps <= 64 && s.cameras >= 1 && s.cameras <= 4 &&
          s.joints >= 1 && s.joints <= 128 && s.min_steps >= 1 && s.min_steps <= s.steps,
          "invalid critic dimensions");
  require(std::isfinite(s.sample_hz) && s.sample_hz > 0 && s.sample_hz <= 30 &&
          std::isfinite(s.max_age_s) && s.max_age_s > 0 && s.max_age_s <= 10, "invalid critic timing");
  return s;
}
Tensor::Tensor(std::string n, std::vector<std::int64_t> s) : name(std::move(n)), shape(std::move(s)) {
  std::size_t count = 1;
  for (const auto d : shape) {
    require(d > 0 && d <= 1000000 && count <= 10000000 / static_cast<std::size_t>(d), "invalid tensor size");
    count *= static_cast<std::size_t>(d);
  }
  values.resize(count);
}
Frame::Frame(const Spec &s) : vision(s.cameras * feature_size), camera_mask(s.cameras),
                              sensors(s.joints * sensor_size), joint_mask(s.joints) {}
void preprocess(std::span<const std::uint8_t> rgb, std::size_t w, std::size_t h,
                std::size_t stride, std::span<float> out) {
  require(w > 0 && h > 0 && w <= 16384 && h <= 16384 && w * h <= 16777216 &&
          stride >= w * 3 && stride <= 1000000 && rgb.size() == h * stride && out.size() == 3 * 224 * 224,
          "invalid RGB frame");
  const std::size_t rw = w <= h ? 224 : 224 * w / h, rh = w <= h ? 224 * h / w : 224;
  constexpr std::array<float, 3> mean{0.48145466F, 0.4578275F, 0.40821073F};
  constexpr std::array<float, 3> deviation{0.26862954F, 0.26130258F, 0.27577711F};
  for (std::size_t y = 0; y < 224; ++y) {
    const double fy = std::clamp((static_cast<double>(y + (rh - 224) / 2) + 0.5) *
                      static_cast<double>(h) / static_cast<double>(rh) - 0.5, 0.0, static_cast<double>(h - 1));
    const auto y0 = static_cast<std::size_t>(fy), y1 = std::min(y0 + 1, h - 1);
    const auto dy = static_cast<float>(fy - static_cast<double>(y0));
    for (std::size_t x = 0; x < 224; ++x) {
      const double fx = std::clamp((static_cast<double>(x + (rw - 224) / 2) + 0.5) *
                        static_cast<double>(w) / static_cast<double>(rw) - 0.5, 0.0, static_cast<double>(w - 1));
      const auto x0 = static_cast<std::size_t>(fx), x1 = std::min(x0 + 1, w - 1);
      const auto dx = static_cast<float>(fx - static_cast<double>(x0));
      for (std::size_t c = 0; c < 3; ++c) {
        const float top = static_cast<float>(rgb[y0 * stride + x0 * 3 + c]) * (1 - dx) +
                          static_cast<float>(rgb[y0 * stride + x1 * 3 + c]) * dx;
        const float bottom = static_cast<float>(rgb[y1 * stride + x0 * 3 + c]) * (1 - dx) +
                             static_cast<float>(rgb[y1 * stride + x1 * 3 + c]) * dx;
        out[c * 224 * 224 + y * 224 + x] = ((top * (1 - dy) + bottom * dy) / 255 - mean[c]) / deviation[c];
      }
    }
  }
}
namespace {
std::vector<Tensor> make_inputs(const Spec &s) {
  const auto t = static_cast<std::int64_t>(s.steps), c = static_cast<std::int64_t>(s.cameras),
             j = static_cast<std::int64_t>(s.joints);
  return {Tensor("vision", {1,t,c,5,256}), Tensor("camera_mask", {1,t,c}),
          Tensor("sensors", {1,t,j,20}), Tensor("joint_mask", {1,t,j}),
          Tensor("step_mask", {1,t}), Tensor("text", {1,512})};
}
void append(Tensor &tensor, std::span<const float> frame) {
  require(tensor.values.size() >= frame.size(), "frame shape mismatch");
  const auto shift = frame.size();
  std::memmove(tensor.values.data(), tensor.values.data() + shift,
               (tensor.values.size() - shift) * sizeof(float));
  std::copy(frame.begin(), frame.end(), tensor.values.end() - static_cast<std::ptrdiff_t>(shift));
}
}
Critic::Critic(const fs::path &bundle, const std::string &backend)
    : manifest_(read_json(bundle / "manifest.json")), spec_(Spec::parse(manifest_.at("window"))),
      image_{Tensor("pixels", {1,3,224,224})}, inputs_(make_inputs(spec_)),
      features_("features", {1,5,256}),
      logits_("logits", {1,static_cast<std::int64_t>(manifest_.at("classes").size())}) {
  require(manifest_.at("schema_version") == 1 && manifest_.at("architecture") == "tinyclip8m_causal96_v1" &&
          manifest_.at("preprocess") == "rgb_shortest224_center_bilinear_v1" &&
          manifest_.at("trained") == true && manifest_.at("score_kind") == "uncalibrated_sigmoid", "unsupported critic contract");
  require(logits_.values.size() <= 64, "too many evidence classes");
  for (const auto &name : {"vision.onnx", "head.onnx", "text.f32"})
    require(file_hash(bundle / name) == manifest_.at("files").at(name).get<std::string>(), "critic artifact hash mismatch");
  require(fs::file_size(bundle / "text.f32") == 512 * sizeof(float), "invalid text embedding size");
  std::ifstream text(bundle / "text.f32", std::ios::binary);
  text.read(reinterpret_cast<char *>(inputs_.back().values.data()), 512 * sizeof(float));
  require(text.good() && std::all_of(inputs_.back().values.begin(), inputs_.back().values.end(),
          [](float f) { return std::isfinite(f); }), "invalid text embedding");
  if (backend == "onnx") {
    vision_engine_ = make_onnx(bundle / "vision.onnx", image_, features_);
    head_engine_ = make_onnx(bundle / "head.onnx", inputs_, logits_);
  } else if (backend == "tensorrt") {
    const auto engines = read_json(bundle / "engines.json");
    require(engines.at("manifest_hash") == file_hash(bundle / "manifest.json"), "engine manifest differs from model");
    for (const auto &name : {"vision.engine", "head.engine"})
      require(engines.at(name) == file_hash(bundle / name), "TensorRT engine hash mismatch");
    vision_engine_ = make_tensorrt(bundle / "vision.engine", image_, features_);
    head_engine_ = make_tensorrt(bundle / "head.engine", inputs_, logits_);
  } else throw std::runtime_error("backend must be onnx or tensorrt");
}
void Critic::reset() {
  for (std::size_t i = 0; i < 5; ++i)
    std::fill(inputs_[i].values.begin(), inputs_[i].values.end(), 0);
  last_time_ = -1; epoch_ = -1;
}
void Critic::encode(std::span<const std::uint8_t> rgb, std::size_t w, std::size_t h,
                    std::size_t stride, std::span<float> features) {
  require(features.size() == feature_size, "feature buffer size mismatch");
  preprocess(rgb, w, h, stride, image_[0].values);
  vision_engine_->run(image_, features_);
  require(std::all_of(features_.values.begin(), features_.values.end(), [](float f) { return std::isfinite(f); }),
          "nonfinite vision output");
  std::copy(features_.values.begin(), features_.values.end(), features.begin());
}
Json Critic::push(const Frame &f) {
  require(f.t_ns >= 0 && f.epoch >= 0 && f.vision.size() == spec_.cameras * feature_size &&
          f.camera_mask.size() == spec_.cameras && f.sensors.size() == spec_.joints * sensor_size &&
          f.joint_mask.size() == spec_.joints, "invalid critic frame");
  const auto period = static_cast<Ns>(std::llround(1e9 / spec_.sample_hz));
  if (f.epoch != epoch_ || f.t_ns < last_time_ || (last_time_ >= 0 && f.t_ns - last_time_ > period * 2)) reset();
  require(last_time_ < 0 || f.t_ns > last_time_, "duplicate critic timestamp");
  last_time_ = f.t_ns; epoch_ = f.epoch;
  for (const auto *values : {&f.vision, &f.camera_mask, &f.sensors, &f.joint_mask})
    require(std::all_of(values->begin(), values->end(), [](float v) { return std::isfinite(v); }), "nonfinite critic input");
  for (const auto *mask : {&f.camera_mask, &f.joint_mask})
    require(std::all_of(mask->begin(), mask->end(), [](float v) { return v == 0 || v == 1; }), "invalid critic mask");
  const bool valid = f.valid && std::any_of(f.camera_mask.begin(), f.camera_mask.end(), [](float v){return v == 1;}) &&
                    std::any_of(f.joint_mask.begin(), f.joint_mask.end(), [](float v){return v == 1;});
  append(inputs_[0], f.vision); append(inputs_[1], f.camera_mask);
  append(inputs_[2], f.sensors); append(inputs_[3], f.joint_mask);
  const std::array<float,1> step{valid ? 1.F : 0.F}; append(inputs_[4], step);
  Json result{{"t_ns", f.t_ns}, {"epoch", f.epoch}, {"source", "experimental_critic"},
              {"deployment_validated", false}, {"scores", Json::object()}};
  if (!valid) { result["status"] = "insufficient_evidence"; return result; }
  const float count = std::accumulate(inputs_[4].values.begin(), inputs_[4].values.end(), 0.F);
  if (count < static_cast<float>(spec_.min_steps)) { result["status"] = "warming_up"; return result; }
  head_engine_->run(inputs_, logits_);
  result["status"] = "valid";
  result["score_kind"] = "uncalibrated_sigmoid";
  for (std::size_t i = 0; i < logits_.values.size(); ++i) {
    const float logit = logits_.values[i];
    require(std::isfinite(logit), "nonfinite critic score");
    result["scores"][manifest_.at("classes").at(i).get<std::string>()] =
        1.0 / (1.0 + std::exp(-static_cast<double>(logit)));
  }
  return result;
}
} // namespace harness::tiny
