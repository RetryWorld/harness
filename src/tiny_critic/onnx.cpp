#include "critic.hpp"
#if defined(HARNESS_WITH_ONNX)
#include <onnxruntime_cxx_api.h>
#include <algorithm>

namespace harness::tiny {
namespace {
class Onnx final : public Engine {
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "harness-critic"};
  Ort::SessionOptions options_;
  Ort::Session session_{nullptr};
  Ort::MemoryInfo memory_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
  std::vector<Tensor> buffers_;
  Tensor result_;
  std::vector<Ort::Value> inputs_, outputs_;
  std::vector<const char *> names_;
  std::array<const char *,1> output_name_;
public:
  Onnx(const fs::path &path, const std::vector<Tensor> &inputs, const Tensor &output)
      : buffers_(inputs), result_(output), output_name_{result_.name.c_str()} {
    options_.SetIntraOpNumThreads(1); options_.SetInterOpNumThreads(1);
    options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = Ort::Session(env_, path.c_str(), options_);
    observation::require(session_.GetInputCount() == inputs.size() && session_.GetOutputCount() == 1,
                         "ONNX input/output count differs");
    Ort::AllocatorWithDefaultOptions allocator;
    for (std::size_t i = 0; i < buffers_.size(); ++i) {
      auto &b = buffers_[i];
      const auto info = session_.GetInputTypeInfo(i);
      const auto type = info.GetTensorTypeAndShapeInfo();
      observation::require(session_.GetInputNameAllocated(i, allocator).get() == b.name &&
          type.GetShape() == b.shape && type.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
          "ONNX input contract differs");
      names_.push_back(b.name.c_str());
      inputs_.push_back(Ort::Value::CreateTensor<float>(memory_, b.values.data(), b.values.size(), b.shape.data(), b.shape.size()));
    }
    const auto info = session_.GetOutputTypeInfo(0);
    const auto type = info.GetTensorTypeAndShapeInfo();
    observation::require(session_.GetOutputNameAllocated(0, allocator).get() == result_.name &&
        type.GetShape() == result_.shape && type.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
        "ONNX output contract differs");
    outputs_.push_back(Ort::Value::CreateTensor<float>(memory_, result_.values.data(), result_.values.size(), result_.shape.data(), result_.shape.size()));
  }
  void run(std::span<const Tensor> inputs, Tensor &output) override {
    observation::require(inputs.size() == buffers_.size() && output.values.size() == result_.values.size(),
                         "ONNX buffer size mismatch");
    for (std::size_t i = 0; i < inputs.size(); ++i) {
      observation::require(inputs[i].values.size() == buffers_[i].values.size(), "ONNX input buffer size mismatch");
      std::copy(inputs[i].values.begin(), inputs[i].values.end(), buffers_[i].values.begin());
    }
    session_.Run(Ort::RunOptions{nullptr}, names_.data(), inputs_.data(), inputs_.size(),
                 output_name_.data(), outputs_.data(), outputs_.size());
    std::copy(result_.values.begin(), result_.values.end(), output.values.begin());
  }
};
}
std::unique_ptr<Engine> make_onnx(const fs::path &path, const std::vector<Tensor> &inputs, const Tensor &output) {
  return std::make_unique<Onnx>(path, inputs, output);
}
}
#else
namespace harness::tiny {
std::unique_ptr<Engine> make_onnx(const fs::path &, const std::vector<Tensor> &, const Tensor &) {
  throw std::runtime_error("configure HARNESS_WITH_ONNX=ON for native CPU model inference");
}
}
#endif
