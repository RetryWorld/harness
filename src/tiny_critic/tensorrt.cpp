#include "critic.hpp"
#if defined(HARNESS_WITH_TENSORRT)
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>

namespace harness::tiny {
namespace {
void cuda_check(cudaError_t error) {
  if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
}
class Logger final : public nvinfer1::ILogger {
  void log(Severity, const char *) noexcept override {}
};
struct Allocation {
  void *device = nullptr;
  explicit Allocation(std::size_t bytes) { cuda_check(cudaMalloc(&device, bytes)); }
  ~Allocation() { if (device) cudaFree(device); }
  Allocation(const Allocation &) = delete;
  Allocation &operator=(const Allocation &) = delete;
};
struct Stream {
  cudaStream_t stream = nullptr;
  Stream() {
    int least = 0, greatest = 0;
    cuda_check(cudaDeviceGetStreamPriorityRange(&least, &greatest));
    cuda_check(cudaStreamCreateWithPriority(&stream, cudaStreamNonBlocking, least));
  }
  ~Stream() { if (stream) cudaStreamDestroy(stream); }
};
class TensorRt final : public Engine {
  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  Stream stream_;
  std::vector<std::unique_ptr<Allocation>> buffers_;
  std::vector<std::size_t> sizes_;
public:
  TensorRt(const fs::path &path, const std::vector<Tensor> &inputs, const Tensor &output) {
    const auto bytes = fs::file_size(path);
    observation::require(bytes > 0 && bytes <= 256 * 1024 * 1024, "engine size outside limit");
    std::vector<char> data(static_cast<std::size_t>(bytes));
    std::ifstream file(path, std::ios::binary);
    file.read(data.data(), static_cast<std::streamsize>(data.size()));
    observation::require(file.good(), "cannot read TensorRT engine");
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    observation::require(runtime_ != nullptr, "TensorRT runtime unavailable");
    engine_.reset(runtime_->deserializeCudaEngine(data.data(), data.size()));
    observation::require(engine_ != nullptr, "TensorRT engine incompatible with this device/runtime");
    context_.reset(engine_->createExecutionContext());
    observation::require(context_ != nullptr && engine_->getNbIOTensors() == static_cast<int>(inputs.size() + 1), "TensorRT IO contract differs");
    auto bind = [&](const Tensor &tensor, nvinfer1::TensorIOMode mode) {
      const auto *name = tensor.name.c_str();
      const auto shape = engine_->getTensorShape(name);
      observation::require(engine_->getTensorIOMode(name) == mode &&
          engine_->getTensorDataType(name) == nvinfer1::DataType::kFLOAT &&
          engine_->getTensorFormat(name) == nvinfer1::TensorFormat::kLINEAR &&
          shape.nbDims == static_cast<int>(tensor.shape.size()), "TensorRT tensor format differs");
      for (int d = 0; d < shape.nbDims; ++d)
        observation::require(shape.d[d] == tensor.shape[static_cast<std::size_t>(d)], "TensorRT shape differs");
      sizes_.push_back(tensor.values.size() * sizeof(float));
      buffers_.push_back(std::make_unique<Allocation>(sizes_.back()));
      observation::require(context_->setTensorAddress(name, buffers_.back()->device), "cannot bind TensorRT buffer");
    };
    for (const auto &input : inputs) bind(input, nvinfer1::TensorIOMode::kINPUT);
    bind(output, nvinfer1::TensorIOMode::kOUTPUT);
  }
  void run(std::span<const Tensor> inputs, Tensor &output) override {
    // Buffer allocation and tensor binding happened at load time. Synchronize
    // only this worker's stream, never the policy's entire CUDA device.
    observation::require(inputs.size() + 1 == sizes_.size() && output.values.size() * sizeof(float) == sizes_.back(),
                         "TensorRT buffer size mismatch");
    for (std::size_t i = 0; i < inputs.size(); ++i) {
      observation::require(inputs[i].values.size() * sizeof(float) == sizes_[i], "TensorRT input buffer size mismatch");
      cuda_check(cudaMemcpyAsync(buffers_[i]->device, inputs[i].values.data(), sizes_[i], cudaMemcpyHostToDevice, stream_.stream));
    }
    observation::require(context_->enqueueV3(stream_.stream), "TensorRT execution failed");
    cuda_check(cudaMemcpyAsync(output.values.data(), buffers_.back()->device, sizes_.back(), cudaMemcpyDeviceToHost, stream_.stream));
    cuda_check(cudaStreamSynchronize(stream_.stream));
  }
};
}
std::unique_ptr<Engine> make_tensorrt(const fs::path &path, const std::vector<Tensor> &inputs, const Tensor &output) {
  return std::make_unique<TensorRt>(path, inputs, output);
}
}
#else
namespace harness::tiny {
std::unique_ptr<Engine> make_tensorrt(const fs::path &, const std::vector<Tensor> &, const Tensor &) {
  throw std::runtime_error("configure HARNESS_WITH_TENSORRT=ON with the Jetson TensorRT SDK");
}
}
#endif
