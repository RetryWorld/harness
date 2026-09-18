#pragma once
#include "store.hpp"
#include <optional>
#include <vector>
namespace harness::observation {
struct Detection {
  std::string detector_id, evidence_class;
  double confidence = 0, before_s = 5, after_s = 2;
  std::string source = "trained_critic";
  Json json() const {
    return {{"detector_id", detector_id}, {"evidence_class", evidence_class},
            {"confidence", confidence},   {"before_s", before_s},
            {"after_s", after_s},         {"source", source}};
  }
};
struct ObservationMessage {
  std::string topic, type;
  std::vector<std::uint8_t> cdr;
  Ns receipt_ros_ns;
};
struct Observations {
  Ns epoch = 0, receipt_ros_ns = 0;
  Json config;
  std::vector<ObservationMessage> messages;
};
struct CriticAdapter {
  virtual ~CriticAdapter() = default;
  virtual bool ready() const = 0;
  virtual bool load(const fs::path &, const Json &, const Json &) = 0;
  virtual std::optional<Detection> evaluate(const Observations &) = 0;
};
struct ControllerAdapter {
  virtual ~ControllerAdapter() = default;
  virtual bool ready() const = 0;
  virtual bool prepare(const fs::path &, const Json &, const Json &) = 0;
  virtual bool begin() = 0;
  virtual std::string poll() = 0;
  virtual bool resume() = 0;
  virtual bool hold(const std::string &) = 0;
};
struct CriticInferencePlaceholder final : CriticAdapter {
  bool ready() const override { return false; }
  bool load(const fs::path &, const Json &, const Json &) override {
    return false;
  }
  std::optional<Detection> evaluate(const Observations &) override {
    return std::nullopt;
  }
};
struct RecoveryControllerPlaceholder final : ControllerAdapter {
  bool ready() const override { return false; }
  bool prepare(const fs::path &, const Json &, const Json &) override {
    return false;
  }
  bool begin() override { return false; }
  std::string poll() override { return "unavailable"; }
  bool resume() override { return false; }
  bool hold(const std::string &) override { return false; }
};
struct DatabaseSyncPlaceholder {
  bool ready() const { return false; }
  void push(const Json &) {
    throw std::runtime_error(
        "database connector not implemented; outbox remains pending");
  }
};
class Runtime {
  const Store &store_;
  CriticAdapter &critic_;
  ControllerAdapter &controller_;
  Json deployment_id_ = nullptr, contract_;
  double deadline_ = 0;
  int retries_ = 0;
  bool last_match_ = false;

public:
  std::string status = "disabled";
  Runtime(const Store &store, CriticAdapter &critic,
          ControllerAdapter &controller)
      : store_(store), critic_(critic), controller_(controller) {}
  Json activate();
  Json snapshot() const;
  Json fallback(const std::string &reason);
  Json deactivate();
  Json step(const Observations &observations, double monotonic_s,
            bool fresh = true);
};
} // namespace harness::observation
