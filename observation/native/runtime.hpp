#pragma once
#include "store.hpp"
#include <optional>
#include <vector>
namespace harness::observation {
struct Detection {
  std::string detector_id, evidence_class;
  double confidence = 0, before_s = 5, after_s = 2;
  std::string source = "trained_critic";
  std::string deployment_id;
  bool deployment_validated = false;
  Detection() = default;
  Detection(std::string detector, std::string evidence, double score,
            double before = 5, double after = 2,
            std::string detection_source = "trained_critic",
            std::string deployment = {}, bool validated = false)
      : detector_id(std::move(detector)), evidence_class(std::move(evidence)),
        confidence(score), before_s(before), after_s(after),
        source(std::move(detection_source)), deployment_id(std::move(deployment)),
        deployment_validated(validated) {}
  Json json() const {
    return {{"detector_id", detector_id}, {"evidence_class", evidence_class},
            {"confidence", confidence},   {"before_s", before_s},
            {"after_s", after_s},         {"source", source},
            {"deployment_id", deployment_id},
            {"deployment_validated", deployment_validated}};
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
class Runtime {
  const Store &store_;
  CriticAdapter &critic_;
  ControllerAdapter &controller_;
  Json deployment_id_ = nullptr, contract_;
  Json failure_id_ = nullptr;
  std::string deployment_content_hash_, enforcement_event_id_;
  Ns enforcement_started_wall_ns_ = 0;
  double deadline_ = 0;
  int retries_ = 0;
  bool last_match_ = false;
  void record_enforcement(const std::string &outcome, const Json &detail,
                          bool terminal);

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
