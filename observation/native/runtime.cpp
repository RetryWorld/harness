#include "runtime.hpp"
#include <cmath>
namespace harness::observation {
Json Runtime::activate() {
  require(status == "disabled" || status == "blocked",
          "deactivate runtime before activation");
  if (!critic_.ready() || !controller_.ready()) {
    status = "blocked";
    throw std::runtime_error("enforcement unavailable: critic inference and "
                             "recovery controller adapters are required");
  }
  const auto s = store_.snapshot();
  require(!s["installed_deployment"].is_null(),
          "no approved deployment installed");
  const auto &d =
      s["deployments"].at(s["installed_deployment"].get<std::string>());
  const auto &b = s["bundles"].at(d["bundle_id"].get<std::string>());
  require(b["status"] == "approved", "deployment not approved");
  const auto &content = b["content"];
  require(
      critic_.load(store_.verify_artifact(content["critic"], "critic_model"),
                   s["binding"], content["contract"]),
      "critic adapter refused deployment");
  require(controller_.prepare(
              store_.verify_artifact(content["recovery"], "recovery_model"),
              s["binding"], content["contract"]),
          "controller adapter refused deployment");
  contract_ = content["contract"];
  deployment_id_ = d["id"];
  last_match_ = false;
  status = "nominal";
  return snapshot();
}
Json Runtime::snapshot() const {
  return {{"status", status},
          {"deployment_id", deployment_id_},
          {"critic_ready", critic_.ready()},
          {"controller_ready", controller_.ready()},
          {"recovery_attempt",
           status == "recovering" ? Json(retries_ + 1) : Json(nullptr)}};
}
Json Runtime::fallback(const std::string &reason) {
  bool held = false;
  try {
    held = controller_.hold(reason);
  } catch (...) {
  }
  status = held ? "held" : "fault_unconfirmed_hold";
  auto result = snapshot();
  result.update(
      {{"kind", "fallback"}, {"reason", reason}, {"hold_acknowledged", held}});
  return result;
}
Json Runtime::deactivate() {
  if (status != "disabled" && status != "blocked") {
    auto result = fallback("runtime deactivated");
    if (status == "fault_unconfirmed_hold")
      return result;
  }
  status = "disabled";
  deployment_id_ = nullptr;
  return snapshot();
}
Json Runtime::step(const Observations &observations, double now, bool fresh) {
  if (status != "nominal" && status != "recovering")
    return nullptr;
  auto event = [&](const char *kind) {
    auto result = snapshot();
    result["kind"] = kind;
    return result;
  };
  try {
    if (store_.snapshot()["installed_deployment"] != deployment_id_)
      return fallback(
          "installed deployment changed; explicit reactivation required");
    if (!fresh || !std::isfinite(now))
      return fallback("observations or clock stale");
    if (status == "recovering") {
      if (now >= deadline_)
        return fallback("recovery timeout");
      const auto outcome = controller_.poll();
      if (outcome == "succeeded") {
        if (!controller_.resume())
          return fallback("nominal reentry not acknowledged");
        status = "nominal";
        return event("recovery_completed");
      }
      if (outcome == "failed") {
        if (retries_ < contract_["retry_budget"].get<int>()) {
          ++retries_;
          if (controller_.begin()) {
            deadline_ = now + contract_["timeout_ms"].get<double>() / 1000;
            return event("recovery_retry");
          }
        }
        return fallback("recovery failed");
      }
      if (outcome != "running")
        return fallback("invalid controller response");
      return nullptr;
    }
    const auto detection = critic_.evaluate(observations);
    if (detection && (!std::isfinite(detection->confidence) ||
                      detection->confidence < 0 || detection->confidence > 1))
      return fallback("invalid critic confidence");
    const bool match =
        detection &&
        detection->evidence_class ==
            contract_["evidence_class"].get<std::string>() &&
        detection->confidence >= contract_["confidence_min"].get<double>();
    const bool rising = match && !last_match_;
    last_match_ = match;
    if (rising) {
      if (!controller_.begin())
        return fallback("recovery authority not acknowledged");
      status = "recovering";
      retries_ = 0;
      deadline_ = now + contract_["timeout_ms"].get<double>() / 1000;
      return event("recovery_started");
    }
  } catch (const std::exception &error) {
    return fallback(std::string("adapter error: ") + error.what());
  }
  return nullptr;
}
} // namespace harness::observation
