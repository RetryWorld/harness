#include "profile.hpp"

#include <cmath>
#include <sstream>

#include "json.hpp"

namespace harness {

namespace {

const json::Value* require(const json::Value& obj, std::string_view key, std::string& err) {
    const json::Value* v = obj.find(key);
    if (v == nullptr && err.empty()) {
        err = "missing required field: " + std::string(key);
    }
    return v;
}

bool read_number(const json::Value& obj, std::string_view key, double& out, std::string& err) {
    const json::Value* v = require(obj, key, err);
    if (v == nullptr) return false;
    if (!v->is_number()) {
        if (err.empty()) err = "field is not a number: " + std::string(key);
        return false;
    }
    out = v->number();
    return true;
}

bool read_string(const json::Value& obj, std::string_view key, std::string& out, std::string& err) {
    const json::Value* v = require(obj, key, err);
    if (v == nullptr) return false;
    if (!v->is_string()) {
        if (err.empty()) err = "field is not a string: " + std::string(key);
        return false;
    }
    out = v->string();
    return true;
}

bool read_doubles(const json::Value& obj, std::string_view key, std::vector<double>& out,
                  std::string& err) {
    const json::Value* v = require(obj, key, err);
    if (v == nullptr) return false;
    if (!v->is_array()) {
        if (err.empty()) err = "field is not an array: " + std::string(key);
        return false;
    }
    out.clear();
    out.reserve(v->array().size());
    for (const json::Value& e : v->array()) {
        if (!e.is_number()) {
            if (err.empty()) err = "non-numeric element in: " + std::string(key);
            return false;
        }
        out.push_back(e.number());
    }
    return true;
}

}  // namespace

std::optional<std::string> HarnessProfile::validate() const {
    if (output_region.lo.size() != output_region.hi.size()) {
        return "output_region lo/hi arity mismatch";
    }
    if (output_region.lo.empty()) {
        return "output_region is empty; a gate with no bounds admits everything";
    }
    for (std::size_t i = 0; i < output_region.lo.size(); ++i) {
        const double lo = output_region.lo[i];
        const double hi = output_region.hi[i];
        if (!std::isfinite(lo) || !std::isfinite(hi)) {
            return "output_region[" + std::to_string(i) + "] non-finite";
        }
        if (lo > hi) {
            return "output_region[" + std::to_string(i) + "] lo > hi";
        }
    }
    if (inference_budget.wcet_ms <= 0.0 || inference_budget.rate_hz <= 0.0) {
        return "inference_budget must be positive";
    }
    // Isolation's joint admission test (paper §5): a reservation feasible on
    // each axis alone can be infeasible when both are claimed at once. Checking
    // it here means an incoherent budget is rejected at configuration time
    // rather than surfacing as a runtime anomaly nobody attributes correctly.
    const double period_ms = 1000.0 / inference_budget.rate_hz;
    if (inference_budget.wcet_ms > period_ms) {
        std::ostringstream oss;
        oss << "wcet_ms " << inference_budget.wcet_ms << " exceeds period " << period_ms
            << "ms at " << inference_budget.rate_hz << "Hz — reservation infeasible";
        return oss.str();
    }
    if (max_staleness_ms < 0.0) return "max_staleness_ms must be non-negative";
    if (fallback_node.empty()) {
        return "fallback_node is empty; Transfer would have nowhere to hand authority";
    }
    return std::nullopt;
}

ProfileParseResult parse_profile(std::string_view json_text) noexcept {
    try {
        json::ParseResult pr = json::parse(json_text);
        if (!pr.ok()) {
            return {std::nullopt, "JSON: " + pr.error->message + " at offset " +
                                      std::to_string(pr.error->offset)};
        }
        const json::Value& root = *pr.value;
        if (!root.is_object()) return {std::nullopt, "profile root is not an object"};

        HarnessProfile p;
        std::string err;

        read_string(root, "profile_version", p.profile_version, err);
        read_string(root, "model_node", p.model_node, err);
        read_string(root, "output_topic", p.output_topic, err);
        read_string(root, "fallback_node", p.fallback_node, err);
        read_number(root, "max_staleness_ms", p.max_staleness_ms, err);

        if (const json::Value* region = require(root, "output_region", err)) {
            read_doubles(*region, "lo", p.output_region.lo, err);
            read_doubles(*region, "hi", p.output_region.hi, err);
        }
        if (const json::Value* ib = require(root, "inference_budget", err)) {
            read_number(*ib, "wcet_ms", p.inference_budget.wcet_ms, err);
            read_number(*ib, "rate_hz", p.inference_budget.rate_hz, err);
        }
        if (const json::Value* tb = require(root, "transport_budget", err)) {
            double bytes = 0.0;
            read_number(*tb, "max_payload_bytes", bytes, err);
            p.transport_budget.max_payload_bytes = static_cast<std::uint32_t>(bytes);
            read_number(*tb, "deadline_ms", p.transport_budget.deadline_ms, err);
        }
        if (const json::Value* orr = require(root, "operating_regime", err)) {
            read_string(*orr, "signal", p.operating_regime.signal, err);
            read_number(*orr, "max", p.operating_regime.max, err);
        }

        if (!err.empty()) return {std::nullopt, err};
        if (std::optional<std::string> v = p.validate()) return {std::nullopt, *v};
        return {std::move(p), {}};
    } catch (const std::exception& e) {
        // noexcept is the contract; an allocation failure inside the parser must
        // become an error code, not terminate a customer's process.
        return {std::nullopt, std::string("internal: ") + e.what()};
    } catch (...) {
        return {std::nullopt, "internal: unknown exception"};
    }
}

}  // namespace harness
