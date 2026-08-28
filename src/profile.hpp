// Harness Profile — the deployment artifact the kernel consumes.
//
// Currently defined here in C++. This is the temporary home: the profile is the
// one schema with three writers (kernel, registry, sim), so it belongs in
// `schemas/` generated into C++, Pydantic and TypeScript from one definition.
//
// Move it BEFORE the registry stores its first profile. Migrating a populated
// registry to a new schema source is the expensive version of this change.

#ifndef HARNESS_PROFILE_HPP
#define HARNESS_PROFILE_HPP

#include <cstdint>
#include <optional>
#include <string_view>
#include <string>

#include "kernel.hpp"

namespace harness {

struct InferenceBudget {
    double wcet_ms = 0.0;
    double rate_hz = 0.0;
};

struct TransportBudget {
    std::uint32_t max_payload_bytes = 0;
    double deadline_ms = 0.0;
};

struct OperatingRegime {
    std::string signal;
    double max = 0.0;
};

// What space Projection's output_region bounds are expressed in. Torque
// thresholds are embodiment-scoped and non-inheriting (sim-stack decision
// record, ROS 2 §4): a torque bound fitted on one robot's actuators has no
// defensible meaning transferred to another's, so a torque-domain profile
// must name the embodiment it was derived for. Velocity and Cartesian bounds
// carry no such requirement — they are the same physical quantity across
// embodiments.
enum class Domain { kTorque, kVelocity, kCartesian };

struct HarnessProfile {
    std::string profile_version;
    std::string model_node;
    std::string output_topic;
    OutputRegion output_region;
    double max_staleness_ms = 0.0;
    InferenceBudget inference_budget;
    TransportBudget transport_budget;
    OperatingRegime operating_regime;
    std::string fallback_node;
    Domain domain = Domain::kTorque;
    std::string embodiment;  // required iff domain == kTorque

    // Fail closed. A profile that does not validate must not load: a robot
    // running an incoherent envelope is worse than one that refuses to start,
    // because the failure is silent.
    [[nodiscard]] std::optional<std::string> validate() const;
};

struct ProfileParseResult {
    std::optional<HarnessProfile> profile;
    std::string error;  // empty iff profile has a value
};

// Never throws; errors are values. Called from the C ABI.
ProfileParseResult parse_profile(std::string_view json) noexcept;

}  // namespace harness

#endif  // HARNESS_PROFILE_HPP
