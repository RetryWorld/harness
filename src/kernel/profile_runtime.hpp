// Flattened, hot-path-friendly view of a validated HarnessProfile. Built once
// in hk_create (allocation is fine there — it is off the control path) and
// never mutated afterwards, so pointers/string_views handed out of it stay
// valid for the handle's lifetime and hk_gate never touches a protobuf
// object.

#ifndef HARNESS_KERNEL_PROFILE_RUNTIME_HPP
#define HARNESS_KERNEL_PROFILE_RUNTIME_HPP

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "harness/v1/profile.pb.h"

namespace harness::kernel {

struct RecoveryRuntime {
    std::string evidence_class;
    std::string action_ref;
    std::uint32_t retry_budget = 0;
    double timeout_ms = 0.0;
    std::int32_t reentry_predicate = 0;  // harness::v1::Predicate
};

struct CriticRuntime {
    std::string critic_id;
    std::string evidence_class;
    double window_ms = 0.0;
    std::uint64_t wcet_us = 0;
};

struct ProfileRuntime {
    std::string profile_id;
    std::string profile_version;
    std::string model_node;
    std::string output_topic;

    std::uint32_t dof = 0;
    std::vector<double> region_lo;
    std::vector<double> region_hi;
    double max_staleness_ms = 0.0;
    std::int32_t on_reject = 0;  // harness::v1::Projection::OnReject

    double inference_wcet_ms = 0.0;
    std::uint64_t critic_budget_total_us = 0;

    std::vector<CriticRuntime> critics;
    std::vector<RecoveryRuntime> recoveries;  // Transfer.band_a.recoveries

    bool band_b_has_reject_rate = false;
    double reject_rate_window_ms = 0.0;
    double reject_rate_max = 0.0;
    bool band_b_has_overrun = false;
    std::uint32_t overrun_count_max = 0;
    bool band_b_has_ood = false;
    std::string ood_signal;
    double ood_max = 0.0;
    std::string fallback_node;
    bool latch = false;

    std::int32_t clock_domain = 0;  // harness::v1::ClockDomain

    // nullopt iff the profile fails structural validation; caller returns
    // HK_ERR_INVALID_PROFILE without constructing a handle.
    static std::optional<ProfileRuntime> build(const harness::v1::HarnessProfile& profile);
};

}  // namespace harness::kernel

#endif  // HARNESS_KERNEL_PROFILE_RUNTIME_HPP
