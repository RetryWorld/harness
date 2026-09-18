#include "kernel/profile_runtime.hpp"

#include "profile/validator.hpp"

namespace harness::kernel {

std::optional<ProfileRuntime> ProfileRuntime::build(const harness::v1::HarnessProfile& profile) {
    if (!harness::profile::validate(profile).empty()) return std::nullopt;

    ProfileRuntime rt;
    rt.profile_id = profile.profile_id();
    rt.profile_version = profile.profile_version();
    rt.model_node = profile.model_binding().model_node();
    rt.output_topic = profile.model_binding().output_topic();

    rt.dof = profile.embodiment().dof();
    rt.region_lo.reserve(static_cast<std::size_t>(profile.projection().output_region_size()));
    rt.region_hi.reserve(static_cast<std::size_t>(profile.projection().output_region_size()));
    for (const auto& bound : profile.projection().output_region()) {
        rt.region_lo.push_back(bound.min());
        rt.region_hi.push_back(bound.max());
    }
    rt.max_staleness_ms = profile.projection().max_staleness_ms();
    rt.on_reject = profile.projection().on_reject();

    rt.inference_wcet_ms = profile.isolation().inference_wcet_ms();
    rt.critic_budget_total_us = profile.isolation().critic_budget_total_us();

    rt.critics.reserve(static_cast<std::size_t>(profile.critics_size()));
    for (const auto& critic : profile.critics()) {
        CriticRuntime cr;
        cr.critic_id = critic.critic_id();
        cr.evidence_class = critic.evidence_class();
        cr.window_ms = critic.window_ms();
        cr.wcet_us = critic.wcet_us();
        rt.critics.push_back(std::move(cr));
    }

    const auto& recoveries = profile.transfer().band_a().recoveries();
    rt.recoveries.reserve(static_cast<std::size_t>(recoveries.size()));
    for (const auto& recovery : recoveries) {
        RecoveryRuntime rr;
        rr.evidence_class = recovery.evidence_class();
        rr.action_ref = recovery.action_ref();
        rr.retry_budget = recovery.retry_budget();
        rr.timeout_ms = recovery.timeout_ms();
        rr.reentry_predicate = recovery.reentry_predicate();
        rt.recoveries.push_back(std::move(rr));
    }

    const auto& band_b = profile.transfer().band_b();
    rt.band_b_has_reject_rate = band_b.has_reject_rate_max();
    rt.reject_rate_window_ms = band_b.reject_rate_window_ms();
    rt.reject_rate_max = band_b.reject_rate_max();
    rt.band_b_has_overrun = band_b.has_overrun_count_max();
    rt.overrun_count_max = band_b.overrun_count_max();
    rt.band_b_has_ood = band_b.has_ood_signal();
    rt.ood_signal = band_b.ood_signal();
    rt.ood_max = band_b.ood_max();
    rt.fallback_node = band_b.fallback_node();
    rt.latch = band_b.latch();

    rt.clock_domain = profile.timing().clock_domain();

    return rt;
}

}  // namespace harness::kernel
