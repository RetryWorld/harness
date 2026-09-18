// Fixed-layout parameter blobs for the two reference critics. Not part of
// schemas/ on purpose: CriticDecl (profile.proto) declares evidence_class,
// window_ms, wcet_us and provenance — the shared, engine-neutral shape every
// critic needs — but not per-critic tuning like "which aux channel" or "what
// epsilon". These reference critics are deliberately trivial (build brief
// §7), so their tuning is a fixed POD struct passed as hkc_create's
// params_pb, not a schema of its own.

#ifndef HARNESS_CRITIC_PARAMS_HPP
#define HARNESS_CRITIC_PARAMS_HPP

#include <cstdint>

namespace harness::critic {

// threshold_critic: fires once `aux[aux_index]` has been on the wrong side
// of `bound` (below, if `trigger_below` else above) for `consecutive_required`
// consecutive hkc_evaluate calls. Proves windowing via cross-call state held
// in hkc_handle, not via hkc_window's history (aux carries only the latest
// sample).
struct ThresholdCriticParams {
    std::uint32_t aux_index = 0;
    double bound = 0.0;
    std::uint8_t trigger_below = 1;
    std::uint32_t consecutive_required = 1;
    std::uint32_t evidence_class_index = 0;
    std::uint64_t declared_wcet_us = 200;
};

// stall_critic: fires when, within the samples handed to one hkc_evaluate
// call, some joint's |position[last] - position[first]| stays under
// `position_epsilon` while |effort| on that joint stays above `effort_bound`
// for every sample in the window. Multi-channel: needs both position and
// effort in the same window to decide.
struct StallCriticParams {
    double position_epsilon = 0.01;
    double effort_bound = 1.0;
    std::uint32_t evidence_class_index = 0;
    std::uint64_t declared_wcet_us = 200;
};

}  // namespace harness::critic

#endif  // HARNESS_CRITIC_PARAMS_HPP
