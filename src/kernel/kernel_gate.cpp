// The enforcement path. See kernel_state.hpp for kMaxJoints, kReentryStableCycles,
// kRejectRateMinSamples — the fixed bounds that keep this file's work
// bounded and allocation-free.
//
// Two design decisions the build brief leaves implicit, made explicit here:
//
// 1. Projection always clamps a violating command to the declared bound in
//    the same cycle (decision=HK_CLAMP, verdict=REPLACE) — that is what
//    "Projection" does mechanically, every cycle, independent of Transfer.
//    `on_reject` (HOLD_LAST/ZERO/REJECT) is NOT that per-cycle behaviour; it
//    is what BAND B EMITS once the kernel has fallen back, since Band B has
//    no model output to clamp against. This reading is what makes golden
//    trace `b_torque_excursion`'s documented shape ("CLAMP then sustained
//    rejects -> transition to BAND_B on rate trigger") coherent: the clamps
//    happen every cycle from the first violation, and only their *rate*
//    crossing the Band B threshold triggers a transfer.
// 2. hkc_proposal.evidence_class is documented on the ABI as "an index into
//    the profile's declared evidence classes" — that list is
//    ProfileRuntime.critics, in profile.proto declaration order, i.e. the
//    proposal names which critic fired by index, and the kernel looks up
//    that critic's evidence_class string to match against declared
//    recoveries.

#include "kernel/kernel_gate.hpp"

#include <algorithm>
#include <array>
#include <limits>

#include "harness/v1/enforcement.pb.h"
#include "harness/v1/profile.pb.h"
#include "kernel/event_codec.hpp"
#include "kernel/event_sink.hpp"

namespace harness::kernel {

namespace {

using harness::v1::Mechanism;
using harness::v1::Predicate;
using harness::v1::Verdict;

// hk_band (C ABI, HK_BAND_NOMINAL=0) and harness::v1::Band (wire, BAND_
// UNSPECIFIED=0/BAND_NOMINAL=1) are deliberately different enumerations —
// the ABI has no "unspecified" state, every hk_gate call reports a real
// band — so a bare cast between them silently shifts every value by one.
std::int32_t wire_band(hk_band b) {
    switch (b) {
        case HK_BAND_NOMINAL: return static_cast<std::int32_t>(harness::v1::BAND_NOMINAL);
        case HK_BAND_A: return static_cast<std::int32_t>(harness::v1::BAND_A);
        case HK_BAND_B: return static_cast<std::int32_t>(harness::v1::BAND_B);
    }
    return static_cast<std::int32_t>(harness::v1::BAND_UNSPECIFIED);
}

EventHeader base_header(const hk_handle& h, Mechanism mechanism, std::uint64_t t_ns) {
    EventHeader header;
    header.mechanism = mechanism;
    header.profile_version = h.profile.profile_version;
    header.model_node = h.profile.model_node;
    header.output_topic = h.profile.output_topic;
    header.step_index = h.step_index;
    header.clock_domain = h.profile.clock_domain;
    if (h.profile.clock_domain == harness::v1::CLOCK_SIM) {
        header.sim_time_s = static_cast<double>(t_ns) / 1e9;
    }
    if (h.profile.clock_domain == harness::v1::CLOCK_WALL) {
        header.has_wall_time_ns = true;
        header.wall_time_ns = t_ns;
    }
    return header;
}

template <typename BodyFields, typename EncodeFn>
void emit(hk_handle& h, hk_event_sink* sink, Mechanism mechanism, std::uint64_t t_ns,
          std::uint32_t body_field, const BodyFields& body, EncodeFn&& encode_fn) {
    if (sink == nullptr) return;
    EventHeader header = base_header(h, mechanism, t_ns);
    // seq advances for every event the kernel ATTEMPTS to emit, including one
    // that is then dropped, so a gap in seq is an independent record of
    // evidence loss even if this field is never read.
    header.seq = h.seq++;
    header.dropped_since_last = h.dropped_pending;

    if (try_write_event(sink, header, body_field, body, encode_fn)) {
        // The drops this event just reported are now accounted for.
        h.dropped_pending = 0;
    } else if (h.dropped_pending < std::numeric_limits<std::uint32_t>::max()) {
        // Saturate rather than wrap: dropped_since_last is a uint32 on the
        // wire, and a wrapped count is worse than a pinned one.
        h.dropped_pending++;
    }
}

}  // namespace

hk_status run_gate_cycle(hk_handle& h, const hk_command& cmd, hk_gated_command& out,
                          hk_event_sink* sink) noexcept {
    if (cmd.effort == nullptr || out.effort == nullptr) return HK_ERR_NULL_ARG;
    if (cmd.n_joints != h.profile.dof || out.n_joints < cmd.n_joints || cmd.n_joints > kMaxJoints) {
        return HK_ERR_JOINT_MISMATCH;
    }
    const std::uint32_t dof = cmd.n_joints;

    // ---- Projection ----
    std::array<double, kMaxJoints> projected{};
    bool any_violation = false;
    double worst_margin = std::numeric_limits<double>::infinity();
    std::int32_t binding_index = -1;
    for (std::uint32_t i = 0; i < dof; ++i) {
        double v = cmd.effort[i];
        double lo = h.profile.region_lo[i];
        double hi = h.profile.region_hi[i];
        double margin = std::min(v - lo, hi - v);
        if (margin < worst_margin) {
            worst_margin = margin;
            binding_index = static_cast<std::int32_t>(i);
        }
        if (margin < 0.0) any_violation = true;
        projected[i] = std::min(std::max(v, lo), hi);
    }

    std::int32_t failed_pred = static_cast<std::int32_t>(Predicate::VALUE_RANGE);
    ProjectionFields pf;
    pf.decided_at_ns = cmd.t_ns;
    pf.verdict = static_cast<std::int32_t>(any_violation ? Verdict::REPLACE : Verdict::ADMIT);
    pf.failed = any_violation ? &failed_pred : nullptr;
    pf.failed_len = any_violation ? 1U : 0U;
    pf.candidate = cmd.effort;
    pf.candidate_len = dof;
    pf.emitted = projected.data();
    pf.emitted_len = dof;
    pf.margin = worst_margin;
    (void)binding_index;
    emit(h, sink, Mechanism::PROJECTION, cmd.t_ns, 10, pf, projection_encoder());

    // Bounded reject-rate lookback window (Band B trigger input).
    h.reject_window[h.reject_window_write % hk_handle::kRejectWindowCap] = {cmd.t_ns, any_violation};
    h.reject_window_write++;
    h.reject_window_count = std::min(h.reject_window_count + 1, hk_handle::kRejectWindowCap);

    // ---- Isolation: flush any budget reports staged since the last cycle ----
    for (std::size_t src = 0; src < h.pending_reports.size(); ++src) {
        auto& report = h.pending_reports[src];
        if (!report.pending) continue;
        report.pending = false;

        double budget_ms = 0.0;
        if (src == 0) {
            budget_ms = h.profile.inference_wcet_ms;
        } else if (src - 1 < h.profile.critics.size()) {
            budget_ms = static_cast<double>(h.profile.critics[src - 1].wcet_us) / 1000.0;
        }
        double elapsed_ms = static_cast<double>(report.elapsed_ns) / 1e6;
        bool overrun = elapsed_ms > budget_ms;
        if (overrun) h.overrun_count++;

        IsolationFields isf;
        isf.decided_at_ns = cmd.t_ns;
        isf.inference_ms = elapsed_ms;
        isf.wcet_budget_ms = budget_ms;
        isf.compute_overrun = overrun;
        isf.slot_utilisation = budget_ms > 0.0 ? elapsed_ms / budget_ms : 0.0;
        emit(h, sink, Mechanism::ISOLATION, cmd.t_ns, 11, isf, isolation_encoder());
    }

    // ---- Transfer ----
    // 1. Band A entry from a staged critic proposal. Band B preempts
    //    unconditionally: no new Band A entry is considered while in Band B.
    if (h.has_pending_proposal && h.band != HK_BAND_B) {
        const hkc_proposal& prop = h.pending_proposal;
        if (prop.evidence_class < h.profile.critics.size()) {
            const CriticRuntime& critic = h.profile.critics[prop.evidence_class];

            CriticFields cf;
            cf.decided_at_ns = cmd.t_ns;
            cf.critic_id = critic.critic_id;
            cf.evidence_class = critic.evidence_class;
            cf.confidence = static_cast<double>(prop.confidence);
            cf.suggested_mode = prop.suggested_mode;
            cf.evidence_payload = prop.payload;
            cf.evidence_payload_len = prop.payload_len;
            cf.eval_declared_wcet_us = critic.wcet_us;
            emit(h, sink, Mechanism::CRITIC, cmd.t_ns, 13, cf, critic_encoder());

            if (h.band == HK_BAND_NOMINAL) {
                int recovery_idx = -1;
                for (std::size_t i = 0; i < h.profile.recoveries.size(); ++i) {
                    if (h.profile.recoveries[i].evidence_class == critic.evidence_class) {
                        recovery_idx = static_cast<int>(i);
                        break;
                    }
                }
                if (recovery_idx >= 0) {
                    hk_band from = h.band;
                    h.band = HK_BAND_A;
                    h.active_recovery_index = recovery_idx;
                    h.active_retry_index = 0;
                    h.band_a_entry_t_ns = cmd.t_ns;
                    h.consecutive_admits = 0;

                    TransferFields tf;
                    tf.decided_at_ns = cmd.t_ns;
                    tf.from_band = wire_band(from);
                    tf.to_band = wire_band(HK_BAND_A);
                    tf.recovery_ref = h.profile.recoveries[static_cast<std::size_t>(recovery_idx)].action_ref;
                    tf.trigger = "critic_evidence";
                    emit(h, sink, Mechanism::TRANSFER, cmd.t_ns, 12, tf, transfer_encoder());
                }
            }
        }
        h.has_pending_proposal = false;
    }

    // 2. Band A progress: reentry on sustained stability, or retry/escalate
    //    on timeout.
    if (h.band == HK_BAND_A) {
        h.consecutive_admits = any_violation ? 0 : (h.consecutive_admits + 1);
        const RecoveryRuntime& recovery =
            h.profile.recoveries[static_cast<std::size_t>(h.active_recovery_index)];
        double elapsed_ms = static_cast<double>(cmd.t_ns - h.band_a_entry_t_ns) / 1e6;

        if (h.consecutive_admits >= kReentryStableCycles) {
            ReentryFields rf;
            rf.decided_at_ns = cmd.t_ns;
            rf.recovery_ref = recovery.action_ref;
            rf.predicate_result = true;
            rf.retry_index = h.active_retry_index;
            emit(h, sink, Mechanism::REENTRY, cmd.t_ns, 14, rf, reentry_encoder());

            hk_band from = h.band;
            h.band = HK_BAND_NOMINAL;
            h.active_recovery_index = -1;

            TransferFields tf;
            tf.decided_at_ns = cmd.t_ns;
            tf.from_band = wire_band(from);
            tf.to_band = wire_band(HK_BAND_NOMINAL);
            tf.recovery_ref = recovery.action_ref;
            tf.trigger = "reentry_pass";
            emit(h, sink, Mechanism::TRANSFER, cmd.t_ns, 12, tf, transfer_encoder());
        } else if (elapsed_ms >= recovery.timeout_ms) {
            if (h.active_retry_index + 1 < recovery.retry_budget) {
                h.active_retry_index++;
                h.band_a_entry_t_ns = cmd.t_ns;
                h.consecutive_admits = 0;

                ReentryFields rf;
                rf.decided_at_ns = cmd.t_ns;
                rf.recovery_ref = recovery.action_ref;
                rf.predicate_result = false;
                rf.denied_reason = "timeout_retry";
                rf.retry_index = h.active_retry_index;
                emit(h, sink, Mechanism::REENTRY, cmd.t_ns, 14, rf, reentry_encoder());
            } else {
                ReentryFields rf;
                rf.decided_at_ns = cmd.t_ns;
                rf.recovery_ref = recovery.action_ref;
                rf.predicate_result = false;
                rf.denied_reason = "retry_budget_exhausted";
                rf.retry_index = h.active_retry_index;
                emit(h, sink, Mechanism::REENTRY, cmd.t_ns, 14, rf, reentry_encoder());

                hk_band from = h.band;
                h.band = HK_BAND_B;
                h.band_b_latched = h.profile.latch;
                h.active_recovery_index = -1;

                TransferFields tf;
                tf.decided_at_ns = cmd.t_ns;
                tf.from_band = wire_band(from);
                tf.to_band = wire_band(HK_BAND_B);
                tf.recovery_ref = recovery.action_ref;
                tf.preempted_band_a = true;
                tf.trigger = "reentry_denied";
                emit(h, sink, Mechanism::TRANSFER, cmd.t_ns, 12, tf, transfer_encoder());
            }
        }
    }

    // 3. Band B triggers. Evaluated every cycle Band B isn't already active;
    //    a latched Band B never releases itself (no reset path in this ABI).
    if (h.band != HK_BAND_B) {
        bool reject_trigger = false;
        if (h.profile.band_b_has_reject_rate && h.reject_window_count >= kRejectRateMinSamples) {
            std::size_t total = 0;
            std::size_t rejected = 0;
            double window_ns = h.profile.reject_rate_window_ms * 1e6;
            for (std::size_t k = 0; k < h.reject_window_count; ++k) {
                std::size_t idx = (h.reject_window_write - 1 - k) % hk_handle::kRejectWindowCap;
                const auto& sample = h.reject_window[idx];
                if (static_cast<double>(cmd.t_ns - sample.t_ns) > window_ns) break;
                total++;
                if (sample.rejected) rejected++;
            }
            if (total >= kRejectRateMinSamples) {
                double rate = static_cast<double>(rejected) / static_cast<double>(total);
                reject_trigger = rate > h.profile.reject_rate_max;
            }
        }
        bool overrun_trigger =
            h.profile.band_b_has_overrun && (h.overrun_count >= h.profile.overrun_count_max);

        if (reject_trigger || overrun_trigger) {
            hk_band from = h.band;
            bool was_band_a = (from == HK_BAND_A);
            h.band = HK_BAND_B;
            h.band_b_latched = h.profile.latch;
            h.active_recovery_index = -1;

            TransferFields tf;
            tf.decided_at_ns = cmd.t_ns;
            tf.from_band = wire_band(from);
            tf.to_band = wire_band(HK_BAND_B);
            tf.to_node = h.profile.fallback_node;
            tf.preempted_band_a = was_band_a;
            tf.trigger = reject_trigger ? "projection_reject_rate" : "isolation_overrun";
            emit(h, sink, Mechanism::TRANSFER, cmd.t_ns, 12, tf, transfer_encoder());
        }
    }

    // ---- Output ----
    if (h.band == HK_BAND_B) {
        using harness::v1::Projection;
        switch (h.profile.on_reject) {
            case Projection::ON_REJECT_HOLD_LAST:
                if (h.have_last_admitted) {
                    std::copy_n(h.last_admitted.begin(), dof, out.effort);
                } else {
                    std::fill_n(out.effort, dof, 0.0);
                }
                out.decision = HK_HOLD_LAST;
                break;
            case Projection::ON_REJECT_REJECT:
                std::fill_n(out.effort, dof, 0.0);
                out.decision = HK_REJECT;
                break;
            case Projection::ON_REJECT_ZERO:
            default:
                std::fill_n(out.effort, dof, 0.0);
                out.decision = HK_CLAMP;
                break;
        }
        out.band = HK_BAND_B;
    } else {
        std::copy_n(projected.begin(), dof, out.effort);
        out.decision = any_violation ? HK_CLAMP : HK_ADMIT;
        out.band = h.band;
        if (!any_violation) {
            if (h.last_admitted.size() != dof) h.last_admitted.assign(dof, 0.0);
            std::copy_n(projected.begin(), dof, h.last_admitted.begin());
            h.have_last_admitted = true;
        }
    }
    out.n_joints = dof;

    h.step_index++;
    return HK_OK;
}

hk_status submit_proposal(hk_handle& h, const hkc_proposal& proposal) noexcept {
    h.pending_proposal = proposal;
    h.has_pending_proposal = true;
    return HK_OK;
}

hk_status report_budget(hk_handle& h, std::uint32_t source_id, std::uint64_t elapsed_ns) noexcept {
    if (source_id >= h.pending_reports.size()) return HK_ERR_NULL_ARG;
    h.pending_reports[source_id].pending = true;
    h.pending_reports[source_id].elapsed_ns = elapsed_ns;
    return HK_OK;
}

}  // namespace harness::kernel
