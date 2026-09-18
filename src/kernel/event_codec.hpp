// Field structs and encoders mirroring enforcement.proto's event messages.
// These are views, not owning containers: every string is a string_view and
// every repeated field is a pointer+count into memory the caller already
// owns (the command buffer, the profile's static strings). Nothing here
// allocates, which is what lets hk_gate build one of these on the stack and
// encode it straight into the caller's event sink.
//
// Field numbers below must match schemas/proto/harness/v1/enforcement.proto
// exactly; there is no generated code cross-checking that here, so a change
// on one side without the other is a silent wire-format bug. See build brief
// §12 "a second definition... it will drift" — this file is exactly that
// risk, accepted because §6 constraint 6 forbids the alternative.

#ifndef HARNESS_KERNEL_EVENT_CODEC_HPP
#define HARNESS_KERNEL_EVENT_CODEC_HPP

#include <cstdint>
#include <string_view>

#include "kernel/wire.hpp"

namespace harness::kernel {

struct EventHeader {
    std::string_view event_id;
    std::int32_t mechanism = 0;
    std::string_view profile_version;
    std::string_view model_node;
    std::string_view output_topic;
    std::uint64_t step_index = 0;
    double sim_time_s = 0.0;
    std::uint64_t seq = 0;
    std::uint32_t dropped_since_last = 0;
    std::int32_t clock_domain = 0;
    bool has_wall_time_ns = false;
    std::uint64_t wall_time_ns = 0;
};

struct ProjectionFields {
    std::uint64_t decided_at_ns = 0;
    std::int32_t verdict = 0;
    const std::int32_t* failed = nullptr;
    std::size_t failed_len = 0;
    const double* candidate = nullptr;
    std::size_t candidate_len = 0;
    const double* emitted = nullptr;
    std::size_t emitted_len = 0;
    double margin = 0.0;
    std::string_view binding_predicate_detail;
    const double* pre_decode_eef = nullptr;
    std::size_t pre_decode_eef_len = 0;
};

struct IsolationFields {
    std::uint64_t decided_at_ns = 0;
    double inference_ms = 0.0;
    double wcet_budget_ms = 0.0;
    std::uint32_t payload_bytes = 0;
    std::uint32_t payload_budget_bytes = 0;
    bool compute_overrun = false;
    bool comm_overrun = false;
    double slot_utilisation = 0.0;
};

struct TransferFields {
    std::uint64_t decided_at_ns = 0;
    bool to_baseline = false;
    std::string_view from_node;
    std::string_view to_node;
    double ood_score = 0.0;
    double ood_threshold = 0.0;
    std::string_view trigger;
    double handoff_transient_ms = 0.0;
    bool baseline_stabilised = false;
    double dwell_ms = 0.0;
    std::int32_t from_band = 0;
    std::int32_t to_band = 0;
    std::string_view recovery_ref;
    std::uint32_t retry_index = 0;
    bool preempted_band_a = false;
};

struct CriticFields {
    std::uint64_t decided_at_ns = 0;
    std::string_view critic_id;
    std::string_view evidence_class;
    double confidence = 0.0;
    std::uint32_t suggested_mode = 0;
    const std::uint8_t* evidence_payload = nullptr;
    std::size_t evidence_payload_len = 0;
    std::uint64_t eval_elapsed_us = 0;
    std::uint64_t eval_declared_wcet_us = 0;
};

struct ReentryFields {
    std::uint64_t decided_at_ns = 0;
    std::string_view recovery_ref;
    bool predicate_result = false;
    std::string_view denied_reason;
    std::uint32_t retry_index = 0;
};

namespace detail {

template <typename W>
void encode_stamp(W& w, std::uint64_t ns) {
    wire::put_uint64(w, 1, ns);
}

template <typename W>
void encode_projection(W& w, const ProjectionFields& f) {
    wire::put_message(w, 1, f.decided_at_ns, [](auto& ww, std::uint64_t ns) { encode_stamp(ww, ns); });
    wire::put_enum(w, 2, f.verdict);
    wire::put_packed_enum(w, 3, f.failed, f.failed_len);
    wire::put_packed_double(w, 4, f.candidate, f.candidate_len);
    wire::put_packed_double(w, 5, f.emitted, f.emitted_len);
    wire::put_double(w, 6, f.margin);
    if (!f.binding_predicate_detail.empty()) wire::put_string(w, 7, f.binding_predicate_detail);
    wire::put_packed_double(w, 8, f.pre_decode_eef, f.pre_decode_eef_len);
}

template <typename W>
void encode_isolation(W& w, const IsolationFields& f) {
    wire::put_message(w, 1, f.decided_at_ns, [](auto& ww, std::uint64_t ns) { encode_stamp(ww, ns); });
    wire::put_double(w, 2, f.inference_ms);
    wire::put_double(w, 3, f.wcet_budget_ms);
    wire::put_uint32(w, 4, f.payload_bytes);
    wire::put_uint32(w, 5, f.payload_budget_bytes);
    wire::put_bool(w, 6, f.compute_overrun);
    wire::put_bool(w, 7, f.comm_overrun);
    wire::put_double(w, 8, f.slot_utilisation);
}

template <typename W>
void encode_transfer(W& w, const TransferFields& f) {
    wire::put_message(w, 1, f.decided_at_ns, [](auto& ww, std::uint64_t ns) { encode_stamp(ww, ns); });
    wire::put_bool(w, 2, f.to_baseline);
    if (!f.from_node.empty()) wire::put_string(w, 3, f.from_node);
    if (!f.to_node.empty()) wire::put_string(w, 4, f.to_node);
    wire::put_double(w, 5, f.ood_score);
    wire::put_double(w, 6, f.ood_threshold);
    if (!f.trigger.empty()) wire::put_string(w, 7, f.trigger);
    wire::put_double(w, 8, f.handoff_transient_ms);
    wire::put_bool(w, 9, f.baseline_stabilised);
    wire::put_double(w, 10, f.dwell_ms);
    wire::put_enum(w, 11, f.from_band);
    wire::put_enum(w, 12, f.to_band);
    if (!f.recovery_ref.empty()) wire::put_string(w, 13, f.recovery_ref);
    wire::put_uint32(w, 14, f.retry_index);
    wire::put_bool(w, 15, f.preempted_band_a);
}

template <typename W>
void encode_critic(W& w, const CriticFields& f) {
    wire::put_message(w, 1, f.decided_at_ns, [](auto& ww, std::uint64_t ns) { encode_stamp(ww, ns); });
    if (!f.critic_id.empty()) wire::put_string(w, 2, f.critic_id);
    if (!f.evidence_class.empty()) wire::put_string(w, 3, f.evidence_class);
    wire::put_double(w, 4, f.confidence);
    wire::put_uint32(w, 5, f.suggested_mode);
    if (f.evidence_payload_len != 0) wire::put_bytes(w, 6, f.evidence_payload, f.evidence_payload_len);
    wire::put_uint64(w, 7, f.eval_elapsed_us);
    wire::put_uint64(w, 8, f.eval_declared_wcet_us);
}

template <typename W>
void encode_reentry(W& w, const ReentryFields& f) {
    wire::put_message(w, 1, f.decided_at_ns, [](auto& ww, std::uint64_t ns) { encode_stamp(ww, ns); });
    if (!f.recovery_ref.empty()) wire::put_string(w, 2, f.recovery_ref);
    wire::put_bool(w, 3, f.predicate_result);
    if (!f.denied_reason.empty()) wire::put_string(w, 4, f.denied_reason);
    wire::put_uint32(w, 5, f.retry_index);
}

template <typename W>
void encode_header(W& w, const EventHeader& h) {
    if (!h.event_id.empty()) wire::put_string(w, 1, h.event_id);
    wire::put_enum(w, 2, h.mechanism);
    if (!h.profile_version.empty()) wire::put_string(w, 3, h.profile_version);
    if (!h.model_node.empty()) wire::put_string(w, 4, h.model_node);
    if (!h.output_topic.empty()) wire::put_string(w, 5, h.output_topic);
    wire::put_uint64(w, 6, h.step_index);
    wire::put_double(w, 7, h.sim_time_s);
    wire::put_uint64(w, 8, h.seq);
    wire::put_uint32(w, 9, h.dropped_since_last);
}

template <typename W>
void encode_footer(W& w, const EventHeader& h) {
    wire::put_enum(w, 15, h.clock_domain);
    if (h.has_wall_time_ns) wire::put_uint64(w, 16, h.wall_time_ns);
}

}  // namespace detail

// Encodes one EnforcementEvent (header + exactly one oneof body variant) with
// `w`, which may be a wire::SizeWriter (to learn the size) or a
// wire::BufWriter (to write the bytes). Field number for the body must match
// the oneof tag in enforcement.proto (10=projection .. 14=reentry).
template <typename W, typename BodyFields, typename BodyEncodeFn>
void encode_enforcement_event(W& w, const EventHeader& header, std::uint32_t body_field,
                               const BodyFields& body, BodyEncodeFn&& body_encode) {
    detail::encode_header(w, header);
    wire::put_message(w, body_field, body, body_encode);
    detail::encode_footer(w, header);
}

inline auto projection_encoder() {
    return [](auto& w, const ProjectionFields& f) { detail::encode_projection(w, f); };
}
inline auto isolation_encoder() {
    return [](auto& w, const IsolationFields& f) { detail::encode_isolation(w, f); };
}
inline auto transfer_encoder() {
    return [](auto& w, const TransferFields& f) { detail::encode_transfer(w, f); };
}
inline auto critic_encoder() {
    return [](auto& w, const CriticFields& f) { detail::encode_critic(w, f); };
}
inline auto reentry_encoder() {
    return [](auto& w, const ReentryFields& f) { detail::encode_reentry(w, f); };
}

}  // namespace harness::kernel

#endif  // HARNESS_KERNEL_EVENT_CODEC_HPP
