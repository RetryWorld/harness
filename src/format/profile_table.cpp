#include "format/profile_table.hpp"

#include <cstdio>
#include <string>
#include <vector>

#include "format/json.hpp"

namespace harness::format {

namespace {

constexpr std::size_t kLabelWidth = 13;

// "label        rest" — label padded, never truncated, for the same reason
// event_table.cpp pads its event column.
std::string labelled(const std::string& label, const std::string& rest) {
    std::string out = label;
    if (label.size() < kLabelWidth) out.append(kLabelWidth - label.size(), ' ');
    out += rest;
    return out;
}

// A continuation line under a label: aligned with `rest` above it.
std::string indented(const std::string& rest) { return std::string(kLabelWidth, ' ') + rest; }

std::string num(double v) {
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%g", v);
    return std::string(buf);
}

// The marker for an absent `optional`. C11 rejects a profile missing one of
// these, so seeing it here in a profile that validates clean means the field
// is one C11 does not require.
const char* kUnset = "<unset>";

std::string opt(bool present, double v) { return present ? num(v) : kUnset; }
std::string opt_u(bool present, std::uint64_t v) {
    return present ? std::to_string(v) : kUnset;
}

std::string or_unset(const std::string& s) { return s.empty() ? kUnset : s; }

void projection_block(std::string& out, const harness::v1::HarnessProfile& p) {
    const auto& proj = p.projection();
    out += labelled("projection",
                    "on_reject=" + harness::v1::Projection_OnReject_Name(proj.on_reject()) +
                        "  max_staleness=" +
                        opt(proj.has_max_staleness_ms(), proj.max_staleness_ms()) + " ms") +
           "\n";

    if (proj.output_region_size() == 0) {
        out += indented("output_region  <empty>  -- nothing is bounded\n");
        return;
    }
    // Named by joint where the arity agrees. Where it does not, C7 is the
    // constraint that fires, and printing a bare index makes the mismatch
    // visible rather than inventing a name for a bound with no joint.
    const bool named = proj.output_region_size() == p.embodiment().joint_names_size();
    for (int i = 0; i < proj.output_region_size(); ++i) {
        const auto& b = proj.output_region(i);
        const std::string who =
            named ? p.embodiment().joint_names(i) : ("[" + std::to_string(i) + "]");
        out += indented("  " + who + "  [" + opt(b.has_min(), b.min()) + ", " +
                        opt(b.has_max(), b.max()) + "]\n");
    }
}

void isolation_block(std::string& out, const harness::v1::Isolation& iso) {
    out += labelled("isolation", "inference_wcet=" +
                                     opt(iso.has_inference_wcet_ms(), iso.inference_wcet_ms()) +
                                     " ms  rate=" +
                                     opt(iso.has_inference_rate_hz(), iso.inference_rate_hz()) +
                                     " Hz") +
           "\n";
    out += indented("max_payload=" + opt_u(iso.has_max_payload_bytes(), iso.max_payload_bytes()) +
                    " B  transport_deadline=" +
                    opt(iso.has_transport_deadline_ms(), iso.transport_deadline_ms()) + " ms\n");
    out += indented("critic_budget_total=" +
                    opt_u(iso.has_critic_budget_total_us(), iso.critic_budget_total_us()) +
                    " us\n");
}

void critics_block(std::string& out, const harness::v1::HarnessProfile& p) {
    if (p.critics_size() == 0) {
        out += labelled("critics", "<none declared>\n");
        return;
    }
    out += labelled("critics", std::to_string(p.critics_size()) + " declared\n");
    for (const auto& c : p.critics()) {
        out += indented("  " + or_unset(c.critic_id()) + "  evidence=" +
                        or_unset(c.evidence_class()) + "  window=" +
                        opt(c.has_window_ms(), c.window_ms()) + " ms  wcet=" +
                        opt_u(c.has_wcet_us(), c.wcet_us()) + " us  impl=" +
                        or_unset(c.impl_hash()) + "\n");
    }
}

void transfer_block(std::string& out, const harness::v1::Transfer& t) {
    out += "transfer\n";

    const auto& a = t.band_a();
    if (a.recoveries_size() == 0) {
        // Band A without a recovery is not an error on its own, but it does
        // mean Band B is terminal: kernel_gate.cpp leaves Band B only through
        // a Band A reentry.
        out += indented("band_a  <no recoveries>  -- Band B, once entered, is terminal\n");
    } else {
        out += indented("band_a  " + std::to_string(a.recoveries_size()) + " recovery(ies)\n");
        for (const auto& r : a.recoveries()) {
            out += indented("  on " + or_unset(r.evidence_class()) + "  -> " +
                            or_unset(r.action_ref()) + "  retry_budget=" +
                            opt_u(r.has_retry_budget(), r.retry_budget()) + "  timeout=" +
                            opt(r.has_timeout_ms(), r.timeout_ms()) + " ms  reentry=" +
                            harness::v1::Predicate_Name(r.reentry_predicate()) + "\n");
        }
    }

    const auto& b = t.band_b();
    out += indented("band_b  fallback=" + or_unset(b.fallback_node()) + "  latch=" +
                    (b.latch() ? "true" : "false") + "\n");
    out += indented("  reject_rate > " + opt(b.has_reject_rate_max(), b.reject_rate_max()) +
                    " over " + opt(b.has_reject_rate_window_ms(), b.reject_rate_window_ms()) +
                    " ms\n");
    out += indented("  overrun_count > " +
                    opt_u(b.has_overrun_count_max(), b.overrun_count_max()) + "\n");
    out += indented("  ood " + (b.has_ood_signal() ? b.ood_signal() : std::string(kUnset)) +
                    " > " + opt(b.has_ood_max(), b.ood_max()) + "\n");
}

void provenance_block(std::string& out, const harness::v1::ProvenanceBlock& p) {
    const auto& d = p.derivation();
    out += labelled("provenance", "source=" + harness::v1::ProvenanceSource_Name(p.source()) +
                                      "  confidence=" +
                                      harness::v1::Confidence_Name(p.confidence()) + "  by=" +
                                      or_unset(p.derived_by()) + "\n");
    out += indented("engine=" + harness::v1::Engine_Name(d.engine()) + " " +
                    or_unset(d.engine_version()) + "  seed=" + opt_u(d.has_seed(), d.seed()) +
                    "  geometry_tier=" +
                    opt_u(p.has_geometry_provenance_tier(), p.geometry_provenance_tier()) + "\n");
    out += indented("model=" + or_unset(d.model_hash()) + "  engine_config=" +
                    or_unset(d.engine_config_hash()) + "\n");
    if (!p.notes().empty()) out += indented("notes: " + p.notes() + "\n");
}

std::string json_bound(const harness::v1::Bound& b) {
    return std::string("{\"min\":") + (b.has_min() ? json::number(b.min()) : "null") +
           ",\"max\":" + (b.has_max() ? json::number(b.max()) : "null") + "}";
}

std::string json_opt(bool present, double v) { return present ? json::number(v) : "null"; }
std::string json_opt_u(bool present, std::uint64_t v) {
    return present ? std::to_string(v) : "null";
}

}  // namespace

std::string profile_text(const harness::v1::HarnessProfile& profile,
                         const std::string& computed_joint_order_hash) {
    const auto& emb = profile.embodiment();
    const auto& mb = profile.model_binding();

    std::string out;
    out += labelled("profile", or_unset(profile.profile_id()) + "@" +
                                   or_unset(profile.profile_version()) + "  schema_version=" +
                                   std::to_string(profile.schema_version())) +
           "\n";
    if (profile.has_parent_profile()) {
        const auto& parent = profile.parent_profile();
        out += indented("parent " + or_unset(parent.profile_id()) + "@" +
                        or_unset(parent.profile_version()) + "  embodiment_class=" +
                        or_unset(parent.embodiment_class_id()) + "\n");
    }
    out += labelled("embodiment", or_unset(emb.class_id()) + "  dof=" + std::to_string(emb.dof())) +
           "\n";

    std::string joints = "[";
    for (int i = 0; i < emb.joint_names_size(); ++i) {
        if (i > 0) joints += ", ";
        joints += emb.joint_names(i);
    }
    joints += "]";
    out += labelled("joints", std::to_string(emb.joint_names_size()) + "  " + joints) + "\n";

    // Same comparison `hash` makes, shown here because a profile whose
    // declared order hash disagrees with its joint list is describing a
    // different robot than the one it lists.
    out += indented("order_hash " + or_unset(emb.joint_order_hash()) + " (declared)");
    if (emb.joint_order_hash() == computed_joint_order_hash) {
        out += "  ok\n";
    } else {
        out += "\n" + indented("           " + computed_joint_order_hash +
                               " (computed)  MISMATCH -- this is what C8 rejects\n");
    }

    out += labelled("binding", or_unset(mb.model_node()) + " -> " + or_unset(mb.output_topic()) +
                                   " as " +
                                   harness::v1::OutputInterface_Name(mb.output_interface())) +
           "\n";
    out += labelled("timing", "clock=" + harness::v1::ClockDomain_Name(profile.timing().clock_domain()) +
                                  "  authoritative=" +
                                  (profile.timing().timing_authoritative() ? "true" : "false")) +
           "\n";
    out += labelled("eligibility", std::string("profile_eligible=") +
                                       (profile.eligibility().profile_eligible() ? "true" : "false")) +
           "\n\n";

    projection_block(out, profile);
    isolation_block(out, profile.isolation());
    critics_block(out, profile);
    transfer_block(out, profile.transfer());
    provenance_block(out, profile.provenance());
    return out;
}

std::string profile_json(const harness::v1::HarnessProfile& profile,
                         const std::string& computed_joint_order_hash) {
    const auto& emb = profile.embodiment();
    const auto& mb = profile.model_binding();
    const auto& proj = profile.projection();
    const auto& iso = profile.isolation();
    const auto& t = profile.transfer();
    const auto& prov = profile.provenance();
    const auto& d = prov.derivation();

    std::vector<std::string> joint_names;
    for (const auto& j : emb.joint_names()) joint_names.push_back(j);

    std::string out = "{";
    out += "\"profile_id\":" + json::str(profile.profile_id());
    out += ",\"profile_version\":" + json::str(profile.profile_version());
    out += ",\"schema_version\":" + std::to_string(profile.schema_version());

    if (profile.has_parent_profile()) {
        const auto& parent = profile.parent_profile();
        out += ",\"parent_profile\":{\"profile_id\":" + json::str(parent.profile_id()) +
               ",\"profile_version\":" + json::str(parent.profile_version()) +
               ",\"embodiment_class_id\":" + json::str(parent.embodiment_class_id()) + "}";
    } else {
        out += ",\"parent_profile\":null";
    }

    out += ",\"embodiment\":{\"class_id\":" + json::str(emb.class_id()) +
           ",\"dof\":" + std::to_string(emb.dof()) +
           ",\"joint_names\":" + json::str_array(joint_names) +
           ",\"joint_order_hash_declared\":" + json::str(emb.joint_order_hash()) +
           ",\"joint_order_hash_computed\":" + json::str(computed_joint_order_hash) +
           ",\"joint_order_hash_match\":" +
           json::boolean(emb.joint_order_hash() == computed_joint_order_hash) + "}";

    out += ",\"model_binding\":{\"model_node\":" + json::str(mb.model_node()) +
           ",\"output_topic\":" + json::str(mb.output_topic()) + ",\"output_interface\":" +
           json::str(harness::v1::OutputInterface_Name(mb.output_interface())) + "}";

    out += ",\"projection\":{\"on_reject\":" +
           json::str(harness::v1::Projection_OnReject_Name(proj.on_reject())) +
           ",\"max_staleness_ms\":" + json_opt(proj.has_max_staleness_ms(), proj.max_staleness_ms()) +
           ",\"output_region\":[";
    for (int i = 0; i < proj.output_region_size(); ++i) {
        if (i > 0) out += ",";
        out += json_bound(proj.output_region(i));
    }
    out += "]}";

    out += ",\"isolation\":{\"inference_wcet_ms\":" +
           json_opt(iso.has_inference_wcet_ms(), iso.inference_wcet_ms()) +
           ",\"inference_rate_hz\":" + json_opt(iso.has_inference_rate_hz(), iso.inference_rate_hz()) +
           ",\"max_payload_bytes\":" +
           json_opt_u(iso.has_max_payload_bytes(), iso.max_payload_bytes()) +
           ",\"transport_deadline_ms\":" +
           json_opt(iso.has_transport_deadline_ms(), iso.transport_deadline_ms()) +
           ",\"critic_budget_total_us\":" +
           json_opt_u(iso.has_critic_budget_total_us(), iso.critic_budget_total_us()) + "}";

    out += ",\"critics\":[";
    for (int i = 0; i < profile.critics_size(); ++i) {
        const auto& c = profile.critics(i);
        if (i > 0) out += ",";
        out += "{\"critic_id\":" + json::str(c.critic_id()) +
               ",\"evidence_class\":" + json::str(c.evidence_class()) +
               ",\"window_ms\":" + json_opt(c.has_window_ms(), c.window_ms()) +
               ",\"wcet_us\":" + json_opt_u(c.has_wcet_us(), c.wcet_us()) +
               ",\"impl_hash\":" + json::str(c.impl_hash()) + "}";
    }
    out += "]";

    out += ",\"transfer\":{\"band_a\":{\"recoveries\":[";
    for (int i = 0; i < t.band_a().recoveries_size(); ++i) {
        const auto& r = t.band_a().recoveries(i);
        if (i > 0) out += ",";
        out += "{\"evidence_class\":" + json::str(r.evidence_class()) +
               ",\"action_ref\":" + json::str(r.action_ref()) +
               ",\"retry_budget\":" + json_opt_u(r.has_retry_budget(), r.retry_budget()) +
               ",\"timeout_ms\":" + json_opt(r.has_timeout_ms(), r.timeout_ms()) +
               ",\"reentry_predicate\":" +
               json::str(harness::v1::Predicate_Name(r.reentry_predicate())) + "}";
    }
    const auto& b = t.band_b();
    out += "]},\"band_b\":{\"reject_rate_window_ms\":" +
           json_opt(b.has_reject_rate_window_ms(), b.reject_rate_window_ms()) +
           ",\"reject_rate_max\":" + json_opt(b.has_reject_rate_max(), b.reject_rate_max()) +
           ",\"overrun_count_max\":" +
           json_opt_u(b.has_overrun_count_max(), b.overrun_count_max()) + ",\"ood_signal\":" +
           (b.has_ood_signal() ? json::str(b.ood_signal()) : "null") +
           ",\"ood_max\":" + json_opt(b.has_ood_max(), b.ood_max()) +
           ",\"fallback_node\":" + json::str(b.fallback_node()) +
           ",\"latch\":" + json::boolean(b.latch()) + "}}";

    out += ",\"timing\":{\"clock_domain\":" +
           json::str(harness::v1::ClockDomain_Name(profile.timing().clock_domain())) +
           ",\"timing_authoritative\":" + json::boolean(profile.timing().timing_authoritative()) +
           "}";
    out += ",\"eligibility\":{\"profile_eligible\":" +
           json::boolean(profile.eligibility().profile_eligible()) + "}";

    out += ",\"provenance\":{\"source\":" +
           json::str(harness::v1::ProvenanceSource_Name(prov.source())) + ",\"confidence\":" +
           json::str(harness::v1::Confidence_Name(prov.confidence())) +
           ",\"derived_by\":" + json::str(prov.derived_by()) +
           ",\"notes\":" + json::str(prov.notes()) + ",\"geometry_provenance_tier\":" +
           json_opt_u(prov.has_geometry_provenance_tier(), prov.geometry_provenance_tier()) +
           ",\"derivation\":{\"engine\":" + json::str(harness::v1::Engine_Name(d.engine())) +
           ",\"engine_version\":" + json::str(d.engine_version()) +
           ",\"model_hash\":" + json::str(d.model_hash()) +
           ",\"engine_config_hash\":" + json::str(d.engine_config_hash()) +
           ",\"seed\":" + json_opt_u(d.has_seed(), d.seed()) +
           ",\"initial_state_hash\":" + json::str(d.initial_state_hash()) +
           ",\"action_trace_hash\":" + json::str(d.action_trace_hash()) +
           ",\"profile_version\":" + json::str(d.profile_version()) + "}}";

    return out + "}";
}

}  // namespace harness::format
