#include "profile/validator.hpp"

#include <array>
#include <cstdio>
#include <set>

#include "harness/v1/enforcement.pb.h"
#include "harness/v1/provenance.pb.h"

namespace harness::profile {

namespace {

using harness::v1::HarnessProfile;

// C1: compile-time constant, not a profile field, so it cannot be widened by
// configuration. Confirmed against the current stack decision: MuJoCo and
// Gazebo rehearsal are both trusted enough to promote thresholds from;
// hardware is always eligible.
constexpr std::array<harness::v1::Engine, 3> kEligibleEngines = {
    harness::v1::ENGINE_MUJOCO,
    harness::v1::ENGINE_GAZEBO,
    harness::v1::ENGINE_HARDWARE,
};

bool is_eligible_engine(harness::v1::Engine e) {
    for (auto eligible : kEligibleEngines) {
        if (e == eligible) return true;
    }
    return false;
}

void push(std::vector<ValidationError>& out, Constraint c, const char* code, std::string msg) {
    out.push_back(ValidationError{c, code, std::move(msg)});
}

void check_c1(const HarnessProfile& p, std::vector<ValidationError>& out) {
    if (!p.eligibility().profile_eligible()) return;
    const auto& derivation = p.provenance().derivation();
    if (!is_eligible_engine(derivation.engine())) {
        push(out, Constraint::C1_INELIGIBLE_ENGINE, "C1",
             "profile_eligible=true requires derivation.engine in {MUJOCO, GAZEBO, HARDWARE} "
             "(got " + harness::v1::Engine_Name(derivation.engine()) + ")");
    }
    if (derivation.engine_version().empty()) {
        push(out, Constraint::C1_INELIGIBLE_ENGINE, "C1",
             "profile_eligible=true requires a non-empty derivation.engine_version");
    }
}

void check_c2(const HarnessProfile& p, std::vector<ValidationError>& out) {
    if (p.timing().timing_authoritative() &&
        p.timing().clock_domain() != harness::v1::CLOCK_WALL) {
        push(out, Constraint::C2_SIM_TIMING_AUTHORITATIVE, "C2",
             "timing_authoritative=true requires clock_domain == CLOCK_WALL "
             "(simulated timing is never authoritative)");
    }
}

void check_c3(const HarnessProfile& p, std::vector<ValidationError>& out) {
    std::set<std::string> declared;
    for (const auto& critic : p.critics()) declared.insert(critic.evidence_class());
    for (const auto& recovery : p.transfer().band_a().recoveries()) {
        if (!declared.contains(recovery.evidence_class())) {
            push(out, Constraint::C3_RECOVERY_EVIDENCE_UNDECLARED, "C3",
                 "recovery '" + recovery.action_ref() + "' references evidence_class '" +
                     recovery.evidence_class() + "' which no critic declares");
        }
    }
}

void check_c4(const HarnessProfile& p, std::vector<ValidationError>& out) {
    if (!p.isolation().has_critic_budget_total_us()) return;  // C11 already flags this
    std::uint64_t sum = 0;
    for (const auto& critic : p.critics()) {
        if (critic.has_wcet_us()) sum += critic.wcet_us();
    }
    if (sum > p.isolation().critic_budget_total_us()) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "sum(critics[].wcet_us)=%llu exceeds critic_budget_total_us=%llu",
                      static_cast<unsigned long long>(sum),
                      static_cast<unsigned long long>(p.isolation().critic_budget_total_us()));
        push(out, Constraint::C4_CRITIC_BUDGET_EXCEEDED, "C4", buf);
    }
}

void check_c5(const HarnessProfile& p, std::vector<ValidationError>& out) {
    const auto& band_b = p.transfer().band_b();
    bool has_trigger = band_b.has_reject_rate_max() || band_b.has_overrun_count_max() ||
                        band_b.has_ood_signal();
    if (!has_trigger) {
        push(out, Constraint::C5_BAND_B_INCOMPLETE, "C5",
             "Band B declares no trigger (need at least one of reject_rate_max, "
             "overrun_count_max, ood_signal)");
    }
    if (band_b.fallback_node().empty()) {
        push(out, Constraint::C5_BAND_B_INCOMPLETE, "C5", "Band B fallback_node must be non-empty");
    }
}

void check_c6(const HarnessProfile& p, std::vector<ValidationError>& out) {
    for (const auto& recovery : p.transfer().band_a().recoveries()) {
        if (recovery.reentry_predicate() == harness::v1::PREDICATE_UNSPECIFIED) {
            push(out, Constraint::C6_MISSING_REENTRY_PREDICATE, "C6",
                 "recovery '" + recovery.action_ref() + "' has no reentry_predicate");
        }
    }
}

void check_c7(const HarnessProfile& p, std::vector<ValidationError>& out) {
    int region_len = p.projection().output_region_size();
    int dof = static_cast<int>(p.embodiment().dof());
    int joint_len = p.embodiment().joint_names_size();
    if (region_len != dof || dof != joint_len) {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "len(output_region)=%d, dof=%d, len(joint_names)=%d must all be equal",
                      region_len, dof, joint_len);
        push(out, Constraint::C7_JOINT_ARITY_MISMATCH, "C7", buf);
    }
}

void check_c8(const HarnessProfile& p, std::vector<ValidationError>& out) {
    std::string expected = canonical_joint_order_hash(p.embodiment().joint_names());
    if (p.embodiment().joint_order_hash() != expected) {
        push(out, Constraint::C8_JOINT_ORDER_HASH_MISMATCH, "C8",
             "joint_order_hash '" + p.embodiment().joint_order_hash() +
                 "' does not match hash of joint_names (expected '" + expected + "')");
    }
}

void check_c9(const HarnessProfile& p, std::vector<ValidationError>& out) {
    if (!p.has_parent_profile()) return;
    const auto& parent_class = p.parent_profile().embodiment_class_id();
    if (!parent_class.empty() && parent_class != p.embodiment().class_id()) {
        push(out, Constraint::C9_PARENT_EMBODIMENT_MISMATCH, "C9",
             "parent_profile.embodiment_class_id '" + parent_class +
                 "' does not match embodiment.class_id '" + p.embodiment().class_id() + "'");
    }
}

void check_c10(const HarnessProfile& p, std::vector<ValidationError>& out) {
    if (p.provenance().confidence() == harness::v1::CONFIDENCE_DEPLOYED &&
        p.provenance().source() != harness::v1::PROVENANCE_SOURCE_HARDWARE) {
        push(out, Constraint::C10_DEPLOYED_CONFIDENCE_NOT_HARDWARE, "C10",
             "confidence=DEPLOYED requires source=HARDWARE (got " +
                 harness::v1::ProvenanceSource_Name(p.provenance().source()) + ")");
    }
}

// C11: every optional scalar in §4.1 that is a bound, budget, threshold, or
// timeout must have presence. Band B's individual trigger fields are exempt
// here — C5 is the normative check for those, and requires only that at
// least one trigger is configured, not all of them.
void check_c11(const HarnessProfile& p, std::vector<ValidationError>& out) {
    auto require = [&](bool present, const char* field) {
        if (!present) {
            push(out, Constraint::C11_MISSING_PRESENCE, "C11",
                 std::string("missing required presence for '") + field + "'");
        }
    };

    require(p.projection().has_max_staleness_ms(), "projection.max_staleness_ms");

    require(p.isolation().has_inference_wcet_ms(), "isolation.inference_wcet_ms");
    require(p.isolation().has_inference_rate_hz(), "isolation.inference_rate_hz");
    require(p.isolation().has_max_payload_bytes(), "isolation.max_payload_bytes");
    require(p.isolation().has_transport_deadline_ms(), "isolation.transport_deadline_ms");
    require(p.isolation().has_critic_budget_total_us(), "isolation.critic_budget_total_us");

    for (int i = 0; i < p.projection().output_region_size(); ++i) {
        const auto& bound = p.projection().output_region(i);
        require(bound.has_min(), ("projection.output_region[" + std::to_string(i) + "].min").c_str());
        require(bound.has_max(), ("projection.output_region[" + std::to_string(i) + "].max").c_str());
    }

    for (int i = 0; i < p.critics_size(); ++i) {
        const auto& critic = p.critics(i);
        require(critic.has_window_ms(), ("critics[" + std::to_string(i) + "].window_ms").c_str());
        require(critic.has_wcet_us(), ("critics[" + std::to_string(i) + "].wcet_us").c_str());
    }

    for (int i = 0; i < p.transfer().band_a().recoveries_size(); ++i) {
        const auto& recovery = p.transfer().band_a().recoveries(i);
        require(recovery.has_retry_budget(),
                ("transfer.band_a.recoveries[" + std::to_string(i) + "].retry_budget").c_str());
        require(recovery.has_timeout_ms(),
                ("transfer.band_a.recoveries[" + std::to_string(i) + "].timeout_ms").c_str());
    }
}

}  // namespace

std::string canonical_joint_order_hash(
    const google::protobuf::RepeatedPtrField<std::string>& joint_names) {
    // FNV-1a 64-bit over the joint names joined with '\n'. Order-sensitive by
    // construction, which is the point: this detects a hand-edited profile
    // whose joint_names were reordered without recomputing the hash.
    constexpr std::uint64_t kOffsetBasis = 0xcbf29ce484222325ULL;
    constexpr std::uint64_t kPrime = 0x100000001b3ULL;
    std::uint64_t hash = kOffsetBasis;
    bool first = true;
    for (const auto& name : joint_names) {
        if (!first) {
            hash ^= static_cast<unsigned char>('\n');
            hash *= kPrime;
        }
        first = false;
        for (char raw_c : name) {
            hash ^= static_cast<unsigned char>(raw_c);
            hash *= kPrime;
        }
    }
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(hash));
    return std::string(buf);
}

std::vector<ValidationError> validate(const HarnessProfile& profile) {
    std::vector<ValidationError> out;
    check_c1(profile, out);
    check_c2(profile, out);
    check_c3(profile, out);
    check_c4(profile, out);
    check_c5(profile, out);
    check_c6(profile, out);
    check_c7(profile, out);
    check_c8(profile, out);
    check_c9(profile, out);
    check_c10(profile, out);
    check_c11(profile, out);
    return out;
}

}  // namespace harness::profile
