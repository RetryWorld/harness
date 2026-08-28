// Tests. No framework — a test dependency would break the offline-build
// property for the same reason a JSON library would.
//
// Two of these are unusual and worth reading: `no_allocation_on_decision_path`
// and `batch_matches_single_bitwise`. They defend properties that C++ does not
// give us and that the previous Rust implementation partly did, so they are the
// compensating control for the language switch rather than ordinary coverage.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <span>
#include <string>
#include <vector>

#include "alloc_counter.hpp"
#include "harness_kernel.h"
#include "kernel.hpp"
#include "profile.hpp"

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool cond, const char* what, int line) {
    ++g_checks;
    if (!cond) {
        ++g_failures;
        std::fprintf(stderr, "FAIL line %d: %s\n", line, what);
    }
}

#define CHECK(cond) check((cond), #cond, __LINE__)

// --- allocation tracker -----------------------------------------------------
//
// Global operator new/delete counters. The decision path must not allocate: at
// 1 kHz an allocator is exactly the unbounded-latency component Isolation
// exists to keep out of the loop, and "we were careful" is not a test.


const char* kProfileJson = R"({
  "profile_version": "test/0.1",
  "model_node": "/vla_policy",
  "output_topic": "/cmd_vel",
  "output_region": {"lo": [-1.0, -0.5], "hi": [1.0, 0.5]},
  "max_staleness_ms": 5.0,
  "inference_budget": {"wcet_ms": 8.0, "rate_hz": 100.0},
  "transport_budget": {"max_payload_bytes": 4096, "deadline_ms": 2.0},
  "operating_regime": {"signal": "/vla_policy/ood_score", "max": 0.7},
  "fallback_node": "/safety_controller",
  "domain": "torque",
  "embodiment": "fr3"
})";

harness::OutputRegion region() {
    return harness::OutputRegion{{-1.0, -0.5}, {1.0, 0.5}};
}

void test_admit_inside() {
    const std::vector<double> cand{0.5, 0.1};
    std::vector<double> emitted(2, 0.0);
    const auto r = harness::project_into(cand, region(), emitted);
    CHECK(r.verdict == harness::Verdict::Admit);
    CHECK(r.margin > 0.0);
    CHECK(r.failed == 0U);
    CHECK(emitted[0] == 0.5);
}

void test_replace_and_clamp() {
    const std::vector<double> cand{2.0, 0.1};
    std::vector<double> emitted(2, 0.0);
    const auto r = harness::project_into(cand, region(), emitted);
    CHECK(r.verdict == harness::Verdict::Replace);
    CHECK(emitted[0] == 1.0);
    CHECK(std::abs(r.margin - -1.0) < 1e-12);
    CHECK(r.binding_index == 0);
}

void test_margin_is_worst_case_not_aggregate() {
    // Both slightly out. Margin must reflect the worse breach, not their sum:
    // an aggregate hides a single violation behind well-behaved neighbours, and
    // margin is what thresholds are fitted to.
    const std::vector<double> cand{1.2, 0.6};
    std::vector<double> emitted(2, 0.0);
    const auto r = harness::project_into(cand, region(), emitted);
    CHECK(std::abs(r.margin - -0.2) < 1e-12);
}

void test_arity_mismatch_is_type_failure() {
    const std::vector<double> cand{0.0};
    std::vector<double> emitted(2, 0.0);
    const auto r = harness::project_into(cand, region(), emitted);
    CHECK(r.verdict == harness::Verdict::Drop);
    CHECK((r.failed & static_cast<std::uint32_t>(harness::Predicate::TypeCompatibility)) != 0U);
}

void test_nonfinite_rejected() {
    for (const double bad : {std::nan(""), std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity()}) {
        const std::vector<double> cand{bad, 0.0};
        std::vector<double> emitted(2, 9.0);
        const auto r = harness::project_into(cand, region(), emitted);
        CHECK(r.verdict == harness::Verdict::Drop);
        CHECK(r.nonfinite_input);
        // Output must be zeroed, not left holding stale values a caller might
        // publish to an actuator.
        CHECK(emitted[0] == 0.0 && emitted[1] == 0.0);
    }
}

void test_no_allocation_on_decision_path() {
    const auto reg = region();
    const std::vector<double> cand{2.0, 0.1};
    std::vector<double> emitted(2, 0.0);
    harness::project_into(cand, reg, emitted);  // warm any lazy init

    harness_test::alloc_count = 0;
    harness_test::tracking = true;
    for (int i = 0; i < 1000; ++i) {
        harness::project_into(cand, reg, emitted);
    }
    harness_test::tracking = false;
    CHECK(harness_test::alloc_count == 0);
    if (harness_test::alloc_count != 0) {
        std::fprintf(stderr, "  %zu allocations in 1000 gate decisions\n", harness_test::alloc_count);
    }
}

void test_profile_parse_and_validate() {
    auto r = harness::parse_profile(kProfileJson);
    CHECK(r.profile.has_value());
    if (r.profile) {
        CHECK(r.profile->output_region.lo.size() == 2);
        CHECK(r.profile->operating_regime.max == 0.7);
        CHECK(r.profile->fallback_node == "/safety_controller");
    }
}

void test_profile_fails_closed() {
    struct Case { const char* json; const char* why; };
    const Case cases[] = {
        {R"({"profile_version":"x"})", "missing fields"},
        {"{ not json", "malformed"},
        {R"({"profile_version":"x","model_node":"/a","output_topic":"/b",
             "output_region":{"lo":[1.0],"hi":[-1.0]},"max_staleness_ms":1,
             "inference_budget":{"wcet_ms":1,"rate_hz":100},
             "transport_budget":{"max_payload_bytes":1,"deadline_ms":1},
             "operating_regime":{"signal":"/s","max":0.5},
             "fallback_node":"/f"})", "lo > hi"},
        // The joint admission test: 20ms WCET cannot fit a 100Hz (10ms) period.
        // Feasible on each axis alone, infeasible together — paper §5.
        {R"({"profile_version":"x","model_node":"/a","output_topic":"/b",
             "output_region":{"lo":[-1.0],"hi":[1.0]},"max_staleness_ms":1,
             "inference_budget":{"wcet_ms":20,"rate_hz":100},
             "transport_budget":{"max_payload_bytes":1,"deadline_ms":1},
             "operating_regime":{"signal":"/s","max":0.5},
             "fallback_node":"/f"})", "wcet exceeds period"},
        // An empty region would admit everything — the silent failure mode.
        {R"({"profile_version":"x","model_node":"/a","output_topic":"/b",
             "output_region":{"lo":[],"hi":[]},"max_staleness_ms":1,
             "inference_budget":{"wcet_ms":1,"rate_hz":100},
             "transport_budget":{"max_payload_bytes":1,"deadline_ms":1},
             "operating_regime":{"signal":"/s","max":0.5},
             "fallback_node":"/f"})", "empty output region"},
        // A torque-domain profile with no named embodiment can't be scoped to
        // one — the concrete form of "torque thresholds are non-inheriting."
        {R"({"profile_version":"x","model_node":"/a","output_topic":"/b",
             "output_region":{"lo":[-1.0],"hi":[1.0]},"max_staleness_ms":1,
             "inference_budget":{"wcet_ms":1,"rate_hz":100},
             "transport_budget":{"max_payload_bytes":1,"deadline_ms":1},
             "operating_regime":{"signal":"/s","max":0.5},
             "fallback_node":"/f","domain":"torque"})", "torque domain without embodiment"},
        {R"({"profile_version":"x","model_node":"/a","output_topic":"/b",
             "output_region":{"lo":[-1.0],"hi":[1.0]},"max_staleness_ms":1,
             "inference_budget":{"wcet_ms":1,"rate_hz":100},
             "transport_budget":{"max_payload_bytes":1,"deadline_ms":1},
             "operating_regime":{"signal":"/s","max":0.5},
             "fallback_node":"/f","domain":"not_a_domain"})", "unknown domain"},
    };
    for (const auto& c : cases) {
        auto r = harness::parse_profile(c.json);
        check(!r.profile.has_value(), c.why, __LINE__);
    }
}

void test_domain_velocity_needs_no_embodiment() {
    // Velocity/Cartesian bounds are the same physical quantity across
    // embodiments, so — unlike torque — they carry no embodiment requirement.
    const char* json = R"({"profile_version":"x","model_node":"/a","output_topic":"/b",
        "output_region":{"lo":[-1.0],"hi":[1.0]},"max_staleness_ms":1,
        "inference_budget":{"wcet_ms":1,"rate_hz":100},
        "transport_budget":{"max_payload_bytes":1,"deadline_ms":1},
        "operating_regime":{"signal":"/s","max":0.5},
        "fallback_node":"/f","domain":"velocity"})";
    auto r = harness::parse_profile(json);
    CHECK(r.profile.has_value());
    if (r.profile) CHECK(r.profile->domain == harness::Domain::kVelocity);
}

void test_json_hostile_input_does_not_crash() {
    // Not exhaustive fuzzing — that belongs in CI. These are the shapes that
    // break hand-written parsers: unterminated everything, deep nesting,
    // truncated escapes, numbers that are not numbers.
    const char* inputs[] = {
        "", "{", "[", "\"", "\"\\", "\"\\u12", "{\"a\":", "{\"a\":}",
        "[1,", "1e", "-", "--1", "0x10", "{\"a\":1,,}", "\x01\x02",
        "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[",",",
    };
    for (const char* in : inputs) {
        auto r = harness::parse_profile(in);
        CHECK(!r.profile.has_value());  // must reject, must not crash
    }
}

void test_c_abi_round_trip() {
    harness_profile_t* p = nullptr;
    CHECK(harness_profile_load(kProfileJson, std::strlen(kProfileJson), &p) == HARNESS_OK);
    CHECK(p != nullptr);
    if (p == nullptr) return;

    std::size_t arity = 0;
    CHECK(harness_profile_arity(p, &arity) == HARNESS_OK);
    CHECK(arity == 2);

    double emitted[2] = {0.0, 0.0};
    harness_projection_result_t res{};

    const double inside[2] = {0.5, 0.1};
    CHECK(harness_project(p, inside, 2, emitted, 2, &res) == HARNESS_OK);
    CHECK(res.verdict == HARNESS_VERDICT_ADMIT);

    const double outside[2] = {2.0, 0.1};
    CHECK(harness_project(p, outside, 2, emitted, 2, &res) == HARNESS_OK);
    CHECK(res.verdict == HARNESS_VERDICT_REPLACE);
    CHECK(emitted[0] == 1.0);
    CHECK((res.failed & HARNESS_PRED_VALUE_RANGE) != 0U);

    // Error paths must be codes, never exceptions or crashes.
    CHECK(harness_project(p, inside, 1, emitted, 2, &res) == HARNESS_ERR_ARITY);
    CHECK(harness_project(p, inside, 2, emitted, 1, &res) == HARNESS_ERR_BUFFER_TOO_SMALL);
    CHECK(harness_project(nullptr, inside, 2, emitted, 2, &res) == HARNESS_ERR_NULL_ARG);
    CHECK(harness_project(p, nullptr, 2, emitted, 2, &res) == HARNESS_ERR_NULL_ARG);

    harness_profile_free(p);
    harness_profile_free(nullptr);  // must be a no-op
}

void test_batch_matches_single_bitwise() {
    // The simulation rig's conformance check compares the batch path against a
    // JAX shadow gate. If batch and single could diverge here, that comparison
    // would be checking the wrong thing. Bitwise, not approximate: -ffp-contract
    // and -fno-fast-math in CMakeLists exist to make this hold.
    harness_profile_t* p = nullptr;
    CHECK(harness_profile_load(kProfileJson, std::strlen(kProfileJson), &p) == HARNESS_OK);
    if (p == nullptr) return;

    const std::size_t n = 5;
    const double cands[n * 2] = {0.5, 0.1, 2.0, 0.1, -3.0, 0.9, 0.0, 0.0, 1.0, 0.5};
    double batch_emitted[n * 2] = {};
    harness_projection_result_t batch_res[n]{};
    CHECK(harness_project_batch(p, cands, n, 2, batch_emitted, batch_res) == HARNESS_OK);

    for (std::size_t i = 0; i < n; ++i) {
        double one_emitted[2] = {};
        harness_projection_result_t one{};
        CHECK(harness_project(p, cands + (i * 2), 2, one_emitted, 2, &one) == HARNESS_OK);
        CHECK(batch_res[i].verdict == one.verdict);
        CHECK(batch_res[i].binding_index == one.binding_index);
        CHECK(std::memcmp(&batch_res[i].margin, &one.margin, sizeof(double)) == 0);
        CHECK(std::memcmp(batch_emitted + (i * 2), one_emitted, sizeof(one_emitted)) == 0);
    }
    harness_profile_free(p);
}

void test_abi_version_matches_header() {
    CHECK(harness_abi_version() == HARNESS_ABI_VERSION);
    CHECK(std::strlen(harness_kernel_version()) > 0);
    CHECK(std::strcmp(harness_schema_revision(), "harness.v1") == 0);
    CHECK(std::strcmp(harness_status_str(HARNESS_OK), "ok") == 0);
}

}  // namespace


int main() {
    test_admit_inside();
    test_replace_and_clamp();
    test_margin_is_worst_case_not_aggregate();
    test_arity_mismatch_is_type_failure();
    test_nonfinite_rejected();
    test_no_allocation_on_decision_path();
    test_profile_parse_and_validate();
    test_profile_fails_closed();
    test_domain_velocity_needs_no_embodiment();
    test_json_hostile_input_does_not_crash();
    test_c_abi_round_trip();
    test_batch_matches_single_bitwise();
    test_abi_version_matches_header();

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
