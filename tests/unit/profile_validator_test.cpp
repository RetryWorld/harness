// C1-C11 fixture tests (build brief §4.4, §9 "Negative schema"). Each fixture
// in schemas/examples/profiles/invalid/ must fail validation with its named
// constraint's error code. A constraint without a failing fixture here is not
// implemented, per the brief.

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "profile/loader.hpp"
#include "profile/validator.hpp"

namespace {

int g_failures = 0;

void expect(bool cond, const std::string& msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg.c_str());
        g_failures++;
    }
}

std::string fixture_path(const std::string& rel) {
    return std::string(HARNESS_EXAMPLES_DIR) + "/" + rel;
}

bool has_code(const std::vector<harness::profile::ValidationError>& errors, const std::string& code) {
    for (const auto& e : errors) {
        if (e.code == code) return true;
    }
    return false;
}

void test_valid_profile_loads_clean() {
    auto result = harness::profile::load_from_yaml_file(fixture_path("profiles/bench3.valid.yaml"));
    expect(result.profile.has_value(), "bench3.valid.yaml should parse: " + result.error);
    if (!result.profile.has_value()) return;
    auto errors = harness::profile::validate(*result.profile);
    std::string joined;
    for (const auto& e : errors) joined += e.code + ": " + e.message + "\n";
    expect(errors.empty(), "bench3.valid.yaml should validate clean, got:\n" + joined);
}

void test_invalid_fixture(const std::string& file, const std::string& expected_code) {
    auto result = harness::profile::load_from_yaml_file(fixture_path("profiles/invalid/" + file));
    expect(result.profile.has_value(), file + " should parse (structural constraints, not JSON, "
                                               "are under test): " + result.error);
    if (!result.profile.has_value()) return;
    auto errors = harness::profile::validate(*result.profile);
    expect(has_code(errors, expected_code), file + " should fail with " + expected_code);
}

}  // namespace

int main() {
    test_valid_profile_loads_clean();

    test_invalid_fixture("c1_ineligible_engine.yaml", "C1");
    test_invalid_fixture("c2_sim_timing_authoritative.yaml", "C2");
    test_invalid_fixture("c3_recovery_evidence_undeclared.yaml", "C3");
    test_invalid_fixture("c4_critic_budget_exceeded.yaml", "C4");
    test_invalid_fixture("c5_band_b_incomplete.yaml", "C5");
    test_invalid_fixture("c6_missing_reentry_predicate.yaml", "C6");
    test_invalid_fixture("c7_joint_arity_mismatch.yaml", "C7");
    test_invalid_fixture("c8_joint_order_hash_mismatch.yaml", "C8");
    test_invalid_fixture("c9_parent_embodiment_mismatch.yaml", "C9");
    test_invalid_fixture("c10_deployed_confidence_not_hardware.yaml", "C10");
    test_invalid_fixture("c11_missing_presence.yaml", "C11");

    if (g_failures > 0) {
        std::fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("all profile validator tests passed\n");
    return 0;
}
