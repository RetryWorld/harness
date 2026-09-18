// Golden trace + determinism tests (build brief §8, §9). Each trace must
// reproduce its committed hash, both from two in-process runs (catches
// iteration-order bugs) and from a fresh subprocess of rearguard (catches
// address-dependent behaviour) — same profile, same trace, byte-identical
// event stream.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include "profile/loader.hpp"
#include "replay/replay.hpp"
#include "replay/trace.hpp"

namespace {

int g_failures = 0;

void expect(bool cond, const std::string& msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg.c_str());
        g_failures++;
    }
}

std::string read_golden_hash(const std::string& trace_name) {
    std::ifstream f(std::string(HARNESS_GOLDEN_DIR) + "/" + trace_name + ".hash");
    std::string hash;
    std::getline(f, hash);
    while (!hash.empty() && (hash.back() == '\n' || hash.back() == '\r')) hash.pop_back();
    return hash;
}

std::string run_fresh_process_hash(const std::string& profile_path, const std::string& trace_path) {
    std::string cmd = std::string(HARNESS_CLI_BIN) + " replay --profile " + profile_path +
                       " --trace " + trace_path + " 2>/dev/null | grep event_stream_hash";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (pipe == nullptr) return "";
    char buf[256];
    std::string out;
    while (std::fgets(buf, sizeof(buf), pipe) != nullptr) out += buf;
    pclose(pipe);
    auto pos = out.find(": ");
    if (pos == std::string::npos) return "";
    std::string hash = out.substr(pos + 2);
    while (!hash.empty() && (hash.back() == '\n' || hash.back() == '\r')) hash.pop_back();
    return hash;
}

void test_trace(const std::string& trace_name) {
    std::string profile_path = std::string(HARNESS_EXAMPLES_DIR) + "/profiles/bench3.valid.yaml";
    std::string trace_path = std::string(HARNESS_EXAMPLES_DIR) + "/traces/" + trace_name + ".jsonl";

    auto profile_result = harness::profile::load_from_yaml_file(profile_path);
    expect(profile_result.profile.has_value(), "profile should load: " + profile_result.error);
    if (!profile_result.profile.has_value()) return;

    auto trace_result = harness::replay::load_trace_file(trace_path);
    expect(trace_result.samples.has_value(), "trace should load: " + trace_result.error);
    if (!trace_result.samples.has_value()) return;

    harness::replay::ReplayOptions options;
    options.critic_lib_dir = HARNESS_CRITIC_LIB_DIR;
    options.scenario_id = trace_name;

    auto run1 = harness::replay::run_replay(*profile_result.profile, *trace_result.samples, options);
    auto run2 = harness::replay::run_replay(*profile_result.profile, *trace_result.samples, options);

    expect(run1.ok, trace_name + " run1 should succeed: " + run1.error);
    expect(run2.ok, trace_name + " run2 should succeed: " + run2.error);
    expect(run1.event_stream_hash == run2.event_stream_hash,
           trace_name + " two in-process runs must produce identical hashes (" +
               run1.event_stream_hash + " vs " + run2.event_stream_hash + ")");

    std::string golden = read_golden_hash(trace_name);
    expect(run1.event_stream_hash == golden, trace_name + " hash " + run1.event_stream_hash +
                                                  " does not match committed golden " + golden);

    std::string fresh = run_fresh_process_hash(profile_path, trace_path);
    expect(fresh == golden, trace_name + " fresh-process hash " + fresh + " does not match golden " + golden);
}

}  // namespace

int main() {
    test_trace("a_nominal");
    test_trace("b_torque_excursion");
    test_trace("c_evidence_recovery");
    test_trace("d_critic_overrun");

    if (g_failures > 0) {
        std::fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("all golden trace tests passed\n");
    return 0;
}
