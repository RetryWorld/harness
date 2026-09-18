// Explicit-loop property tests (build brief §9 "Property tests" — no
// proptest-equivalent library, just loops over generated inputs).

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "harness/harness_kernel.h"
#include "harness/v1/enforcement.pb.h"
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

std::uint64_t xorshift(std::uint64_t& state) {
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    return state;
}

double rand_effort(std::uint64_t& state, double range) {
    double u = static_cast<double>(xorshift(state) % 1000000ULL) / 1000000.0;  // [0,1)
    return (u * 2.0 - 1.0) * range;
}

std::string profile_path() {
    return std::string(HARNESS_EXAMPLES_DIR) + "/profiles/bench3.valid.yaml";
}

// Property 1: emitted effort is within output_region for every decision
// except REJECT. Property 5: a full ring buffer increments dropped and
// never returns anything other than HK_OK.
void test_emitted_within_region_and_full_sink() {
    auto loaded = harness::profile::load_from_yaml_file(profile_path());
    expect(loaded.profile.has_value(), "profile should load");
    if (!loaded.profile.has_value()) return;
    std::string bytes = loaded.profile->SerializeAsString();

    hk_handle* handle = nullptr;
    expect(hk_create(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size(), &handle) == HK_OK,
           "hk_create should succeed");
    if (handle == nullptr) return;

    // Deliberately tiny sink: guarantees drops without disabling the guard
    // against ever aborting or returning a non-OK status because of them.
    std::vector<std::uint8_t> tiny_sink(4);

    std::uint64_t rng = 0x9e3779b97f4a7c15ULL;
    for (int i = 0; i < 2000; ++i) {
        double candidate[3] = {rand_effort(rng, 15.0), rand_effort(rng, 15.0), rand_effort(rng, 15.0)};
        double emitted[3] = {0.0, 0.0, 0.0};
        hk_command cmd{static_cast<std::uint64_t>(i) * static_cast<std::uint64_t>(1'000'000), 3, candidate};
        hk_gated_command out{HK_ADMIT, HK_BAND_NOMINAL, 3, emitted};

        std::size_t sink_len = 0;
        std::uint32_t sink_dropped = 0;
        hk_event_sink sink{tiny_sink.data(), tiny_sink.size(), &sink_len, &sink_dropped};

        hk_status st = hk_gate(handle, &cmd, &out, &sink);
        expect(st == HK_OK, "hk_gate must always return HK_OK for well-formed input, even with a full sink");

        if (out.decision != HK_REJECT) {
            for (int j = 0; j < 3; ++j) {
                expect(emitted[j] >= loaded.profile->projection().output_region(j).min() - 1e-9 &&
                           emitted[j] <= loaded.profile->projection().output_region(j).max() + 1e-9,
                       "emitted[" + std::to_string(j) + "]=" + std::to_string(emitted[j]) +
                           " out of declared bounds on cycle " + std::to_string(i));
            }
        }
        expect(sink_dropped > 0 || sink_len <= tiny_sink.size(),
               "a 4-byte sink should be dropping records, not silently growing");
    }
    hk_destroy(handle);
}

// Property 2 & 4: seq strictly increasing with no gaps in a fully-drained
// stream, and no reentry to nominal without a passing ReentryEvent
// immediately preceding the Band A -> Nominal transfer.
void test_seq_monotone_and_reentry_ordering(const std::string& trace_name) {
    auto loaded = harness::profile::load_from_yaml_file(profile_path());
    if (!loaded.profile.has_value()) return;
    auto trace =
        harness::replay::load_trace_file(std::string(HARNESS_EXAMPLES_DIR) + "/traces/" + trace_name + ".jsonl");
    if (!trace.samples.has_value()) return;

    // Re-run manually (not via run_replay) so every event, not just the
    // hash, is available for inspection.
    std::string bytes = loaded.profile->SerializeAsString();
    hk_handle* handle = nullptr;
    if (hk_create(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size(), &handle) != HK_OK) return;

    std::vector<harness::v1::EnforcementEvent> events;
    std::vector<std::uint8_t> sink_buf(1U << 16U);
    for (const auto& sample : *trace.samples) {
        double emitted[64] = {0};
        hk_command cmd{sample.t_ns, static_cast<std::uint32_t>(sample.effort.size()), sample.effort.data()};
        hk_gated_command out{HK_ADMIT, HK_BAND_NOMINAL, cmd.n_joints, emitted};
        std::size_t sink_len = 0;
        std::uint32_t sink_dropped = 0;
        hk_event_sink sink{sink_buf.data(), sink_buf.size(), &sink_len, &sink_dropped};
        hk_gate(handle, &cmd, &out, &sink);

        std::size_t pos = 0;
        while (pos < sink_len) {
            std::uint64_t len = 0;
            int shift = 0;
            while (pos < sink_len) {
                std::uint8_t b = sink_buf[pos++];
                len |= static_cast<std::uint64_t>(b & 0x7F) << shift;
                if ((b & 0x80U) == 0U) break;
                shift += 7;
            }
            harness::v1::EnforcementEvent ev;
            if (ev.ParseFromArray(sink_buf.data() + pos, static_cast<int>(len))) events.push_back(ev);
            pos += len;
        }
    }
    hk_destroy(handle);

    for (std::size_t i = 1; i < events.size(); ++i) {
        expect(events[i].seq() == events[i - 1].seq() + 1,
               trace_name + ": seq must be gap-free in a fully drained stream (" +
                   std::to_string(events[i - 1].seq()) + " -> " + std::to_string(events[i].seq()) + ")");
    }

    for (std::size_t i = 0; i < events.size(); ++i) {
        if (!events[i].has_transfer()) continue;
        const auto& t = events[i].transfer();
        if (t.from_band() == harness::v1::BAND_A && t.to_band() == harness::v1::BAND_NOMINAL) {
            expect(i > 0 && events[i - 1].has_reentry() && events[i - 1].reentry().predicate_result(),
                   trace_name + ": Band A -> Nominal must be immediately preceded by a passing ReentryEvent");
        }
    }
}

}  // namespace

int main() {
    test_emitted_within_region_and_full_sink();
    test_seq_monotone_and_reentry_ordering("a_nominal");
    test_seq_monotone_and_reentry_ordering("b_torque_excursion");
    test_seq_monotone_and_reentry_ordering("c_evidence_recovery");
    test_seq_monotone_and_reentry_ordering("d_critic_overrun");

    if (g_failures > 0) {
        std::fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("all property tests passed\n");
    return 0;
}
