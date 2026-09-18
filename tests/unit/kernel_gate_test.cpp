// Direct hk_create/hk_gate unit tests against the C ABI (build brief §6).

#include <cstdio>
#include <string>
#include <vector>

#include "harness/harness_kernel.h"
#include "harness/v1/enforcement.pb.h"
#include "profile/loader.hpp"

namespace {

int g_failures = 0;

void expect(bool cond, const std::string& msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg.c_str());
        g_failures++;
    }
}

std::string profile_path() {
    return std::string(HARNESS_EXAMPLES_DIR) + "/profiles/bench3.valid.yaml";
}

hk_handle* make_handle() {
    auto loaded = harness::profile::load_from_yaml_file(profile_path());
    if (!loaded.profile.has_value()) return nullptr;
    std::string bytes = loaded.profile->SerializeAsString();
    hk_handle* handle = nullptr;
    hk_status st = hk_create(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size(), &handle);
    if (st != HK_OK) return nullptr;
    return handle;
}

void test_abi_version() { expect(hk_abi_version() == HK_ABI_VERSION, "hk_abi_version mismatch"); }

void test_create_and_destroy() {
    hk_handle* h = make_handle();
    expect(h != nullptr, "hk_create should succeed on a valid profile");
    hk_destroy(h);
}

void test_create_rejects_invalid_profile() {
    // Empty bytes do not even parse as a HarnessProfile with the required
    // shape (dof=0, embodiment unset) -- ProfileRuntime::build's validator
    // pass should reject it (fails C7 at minimum: 0 == 0 == 0 actually
    // passes trivially, but C1 profile_eligible defaults false so C1 is
    // skipped; use garbage bytes instead to hit ParseFromArray failure).
    std::vector<std::uint8_t> garbage = {0xFF, 0xFF, 0xFF, 0x00, 0x01};
    hk_handle* handle = nullptr;
    hk_status st = hk_create(garbage.data(), garbage.size(), &handle);
    expect(st != HK_OK, "hk_create should reject malformed profile bytes");
    expect(handle == nullptr, "hk_create must not set *out on failure");
}

void test_admit_within_bounds() {
    hk_handle* h = make_handle();
    expect(h != nullptr, "handle required");
    if (h == nullptr) return;

    double candidate[3] = {1.0, -1.0, 0.5};
    double emitted[3] = {0.0, 0.0, 0.0};
    hk_command cmd{0, 3, candidate};
    hk_gated_command out{HK_ADMIT, HK_BAND_NOMINAL, 3, emitted};

    hk_status st = hk_gate(h, &cmd, &out, nullptr);
    expect(st == HK_OK, "hk_gate should succeed");
    expect(out.decision == HK_ADMIT, "in-bounds command should be admitted");
    expect(emitted[0] == 1.0 && emitted[1] == -1.0 && emitted[2] == 0.5,
           "admitted command should pass through unchanged");
    hk_destroy(h);
}

void test_clamp_out_of_bounds() {
    hk_handle* h = make_handle();
    expect(h != nullptr, "handle required");
    if (h == nullptr) return;

    double candidate[3] = {15.0, 0.0, 0.0};
    double emitted[3] = {0.0, 0.0, 0.0};
    hk_command cmd{0, 3, candidate};
    hk_gated_command out{HK_ADMIT, HK_BAND_NOMINAL, 3, emitted};

    hk_status st = hk_gate(h, &cmd, &out, nullptr);
    expect(st == HK_OK, "hk_gate should succeed");
    expect(out.decision == HK_CLAMP, "out-of-bounds command should be clamped");
    expect(emitted[0] == 10.0, "emitted value should clamp to the declared bound");
    hk_destroy(h);
}

void test_joint_mismatch() {
    hk_handle* h = make_handle();
    expect(h != nullptr, "handle required");
    if (h == nullptr) return;

    double candidate[2] = {1.0, 1.0};
    double emitted[2] = {0.0, 0.0};
    hk_command cmd{0, 2, candidate};  // profile declares dof=3
    hk_gated_command out{HK_ADMIT, HK_BAND_NOMINAL, 2, emitted};

    hk_status st = hk_gate(h, &cmd, &out, nullptr);
    expect(st == HK_ERR_JOINT_MISMATCH, "mismatched joint count should be rejected");
    hk_destroy(h);
}

// Reads the first length-prefixed EnforcementEvent out of a sink buffer.
bool decode_first_event(const std::uint8_t* buf, std::size_t len,
                        harness::v1::EnforcementEvent& out) {
    std::uint64_t record_len = 0;
    int shift = 0;
    std::size_t pos = 0;
    while (pos < len) {
        std::uint8_t b = buf[pos++];
        record_len |= static_cast<std::uint64_t>(b & 0x7F) << shift;
        if ((b & 0x80U) == 0U) break;
        shift += 7;
    }
    if (pos + record_len > len) return false;
    return out.ParseFromArray(buf + pos, static_cast<int>(record_len));
}

// Regression: dropped_since_last used to be derived by differencing the
// CALLER-owned hk_event_sink::dropped against a snapshot held in the handle.
// The ABI lets the caller reset that counter on any cadence it likes (the
// replay rig builds a fresh one every cycle), so the first event after any
// drop computed `0 - N` on a uint32 and reported ~4.29e9 losses.
void test_dropped_since_last_survives_a_caller_reset_counter() {
    hk_handle* h = make_handle();
    expect(h != nullptr, "handle required");
    if (h == nullptr) return;

    double candidate[3] = {1.0, -1.0, 0.5};
    double emitted[3] = {0.0, 0.0, 0.0};
    hk_command cmd{0, 3, candidate};
    hk_gated_command out{HK_ADMIT, HK_BAND_NOMINAL, 3, emitted};

    // Cycle 1: a sink far too small to hold even one event, so every event
    // this cycle is dropped.
    std::uint8_t tiny[4] = {};
    std::size_t tiny_len = 0;
    std::uint32_t tiny_dropped = 0;
    hk_event_sink tiny_sink{tiny, sizeof(tiny), &tiny_len, &tiny_dropped};
    expect(hk_gate(h, &cmd, &out, &tiny_sink) == HK_OK, "gate should succeed despite a full sink");
    expect(tiny_len == 0, "nothing should be written to a sink that cannot hold an event");
    expect(tiny_dropped > 0, "a sink too small for an event must count drops");

    // Cycle 2: a roomy sink with a freshly zeroed drop counter -- exactly
    // what the replay rig hands the kernel every cycle.
    std::vector<std::uint8_t> buf(4096, 0);
    std::size_t len = 0;
    std::uint32_t dropped = 0;
    hk_event_sink sink{buf.data(), buf.size(), &len, &dropped};
    expect(hk_gate(h, &cmd, &out, &sink) == HK_OK, "gate should succeed with a roomy sink");
    expect(len > 0, "a roomy sink should receive events");

    harness::v1::EnforcementEvent event;
    expect(decode_first_event(buf.data(), len, event), "first record should decode");
    expect(event.dropped_since_last() == tiny_dropped,
           "the first event after a drop must report exactly the drops that occurred (got " +
               std::to_string(event.dropped_since_last()) + ", expected " +
               std::to_string(tiny_dropped) + ")");
    expect(event.dropped_since_last() < 1000,
           "dropped_since_last must never underflow (got " +
               std::to_string(event.dropped_since_last()) + ")");
}

// A drop must be attributed to the next event that survives, not absorbed.
// The old code updated its snapshot immediately after the failed write, so
// the drop was silently swallowed and reported by nobody.
void test_dropped_since_last_is_cleared_once_reported() {
    hk_handle* h = make_handle();
    expect(h != nullptr, "handle required");
    if (h == nullptr) return;

    double candidate[3] = {1.0, -1.0, 0.5};
    double emitted[3] = {0.0, 0.0, 0.0};
    hk_command cmd{0, 3, candidate};
    hk_gated_command out{HK_ADMIT, HK_BAND_NOMINAL, 3, emitted};

    std::uint8_t tiny[4] = {};
    std::size_t tiny_len = 0;
    std::uint32_t tiny_dropped = 0;
    hk_event_sink tiny_sink{tiny, sizeof(tiny), &tiny_len, &tiny_dropped};
    hk_gate(h, &cmd, &out, &tiny_sink);

    std::vector<std::uint8_t> buf(4096, 0);
    std::size_t len = 0;
    std::uint32_t dropped = 0;
    hk_event_sink sink{buf.data(), buf.size(), &len, &dropped};
    hk_gate(h, &cmd, &out, &sink);

    harness::v1::EnforcementEvent first;
    expect(decode_first_event(buf.data(), len, first), "first record should decode");
    expect(first.dropped_since_last() > 0, "the surviving event must report the earlier drops");

    // A third cycle with room and nothing dropped since: back to zero.
    len = 0;
    dropped = 0;
    hk_event_sink clean{buf.data(), buf.size(), &len, &dropped};
    hk_gate(h, &cmd, &out, &clean);

    harness::v1::EnforcementEvent later;
    expect(decode_first_event(buf.data(), len, later), "later record should decode");
    expect(later.dropped_since_last() == 0,
           "drops must be reported once, not repeated on every later event (got " +
               std::to_string(later.dropped_since_last()) + ")");
}

void test_null_args() {
    hk_handle* h = make_handle();
    expect(h != nullptr, "handle required");
    if (h == nullptr) return;

    hk_status st = hk_gate(h, nullptr, nullptr, nullptr);
    expect(st == HK_ERR_NULL_ARG, "null command/output should be rejected");
    hk_destroy(h);

    hk_handle* created = nullptr;
    st = hk_create(nullptr, 0, &created);
    expect(st == HK_ERR_NULL_ARG, "hk_create should reject a null profile pointer");
}

}  // namespace

int main() {
    test_abi_version();
    test_create_and_destroy();
    test_create_rejects_invalid_profile();
    test_admit_within_bounds();
    test_clamp_out_of_bounds();
    test_joint_mismatch();
    test_null_args();
    test_dropped_since_last_survives_a_caller_reset_counter();
    test_dropped_since_last_is_cleared_once_reported();

    if (g_failures > 0) {
        std::fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("all kernel gate tests passed\n");
    return 0;
}
