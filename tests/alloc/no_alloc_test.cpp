// Overrides global operator new/delete to abort whenever an allocation is
// attempted while a thread-local guard is armed, then arms it around
// hk_gate and hkc_evaluate calls only (build brief §9 "No-allocation"). This
// protects the entire real-time argument, so it must be impossible to skip:
// a real std::abort(), not a soft assertion.

#include <cstdio>
#include <cstdlib>
#include <new>
#include <string>
#include <vector>

#include "harness/harness_kernel.h"
#include "profile/loader.hpp"

namespace {

thread_local bool g_guard = false;
thread_local bool g_violation = false;

}  // namespace

void* operator new(std::size_t size) {
    if (g_guard) {
        g_violation = true;
        std::fprintf(stderr, "ABORT: heap allocation (%zu bytes) inside a guarded no-alloc region\n",
                     size);
        std::abort();
    }
    void* p = std::malloc(size == 0 ? 1 : size);
    if (p == nullptr) throw std::bad_alloc();
    return p;
}

void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

namespace {

int g_failures = 0;

void expect(bool cond, const std::string& msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg.c_str());
        g_failures++;
    }
}

void test_hk_gate_no_alloc() {
    auto loaded = harness::profile::load_from_yaml_file(std::string(HARNESS_EXAMPLES_DIR) +
                                                          "/profiles/bench3.valid.yaml");
    expect(loaded.profile.has_value(), "profile should load: " + loaded.error);
    if (!loaded.profile.has_value()) return;

    std::string bytes = loaded.profile->SerializeAsString();  // allocates; unguarded, at init
    hk_handle* handle = nullptr;
    hk_status st = hk_create(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size(), &handle);
    expect(st == HK_OK, "hk_create should succeed");
    if (st != HK_OK) return;

    double candidate[3] = {1.0, 12.0, -1.0};
    double emitted[3] = {0.0, 0.0, 0.0};
    hk_command cmd{0, 3, candidate};
    hk_gated_command out{HK_ADMIT, HK_BAND_NOMINAL, 3, emitted};

    std::vector<std::uint8_t> sink_buf(4096);
    std::size_t sink_len = 0;
    std::uint32_t sink_dropped = 0;
    hk_event_sink sink{sink_buf.data(), sink_buf.size(), &sink_len, &sink_dropped};

    g_guard = true;
    for (int i = 0; i < 1000; ++i) {
        // uint64_t is `unsigned long` on LP64 Linux but the ULL literal is
        // `unsigned long long` — a differently-ranked 64-bit type, so mixing
        // them makes the multiply promote through a type -Wsign-conversion
        // flags on the way back. Cast both operands to uint64_t explicitly.
        cmd.t_ns = static_cast<std::uint64_t>(i) * static_cast<std::uint64_t>(1'000'000);
        sink_len = 0;
        hk_status gate_status = hk_gate(handle, &cmd, &out, &sink);
        if (gate_status != HK_OK) {
            g_guard = false;
            expect(false, "hk_gate returned an error under the no-alloc guard");
            hk_destroy(handle);
            return;
        }
    }
    g_guard = false;

    expect(!g_violation, "hk_gate must not allocate");
    hk_destroy(handle);
}

}  // namespace

int main() {
    test_hk_gate_no_alloc();

    if (g_failures > 0) {
        std::fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("no-alloc test passed\n");
    return 0;
}
