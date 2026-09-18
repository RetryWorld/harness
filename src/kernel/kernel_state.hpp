// hk_handle's concrete definition — opaque to every consumer of
// harness_kernel.h, private to the kernel implementation.

#ifndef HARNESS_KERNEL_STATE_HPP
#define HARNESS_KERNEL_STATE_HPP

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "harness/harness_critic.h"
#include "harness/harness_kernel.h"
#include "kernel/profile_runtime.hpp"

// Compile-time joint-count ceiling for the fixed-size scratch arrays hk_gate
// uses to stay allocation-free. 64 covers every embodiment class this schema
// targets (a G1 is 29 DOF); hk_create rejects a profile whose dof exceeds it.
inline constexpr std::uint32_t kMaxJoints = 64;

// Consecutive ADMIT cycles required for a Band A recovery's reentry
// predicate to read as satisfied. build brief §4.3 leaves "the predicate
// holds" abstract (SCHEDULABILITY, LIFECYCLE_STATE, ...); this reference
// kernel has no per-predicate telemetry channel, so it uses a stability
// proxy common to all of them — sustained clean Projection decisions — and
// documents the substitution here rather than pretending to evaluate the
// declared predicate class.
inline constexpr std::uint32_t kReentryStableCycles = 3;

// Minimum samples in the reject-rate window before Band B's rate trigger is
// allowed to fire, so one bad cycle can't look like a 100% reject rate.
inline constexpr std::size_t kRejectRateMinSamples = 4;

struct hk_handle {
    harness::kernel::ProfileRuntime profile;
    std::string last_error;

    std::uint64_t seq = 0;
    std::uint64_t step_index = 0;

    // Drops since the last event that was actually WRITTEN, carried until an
    // event survives to report them (EnforcementEvent.dropped_since_last).
    //
    // Kernel-owned on purpose. `hk_event_sink::dropped` belongs to the caller,
    // which resets it on whatever cadence it likes (the replay rig builds a
    // fresh counter every cycle), so the kernel cannot derive its own state by
    // differencing it -- doing so underflowed this field to ~4.29e9 on the
    // first event after any drop.
    std::uint32_t dropped_pending = 0;

    // Projection hold-last state.
    std::vector<double> last_admitted;  // size dof
    bool have_last_admitted = false;

    // Transfer state machine.
    hk_band band = HK_BAND_NOMINAL;
    int active_recovery_index = -1;
    std::uint32_t active_retry_index = 0;
    std::uint64_t band_a_entry_t_ns = 0;
    std::uint32_t consecutive_admits = 0;
    bool band_b_latched = false;

    // Isolation: per-source (0 = inference, 1+i = critics[i]) pending report,
    // consumed and cleared by the next hk_gate call.
    struct PendingBudgetReport {
        bool pending = false;
        std::uint64_t elapsed_ns = 0;
    };
    static constexpr std::size_t kMaxBudgetSources = 65;  // inference + up to 64 critics
    std::array<PendingBudgetReport, kMaxBudgetSources> pending_reports{};
    std::uint32_t overrun_count = 0;

    // Critic proposal staged by hk_submit_proposal, consumed by the next
    // hk_gate call. `evidence_class` in hkc_proposal is an index into
    // profile.critics (build brief's ABI doc calls it "an index into the
    // profile's declared evidence classes"; here that list is exactly the
    // declared critics, in profile.proto order).
    bool has_pending_proposal = false;
    hkc_proposal pending_proposal{};

    // Band B's reject-rate trigger needs a bounded lookback window. A fixed
    // compile-time cap keeps this O(1) regardless of trace length or the
    // profile's window_ms, at the cost of not exactly reproducing an
    // arbitrarily long window — acceptable for the windows this schema's
    // profiles actually declare (hundreds of ms at kHz control rates).
    struct RejectSample {
        std::uint64_t t_ns = 0;
        bool rejected = false;
    };
    static constexpr std::size_t kRejectWindowCap = 256;
    std::array<RejectSample, kRejectWindowCap> reject_window{};
    std::size_t reject_window_write = 0;
    std::size_t reject_window_count = 0;
};

#endif  // HARNESS_KERNEL_STATE_HPP
