// Harness kernel — the enforcement path.
//
// # The one rule in this file
//
// The projection predicate here is the ONLY implementation. The simulation
// rig's JAX gate is a declared batching optimisation, not a second
// implementation, and it is held to that by a conformance check that replays
// recorded action traces through this code and requires identical verdicts.
// If the two disagree, the sweep's thresholds are non-authoritative — a
// threshold derived by code that is not the code that enforces it cannot be
// defended.
//
// # Constraints on the decision path (project_into)
//
//   * No allocation. The caller owns the output buffer. At 1 kHz an allocator
//     is exactly the unbounded-latency component Isolation exists to exclude.
//   * No exceptions. This runs behind a C ABI and inside a customer's process;
//     an exception escaping the boundary is undefined behaviour.
//   * No order-dependent or fused floating-point. The result must be
//     bit-identical across platforms for identical inputs, which is what makes
//     the conformance check exact rather than approximate. See the -ffp-contract
//     and -fno-fast-math flags in CMakeLists.txt — they are load-bearing.

#ifndef HARNESS_KERNEL_HPP
#define HARNESS_KERNEL_HPP

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace harness {

enum class Predicate : std::uint32_t {
    TypeCompatibility = 1U << 0U,
    TimestampFreshness = 1U << 1U,
    ValueRange = 1U << 2U,
    TrajectoryStability = 1U << 3U,
    Schedulability = 1U << 4U,
    DeliveryFeasibility = 1U << 5U,
    LifecycleState = 1U << 6U,
};

enum class Verdict : std::int32_t { Admit = 1, Drop = 2, Replace = 3 };

// Fixed-size and trivially copyable: no Vec, no string, nothing that allocates.
struct ProjectionRaw {
    Verdict verdict = Verdict::Drop;
    std::uint32_t failed = 0;
    // Signed distance to the binding predicate; negative means violated.
    // Defined as the WORST per-actuator violation rather than an aggregate: an
    // aggregate hides a single breach behind well-behaved neighbours, and this
    // is the number thresholds are fitted to.
    double margin = 0.0;
    std::int32_t binding_index = -1;
    bool nonfinite_input = false;
};

// The declared output region from the Harness Profile.
struct OutputRegion {
    std::vector<double> lo;
    std::vector<double> hi;
};

// Gate one candidate action. `emitted` is caller-owned and must be at least as
// long as `candidate`.
//
// noexcept is part of the contract, not decoration: this is called through
// extern "C".
ProjectionRaw project_into(std::span<const double> candidate,
                           const OutputRegion& region,
                           std::span<double> emitted) noexcept;

// Allocating convenience wrapper for tests and offline tooling.
// NEVER call this on the control path.
struct ProjectionOutcome {
    ProjectionRaw raw;
    std::vector<double> emitted;
};
ProjectionOutcome project(std::span<const double> candidate, const OutputRegion& region);

extern const char* const kKernelVersion;
extern const char* const kSchemaRevision;
extern const char* const kBuildCommit;

}  // namespace harness

#endif  // HARNESS_KERNEL_HPP
