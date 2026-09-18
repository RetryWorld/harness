#ifndef HARNESS_KERNEL_GATE_HPP
#define HARNESS_KERNEL_GATE_HPP

#include "harness/harness_kernel.h"
#include "kernel/kernel_state.hpp"

namespace harness::kernel {

// The full hk_gate cycle: Projection, Isolation event flush, Transfer state
// machine. Alloc-free, no exceptions, bounded work. See kernel_gate.cpp for
// the design notes on the judgment calls this makes where build brief §4.3/
// §6 leave the exact mechanics underspecified.
hk_status run_gate_cycle(hk_handle& h, const hk_command& cmd, hk_gated_command& out,
                          hk_event_sink* sink) noexcept;

hk_status submit_proposal(hk_handle& h, const hkc_proposal& proposal) noexcept;

hk_status report_budget(hk_handle& h, std::uint32_t source_id, std::uint64_t elapsed_ns) noexcept;

}  // namespace harness::kernel

#endif  // HARNESS_KERNEL_GATE_HPP
