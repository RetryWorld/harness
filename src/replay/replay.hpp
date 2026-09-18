// Drives a trace through the kernel and critics, writes an MCAP episode, and
// reports a byte-stream hash of the emitted events for the golden-trace
// tests (build brief §9 "Determinism").

#ifndef HARNESS_REPLAY_REPLAY_HPP
#define HARNESS_REPLAY_REPLAY_HPP

#include <cstdint>
#include <optional>
#include <string>

#include "harness/v1/profile.pb.h"
#include "replay/trace.hpp"

namespace harness::replay {

struct ReplayOptions {
    std::string critic_lib_dir;   // directory containing lib<critic_id>.{dylib,so}
    std::string scenario_id;
    std::optional<std::string> mcap_out_path;  // nullopt: no MCAP written
    bool print_summary = false;   // build brief §10 demo table, to stdout
};

struct ReplayResult {
    bool ok = false;
    std::string error;
    std::uint64_t cycles = 0;
    std::uint64_t event_count = 0;
    std::uint32_t dropped_total = 0;
    std::string event_stream_hash;  // hex FNV-1a 64 over every emitted event's wire bytes, in order
    double gate_p50_us = 0.0;
    double gate_p99_us = 0.0;
    double gate_p999_us = 0.0;
};

ReplayResult run_replay(const harness::v1::HarnessProfile& profile,
                         const std::vector<TraceSample>& trace, const ReplayOptions& options);

}  // namespace harness::replay

#endif  // HARNESS_REPLAY_REPLAY_HPP
