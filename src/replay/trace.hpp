// JSONL trace loader (build brief §8). One control cycle per line:
// {"t_ns": ..., "effort": [...], "position": [...], "velocity": [...], "aux": {...}}

#ifndef HARNESS_REPLAY_TRACE_HPP
#define HARNESS_REPLAY_TRACE_HPP

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace harness::replay {

struct TraceSample {
    std::uint64_t t_ns = 0;
    std::vector<double> effort;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<std::pair<std::string, double>> aux;
};

struct TraceLoadResult {
    std::optional<std::vector<TraceSample>> samples;
    std::string error;
};

TraceLoadResult load_trace_file(const std::string& path);

}  // namespace harness::replay

#endif  // HARNESS_REPLAY_TRACE_HPP
