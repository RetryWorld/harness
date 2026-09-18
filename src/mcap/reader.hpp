// Minimal sequential MCAP reader. Reads Message records only (Schema/Channel
// records are skipped — this reader knows the two channel ids the writer
// uses by convention, build brief §8 "minimal reader"). Stops cleanly at
// EOF, a Footer record, or a truncated trailing record — the last case is
// what makes the SIGKILL crash test meaningful: a partially written record
// at the tail of the file is simply not returned, never a parse error.

#ifndef HARNESS_MCAP_READER_HPP
#define HARNESS_MCAP_READER_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace harness::mcap {

struct Message {
    std::uint16_t channel_id = 0;
    std::uint32_t sequence = 0;
    std::uint64_t log_time_ns = 0;
    std::uint64_t publish_time_ns = 0;
    std::string data;
};

struct ReadResult {
    bool valid_magic = false;
    bool saw_footer = false;
    std::vector<Message> messages;
};

ReadResult read_episode(const std::string& path);

}  // namespace harness::mcap

#endif  // HARNESS_MCAP_READER_HPP
