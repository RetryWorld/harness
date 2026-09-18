// Unchunked MCAP writer for one episode file. Two channels: the episode
// header (written once, seq==0) and the enforcement event stream. Off the
// control path entirely — the replay rig drains hk_gate's event sink and
// hands the already-serialised bytes here; nothing is re-encoded.

#ifndef HARNESS_MCAP_WRITER_HPP
#define HARNESS_MCAP_WRITER_HPP

#include <cstdint>
#include <fstream>
#include <string>

#include <google/protobuf/descriptor.h>

namespace harness::mcap {

class EpisodeWriter {
public:
    // Returns nullopt-like failure via the bool return of open(); constructed
    // in two steps so a failed open doesn't leave a half-constructed object
    // whose destructor writes an unclosed file.
    bool open(const std::string& path, const google::protobuf::Descriptor* header_descriptor,
              const google::protobuf::Descriptor* event_descriptor);

    // Writes one message on the "harness/episode_header" channel. Call
    // exactly once, before any WriteEvent call.
    void write_header(const void* data, std::size_t len, std::uint64_t log_time_ns);

    // Writes one message on the "harness/enforcement_event" channel. `data`
    // is an already wire-encoded harness.v1.EnforcementEvent.
    void write_event(const void* data, std::size_t len, std::uint64_t log_time_ns);

    // Footer + closing magic. Idempotent; also runs from the destructor so a
    // normal (non-crashed) writer always leaves a fully closed file.
    void close();

    ~EpisodeWriter();

    EpisodeWriter() = default;
    EpisodeWriter(const EpisodeWriter&) = delete;
    EpisodeWriter& operator=(const EpisodeWriter&) = delete;

private:
    std::ofstream out_;
    std::uint32_t header_sequence_ = 0;
    std::uint32_t event_sequence_ = 0;
    bool closed_ = true;

    void write_schema(std::uint16_t id, const std::string& name,
                       const google::protobuf::Descriptor* descriptor);
    void write_channel(std::uint16_t id, std::uint16_t schema_id, const std::string& topic);
    void write_message(std::uint16_t channel_id, std::uint32_t sequence, std::uint64_t log_time_ns,
                        const void* data, std::size_t len);
};

}  // namespace harness::mcap

#endif  // HARNESS_MCAP_WRITER_HPP
