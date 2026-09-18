// The event-stream hash (build brief §9 "Determinism").
//
// FNV-1a 64 over every emitted EnforcementEvent's wire bytes, in emission
// order, rendered as lowercase hex. Shared rather than reimplemented: the
// replay rig computes it as it writes an episode, and `rearguard verify`
// recomputes it from the MCAP to prove the file on disk is exactly the event
// stream the kernel produced. If those two ever computed it differently the
// check would be worthless, and the difference would be invisible.

#ifndef HARNESS_REPLAY_EVENT_HASH_HPP
#define HARNESS_REPLAY_EVENT_HASH_HPP

#include <cstddef>
#include <cstdint>
#include <string>

namespace harness::replay {

class EventStreamHasher {
  public:
    void update(const void* data, std::size_t len);

    // Lowercase, zero-padded to 16 chars.
    std::string hex() const;

  private:
    std::uint64_t hash_ = 0xcbf29ce484222325ULL;
};

}  // namespace harness::replay

#endif  // HARNESS_REPLAY_EVENT_HASH_HPP
