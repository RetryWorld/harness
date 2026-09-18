#include "replay/event_hash.hpp"

#include <cstdio>

namespace harness::replay {

void EventStreamHasher::update(const void* data, std::size_t len) {
    constexpr std::uint64_t kPrime = 0x100000001b3ULL;
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < len; ++i) {
        hash_ ^= bytes[i];
        hash_ *= kPrime;
    }
}

std::string EventStreamHasher::hex() const {
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(hash_));
    return std::string(buf);
}

}  // namespace harness::replay
