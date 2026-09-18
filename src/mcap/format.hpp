// Minimal MCAP (https://mcap.dev) record layout — just enough of the spec to
// write and read one unchunked episode file. No chunking, no summary/index
// section: build brief §8 wants crash-safety over performance, and an
// unchunked writer means every record on disk before a SIGKILL is already a
// complete, independently parseable record.

#ifndef HARNESS_MCAP_FORMAT_HPP
#define HARNESS_MCAP_FORMAT_HPP

#include <array>
#include <cstdint>

namespace harness::mcap {

inline constexpr std::array<std::uint8_t, 8> kMagic = {0x89, 'M', 'C', 'A', 'P', '0', '\r', '\n'};

enum class Opcode : std::uint8_t {
    kHeader = 0x01,
    kFooter = 0x02,
    kSchema = 0x03,
    kChannel = 0x04,
    kMessage = 0x05,
    kDataEnd = 0x0F,
};

}  // namespace harness::mcap

#endif  // HARNESS_MCAP_FORMAT_HPP
