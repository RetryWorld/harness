#include "mcap/reader.hpp"

#include <array>
#include <fstream>

#include "mcap/format.hpp"

namespace harness::mcap {

namespace {

bool read_exact(std::ifstream& in, char* dst, std::size_t n) {
    in.read(dst, static_cast<std::streamsize>(n));
    return in.good() && static_cast<std::size_t>(in.gcount()) == n;
}

std::uint16_t read_u16(const std::string& s, std::size_t& pos) {
    std::uint16_t v = 0;
    // uint16_t promotes to int for the shift (unlike uint32_t/uint64_t below,
    // which don't), so the `|=` narrows an int back to uint16_t. The value is
    // always in range (a byte shifted 0 or 8 bits fits in 16 bits), but
    // -Werror=conversion doesn't reason about ranges — cast explicitly.
    for (int i = 0; i < 2; ++i) {
        v = static_cast<std::uint16_t>(
            v | (static_cast<std::uint16_t>(static_cast<unsigned char>(s[pos++])) << (8 * i)));
    }
    return v;
}

std::uint32_t read_u32(const std::string& s, std::size_t& pos) {
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(static_cast<unsigned char>(s[pos++])) << (8 * i);
    return v;
}

std::uint64_t read_u64(const std::string& s, std::size_t& pos) {
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= static_cast<std::uint64_t>(static_cast<unsigned char>(s[pos++])) << (8 * i);
    return v;
}

}  // namespace

ReadResult read_episode(const std::string& path) {
    ReadResult result;
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return result;

    std::array<char, kMagic.size()> magic{};
    if (!read_exact(in, magic.data(), magic.size())) return result;
    for (std::size_t i = 0; i < kMagic.size(); ++i) {
        if (static_cast<std::uint8_t>(magic[static_cast<std::size_t>(i)]) != kMagic[i]) return result;
    }
    result.valid_magic = true;

    while (true) {
        char opcode_byte = 0;
        if (!read_exact(in, &opcode_byte, 1)) break;  // EOF, or trailing partial record: stop cleanly

        std::array<char, 8> len_bytes{};
        if (!read_exact(in, len_bytes.data(), len_bytes.size())) break;
        std::uint64_t len = 0;
        for (int i = 0; i < 8; ++i) {
            len |= static_cast<std::uint64_t>(static_cast<unsigned char>(len_bytes[static_cast<std::size_t>(i)]))
                   << (8 * i);
        }

        std::string content(len, '\0');
        if (len > 0 && !read_exact(in, content.data(), static_cast<std::size_t>(len))) break;

        auto opcode = static_cast<Opcode>(static_cast<std::uint8_t>(opcode_byte));
        if (opcode == Opcode::kFooter) {
            result.saw_footer = true;
            break;
        }
        if (opcode == Opcode::kMessage) {
            std::size_t pos = 0;
            Message m;
            if (content.size() < 2 + 4 + 8 + 8) continue;  // malformed/truncated record body
            m.channel_id = read_u16(content, pos);
            m.sequence = read_u32(content, pos);
            m.log_time_ns = read_u64(content, pos);
            m.publish_time_ns = read_u64(content, pos);
            m.data = content.substr(pos);
            result.messages.push_back(std::move(m));
        }
        // Schema/Channel/Header/DataEnd records: skipped, channel ids are a
        // fixed writer convention (1=episode_header, 2=enforcement_event).
    }
    return result;
}

}  // namespace harness::mcap
