#include "mcap/writer.hpp"

#include <set>
#include <string>

#include <google/protobuf/descriptor.pb.h>

#include "mcap/format.hpp"

namespace harness::mcap {

namespace {

void put_u16(std::string& s, std::uint16_t v) {
    for (int i = 0; i < 2; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
}

void put_u32(std::string& s, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
}

void put_u64(std::string& s, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
}

void put_str(std::string& s, const std::string& v) {
    put_u32(s, static_cast<std::uint32_t>(v.size()));
    s.append(v);
}

void put_bytes(std::string& s, const void* data, std::size_t len) {
    put_u32(s, static_cast<std::uint32_t>(len));
    s.append(static_cast<const char*>(data), len);
}

void collect_file_descriptors(const google::protobuf::FileDescriptor* file,
                               std::set<std::string>& seen,
                               google::protobuf::FileDescriptorSet& out) {
    if (!seen.insert(std::string(file->name())).second) return;
    for (int i = 0; i < file->dependency_count(); ++i) {
        collect_file_descriptors(file->dependency(i), seen, out);
    }
    file->CopyTo(out.add_file());
}

std::string build_file_descriptor_set(const google::protobuf::Descriptor* descriptor) {
    google::protobuf::FileDescriptorSet fds;
    std::set<std::string> seen;
    collect_file_descriptors(descriptor->file(), seen, fds);
    return fds.SerializeAsString();
}

}  // namespace

bool EpisodeWriter::open(const std::string& path,
                          const google::protobuf::Descriptor* header_descriptor,
                          const google::protobuf::Descriptor* event_descriptor) {
    out_.open(path, std::ios::binary | std::ios::trunc);
    if (!out_.is_open()) return false;
    closed_ = false;

    out_.write(reinterpret_cast<const char*>(kMagic.data()),
               static_cast<std::streamsize>(kMagic.size()));

    {
        std::string content;
        put_str(content, "");                    // profile
        put_str(content, "harness_kernel_v2");    // library
        out_.put(static_cast<char>(Opcode::kHeader));
        std::string len_bytes;
        put_u64(len_bytes, content.size());
        out_.write(len_bytes.data(), static_cast<std::streamsize>(len_bytes.size()));
        out_.write(content.data(), static_cast<std::streamsize>(content.size()));
    }

    write_schema(1, std::string(header_descriptor->full_name()), header_descriptor);
    write_schema(2, std::string(event_descriptor->full_name()), event_descriptor);
    write_channel(1, 1, "harness/episode_header");
    write_channel(2, 2, "harness/enforcement_event");

    out_.flush();
    return out_.good();
}

void EpisodeWriter::write_schema(std::uint16_t id, const std::string& name,
                                  const google::protobuf::Descriptor* descriptor) {
    std::string data = build_file_descriptor_set(descriptor);
    std::string content;
    put_u16(content, id);
    put_str(content, name);
    put_str(content, "protobuf");
    put_bytes(content, data.data(), data.size());

    out_.put(static_cast<char>(Opcode::kSchema));
    std::string len_bytes;
    put_u64(len_bytes, content.size());
    out_.write(len_bytes.data(), static_cast<std::streamsize>(len_bytes.size()));
    out_.write(content.data(), static_cast<std::streamsize>(content.size()));
    out_.flush();
}

void EpisodeWriter::write_channel(std::uint16_t id, std::uint16_t schema_id,
                                   const std::string& topic) {
    std::string content;
    put_u16(content, id);
    put_u16(content, schema_id);
    put_str(content, topic);
    put_str(content, "protobuf");
    put_u32(content, 0);  // empty metadata map

    out_.put(static_cast<char>(Opcode::kChannel));
    std::string len_bytes;
    put_u64(len_bytes, content.size());
    out_.write(len_bytes.data(), static_cast<std::streamsize>(len_bytes.size()));
    out_.write(content.data(), static_cast<std::streamsize>(content.size()));
    out_.flush();
}

void EpisodeWriter::write_message(std::uint16_t channel_id, std::uint32_t sequence,
                                   std::uint64_t log_time_ns, const void* data, std::size_t len) {
    std::string content;
    put_u16(content, channel_id);
    put_u32(content, sequence);
    put_u64(content, log_time_ns);
    put_u64(content, log_time_ns);  // publish_time == log_time: the kernel's own decision stamp
    content.append(static_cast<const char*>(data), len);

    out_.put(static_cast<char>(Opcode::kMessage));
    std::string len_bytes;
    put_u64(len_bytes, content.size());
    out_.write(len_bytes.data(), static_cast<std::streamsize>(len_bytes.size()));
    out_.write(content.data(), static_cast<std::streamsize>(content.size()));
    // Flushed per record on purpose: build brief §9 "MCAP" test SIGKILLs the
    // writer mid-stream and requires the partial file to parse up to the
    // last *committed* record. A record only counts as committed once it has
    // left process memory.
    out_.flush();
}

void EpisodeWriter::write_header(const void* data, std::size_t len, std::uint64_t log_time_ns) {
    write_message(1, header_sequence_++, log_time_ns, data, len);
}

void EpisodeWriter::write_event(const void* data, std::size_t len, std::uint64_t log_time_ns) {
    write_message(2, event_sequence_++, log_time_ns, data, len);
}

void EpisodeWriter::close() {
    if (closed_) return;
    closed_ = true;

    {
        std::string content;
        put_u32(content, 0);  // data_section_crc: not computed
        out_.put(static_cast<char>(Opcode::kDataEnd));
        std::string len_bytes;
        put_u64(len_bytes, content.size());
        out_.write(len_bytes.data(), static_cast<std::streamsize>(len_bytes.size()));
        out_.write(content.data(), static_cast<std::streamsize>(content.size()));
    }
    {
        std::string content;
        put_u64(content, 0);  // summary_start: no summary section (unchunked writer)
        put_u64(content, 0);  // summary_offset_start
        put_u32(content, 0);  // summary_crc: not computed
        out_.put(static_cast<char>(Opcode::kFooter));
        std::string len_bytes;
        put_u64(len_bytes, content.size());
        out_.write(len_bytes.data(), static_cast<std::streamsize>(len_bytes.size()));
        out_.write(content.data(), static_cast<std::streamsize>(content.size()));
    }
    out_.write(reinterpret_cast<const char*>(kMagic.data()),
               static_cast<std::streamsize>(kMagic.size()));
    out_.flush();
    out_.close();
}

EpisodeWriter::~EpisodeWriter() { close(); }

}  // namespace harness::mcap
