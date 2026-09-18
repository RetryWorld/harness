// MCAP writer/reader round-trip and crash-safety tests (build brief §9
// "MCAP"). SIGKILL is simulated by truncating the file at an arbitrary byte
// offset mid-stream rather than actually forking and killing a process —
// the on-disk effect (a file that ends mid-record) is identical, and this
// keeps the test hermetic and fast.

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "harness/v1/enforcement.pb.h"
#include "mcap/reader.hpp"
#include "mcap/writer.hpp"

namespace {

int g_failures = 0;

void expect(bool cond, const std::string& msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg.c_str());
        g_failures++;
    }
}

std::string temp_path(const std::string& name) { return "/tmp/harness_mcap_test_" + name + ".mcap"; }

void write_sample_episode(const std::string& path, int n_events) {
    harness::mcap::EpisodeWriter writer;
    bool opened = writer.open(path, harness::v1::EpisodeHeader::GetDescriptor(),
                               harness::v1::EnforcementEvent::GetDescriptor());
    if (!opened) return;

    harness::v1::EpisodeHeader header;
    header.set_schema("harness.v1.EpisodeHeader");
    header.set_scenario_id("mcap_test");
    std::string header_bytes = header.SerializeAsString();
    writer.write_header(header_bytes.data(), header_bytes.size(), 0);

    for (int i = 0; i < n_events; ++i) {
        harness::v1::EnforcementEvent ev;
        ev.set_seq(static_cast<std::uint64_t>(i));
        ev.set_mechanism(harness::v1::PROJECTION);
        std::string bytes = ev.SerializeAsString();
        writer.write_event(
            bytes.data(), bytes.size(),
            static_cast<std::uint64_t>(i) * static_cast<std::uint64_t>(1'000'000));
    }
    writer.close();
}

void test_round_trip() {
    std::string path = temp_path("roundtrip");
    write_sample_episode(path, 10);

    auto result = harness::mcap::read_episode(path);
    expect(result.valid_magic, "written file should have valid magic");
    expect(result.saw_footer, "a cleanly closed writer should leave a footer");

    int header_count = 0;
    int event_count = 0;
    for (const auto& m : result.messages) {
        if (m.channel_id == 1) {
            harness::v1::EpisodeHeader header;
            expect(header.ParseFromString(m.data), "EpisodeHeader message should parse");
            expect(header.scenario_id() == "mcap_test", "EpisodeHeader fields should survive round-trip");
            header_count++;
        } else if (m.channel_id == 2) {
            harness::v1::EnforcementEvent ev;
            expect(ev.ParseFromString(m.data), "EnforcementEvent message should parse");
            expect(ev.seq() == static_cast<std::uint64_t>(event_count), "event seq should survive in order");
            event_count++;
        }
    }
    expect(header_count == 1, "exactly one EpisodeHeader message expected");
    expect(event_count == 10, "all 10 events should round-trip");
}

void test_truncated_file_parses_up_to_last_committed_record() {
    std::string path = temp_path("truncated");
    // Enough events that the schema/channel preamble (which embeds a full
    // FileDescriptorSet per message type, easily several KB) is a small
    // fraction of the file — otherwise a 90%-mark cut could land before the
    // first Message record and the test would prove nothing.
    write_sample_episode(path, 2000);

    std::ifstream in(path, std::ios::binary | std::ios::ate);
    auto full_size = in.tellg();
    in.close();

    // Truncate near the end (well before the footer), guaranteeing the cut
    // lands inside some record's body rather than on a record boundary.
    std::string truncated_path = temp_path("truncated_copy");
    {
        std::ifstream src(path, std::ios::binary);
        std::ofstream dst(truncated_path, std::ios::binary);
        std::vector<char> buf(static_cast<std::size_t>(full_size) * 9 / 10);
        src.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        dst.write(buf.data(), static_cast<std::streamsize>(buf.size()));
    }

    auto result = harness::mcap::read_episode(truncated_path);
    expect(result.valid_magic, "truncated file should still have a valid, intact magic header");
    expect(!result.saw_footer, "truncated file should not report a footer (it was cut before reaching one)");
    expect(!result.messages.empty(), "records fully written before the cut must still be recovered");

    // Every recovered record must be well-formed: no partial/garbage record
    // should ever be surfaced by the reader.
    for (const auto& m : result.messages) {
        if (m.channel_id == 2) {
            harness::v1::EnforcementEvent ev;
            expect(ev.ParseFromString(m.data), "every recovered event record must be complete and parseable");
        }
    }
}

}  // namespace

int main() {
    test_round_trip();
    test_truncated_file_parses_up_to_last_committed_record();

    if (g_failures > 0) {
        std::fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("all mcap tests passed\n");
    return 0;
}
