// Rearguard CLI — validate | show | replay | verify | abi. Build brief §10.
//
// Argument parsing here is strict for the same reason the profile JSON parser
// is (see edge/README.md, "Why the JSON parser is ours"): this tool produces
// and inspects safety evidence, and silently accepting a nearly-valid
// invocation is the wrong failure mode. A misspelled `--out` must not run the
// replay, write no MCAP, and exit 0.

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "common.hpp"
#include "connect.hpp"
#include "format/event_table.hpp"
#include "format/json.hpp"
#include "format/profile_table.hpp"
#include "harness/harness_kernel.h"
#include "harness/v1/enforcement.pb.h"
#include "mcap/reader.hpp"
#include "profile/loader.hpp"
#include "profile/validator.hpp"
#include "replay/event_hash.hpp"
#include "replay/replay.hpp"
#include "replay/trace.hpp"
#include "uninstall.hpp"

namespace {

bool looks_like_flag(const std::string& s) { return s.rfind("--", 0) == 0; }

// JSON emission lives in src/format/json.hpp — `show` renders the profile as
// both a table and JSON from one place, and the escaper has to be reachable
// from there. These are thin aliases so call sites here stay readable.
std::string json_string(const std::string& in) { return harness::format::json::str(in); }
std::string json_bool(bool b) { return harness::format::json::boolean(b); }
std::string json_string_array(const std::vector<std::string>& items) {
    return harness::format::json::str_array(items);
}

// Every flag is `--name value`, every flag may appear at most once, and
// anything unrecognised is an error rather than a silent no-op.
class FlagParser {
  public:
    void add(const char* name, std::string* out) {
        specs_.push_back({name, out, nullptr, false});
    }

    // A boolean flag takes no value: `--json`, never `--json true`.
    void add_bool(const char* name, bool* out) {
        specs_.push_back({name, nullptr, out, false});
    }

    bool parse(int argc, char** argv, int first) {
        for (int i = first; i < argc; ++i) {
            const std::string arg = argv[i];
            Spec* spec = find(arg);
            if (spec == nullptr) {
                if (looks_like_flag(arg)) {
                    std::fprintf(stderr, "error: unknown flag '%s'\n", arg.c_str());
                } else {
                    std::fprintf(stderr, "error: unexpected argument '%s'\n", arg.c_str());
                }
                return false;
            }
            if (spec->seen) {
                std::fprintf(stderr, "error: %s given more than once\n", spec->name.c_str());
                return false;
            }
            if (spec->flag_out != nullptr) {
                spec->seen = true;
                *spec->flag_out = true;
                continue;
            }
            if (i + 1 >= argc) {
                std::fprintf(stderr, "error: %s requires a value\n", spec->name.c_str());
                return false;
            }
            const std::string value = argv[++i];
            // `--profile --trace t.jsonl` is a missing value, not a profile
            // literally named "--trace".
            if (looks_like_flag(value)) {
                std::fprintf(stderr, "error: %s requires a value, got flag '%s'\n",
                             spec->name.c_str(), value.c_str());
                return false;
            }
            spec->seen = true;
            *spec->out = value;
        }
        return true;
    }

  private:
    struct Spec {
        std::string name;
        std::string* out;       // null for boolean flags
        bool* flag_out;         // null for value flags
        bool seen;
    };

    Spec* find(const std::string& name) {
        for (auto& s : specs_) {
            if (s.name == name) return &s;
        }
        return nullptr;
    }

    std::vector<Spec> specs_;
};

int cmd_validate(const std::string& profile_path, bool json) {
    auto loaded = harness::profile::load_from_yaml_file(profile_path);
    if (!loaded.profile.has_value()) {
        if (json) {
            std::printf("{\"path\":%s,\"valid\":false,\"parse_error\":%s,\"errors\":[]}\n",
                        json_string(profile_path).c_str(), json_string(loaded.error).c_str());
        } else {
            std::fprintf(stderr, "error: failed to parse %s: %s\n", profile_path.c_str(),
                         loaded.error.c_str());
        }
        return 1;
    }
    auto errors = harness::profile::validate(*loaded.profile);

    if (json) {
        std::string items;
        for (std::size_t i = 0; i < errors.size(); ++i) {
            if (i > 0) items += ",";
            items += "{\"code\":" + json_string(errors[i].code) +
                     ",\"message\":" + json_string(errors[i].message) + "}";
        }
        std::printf("{\"path\":%s,\"valid\":%s,\"errors\":[%s]}\n", json_string(profile_path).c_str(),
                    json_bool(errors.empty()).c_str(), items.c_str());
        return errors.empty() ? 0 : 1;
    }

    if (errors.empty()) {
        std::printf("%s: valid\n", profile_path.c_str());
        return 0;
    }
    for (const auto& e : errors) {
        std::printf("error[%s]: %s\n", e.code.c_str(), e.message.c_str());
    }
    return 1;
}

// `show` prints the profile the kernel would be handed, not the YAML on
// disk. It deliberately does NOT gate on validate(): the profile you most
// need to look at is usually the one that just failed validation, and
// refusing to show it would refuse exactly that case. It does report the
// verdict at the end, so `show` is never mistaken for a clean bill of health.
int cmd_show(const std::string& profile_path, bool json) {
    auto loaded = harness::profile::load_from_yaml_file(profile_path);
    if (!loaded.profile.has_value()) {
        if (json) {
            std::printf("{\"path\":%s,\"parse_error\":%s}\n", json_string(profile_path).c_str(),
                        json_string(loaded.error).c_str());
        } else {
            std::fprintf(stderr, "error: failed to parse %s: %s\n", profile_path.c_str(),
                         loaded.error.c_str());
        }
        return 1;
    }
    const auto& profile = *loaded.profile;
    const std::string computed =
        harness::profile::canonical_joint_order_hash(profile.embodiment().joint_names());
    auto errors = harness::profile::validate(profile);

    if (json) {
        std::string items;
        for (std::size_t i = 0; i < errors.size(); ++i) {
            if (i > 0) items += ",";
            items += "{\"code\":" + json_string(errors[i].code) +
                     ",\"message\":" + json_string(errors[i].message) + "}";
        }
        // The profile object is spliced in rather than nested so a consumer
        // reads the same field names `show` prints, with the verdict beside
        // them. Both objects are ours and neither key set collides.
        std::string body = harness::format::profile_json(profile, computed);
        body.pop_back();  // drop the closing brace to append the verdict
        std::printf("%s,\"path\":%s,\"valid\":%s,\"errors\":[%s]}\n", body.c_str(),
                    json_string(profile_path).c_str(), json_bool(errors.empty()).c_str(),
                    items.c_str());
        return errors.empty() ? 0 : 1;
    }

    std::printf("%s", harness::format::profile_text(profile, computed).c_str());

    if (errors.empty()) {
        std::printf("\nvalidate     ok -- C1-C11 clean\n");
        return 0;
    }
    std::printf("\nvalidate     %zu constraint failure(s)\n", errors.size());
    for (const auto& e : errors) {
        std::printf("             error[%s]: %s\n", e.code.c_str(), e.message.c_str());
    }
    return 1;
}

// Splits "j0,j1,j2". Empty segments are kept: a joint order with a blank
// entry is wrong, and hashing it to something plausible would hide that.
std::vector<std::string> split_commas(const std::string& s) {
    std::vector<std::string> out;
    std::string current;
    for (char c : s) {
        if (c == ',') {
            out.push_back(current);
            current.clear();
        } else {
            current += c;
        }
    }
    out.push_back(current);
    return out;
}

// `hash` deliberately does NOT run validate(). Its whole purpose is to debug a
// C8 (joint-order hash mismatch) failure, and a profile that fails C8 fails
// validation — refusing to hash it would refuse exactly the case you need.
int cmd_hash_joints(const std::vector<std::string>& joint_names, bool json) {
    harness::v1::HarnessProfile scratch;
    for (const auto& j : joint_names) scratch.mutable_embodiment()->add_joint_names(j);
    const std::string computed =
        harness::profile::canonical_joint_order_hash(scratch.embodiment().joint_names());

    if (json) {
        std::printf("{\"joint_names\":%s,\"joint_order_hash\":%s}\n",
                    json_string_array(joint_names).c_str(), json_string(computed).c_str());
        return 0;
    }

    std::printf("joints            %zu  [", joint_names.size());
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        std::printf("%s%s", i == 0 ? "" : ", ", joint_names[i].c_str());
    }
    std::printf("]\n");
    std::printf("joint_order_hash  %s\n", computed.c_str());
    return 0;
}

int cmd_hash_profile(const std::string& profile_path, bool json) {
    auto loaded = harness::profile::load_from_yaml_file(profile_path);
    if (!loaded.profile.has_value()) {
        if (json) {
            std::printf("{\"path\":%s,\"parse_error\":%s}\n", json_string(profile_path).c_str(),
                        json_string(loaded.error).c_str());
        } else {
            std::fprintf(stderr, "error: failed to parse %s: %s\n", profile_path.c_str(),
                         loaded.error.c_str());
        }
        return 1;
    }
    const auto& profile = *loaded.profile;
    const auto& embodiment = profile.embodiment();

    const std::string computed =
        harness::profile::canonical_joint_order_hash(embodiment.joint_names());
    const std::string declared = embodiment.joint_order_hash();

    if (json) {
        std::vector<std::string> names;
        for (const auto& j : embodiment.joint_names()) names.push_back(j);
        std::printf("{\"profile\":%s,\"dof\":%u,\"joint_names\":%s,"
                    "\"joint_order_hash_computed\":%s,\"joint_order_hash_declared\":%s,"
                    "\"match\":%s}\n",
                    json_string(profile.profile_id() + "@" + profile.profile_version()).c_str(),
                    embodiment.dof(), json_string_array(names).c_str(),
                    json_string(computed).c_str(), json_string(declared).c_str(),
                    json_bool(computed == declared).c_str());
        return computed == declared ? 0 : 1;
    }

    // Same string EpisodeHeader.profile_hash carries, so an episode can be
    // traced back to the profile that produced it.
    std::printf("profile           %s@%s\n", profile.profile_id().c_str(),
                profile.profile_version().c_str());
    std::printf("dof               %u\n", embodiment.dof());
    std::printf("joints            %d  [", embodiment.joint_names_size());
    for (int i = 0; i < embodiment.joint_names_size(); ++i) {
        std::printf("%s%s", i == 0 ? "" : ", ", embodiment.joint_names(i).c_str());
    }
    std::printf("]\n");
    std::printf("joint_order_hash  %s  (computed)\n", computed.c_str());
    std::printf("declared          %s  (embodiment.joint_order_hash)\n", declared.c_str());

    if (computed == declared) {
        std::printf("\n  ok    computed matches declared\n");
        return 0;
    }
    std::printf("\n  FAIL  computed does not match declared — this is what C8 rejects\n");
    return 1;
}

int cmd_replay(const std::string& profile_path, const std::string& trace_path,
                const std::string& out_path, const std::string& critic_lib_dir, bool json) {
    auto loaded = harness::profile::load_from_yaml_file(profile_path);
    if (!loaded.profile.has_value()) {
        std::fprintf(stderr, "error: failed to parse profile: %s\n", loaded.error.c_str());
        return 1;
    }
    auto errors = harness::profile::validate(*loaded.profile);
    if (!errors.empty()) {
        std::fprintf(stderr, "error: profile fails validation, refusing to replay:\n");
        for (const auto& e : errors) std::fprintf(stderr, "  error[%s]: %s\n", e.code.c_str(), e.message.c_str());
        return 1;
    }

    auto trace = harness::replay::load_trace_file(trace_path);
    if (!trace.samples.has_value()) {
        std::fprintf(stderr, "error: failed to load trace: %s\n", trace.error.c_str());
        return 1;
    }

    harness::replay::ReplayOptions options;
    options.critic_lib_dir = critic_lib_dir;
    options.scenario_id = trace_path;
    options.print_summary = !json;
    if (!out_path.empty()) options.mcap_out_path = out_path;

    auto result = harness::replay::run_replay(*loaded.profile, *trace.samples, options);
    if (!result.ok) {
        std::fprintf(stderr, "error: replay failed: %s\n", result.error.c_str());
        return 1;
    }
    if (json) {
        std::printf("{\"profile\":%s,\"trace\":%s,\"mcap_out\":%s,"
                    "\"event_stream_hash\":%s,\"cycles\":%llu,\"event_count\":%llu,"
                    "\"dropped\":%u,\"gate_p50_us\":%.3f,\"gate_p99_us\":%.3f,"
                    "\"gate_p999_us\":%.3f}\n",
                    json_string(profile_path).c_str(), json_string(trace_path).c_str(),
                    json_string(out_path).c_str(), json_string(result.event_stream_hash).c_str(),
                    static_cast<unsigned long long>(result.cycles),
                    static_cast<unsigned long long>(result.event_count), result.dropped_total,
                    result.gate_p50_us, result.gate_p99_us, result.gate_p999_us);
        return 0;
    }
    std::printf("event_stream_hash: %s\n", result.event_stream_hash.c_str());
    return 0;
}

// One line of the verification report. Checks are named and reported
// individually rather than collapsed into a single pass/fail, because "this
// episode is unusable" and "this episode lost 3 events to a full ring buffer"
// call for different responses from whoever is reading it.
struct Check {
    enum class Result { kOk, kWarn, kFail };
    Result result;
    std::string detail;
};

const char* label(Check::Result r) {
    switch (r) {
        case Check::Result::kOk: return "ok  ";
        case Check::Result::kWarn: return "warn";
        case Check::Result::kFail: return "FAIL";
    }
    return "????";
}

int cmd_verify(const std::string& mcap_path, const std::string& expect_hash, bool json) {
    auto result = harness::mcap::read_episode(mcap_path);
    if (!result.valid_magic) {
        if (json) {
            std::printf("{\"path\":%s,\"ok\":false,\"error\":\"bad magic\"}\n",
                        json_string(mcap_path).c_str());
        } else {
            std::fprintf(stderr, "error: %s is not a valid MCAP file (bad magic)\n",
                         mcap_path.c_str());
        }
        return 1;
    }

    if (!json) std::printf("%s\n", harness::format::table_header().c_str());

    // The writer stamps the header record with the first sample's time, so
    // the first record's log time is the episode's t=0.
    const std::uint64_t t0_ns = result.messages.empty() ? 0 : result.messages.front().log_time_ns;

    std::size_t header_count = 0;
    std::size_t event_count = 0;
    std::size_t undecodable = 0;
    std::uint64_t dropped_reported = 0;
    std::uint64_t missing_events = 0;
    std::size_t seq_regressions = 0;
    std::size_t time_regressions = 0;
    std::uint32_t file_abi = 0;
    bool have_prev_seq = false;
    std::uint64_t prev_seq = 0;
    std::uint64_t prev_log_time = 0;
    harness::replay::EventStreamHasher hasher;

    for (const auto& m : result.messages) {
        if (m.log_time_ns < prev_log_time) time_regressions++;
        prev_log_time = m.log_time_ns;

        if (m.channel_id == 1) {
            harness::v1::EpisodeHeader header;
            if (!header.ParseFromString(m.data)) {
                if (!json) {
                    std::fprintf(stderr,
                                 "error: header record is not a decodable EpisodeHeader\n");
                }
                undecodable++;
                continue;
            }
            file_abi = header.kernel_abi_version();
            if (!json) {
                std::printf("%s\n", harness::format::episode_header_row(header).c_str());
            }
            header_count++;
        } else if (m.channel_id == 2) {
            harness::v1::EnforcementEvent event;
            if (!event.ParseFromString(m.data)) {
                if (!json) {
                    std::fprintf(stderr, "error: event record seq~%u is not a decodable "
                                         "EnforcementEvent\n", m.sequence);
                }
                undecodable++;
                continue;
            }

            // Hash the stored bytes, not a re-serialisation of the parsed
            // message: protobuf does not guarantee byte-identical round-trips,
            // and it is the bytes on disk this check is about.
            hasher.update(m.data.data(), m.data.size());

            // The kernel assigns a seq to every event it ATTEMPTS to emit
            // (kernel_gate.cpp: `header.seq = h.seq++`, before the sink write
            // that may drop it). So a gap in seq is the ground truth for how
            // many events were lost, derived independently of the
            // dropped_since_last field the events themselves carry.
            if (have_prev_seq) {
                if (event.seq() <= prev_seq) {
                    seq_regressions++;
                } else {
                    missing_events += event.seq() - prev_seq - 1;
                }
            }
            prev_seq = event.seq();
            have_prev_seq = true;

            // Clamped rather than wrapped: out-of-order log times are a real
            // condition (checked above), not a reason to print a nonsense
            // 1.8e10 ms offset from an unsigned underflow.
            const double t_ms = m.log_time_ns >= t0_ns
                                     ? static_cast<double>(m.log_time_ns - t0_ns) / 1e6
                                     : 0.0;
            if (!json) {
                std::printf("%s\n",
                            harness::format::event_row(event.seq(), t_ms, event).c_str());
            }
            dropped_reported += event.dropped_since_last();
            event_count++;
        }
    }

    const std::string stream_hash = hasher.hex();

    if (!json) {
        std::printf("\n%s\n", mcap_path.c_str());
        std::printf("event_stream_hash: %s\n\n", stream_hash.c_str());
    }

    std::vector<Check> checks;
    auto add = [&checks](Check::Result r, std::string detail) {
        checks.push_back({r, std::move(detail)});
    };

    add(Check::Result::kOk, "MCAP magic");

    add(result.saw_footer ? Check::Result::kOk : Check::Result::kFail,
        result.saw_footer ? "footer present (file is complete)"
                          : "footer MISSING — file is truncated; events above are still valid, "
                            "but the episode did not finish");

    add(header_count == 1 ? Check::Result::kOk : Check::Result::kFail,
        "exactly one EpisodeHeader (found " + std::to_string(header_count) + ")");

    add(undecodable == 0 ? Check::Result::kOk : Check::Result::kFail,
        std::to_string(event_count) + " event(s) decoded, " + std::to_string(undecodable) +
            " undecodable");

    if (seq_regressions > 0) {
        add(Check::Result::kFail, std::to_string(seq_regressions) +
                                       " seq regression(s) — events out of order or duplicated");
    } else if (missing_events > 0) {
        // Not a failure: the kernel is specified to drop rather than block
        // when the sink is full. It IS evidence loss, and must be visible.
        add(Check::Result::kWarn,
            std::to_string(missing_events) + " event(s) missing from the seq sequence (dropped)");
    } else {
        add(Check::Result::kOk, "seq contiguous, no gaps");
    }

    add(time_regressions == 0 ? Check::Result::kOk : Check::Result::kFail,
        time_regressions == 0 ? "log_time monotonic"
                              : std::to_string(time_regressions) + " log_time regression(s)");

    // A warning, not a failure: an episode recorded by a different kernel ABI
    // is still valid evidence of what THAT kernel did. It just cannot be
    // compared against this binary's behaviour.
    if (file_abi == hk_abi_version()) {
        add(Check::Result::kOk, "kernel ABI " + std::to_string(file_abi) + " matches this binary");
    } else {
        add(Check::Result::kWarn, "kernel ABI " + std::to_string(file_abi) +
                                       " differs from this binary's " +
                                       std::to_string(hk_abi_version()));
    }

    if (!expect_hash.empty()) {
        add(stream_hash == expect_hash ? Check::Result::kOk : Check::Result::kFail,
            stream_hash == expect_hash
                ? "event_stream_hash matches --expect-hash"
                : "event_stream_hash " + stream_hash + " does not match --expect-hash " +
                      expect_hash);
    }

    // The dropped_since_last field is reported alongside the seq-derived
    // count deliberately: they are two independent accounts of the same loss,
    // and a disagreement between them is itself worth seeing.
    if (dropped_reported != missing_events) {
        add(Check::Result::kWarn,
            "events report " + std::to_string(dropped_reported) +
                " drop(s) via dropped_since_last, but seq gaps imply " +
                std::to_string(missing_events));
    }

    bool failed = false;
    for (const auto& c : checks) {
        if (c.result == Check::Result::kFail) failed = true;
    }

    if (json) {
        // Deliberately no per-event array: the event table has exactly one
        // rendering (format/event_table.cpp), and a second, JSON-shaped
        // definition of the same records is the drift §12 warns about.
        // Anything needing per-event data should read the MCAP directly --
        // this command's output is the verdict.
        std::string items;
        for (std::size_t i = 0; i < checks.size(); ++i) {
            if (i > 0) items += ",";
            const char* r = checks[i].result == Check::Result::kOk     ? "ok"
                            : checks[i].result == Check::Result::kWarn ? "warn"
                                                                       : "fail";
            items += "{\"result\":\"" + std::string(r) + "\",\"detail\":" +
                     json_string(checks[i].detail) + "}";
        }
        std::printf("{\"path\":%s,\"ok\":%s,\"event_stream_hash\":%s,\"header_count\":%zu,"
                    "\"event_count\":%zu,\"undecodable\":%zu,\"footer_present\":%s,"
                    "\"dropped_reported\":%llu,\"missing_events\":%llu,\"seq_regressions\":%zu,"
                    "\"time_regressions\":%zu,\"kernel_abi_file\":%u,\"kernel_abi_binary\":%u,"
                    "\"checks\":[%s]}\n",
                    json_string(mcap_path).c_str(), json_bool(!failed).c_str(),
                    json_string(stream_hash).c_str(), header_count, event_count, undecodable,
                    json_bool(result.saw_footer).c_str(),
                    static_cast<unsigned long long>(dropped_reported),
                    static_cast<unsigned long long>(missing_events), seq_regressions,
                    time_regressions, file_abi, hk_abi_version(), items.c_str());
        return failed ? 1 : 0;
    }

    for (const auto& c : checks) {
        std::printf("  %s  %s\n", label(c.result), c.detail.c_str());
    }

    return failed ? 1 : 0;
}

// ---------------------------------------------------------------------------
// diff: the determinism regression check, as a command.
//
// Two episodes that were supposed to be produced identically -- the same
// profile and trace before and after a change, or the rig's shadow gate
// against the kernel -- must have the same event_stream_hash. When they do
// not, the only useful next question is "which event first differed", and
// answering it by hand means decoding two MCAPs.
// ---------------------------------------------------------------------------
struct LoadedEpisode {
    bool ok = false;
    std::string error;
    bool has_header = false;
    harness::v1::EpisodeHeader header;
    std::vector<std::string> event_bytes;
    std::vector<harness::v1::EnforcementEvent> events;
    std::vector<double> t_ms;
    std::string stream_hash;
};

LoadedEpisode load_episode(const std::string& path) {
    LoadedEpisode ep;
    auto read = harness::mcap::read_episode(path);
    if (!read.valid_magic) {
        ep.error = path + " is not a valid MCAP file (bad magic)";
        return ep;
    }
    const std::uint64_t t0_ns = read.messages.empty() ? 0 : read.messages.front().log_time_ns;
    harness::replay::EventStreamHasher hasher;

    for (const auto& m : read.messages) {
        if (m.channel_id == 1) {
            if (!ep.has_header && ep.header.ParseFromString(m.data)) ep.has_header = true;
        } else if (m.channel_id == 2) {
            harness::v1::EnforcementEvent event;
            if (!event.ParseFromString(m.data)) {
                ep.error = path + " contains an undecodable EnforcementEvent";
                return ep;
            }
            hasher.update(m.data.data(), m.data.size());
            ep.event_bytes.push_back(m.data);
            ep.events.push_back(event);
            ep.t_ms.push_back(m.log_time_ns >= t0_ns
                                   ? static_cast<double>(m.log_time_ns - t0_ns) / 1e6
                                   : 0.0);
        }
    }
    ep.stream_hash = hasher.hex();
    ep.ok = true;
    return ep;
}

// Only the fields that decide whether two episodes are COMPARABLE at all.
// scenario_id and sweep_id differ routinely and legitimately; a differing
// profile_hash or joint order means the two files are not two runs of the
// same thing, and that explains a divergence rather than being one.
std::vector<std::string> header_differences(const LoadedEpisode& a, const LoadedEpisode& b) {
    std::vector<std::string> diffs;
    auto compare = [&diffs](const char* name, const std::string& lhs, const std::string& rhs) {
        if (lhs != rhs) diffs.push_back(std::string(name) + ": " + lhs + " vs " + rhs);
    };
    compare("profile_hash", a.header.profile_hash(), b.header.profile_hash());
    compare("joint_order_hash", a.header.joint_order_hash(), b.header.joint_order_hash());
    compare("kernel_abi_version", std::to_string(a.header.kernel_abi_version()),
            std::to_string(b.header.kernel_abi_version()));
    compare("clock_domain", harness::v1::ClockDomain_Name(a.header.clock_domain()),
            harness::v1::ClockDomain_Name(b.header.clock_domain()));
    compare("output_interface", harness::v1::OutputInterface_Name(a.header.output_interface()),
            harness::v1::OutputInterface_Name(b.header.output_interface()));
    return diffs;
}

int cmd_diff(const std::string& path_a, const std::string& path_b, bool json) {
    LoadedEpisode a = load_episode(path_a);
    LoadedEpisode b = load_episode(path_b);
    if (!a.ok || !b.ok) {
        const std::string& err = a.ok ? b.error : a.error;
        if (json) {
            std::printf("{\"ok\":false,\"error\":%s}\n", json_string(err).c_str());
        } else {
            std::fprintf(stderr, "error: %s\n", err.c_str());
        }
        return 1;
    }

    const bool identical = a.stream_hash == b.stream_hash;
    const std::vector<std::string> hdr_diffs = header_differences(a, b);

    // Compare the stored bytes, not the parsed messages: the hash is over
    // bytes, so this locates exactly what made the hashes differ.
    std::size_t first_divergence = 0;
    bool found_divergence = false;
    const std::size_t common = std::min(a.event_bytes.size(), b.event_bytes.size());
    for (std::size_t i = 0; i < common; ++i) {
        if (a.event_bytes[i] != b.event_bytes[i]) {
            first_divergence = i;
            found_divergence = true;
            break;
        }
    }
    if (!found_divergence && a.event_bytes.size() != b.event_bytes.size()) {
        first_divergence = common;
        found_divergence = true;
    }

    if (json) {
        std::string diffs_json = "[";
        for (std::size_t i = 0; i < hdr_diffs.size(); ++i) {
            if (i > 0) diffs_json += ",";
            diffs_json += json_string(hdr_diffs[i]);
        }
        diffs_json += "]";
        std::printf("{\"a\":%s,\"b\":%s,\"identical\":%s,\"hash_a\":%s,\"hash_b\":%s,"
                    "\"events_a\":%zu,\"events_b\":%zu,\"header_differences\":%s,"
                    "\"first_divergence_index\":%s}\n",
                    json_string(path_a).c_str(), json_string(path_b).c_str(),
                    json_bool(identical).c_str(), json_string(a.stream_hash).c_str(),
                    json_string(b.stream_hash).c_str(), a.event_bytes.size(),
                    b.event_bytes.size(), diffs_json.c_str(),
                    found_divergence ? std::to_string(first_divergence).c_str() : "null");
        return identical ? 0 : 1;
    }

    std::printf("a  %s\n   hash=%s  %zu event(s)\n", path_a.c_str(), a.stream_hash.c_str(),
                a.event_bytes.size());
    std::printf("b  %s\n   hash=%s  %zu event(s)\n\n", path_b.c_str(), b.stream_hash.c_str(),
                b.event_bytes.size());

    if (hdr_diffs.empty()) {
        std::printf("  ok    EpisodeHeaders describe the same run\n");
    } else {
        for (const auto& d : hdr_diffs) {
            std::printf("  FAIL  header differs -- %s\n", d.c_str());
        }
        std::printf("        these episodes are not two runs of the same thing\n");
    }

    if (identical) {
        std::printf("  ok    event streams are byte-identical\n");
        return 0;
    }

    std::printf("  FAIL  event streams differ\n");
    if (found_divergence) {
        std::printf("\nfirst divergence at event index %zu:\n\n", first_divergence);
        std::printf("     %s\n", harness::format::table_header().c_str());
        if (first_divergence < a.events.size()) {
            std::printf("  a  %s\n",
                        harness::format::event_row(a.events[first_divergence].seq(),
                                                    a.t_ms[first_divergence],
                                                    a.events[first_divergence])
                            .c_str());
        } else {
            std::printf("  a  <no event at this index -- stream ends here>\n");
        }
        if (first_divergence < b.events.size()) {
            std::printf("  b  %s\n",
                        harness::format::event_row(b.events[first_divergence].seq(),
                                                    b.t_ms[first_divergence],
                                                    b.events[first_divergence])
                            .c_str());
        } else {
            std::printf("  b  <no event at this index -- stream ends here>\n");
        }
    }
    return 1;
}

void print_usage() {
    std::fprintf(stderr,
                  "usage:\n"
                  "  rearguard --version\n"
                  "  rearguard validate <profile.yaml> [--json]\n"
                  "  rearguard show <profile.yaml> [--json]\n"
                  "  rearguard replay --profile <profile.yaml> --trace <trace.jsonl> "
                  "[--out <episode.mcap>] [--critic-lib-dir <dir>] [--json]\n"
                  "  rearguard verify <episode.mcap> [--expect-hash <hex>] [--json]\n"
                  "  rearguard hash <profile.yaml> [--json]\n"
                  "  rearguard hash --joints <j0,j1,j2> [--json]\n"
                  "  rearguard diff <a.mcap> <b.mcap> [--json]\n"
                  "  rearguard abi\n"
                  "  rearguard connect [--name <device-name>] [--scan-settle-ms 750] "
                  "[--scan-parallelism 16] [--json]\n"
                  "  rearguard scan <setup|runtime> [options]\n"
                  "  rearguard observe <start|profile|status|capture|windows|promote|proposals> [options]\n"
                  "  rearguard workflow <init|show|outbox|artifact|apply|export-window|generation-request|sync|sync-once> [options]\n"
                  "  rearguard uninstall [--prefix <dir>] [--dry-run] [--yes] [--json]\n");
}

// Shared by the commands that take a fixed number of positional arguments, so
// `validate a.yaml b.yaml` is refused instead of quietly ignoring b.yaml.
bool expect_exact_args(int argc, int wanted, const char* command) {
    if (argc - 2 == wanted) return true;
    std::fprintf(stderr, "error: %s takes exactly %d argument(s), got %d\n", command, wanted,
                 argc - 2);
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage();
        return 1;
    }
    const std::string cmd = argv[1];

    if (cmd == "--version") {
        if (!expect_exact_args(argc, 0, "--version")) {
            print_usage();
            return 1;
        }
        std::printf("rearguard %s\n", REARGUARD_VERSION);
        return 0;
    }

    if (cmd == "scan" || cmd == "observe" || cmd == "workflow") {
        return harness::observation::command_main(cmd, argc - 2, argv + 2);
    }

    if (cmd == "abi") {
        if (!expect_exact_args(argc, 0, "abi")) {
            print_usage();
            return 1;
        }
        std::printf("%u\n", hk_abi_version());
        return 0;
    }

    if (cmd == "connect") {
        harness::cli::ConnectOptions opts;
        std::string scan_settle_ms = "750";
        std::string scan_parallelism = "16";
        FlagParser flags;
        flags.add("--name", &opts.device_name);
        flags.add("--scan-settle-ms", &scan_settle_ms);
        flags.add("--scan-parallelism", &scan_parallelism);
        flags.add_bool("--json", &opts.json);
        if (!flags.parse(argc, argv, 2)) {
            print_usage();
            return 1;
        }
        try {
            std::size_t used = 0;
            opts.scan_settle_ms = std::stoi(scan_settle_ms, &used);
            if (used != scan_settle_ms.size() || opts.scan_settle_ms < 0 ||
                opts.scan_settle_ms > 10000)
                throw std::invalid_argument("range");
            used = 0;
            opts.scan_parallelism = std::stoi(scan_parallelism, &used);
            if (used != scan_parallelism.size() || opts.scan_parallelism < 1 ||
                opts.scan_parallelism > 32)
                throw std::invalid_argument("range");
        } catch (const std::exception&) {
            std::fprintf(stderr,
                         "error: scan settle must be 0..10000 ms and parallelism 1..32\n");
            return 1;
        }
        return harness::cli::run_connect(opts);
    }

    if (cmd == "validate") {
        if (argc < 3 || looks_like_flag(argv[2])) {
            std::fprintf(stderr, "error: validate requires a <profile.yaml> path\n");
            print_usage();
            return 1;
        }
        bool json = false;
        FlagParser flags;
        flags.add_bool("--json", &json);
        if (!flags.parse(argc, argv, 3)) {
            print_usage();
            return 1;
        }
        return cmd_validate(argv[2], json);
    }

    if (cmd == "show") {
        if (argc < 3 || looks_like_flag(argv[2])) {
            std::fprintf(stderr, "error: show requires a <profile.yaml> path\n");
            print_usage();
            return 1;
        }
        bool json = false;
        FlagParser flags;
        flags.add_bool("--json", &json);
        if (!flags.parse(argc, argv, 3)) {
            print_usage();
            return 1;
        }
        return cmd_show(argv[2], json);
    }

    if (cmd == "verify") {
        if (argc < 3 || looks_like_flag(argv[2])) {
            std::fprintf(stderr, "error: verify requires an <episode.mcap> path\n");
            print_usage();
            return 1;
        }
        std::string expect_hash;
        bool json = false;
        FlagParser flags;
        flags.add("--expect-hash", &expect_hash);
        flags.add_bool("--json", &json);
        if (!flags.parse(argc, argv, 3)) {
            print_usage();
            return 1;
        }
        return cmd_verify(argv[2], expect_hash, json);
    }

    if (cmd == "hash") {
        if (argc < 3) {
            std::fprintf(stderr, "error: hash requires <profile.yaml> or --joints <j0,j1,j2>\n");
            print_usage();
            return 1;
        }
        bool json = false;
        if (std::string(argv[2]) == "--joints") {
            std::string joints;
            FlagParser flags;
            flags.add("--joints", &joints);
            flags.add_bool("--json", &json);
            if (!flags.parse(argc, argv, 2)) {
                print_usage();
                return 1;
            }
            return cmd_hash_joints(split_commas(joints), json);
        }
        if (looks_like_flag(argv[2])) {
            std::fprintf(stderr, "error: hash requires <profile.yaml> or --joints <j0,j1,j2>\n");
            print_usage();
            return 1;
        }
        FlagParser flags;
        flags.add_bool("--json", &json);
        if (!flags.parse(argc, argv, 3)) {
            print_usage();
            return 1;
        }
        return cmd_hash_profile(argv[2], json);
    }

    if (cmd == "diff") {
        if (argc < 4 || looks_like_flag(argv[2]) || looks_like_flag(argv[3])) {
            std::fprintf(stderr, "error: diff requires two <episode.mcap> paths\n");
            print_usage();
            return 1;
        }
        bool json = false;
        FlagParser flags;
        flags.add_bool("--json", &json);
        if (!flags.parse(argc, argv, 4)) {
            print_usage();
            return 1;
        }
        return cmd_diff(argv[2], argv[3], json);
    }

    if (cmd == "replay") {
        std::string profile_path;
        std::string trace_path;
        std::string out_path;
        std::string critic_lib_dir = HARNESS_CRITIC_LIB_DIR;
        bool json = false;

        FlagParser flags;
        flags.add("--profile", &profile_path);
        flags.add("--trace", &trace_path);
        flags.add("--out", &out_path);
        flags.add("--critic-lib-dir", &critic_lib_dir);
        flags.add_bool("--json", &json);
        if (!flags.parse(argc, argv, 2)) {
            print_usage();
            return 1;
        }
        if (profile_path.empty()) {
            std::fprintf(stderr, "error: replay requires --profile\n");
            print_usage();
            return 1;
        }
        if (trace_path.empty()) {
            std::fprintf(stderr, "error: replay requires --trace\n");
            print_usage();
            return 1;
        }
        return cmd_replay(profile_path, trace_path, out_path, critic_lib_dir, json);
    }

    if (cmd == "uninstall") {
        harness::cli::UninstallOptions opts;
        std::string prefix;
        bool dry_run = false;
        bool assume_yes = false;
        bool json = false;

        FlagParser flags;
        flags.add("--prefix", &prefix);
        flags.add_bool("--dry-run", &dry_run);
        flags.add_bool("--yes", &assume_yes);
        flags.add_bool("--json", &json);
        if (!flags.parse(argc, argv, 2)) {
            print_usage();
            return 1;
        }
        opts.prefix = prefix;
        opts.dry_run = dry_run;
        opts.assume_yes = assume_yes;
        opts.json = json;
        return harness::cli::run_uninstall(opts);
    }

    std::fprintf(stderr, "error: unknown command '%s'\n", cmd.c_str());
    if (cmd == "conect") {
        std::fprintf(stderr, "hint: did you mean 'rearguard connect'?\n");
    }
    print_usage();
    return 1;
}
