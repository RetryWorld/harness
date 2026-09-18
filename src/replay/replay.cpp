#include "replay/replay.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

#include "critic/critic_params.hpp"
#include "format/event_table.hpp"
#include "replay/event_hash.hpp"
#include "harness/harness_kernel.h"
#include "harness/v1/enforcement.pb.h"
#include "mcap/writer.hpp"
#include "replay/critic_loader.hpp"

namespace harness::replay {

namespace {

#if defined(__APPLE__)
constexpr const char* kSharedLibExt = ".dylib";
#else
constexpr const char* kSharedLibExt = ".so";
#endif

double aux_lookup(const TraceSample& sample, const std::string& key, double fallback) {
    for (const auto& [k, v] : sample.aux) {
        if (k == key) return v;
    }
    return fallback;
}

// The two reference critics are "deliberately trivial" (build brief §7) and
// their tuning isn't part of the schema — see critic_params.hpp. This is the
// one place that decides how to tune them for a given profile's critics.
std::vector<std::uint8_t> build_params_for(const std::string& critic_id, std::uint32_t evidence_index,
                                            std::uint64_t declared_wcet_us) {
    if (critic_id == "threshold_critic") {
        harness::critic::ThresholdCriticParams p;
        p.aux_index = 0;
        p.bound = 0.5;
        p.trigger_below = 1;
        p.consecutive_required = 3;
        p.evidence_class_index = evidence_index;
        p.declared_wcet_us = declared_wcet_us;
        std::vector<std::uint8_t> bytes(sizeof(p));
        std::memcpy(bytes.data(), &p, sizeof(p));
        return bytes;
    }
    if (critic_id == "stall_critic") {
        harness::critic::StallCriticParams p;
        p.position_epsilon = 0.01;
        p.effort_bound = 1.0;
        p.evidence_class_index = evidence_index;
        p.declared_wcet_us = declared_wcet_us;
        std::vector<std::uint8_t> bytes(sizeof(p));
        std::memcpy(bytes.data(), &p, sizeof(p));
        return bytes;
    }
    return {};
}

std::uint64_t decode_varint(const std::uint8_t* buf, std::size_t len, std::size_t& pos) {
    std::uint64_t result = 0;
    int shift = 0;
    while (pos < len) {
        std::uint8_t b = buf[pos++];
        result |= static_cast<std::uint64_t>(b & 0x7F) << shift;
        if ((b & 0x80U) == 0U) break;
        shift += 7;
    }
    return result;
}

// Returns the message, not its bytes: the summary table renders from the
// same EpisodeHeader that gets serialised into the MCAP, so what `replay`
// prints and what `verify` reads back cannot drift apart.
harness::v1::EpisodeHeader build_episode_header(const harness::v1::HarnessProfile& profile,
                                                 const ReplayOptions& options) {
    harness::v1::EpisodeHeader header;
    header.set_schema("harness.v1.EpisodeHeader");
    header.set_scenario_id(options.scenario_id);
    header.set_profile_version(profile.profile_version());
    *header.mutable_derivation() = profile.provenance().derivation();
    for (const auto& j : profile.embodiment().joint_names()) header.add_joint_names(j);
    header.set_joint_order_hash(profile.embodiment().joint_order_hash());
    header.set_output_interface(profile.model_binding().output_interface());
    header.set_kernel_abi_version(hk_abi_version());
    header.set_kernel_build_hash("dev");
    header.set_profile_hash(profile.profile_id() + "@" + profile.profile_version());
    header.set_clock_domain(profile.timing().clock_domain());
    return header;
}

}  // namespace

ReplayResult run_replay(const harness::v1::HarnessProfile& profile,
                         const std::vector<TraceSample>& trace, const ReplayOptions& options) {
    ReplayResult result;

    std::string profile_bytes = profile.SerializeAsString();
    hk_handle* handle = nullptr;
    hk_status st = hk_create(reinterpret_cast<const std::uint8_t*>(profile_bytes.data()),
                              profile_bytes.size(), &handle);
    if (st != HK_OK) {
        result.error = "hk_create failed: status " + std::to_string(st);
        return result;
    }

    std::vector<LoadedCritic> critics;
    critics.reserve(static_cast<std::size_t>(profile.critics_size()));
    for (int i = 0; i < profile.critics_size(); ++i) {
        const auto& decl = profile.critics(i);
        std::string lib_path = options.critic_lib_dir + "/lib" + decl.critic_id() + kSharedLibExt;
        auto loaded = LoadedCritic::load(lib_path);
        if (!loaded.has_value()) {
            hk_destroy(handle);
            result.error = "failed to load critic library: " + lib_path;
            return result;
        }
        auto params = build_params_for(decl.critic_id(), static_cast<std::uint32_t>(i), decl.wcet_us());
        if (loaded->create(params.empty() ? nullptr : params.data(), params.size()) != HK_OK) {
            hk_destroy(handle);
            result.error = "critic hkc_create failed: " + decl.critic_id();
            return result;
        }
        critics.push_back(std::move(*loaded));
    }

    const harness::v1::EpisodeHeader episode_header = build_episode_header(profile, options);

    std::optional<harness::mcap::EpisodeWriter> writer;
    if (options.mcap_out_path.has_value()) {
        writer.emplace();
        bool opened = writer->open(*options.mcap_out_path, harness::v1::EpisodeHeader::GetDescriptor(),
                                    harness::v1::EnforcementEvent::GetDescriptor());
        if (!opened) {
            hk_destroy(handle);
            result.error = "failed to open MCAP output: " + *options.mcap_out_path;
            return result;
        }
        const std::string header_bytes = episode_header.SerializeAsString();
        std::uint64_t log_time = trace.empty() ? 0 : trace.front().t_ns;
        writer->write_header(header_bytes.data(), header_bytes.size(), log_time);
    }

    if (options.print_summary) {
        std::printf("%s\n", harness::format::table_header().c_str());
        std::printf("%s\n", harness::format::episode_header_row(episode_header).c_str());
    }

    EventStreamHasher hasher;
    std::vector<std::uint8_t> sink_buf(1U << 16U);
    std::vector<double> gate_times_us;
    gate_times_us.reserve(trace.size());
    std::uint64_t t0_ns = trace.empty() ? 0 : trace.front().t_ns;

    for (const auto& sample : trace) {
        std::uint32_t dof = profile.embodiment().dof();
        if (sample.effort.size() != dof) {
            result.error = "trace sample has " + std::to_string(sample.effort.size()) +
                            " effort values, profile declares dof=" + std::to_string(dof);
            hk_destroy(handle);
            return result;
        }

        for (std::size_t ci = 0; ci < critics.size(); ++ci) {
            const auto& decl = profile.critics(static_cast<int>(ci));
            hkc_window window{};
            window.t_ns = sample.t_ns;
            window.n_joints = dof;
            window.n_samples = 1;
            window.effort = sample.effort.empty() ? nullptr : sample.effort.data();
            window.position = sample.position.empty() ? nullptr : sample.position.data();
            window.velocity = sample.velocity.empty() ? nullptr : sample.velocity.data();
            std::vector<double> aux_values;
            aux_values.reserve(sample.aux.size());
            for (const auto& [k, v] : sample.aux) {
                (void)k;
                aux_values.push_back(v);
            }
            window.aux = aux_values.empty() ? nullptr : aux_values.data();
            window.n_aux = static_cast<std::uint32_t>(aux_values.size());

            hkc_proposal proposal{};
            critics[ci].evaluate(window, proposal);

            double elapsed_us = aux_lookup(sample, decl.critic_id() + ".elapsed_us",
                                            static_cast<double>(decl.wcet_us()) * 0.5);
            hk_report_budget(handle, static_cast<std::uint32_t>(1 + ci),
                              static_cast<std::uint64_t>(elapsed_us * 1000.0));

            if (proposal.confidence >= 0.5F) {
                hk_submit_proposal(handle, &proposal);
            }
        }

        std::vector<double> emitted(dof, 0.0);
        hk_command cmd{};
        cmd.t_ns = sample.t_ns;
        cmd.n_joints = dof;
        cmd.effort = sample.effort.data();

        hk_gated_command gated{};
        gated.n_joints = dof;
        gated.effort = emitted.data();

        std::size_t sink_len = 0;
        std::uint32_t sink_dropped = 0;
        hk_event_sink sink{sink_buf.data(), sink_buf.size(), &sink_len, &sink_dropped};

        auto t_start = std::chrono::steady_clock::now();
        hk_status gate_status = hk_gate(handle, &cmd, &gated, &sink);
        auto t_end = std::chrono::steady_clock::now();
        gate_times_us.push_back(
            std::chrono::duration<double, std::micro>(t_end - t_start).count());

        if (gate_status != HK_OK) {
            result.error = "hk_gate failed: status " + std::to_string(gate_status);
            hk_destroy(handle);
            return result;
        }

        result.dropped_total += sink_dropped;

        std::size_t pos = 0;
        while (pos < sink_len) {
            std::uint64_t record_len = decode_varint(sink_buf.data(), sink_len, pos);
            if (pos + record_len > sink_len) break;
            const std::uint8_t* record = sink_buf.data() + pos;
            hasher.update(record, record_len);
            if (writer.has_value()) writer->write_event(record, record_len, sample.t_ns);

            if (options.print_summary) {
                harness::v1::EnforcementEvent ev;
                if (ev.ParseFromArray(record, static_cast<int>(record_len))) {
                    double t_ms = static_cast<double>(sample.t_ns - t0_ns) / 1e6;
                    std::printf("%s\n",
                                harness::format::event_row(ev.seq(), t_ms, ev).c_str());
                }
            }
            result.event_count++;
            pos += record_len;
        }
    }

    for (auto& c : critics) c.destroy();
    if (writer.has_value()) writer->close();
    hk_destroy(handle);

    std::sort(gate_times_us.begin(), gate_times_us.end());
    auto percentile = [&](double p) -> double {
        if (gate_times_us.empty()) return 0.0;
        std::size_t idx = static_cast<std::size_t>(p * static_cast<double>(gate_times_us.size() - 1));
        return gate_times_us[idx];
    };
    result.gate_p50_us = percentile(0.50);
    result.gate_p99_us = percentile(0.99);
    result.gate_p999_us = percentile(0.999);

    if (options.print_summary) {
        std::printf("\n%llu cycles \xC2\xB7 %llu events \xC2\xB7 %u dropped\n",
                    static_cast<unsigned long long>(trace.size()),
                    static_cast<unsigned long long>(result.event_count), result.dropped_total);
        std::printf("gate p50 %.1f \xC2\xB5s \xC2\xB7 p99 %.1f \xC2\xB5s \xC2\xB7 p99.9 %.1f \xC2\xB5s   "
                    "[informational -- shared hardware]\n",
                    result.gate_p50_us, result.gate_p99_us, result.gate_p999_us);
        if (options.mcap_out_path.has_value()) {
            std::printf("wrote %s (replay tuple attached)\n", options.mcap_out_path->c_str());
        }
    }

    result.ok = true;
    result.cycles = trace.size();
    result.event_stream_hash = hasher.hex();
    return result;
}

}  // namespace harness::replay
