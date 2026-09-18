#include "format/event_table.hpp"

#include <cstdio>

namespace harness::format {

namespace {

constexpr std::size_t kEventColumnWidth = 20;

// Padded rather than "%-20s" so an event name longer than the column pushes
// the detail across instead of being silently truncated. Losing part of an
// event name in a safety log is the wrong failure mode; a ragged row is not.
std::string row(const std::string& seq, const std::string& t_ms, const std::string& event,
                const std::string& detail) {
    std::string out;
    out.reserve(48 + event.size() + detail.size());
    out += seq;
    if (seq.size() < 6) out.append(6 - seq.size(), ' ');
    out += ' ';
    if (t_ms.size() < 8) out.append(8 - t_ms.size(), ' ');
    out += t_ms;
    out += "   ";
    out += event;
    if (event.size() < kEventColumnWidth) out.append(kEventColumnWidth - event.size(), ' ');
    out += "  ";
    out += detail;
    return out;
}

std::string fixed1(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f", v);
    return std::string(buf);
}

std::string u64(std::uint64_t v) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(v));
    return std::string(buf);
}

// The joint-order hash is 16 hex chars; 8 is enough to tell two apart at a
// glance, and `verify --json` (and the header record itself) still carry it
// in full.
std::string short_hash(const std::string& h) { return h.substr(0, 8); }

}  // namespace

std::string table_header() { return row("seq", "t_ms", "event", "detail"); }

std::string episode_header_row(const harness::v1::EpisodeHeader& header) {
    std::string detail = "joints=" + std::to_string(header.joint_names_size()) +
                         " order=" + short_hash(header.joint_order_hash()) +
                         " abi=" + std::to_string(header.kernel_abi_version()) +
                         " profile=" + header.profile_hash() +
                         " clock=" + harness::v1::ClockDomain_Name(header.clock_domain());
    return row("0", "0.0", "EpisodeHeader", detail);
}

std::string event_row(std::uint64_t seq, double t_ms,
                      const harness::v1::EnforcementEvent& event) {
    std::string name;
    std::string detail;
    switch (event.body_case()) {
        case harness::v1::EnforcementEvent::kProjection: {
            const auto& p = event.projection();
            name = "ProjectionEvent";
            detail = harness::v1::Verdict_Name(p.verdict()) +
                     std::string(" margin=") + std::to_string(p.margin());
            break;
        }
        case harness::v1::EnforcementEvent::kIsolation: {
            const auto& iso = event.isolation();
            name = "IsolationEvent";
            detail = std::string("elapsed=") + std::to_string(iso.inference_ms()) +
                     "ms budget=" + std::to_string(iso.wcet_budget_ms()) +
                     "ms overrun=" + (iso.compute_overrun() ? "true" : "false");
            break;
        }
        case harness::v1::EnforcementEvent::kTransfer: {
            const auto& t = event.transfer();
            name = "TransferEvent";
            detail = harness::v1::Band_Name(t.from_band()) + " -> " +
                     harness::v1::Band_Name(t.to_band());
            if (!t.recovery_ref().empty()) detail += "  recovery=" + t.recovery_ref();
            if (!t.trigger().empty()) detail += "  trigger=" + t.trigger();
            break;
        }
        case harness::v1::EnforcementEvent::kCritic: {
            const auto& c = event.critic();
            name = "CriticEvent";
            detail = c.critic_id() + "  " + c.evidence_class() +
                     "  conf=" + std::to_string(c.confidence());
            break;
        }
        case harness::v1::EnforcementEvent::kReentry: {
            const auto& r = event.reentry();
            name = "ReentryEvent";
            detail = r.recovery_ref() + "  predicate=" + (r.predicate_result() ? "PASS" : "FAIL") +
                     "  retry=" + std::to_string(r.retry_index());
            break;
        }
        default:
            // Not an error here: a decoder older than the writer sees an
            // unset oneof for an event type added later. Naming it and
            // carrying on is the compatibility rule enforcement.proto states.
            name = "UnknownEvent";
            break;
    }

    // A dropped-event count belongs on every row that reports one — it says
    // this row is preceded by evidence that no longer exists.
    if (event.dropped_since_last() > 0) {
        detail += "  [dropped_since_last=" + std::to_string(event.dropped_since_last()) + "]";
    }

    return row(u64(seq), fixed1(t_ms), name, detail);
}

}  // namespace harness::format
