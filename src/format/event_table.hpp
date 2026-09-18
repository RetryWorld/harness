// The build brief §10 evidence table, in one place.
//
// §12 non-goals names "building a viewer" as out of scope precisely because
// this table IS the viewer — and §12 also warns that a second definition of a
// generated message "will appear somewhere, usually in the CLI, and it will
// drift". Both the replay rig (writing events as they happen) and
// `rearguard verify` (reading them back out of an MCAP) render the same
// rows, so the rendering lives here rather than in either caller.
//
// These functions RETURN strings rather than printing them: `verify` needs to
// count and inspect rows, and a test needs to assert on one. Nothing here is
// on the decision path, so std::string is free.

#ifndef HARNESS_FORMAT_EVENT_TABLE_HPP
#define HARNESS_FORMAT_EVENT_TABLE_HPP

#include <cstdint>
#include <string>

#include "harness/v1/enforcement.pb.h"

namespace harness::format {

// "seq    t_ms     event                 detail"
std::string table_header();

// The seq-0 row. Rendered from the EpisodeHeader message itself — not from
// the profile it was built from — so that what `replay` prints and what
// `verify` reads back out of the file cannot disagree.
std::string episode_header_row(const harness::v1::EpisodeHeader& header);

// One event row. `t_ms` is relative to the episode's first log time.
std::string event_row(std::uint64_t seq, double t_ms,
                      const harness::v1::EnforcementEvent& event);

}  // namespace harness::format

#endif  // HARNESS_FORMAT_EVENT_TABLE_HPP
