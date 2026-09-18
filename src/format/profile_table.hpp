// The Harness Profile, rendered for a human — build brief §4.1's field set in
// one place.
//
// `validate` says whether a profile is legal and `hash` says whether its
// joint order matches, but neither shows what the kernel will actually
// enforce. The interesting failure is not a malformed profile — the validator
// catches that — it is a profile that is valid and wrong: a bound off by a
// decimal place, a Band B threshold left at what the author thought was a
// default, a critic believed to be declared that is not.
//
// This renders the LOADED profile, post-parse, so what it prints is by
// construction the message the kernel is handed. It deliberately does not
// re-read the YAML: a second reading of the authoring format would be a
// second definition of the profile, which is exactly the drift §12 warns
// about.
//
// Presence is printed, not defaulted away. Every `optional` scalar in §4.1
// exists so an omitted bound cannot deserialise as 0.0 (see profile.proto's
// header); an inspector that prints `0` for an absent budget would undo that
// distinction at the moment it matters most. Absent fields print `<unset>`.

#ifndef HARNESS_FORMAT_PROFILE_TABLE_HPP
#define HARNESS_FORMAT_PROFILE_TABLE_HPP

#include <string>

#include "harness/v1/profile.pb.h"

namespace harness::format {

// Multi-line, grouped by enforcement mechanism. `computed_joint_order_hash`
// is passed in rather than computed here so this renderer stays free of the
// profile library: the caller already has it, and printing the declared hash
// beside the computed one is what makes a C8 failure legible.
std::string profile_text(const harness::v1::HarnessProfile& profile,
                         const std::string& computed_joint_order_hash);

// The same field set as one JSON object. Unset optionals are `null` — the
// JSON counterpart of `<unset>`, and distinguishable from a real 0.
std::string profile_json(const harness::v1::HarnessProfile& profile,
                         const std::string& computed_joint_order_hash);

}  // namespace harness::format

#endif  // HARNESS_FORMAT_PROFILE_TABLE_HPP
