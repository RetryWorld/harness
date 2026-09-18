// YAML -> JSON -> protobuf loader for the Harness Profile.
//
// Authoring format is YAML; the kernel and validator both consume the
// deserialised harness::v1::HarnessProfile. Nothing on the kernel's hot path
// parses YAML — this loader runs at profile-load time only.
//
// ignore_unknown_fields is false: a misspelled key is a load error, not a
// silently dropped field.

#ifndef HARNESS_PROFILE_LOADER_HPP
#define HARNESS_PROFILE_LOADER_HPP

#include <optional>
#include <string>
#include <string_view>

#include "harness/v1/profile.pb.h"

namespace harness::profile {

struct LoadResult {
    std::optional<harness::v1::HarnessProfile> profile;
    std::string error;  // empty iff profile has a value
};

// Parses `yaml_text` as a Harness Profile. Never throws.
LoadResult load_from_yaml(std::string_view yaml_text) noexcept;

// Convenience: reads `path` and loads it.
LoadResult load_from_yaml_file(const std::string& path) noexcept;

}  // namespace harness::profile

#endif  // HARNESS_PROFILE_LOADER_HPP
