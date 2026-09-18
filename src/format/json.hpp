// Minimal JSON emission, in one place.
//
// These helpers were local to tools/harness_cli/main.cpp until `show` needed
// to emit a profile as JSON. Rendering the profile's field set twice — once
// as a table, once as JSON — in two different files is the drift build brief
// §12 warns about, so the profile renderer emits both and needs the escaper
// here rather than in the CLI.
//
// Hand-rolled for the same reason edge/'s JSON parser is (see edge/README.md):
// this code ships in the release tarball and must build standalone and
// offline, and the shapes emitted are small, fixed, and ours.

#ifndef HARNESS_FORMAT_JSON_HPP
#define HARNESS_FORMAT_JSON_HPP

#include <string>
#include <vector>

namespace harness::format::json {

// Escapes the characters JSON requires, and control characters as \u00xx.
std::string escape(const std::string& in);

// `escape`d and wrapped in quotes.
std::string str(const std::string& in);

std::string boolean(bool b);

// A double as a JSON number. Non-finite values are emitted as null: JSON has
// no NaN or Infinity literal, and a parser that accepts one is not reading
// JSON. A NaN bound is a real thing a profile can carry (see the NaN
// characterization test in edge/ros2/harness_controller/test/), so this case
// is reachable, not theoretical.
std::string number(double v);

std::string str_array(const std::vector<std::string>& items);

}  // namespace harness::format::json

#endif  // HARNESS_FORMAT_JSON_HPP
