// Small recursive-descent JSON parser for trace lines only (build brief §8:
// JSONL, one control cycle per line, hand-editable on purpose). Not a
// general-purpose JSON library — no comments, no trailing commas, no
// streaming. Off the hot path (trace loading happens before replay starts),
// so ordinary allocation is fine here.

#ifndef HARNESS_REPLAY_JSON_LITE_HPP
#define HARNESS_REPLAY_JSON_LITE_HPP

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace harness::replay::json {

struct Value {
    enum class Kind { Null, Bool, Number, String, Array, Object };
    Kind kind = Kind::Null;
    bool b = false;
    double num = 0.0;
    std::string str;
    std::vector<Value> arr;
    std::vector<std::pair<std::string, Value>> obj;

    const Value* find(std::string_view key) const {
        for (const auto& [k, v] : obj) {
            if (k == key) return &v;
        }
        return nullptr;
    }
};

std::optional<Value> parse(std::string_view text);

}  // namespace harness::replay::json

#endif  // HARNESS_REPLAY_JSON_LITE_HPP
