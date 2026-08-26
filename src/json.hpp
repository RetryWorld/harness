// Minimal JSON reader for the Harness Profile. No third-party dependencies.
//
// # Why not nlohmann/json
//
// Three reasons, in order of weight:
//
//  1. A customer building the enforcement path from source, or an auditor doing
//     the same, must not need a package manager or network access. `edge/`
//     builds offline from a bare checkout; that property is tested in CI.
//  2. Zero third-party code in the safety artifact. The whole reason we ship a
//     compiled interlock is that a reviewer can read what runs.
//  3. The profile schema is small, fixed, and ours. A general JSON library
//     solves a much larger problem than we have.
//
// # Scope and safety
//
// This parses a bounded subset: objects, arrays, strings, numbers, booleans,
// null. No unicode escapes beyond the basic ones, no big-number handling, no
// comments. Recursion is depth-limited, every index is bounds-checked, and the
// parser never allocates based on a length field read from the input.
//
// The profile is a deployment artifact from our own registry rather than
// untrusted network input, but a parser that runs at boot on a robot is worth
// being paranoid about regardless. Errors are values, never exceptions crossing
// into the C ABI.

#ifndef HARNESS_JSON_HPP
#define HARNESS_JSON_HPP

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace harness::json {

class Value;
using Object = std::map<std::string, Value>;
using Array = std::vector<Value>;

// Enumerator names differ from the Object/Array aliases above deliberately:
// -Wshadow is an error in this build and reusing the names is a real ambiguity.
enum class Type { Null, Bool, Number, Str, Obj, Arr };

class Value {
public:
    Value() = default;
    explicit Value(bool b) : type_(Type::Bool), bool_(b) {}
    explicit Value(double d) : type_(Type::Number), num_(d) {}
    explicit Value(std::string s) : type_(Type::Str), str_(std::move(s)) {}
    explicit Value(Object o) : type_(Type::Obj), obj_(std::move(o)) {}
    explicit Value(Array a) : type_(Type::Arr), arr_(std::move(a)) {}

    [[nodiscard]] Type type() const { return type_; }
    [[nodiscard]] bool is_object() const { return type_ == Type::Obj; }
    [[nodiscard]] bool is_array() const { return type_ == Type::Arr; }
    [[nodiscard]] bool is_number() const { return type_ == Type::Number; }
    [[nodiscard]] bool is_string() const { return type_ == Type::Str; }

    [[nodiscard]] double number() const { return num_; }
    [[nodiscard]] bool boolean() const { return bool_; }
    [[nodiscard]] const std::string& string() const { return str_; }
    [[nodiscard]] const Array& array() const { return arr_; }
    [[nodiscard]] const Object& object() const { return obj_; }

    // Lookup returning nullptr rather than throwing or default-constructing:
    // a missing field must be distinguishable from a field set to zero, because
    // "no inference budget declared" and "inference budget of 0ms" are very
    // different profiles and only one of them is a mistake.
    [[nodiscard]] const Value* find(std::string_view key) const;

private:
    Type type_ = Type::Null;
    bool bool_ = false;
    double num_ = 0.0;
    std::string str_;
    Object obj_;
    Array arr_;
};

struct ParseError {
    std::string message;
    std::size_t offset = 0;
};

struct ParseResult {
    std::optional<Value> value;
    std::optional<ParseError> error;

    [[nodiscard]] bool ok() const { return value.has_value(); }
};

// Depth limit. A profile is three levels deep; anything approaching this is
// malformed or hostile, and either way should be rejected rather than recursed.
inline constexpr int kMaxDepth = 32;

ParseResult parse(std::string_view text);

}  // namespace harness::json

#endif  // HARNESS_JSON_HPP
