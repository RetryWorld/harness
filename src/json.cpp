#include "json.hpp"

#include <cctype>
#include <cerrno>
#include <cstdlib>

namespace harness::json {

const Value* Value::find(std::string_view key) const {
    if (type_ != Type::Obj) return nullptr;
    auto it = obj_.find(std::string(key));
    return it == obj_.end() ? nullptr : &it->second;
}

namespace {

class Parser {
public:
    explicit Parser(std::string_view text) : s_(text) {}

    ParseResult run() {
        skip_ws();
        auto v = parse_value(0);
        if (!v) return {std::nullopt, err_};
        skip_ws();
        if (i_ != s_.size()) return fail("trailing content after JSON value");
        return {std::move(v), std::nullopt};
    }

private:
    std::string_view s_;
    std::size_t i_ = 0;
    std::optional<ParseError> err_;

    ParseResult fail(std::string msg) {
        if (!err_) err_ = ParseError{std::move(msg), i_};
        return {std::nullopt, err_};
    }

    std::nullopt_t bad(std::string msg) {
        if (!err_) err_ = ParseError{std::move(msg), i_};
        return std::nullopt;
    }

    [[nodiscard]] bool at_end() const { return i_ >= s_.size(); }
    [[nodiscard]] char peek() const { return at_end() ? '\0' : s_[i_]; }

    void skip_ws() {
        while (!at_end()) {
            const char c = s_[i_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                ++i_;
            } else {
                break;
            }
        }
    }

    bool literal(std::string_view lit) {
        if (s_.substr(i_, lit.size()) != lit) return false;
        i_ += lit.size();
        return true;
    }

    std::optional<Value> parse_value(int depth) {
        if (depth > kMaxDepth) return bad("maximum nesting depth exceeded");
        skip_ws();
        if (at_end()) return bad("unexpected end of input");

        switch (peek()) {
            case '{': return parse_object(depth);
            case '[': return parse_array(depth);
            case '"': {
                auto str = parse_string();
                if (!str) return std::nullopt;
                return Value(std::move(*str));
            }
            case 't':
                if (literal("true")) return Value(true);
                return bad("invalid literal");
            case 'f':
                if (literal("false")) return Value(false);
                return bad("invalid literal");
            case 'n':
                if (literal("null")) return Value();
                return bad("invalid literal");
            default: return parse_number();
        }
    }

    std::optional<Value> parse_object(int depth) {
        ++i_;  // '{'
        Object obj;
        skip_ws();
        if (peek() == '}') { ++i_; return Value(std::move(obj)); }

        while (true) {
            skip_ws();
            if (peek() != '"') return bad("expected object key");
            auto key = parse_string();
            if (!key) return std::nullopt;
            skip_ws();
            if (peek() != ':') return bad("expected ':' after object key");
            ++i_;
            auto val = parse_value(depth + 1);
            if (!val) return std::nullopt;
            // Last-wins on a duplicate key. Silent, but the alternative is
            // rejecting profiles a permissive writer produced; flag it in
            // validation rather than here if it ever matters.
            obj[std::move(*key)] = std::move(*val);
            skip_ws();
            if (peek() == ',') { ++i_; continue; }
            if (peek() == '}') { ++i_; return Value(std::move(obj)); }
            return bad("expected ',' or '}' in object");
        }
    }

    std::optional<Value> parse_array(int depth) {
        ++i_;  // '['
        Array arr;
        skip_ws();
        if (peek() == ']') { ++i_; return Value(std::move(arr)); }

        while (true) {
            auto val = parse_value(depth + 1);
            if (!val) return std::nullopt;
            arr.push_back(std::move(*val));
            skip_ws();
            if (peek() == ',') { ++i_; continue; }
            if (peek() == ']') { ++i_; return Value(std::move(arr)); }
            return bad("expected ',' or ']' in array");
        }
    }

    std::optional<std::string> parse_string() {
        ++i_;  // opening quote
        std::string out;
        while (true) {
            if (at_end()) return bad("unterminated string");
            const char c = s_[i_++];
            if (c == '"') return out;
            if (c != '\\') {
                // Reject raw control characters rather than passing them through
                // into a field that ends up in a log line or a UI.
                if (static_cast<unsigned char>(c) < 0x20) return bad("control character in string");
                out.push_back(c);
                continue;
            }
            if (at_end()) return bad("unterminated escape");
            switch (s_[i_++]) {
                case '"': out.push_back('"'); break;
                case '\\': out.push_back('\\'); break;
                case '/': out.push_back('/'); break;
                case 'b': out.push_back('\b'); break;
                case 'f': out.push_back('\f'); break;
                case 'n': out.push_back('\n'); break;
                case 'r': out.push_back('\r'); break;
                case 't': out.push_back('\t'); break;
                case 'u': {
                    // \uXXXX is accepted syntactically and emitted as '?' unless
                    // ASCII. Profiles are machine-generated identifiers and
                    // numbers; full UTF-16 surrogate handling would be code on a
                    // boot path that nothing needs.
                    if (i_ + 4 > s_.size()) return bad("truncated \\u escape");
                    unsigned code = 0;
                    for (int k = 0; k < 4; ++k) {
                        const char h = s_[i_ + static_cast<std::size_t>(k)];
                        code <<= 4U;
                        if (h >= '0' && h <= '9') code |= static_cast<unsigned>(h - '0');
                        else if (h >= 'a' && h <= 'f') code |= static_cast<unsigned>(h - 'a' + 10);
                        else if (h >= 'A' && h <= 'F') code |= static_cast<unsigned>(h - 'A' + 10);
                        else return bad("invalid hex in \\u escape");
                    }
                    i_ += 4;
                    out.push_back(code < 0x80 ? static_cast<char>(code) : '?');
                    break;
                }
                default: return bad("invalid escape sequence");
            }
        }
    }

    std::optional<Value> parse_number() {
        const std::size_t start = i_;
        if (peek() == '-') ++i_;
        bool any_digit = false;
        while (!at_end() && (std::isdigit(static_cast<unsigned char>(s_[i_])) != 0)) { ++i_; any_digit = true; }
        if (!at_end() && s_[i_] == '.') {
            ++i_;
            while (!at_end() && (std::isdigit(static_cast<unsigned char>(s_[i_])) != 0)) { ++i_; any_digit = true; }
        }
        if (!at_end() && (s_[i_] == 'e' || s_[i_] == 'E')) {
            ++i_;
            if (!at_end() && (s_[i_] == '+' || s_[i_] == '-')) ++i_;
            bool exp_digit = false;
            while (!at_end() && (std::isdigit(static_cast<unsigned char>(s_[i_])) != 0)) { ++i_; exp_digit = true; }
            if (!exp_digit) return bad("malformed exponent");
        }
        if (!any_digit) return bad("malformed number");

        // strtod on a bounded, NUL-terminated copy. Never on the raw view: the
        // input is not guaranteed terminated and strtod would read past the end.
        const std::string tok(s_.substr(start, i_ - start));
        errno = 0;
        char* end = nullptr;
        const double d = std::strtod(tok.c_str(), &end);
        if (end != tok.c_str() + tok.size()) return bad("malformed number");
        if (errno == ERANGE) return bad("number out of range");
        return Value(d);
    }
};

}  // namespace

ParseResult parse(std::string_view text) { return Parser(text).run(); }

}  // namespace harness::json
