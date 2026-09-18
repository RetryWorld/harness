#include "replay/json_lite.hpp"

#include <cctype>
#include <cstdlib>

namespace harness::replay::json {

namespace {

class Parser {
public:
    explicit Parser(std::string_view text) : text_(text) {}

    std::optional<Value> parse_value() {
        skip_ws();
        if (pos_ >= text_.size()) return std::nullopt;
        char c = text_[pos_];
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') return parse_string_value();
        if (c == 't' || c == 'f') return parse_bool();
        if (c == 'n') return parse_null();
        return parse_number();
    }

private:
    std::string_view text_;
    std::size_t pos_ = 0;

    void skip_ws() {
        while (pos_ < text_.size() &&
               (text_[pos_] == ' ' || text_[pos_] == '\t' || text_[pos_] == '\n' || text_[pos_] == '\r')) {
            ++pos_;
        }
    }

    std::optional<std::string> parse_raw_string() {
        if (pos_ >= text_.size() || text_[pos_] != '"') return std::nullopt;
        ++pos_;
        std::string out;
        while (pos_ < text_.size() && text_[pos_] != '"') {
            char c = text_[pos_++];
            if (c == '\\' && pos_ < text_.size()) {
                char esc = text_[pos_++];
                switch (esc) {
                    case 'n': out.push_back('\n'); break;
                    case 't': out.push_back('\t'); break;
                    case 'r': out.push_back('\r'); break;
                    case '"': out.push_back('"'); break;
                    case '\\': out.push_back('\\'); break;
                    case '/': out.push_back('/'); break;
                    default: out.push_back(esc); break;
                }
            } else {
                out.push_back(c);
            }
        }
        if (pos_ >= text_.size()) return std::nullopt;
        ++pos_;  // closing quote
        return out;
    }

    std::optional<Value> parse_string_value() {
        auto s = parse_raw_string();
        if (!s.has_value()) return std::nullopt;
        Value v;
        v.kind = Value::Kind::String;
        v.str = std::move(*s);
        return v;
    }

    std::optional<Value> parse_bool() {
        if (text_.substr(pos_, 4) == "true") {
            pos_ += 4;
            Value v;
            v.kind = Value::Kind::Bool;
            v.b = true;
            return v;
        }
        if (text_.substr(pos_, 5) == "false") {
            pos_ += 5;
            Value v;
            v.kind = Value::Kind::Bool;
            v.b = false;
            return v;
        }
        return std::nullopt;
    }

    std::optional<Value> parse_null() {
        if (text_.substr(pos_, 4) == "null") {
            pos_ += 4;
            return Value{};
        }
        return std::nullopt;
    }

    std::optional<Value> parse_number() {
        std::size_t start = pos_;
        while (pos_ < text_.size() &&
               (std::isdigit(static_cast<unsigned char>(text_[pos_])) || text_[pos_] == '-' ||
                text_[pos_] == '+' || text_[pos_] == '.' || text_[pos_] == 'e' || text_[pos_] == 'E')) {
            ++pos_;
        }
        if (pos_ == start) return std::nullopt;
        std::string token(text_.substr(start, pos_ - start));
        Value v;
        v.kind = Value::Kind::Number;
        v.num = std::strtod(token.c_str(), nullptr);
        return v;
    }

    std::optional<Value> parse_array() {
        ++pos_;  // '['
        Value v;
        v.kind = Value::Kind::Array;
        skip_ws();
        if (pos_ < text_.size() && text_[pos_] == ']') {
            ++pos_;
            return v;
        }
        while (true) {
            auto elem = parse_value();
            if (!elem.has_value()) return std::nullopt;
            v.arr.push_back(std::move(*elem));
            skip_ws();
            if (pos_ >= text_.size()) return std::nullopt;
            if (text_[pos_] == ',') {
                ++pos_;
                skip_ws();
                continue;
            }
            if (text_[pos_] == ']') {
                ++pos_;
                break;
            }
            return std::nullopt;
        }
        return v;
    }

    std::optional<Value> parse_object() {
        ++pos_;  // '{'
        Value v;
        v.kind = Value::Kind::Object;
        skip_ws();
        if (pos_ < text_.size() && text_[pos_] == '}') {
            ++pos_;
            return v;
        }
        while (true) {
            skip_ws();
            auto key = parse_raw_string();
            if (!key.has_value()) return std::nullopt;
            skip_ws();
            if (pos_ >= text_.size() || text_[pos_] != ':') return std::nullopt;
            ++pos_;
            auto val = parse_value();
            if (!val.has_value()) return std::nullopt;
            v.obj.emplace_back(std::move(*key), std::move(*val));
            skip_ws();
            if (pos_ >= text_.size()) return std::nullopt;
            if (text_[pos_] == ',') {
                ++pos_;
                continue;
            }
            if (text_[pos_] == '}') {
                ++pos_;
                break;
            }
            return std::nullopt;
        }
        return v;
    }
};

}  // namespace

std::optional<Value> parse(std::string_view text) {
    Parser p(text);
    return p.parse_value();
}

}  // namespace harness::replay::json
