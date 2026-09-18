#include "format/json.hpp"

#include <cmath>
#include <cstdio>

namespace harness::format::json {

std::string escape(const std::string& in) {
    std::string out;
    out.reserve(in.size() + 2);
    for (char c : in) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[7];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

std::string str(const std::string& in) { return "\"" + escape(in) + "\""; }

std::string boolean(bool b) { return b ? "true" : "false"; }

std::string number(double v) {
    if (!std::isfinite(v)) return "null";
    char buf[40];
    // %.17g round-trips an IEEE-754 double exactly. A bound printed to fewer
    // digits than it was authored with is a different bound.
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    return std::string(buf);
}

std::string str_array(const std::vector<std::string>& items) {
    std::string out = "[";
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (i > 0) out += ",";
        out += str(items[i]);
    }
    return out + "]";
}

}  // namespace harness::format::json
