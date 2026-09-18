#include "profile/loader.hpp"

#include <fstream>
#include <sstream>

#include <google/protobuf/util/json_util.h>
#include <yaml-cpp/yaml.h>

namespace harness::profile {

namespace {

void escape_json_string(const std::string& in, std::string& out) {
    out.push_back('"');
    for (char c : in) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out.push_back(c);
                }
        }
    }
    out.push_back('"');
}

// YAML has no concept of "this scalar is a number vs a string" beyond its own
// resolution rules; we lean on yaml-cpp's tag/Type() plus a numeric-parse
// fallback so authored bare tokens like `1.0` or `true` come through as JSON
// literals rather than quoted strings, which is what protobuf's JSON parser
// expects for scalar fields.
void scalar_to_json(const YAML::Node& node, std::string& out) {
    const std::string& tag = node.Tag();
    const std::string& s = node.Scalar();
    if (tag == "!") {
        // Explicitly quoted in the YAML source: always a string.
        escape_json_string(s, out);
        return;
    }
    if (s == "null" || s == "~" || s.empty()) {
        out += "null";
        return;
    }
    if (s == "true" || s == "false") {
        out += s;
        return;
    }
    try {
        size_t consumed = 0;
        double d = std::stod(s, &consumed);
        if (consumed == s.size()) {
            out += s;
            (void)d;
            return;
        }
    } catch (...) {
        // fall through to string
    }
    escape_json_string(s, out);
}

void node_to_json(const YAML::Node& node, std::string& out) {
    switch (node.Type()) {
        case YAML::NodeType::Null:
            out += "null";
            return;
        case YAML::NodeType::Scalar:
            scalar_to_json(node, out);
            return;
        case YAML::NodeType::Sequence: {
            out.push_back('[');
            bool first = true;
            for (const auto& child : node) {
                if (!first) out.push_back(',');
                first = false;
                node_to_json(child, out);
            }
            out.push_back(']');
            return;
        }
        case YAML::NodeType::Map: {
            out.push_back('{');
            bool first = true;
            for (const auto& kv : node) {
                if (!first) out.push_back(',');
                first = false;
                escape_json_string(kv.first.Scalar(), out);
                out.push_back(':');
                node_to_json(kv.second, out);
            }
            out.push_back('}');
            return;
        }
        case YAML::NodeType::Undefined:
        default:
            out += "null";
            return;
    }
}

}  // namespace

LoadResult load_from_yaml(std::string_view yaml_text) noexcept {
    LoadResult result;
    try {
        YAML::Node root = YAML::Load(std::string(yaml_text));
        std::string json;
        node_to_json(root, json);

        harness::v1::HarnessProfile msg;
        google::protobuf::util::JsonParseOptions opts;
        opts.ignore_unknown_fields = false;
        auto status = google::protobuf::util::JsonStringToMessage(json, &msg, opts);
        if (!status.ok()) {
            result.error = std::string(status.message());
            return result;
        }
        result.profile = std::move(msg);
        return result;
    } catch (const std::exception& e) {
        result.error = std::string("yaml parse error: ") + e.what();
        return result;
    } catch (...) {
        result.error = "yaml parse error: unknown exception";
        return result;
    }
}

LoadResult load_from_yaml_file(const std::string& path) noexcept {
    try {
        std::ifstream f(path);
        if (!f) {
            LoadResult result;
            result.error = "cannot open file: " + path;
            return result;
        }
        std::ostringstream buf;
        buf << f.rdbuf();
        return load_from_yaml(buf.str());
    } catch (const std::exception& e) {
        LoadResult result;
        result.error = std::string("io error: ") + e.what();
        return result;
    }
}

}  // namespace harness::profile
