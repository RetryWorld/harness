#include "replay/trace.hpp"

#include <fstream>
#include <sstream>

#include "replay/json_lite.hpp"

namespace harness::replay {

namespace {

std::vector<double> read_double_array(const json::Value* v) {
    std::vector<double> out;
    if (v == nullptr || v->kind != json::Value::Kind::Array) return out;
    out.reserve(v->arr.size());
    for (const auto& e : v->arr) out.push_back(e.num);
    return out;
}

}  // namespace

TraceLoadResult load_trace_file(const std::string& path) {
    TraceLoadResult result;
    std::ifstream f(path);
    if (!f) {
        result.error = "cannot open trace file: " + path;
        return result;
    }

    std::vector<TraceSample> samples;
    std::string line;
    std::size_t line_no = 0;
    while (std::getline(f, line)) {
        ++line_no;
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;

        auto parsed = json::parse(line);
        if (!parsed.has_value() || parsed->kind != json::Value::Kind::Object) {
            result.error = "trace line " + std::to_string(line_no) + ": invalid JSON";
            return result;
        }

        TraceSample s;
        if (const auto* t = parsed->find("t_ns")) s.t_ns = static_cast<std::uint64_t>(t->num);
        s.effort = read_double_array(parsed->find("effort"));
        s.position = read_double_array(parsed->find("position"));
        s.velocity = read_double_array(parsed->find("velocity"));
        if (const auto* aux = parsed->find("aux"); aux != nullptr && aux->kind == json::Value::Kind::Object) {
            for (const auto& [k, v] : aux->obj) s.aux.emplace_back(k, v.num);
        }
        samples.push_back(std::move(s));
    }

    result.samples = std::move(samples);
    return result;
}

}  // namespace harness::replay
