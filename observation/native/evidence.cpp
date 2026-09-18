#include "evidence.hpp"
#include <cmath>
namespace harness::observation {
Evidence::Evidence(Json binding, const std::string &session)
    : config(std::move(binding)) {
  state = {{"schema_version", 1},
           {"session_id", session},
           {"mode", "observation_only"},
           {"config_sha256", digest(config)},
           {"epoch", 0},
           {"windows", Json::array()},
           {"failures", Json::array()},
           {"proposals", Json::array()},
           {"status", "starting"}};
  for (const auto &t : config["topics"])
    samples[t["name"].get<std::string>()] = {};
}
void Evidence::tick(Ns stamp, Ns wall) {
  if (!now || *now != stamp)
    last_clock_advance = wall;
  if (now && stamp < *now) {
    state["epoch"] = state["epoch"].get<Ns>() + 1;
    for (auto &[topic, history] : samples) {
      (void)topic;
      history.clear();
    }
    for (auto &w : state["windows"])
      if (w["status"] == "collecting")
        w["status"] = "clock_reset";
  }
  now = stamp;
  for (auto &w : state["windows"])
    if (w["status"] == "collecting" && stamp >= w["end_ns"].get<Ns>()) {
      Json coverage = Json::object();
      for (const auto &[topic, history] : samples) {
        Ns count = 0, valid = 0;
        Json first = nullptr, last = nullptr;
        for (const auto &s : history)
          if (s["receipt_ros_ns"].get<Ns>() >= w["start_ns"].get<Ns>() &&
              s["receipt_ros_ns"].get<Ns>() <= w["end_ns"].get<Ns>()) {
            if (count == 0)
              first = s;
            last = s;
            ++count;
            if (s["valid"].get<bool>())
              ++valid;
          }
        coverage[topic] = {
            {"count", count},
            {"valid_count", valid},
            {"first", first},
            {"last", last},
            {"observed_span_ns", count ? last["receipt_ros_ns"].get<Ns>() -
                                             first["receipt_ros_ns"].get<Ns>()
                                       : 0}};
      }
      w.update({{"status", "ready"}, {"coverage", coverage}});
    }
}
void Evidence::sample(const std::string &topic, Ns receipt, Json source,
                      Ns wall, bool valid) {
  auto &history = samples.at(topic);
  history.push_back({{"receipt_ros_ns", receipt},
                     {"source_ns", source},
                     {"receipt_monotonic_ns", wall},
                     {"valid", valid}});
  while (!history.empty() && (history.size() > 20000 ||
                              history.front()["receipt_ros_ns"].get<Ns>() <
                                  receipt - 60000000000LL))
    history.pop_front();
}
void Evidence::check_live(Ns wall) const {
  require(now && *now > 0 && last_clock_advance, "ROS clock has not started");
  require(wall - *last_clock_advance <= 10000000000LL,
          "ROS clock stopped advancing; capture refused");
  for (const auto &t : config["topics"])
    if (t.value("required", true)) {
      const auto name = t["name"].get<std::string>();
      const auto &history = samples.at(name);
      require(!history.empty() && history.back()["valid"].get<bool>() &&
                  wall - history.back()["receipt_monotonic_ns"].get<Ns>() <=
                      10000000000LL,
              "required topic missing, invalid or stale: " + name);
      const auto &source = history.back()["source_ns"];
      if (!source.is_null())
        require(*now - source.get<Ns>() >= -1000000000LL &&
                    *now - source.get<Ns>() <= 5000000000LL,
                "required topic source timestamp stale or ahead: " + name);
    }
}
void Evidence::expire(Ns wall) {
  if (last_clock_advance && wall - *last_clock_advance > 15000000000LL)
    for (auto &w : state["windows"])
      if (w["status"] == "collecting")
        w.update(
            {{"status", "incomplete"},
             {"reason", "ROS clock stopped before post-window completed"}});
}
Json &Evidence::capture(double before, double after, const std::string &label) {
  require(now && *now > 0, "ROS clock has not started");
  require(std::isfinite(before) && std::isfinite(after) && before > 0 &&
              before <= 20 && after > 0 && after <= 20,
          "before and after must be finite, positive and at most 20 seconds");
  unsigned collecting = 0;
  for (const auto &w : state["windows"])
    if (w["status"] == "collecting")
      ++collecting;
  require(collecting < 8, "at most eight windows may collect concurrently");
  state["windows"].push_back(
      {{"id", "w-" + unique_id().substr(0, 12)},
       {"epoch", state["epoch"]},
       {"start_ns", std::max<Ns>(0, *now - static_cast<Ns>(before * 1e9))},
       {"mark_ns", *now},
       {"end_ns", *now + static_cast<Ns>(after * 1e9)},
       {"label", label},
       {"status", "collecting"},
       {"trigger", "operator_mark"},
       {"evidence", "rosbag2"},
       {"clock", config["clock"]}});
  return state["windows"].back();
}
Json Evidence::promote(const std::string &id, const std::string &description,
                       const std::string &actor) {
  required_text(description, "description");
  required_text(actor, "operator");
  Json *window = nullptr;
  for (auto &w : state["windows"])
    if (w["id"] == id)
      window = &w;
  require(window && (*window)["status"] == "ready",
          "window must exist and finish collecting before promotion");
  for (const auto &f : state["failures"])
    if (f["window_id"] == id)
      return f;
  for (const auto &t : config["topics"])
    if (t.value("required", true))
      require((*window)["coverage"]
                      .at(t["name"].get<std::string>())["valid_count"]
                      .get<Ns>() >= 2,
              "insufficient valid observations in window");
  Json failure = {{"id", "f-" + unique_id().substr(0, 12)},
                  {"window_id", id},
                  {"description", description},
                  {"operator", actor},
                  {"promoted_at_ros_ns", *now},
                  {"classification_source", "operator"},
                  {"status", "confirmed_by_operator"}};
  state["failures"].push_back(failure);
  for (const auto *kind : {"critic", "recovery"})
    state["proposals"].push_back(
        {{"id", "p-" + unique_id().substr(0, 12)},
         {"kind", kind},
         {"failure_id", failure["id"]},
         {"window_id", id},
         {"status", "pending"},
         {"failure_description", description},
         {"draft",
          std::string(kind) == "critic"
              ? "Define a measurable predicate using the recorded sensor and "
                "action window; validate on successful and failed trials."
              : "Specify response, entry conditions, completion predicate and "
                "timeout; validate before execution."},
         {"generator", "review_template_v1"},
         {"executable", false},
         {"acceptance_enabled", false}});
  return failure;
}
} // namespace harness::observation
