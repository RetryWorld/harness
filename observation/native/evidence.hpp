#pragma once
#include "common.hpp"
#include <deque>
#include <optional>
namespace harness::observation {
class Evidence {
public:
  Json config, state;
  std::map<std::string, std::deque<Json>> samples;
  std::optional<Ns> now, last_clock_advance;
  Evidence(Json binding, const std::string &session);
  void tick(Ns stamp, Ns wall = monotonic_ns());
  void sample(const std::string &topic, Ns receipt, Json source, Ns wall,
              bool valid = true);
  void check_live(Ns wall = monotonic_ns()) const;
  void expire(Ns wall = monotonic_ns());
  Json &capture(double before, double after, const std::string &label);
  Json promote(const std::string &window, const std::string &description,
               const std::string &actor);
};
} // namespace harness::observation
