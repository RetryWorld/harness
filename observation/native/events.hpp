#pragma once

#include "common.hpp"

namespace harness::observation {

fs::path state_root();
void record_event(const std::string &component, const std::string &action,
                  const std::string &status, const Json &detail = Json::object());
Json read_events(Ns after = 0);
int events_main(int argc, char **argv);

} // namespace harness::observation
