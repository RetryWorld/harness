#pragma once

#include "common.hpp"

namespace harness::observation {

void set_service_executable(const fs::path &path);
Json service_command(const Options &options);
Json start_service_after_connect();
Json stop_service_if_running();

} // namespace harness::observation
