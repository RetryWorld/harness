// Device-code authentication for a headless Linux edge computer.

#ifndef HARNESS_CLI_CONNECT_HPP
#define HARNESS_CLI_CONNECT_HPP

#include <string>

namespace harness::cli {

struct ConnectOptions {
    std::string device_name;
    bool json = false;
};

int run_connect(const ConnectOptions& options);

}  // namespace harness::cli

#endif  // HARNESS_CLI_CONNECT_HPP

