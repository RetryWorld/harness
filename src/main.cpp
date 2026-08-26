// Kernel daemon entry point.
//
// Phase 1 scope is deliberately narrow: report build provenance and validate a
// profile. The ROS 2 binding is a separate rclcpp controller plugin that dlopens
// the shared library — the kernel itself links no ROS, which is what keeps it
// unit-testable without a ROS install and keeps the ROS binding's language
// choice independent of this one.
//
// `--version` output is not cosmetic: it is what the simulation rig records into
// the replay tuple, and what an auditor reads off a deployed robot to check the
// binary matches the profile's stated provenance.

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "harness_kernel.h"
#include "kernel.hpp"
#include "profile.hpp"

namespace {

int usage() {
    std::cerr << "usage: harness-kernel [--version | --abi | validate <profile.json>]\n";
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv, argv + argc);
    if (args.size() < 2) return usage();

    if (args[1] == "--version" || args[1] == "-V") {
        std::cout << "harness-kernel " << harness::kKernelVersion << "\n"
                  << "schema " << harness::kSchemaRevision << "\n"
                  << "commit " << harness::kBuildCommit << "\n"
                  << "abi " << HARNESS_ABI_VERSION << "\n";
        return 0;
    }

    // Read by scripts/build_artifact.sh to confirm the header and the built
    // library agree. A header change without an ABI bump is how a consumer
    // silently misreads the envelope.
    if (args[1] == "--abi") {
        std::cout << HARNESS_ABI_VERSION << "\n";
        return 0;
    }

    if (args[1] == "validate") {
        if (args.size() < 3) return usage();
        std::ifstream in(args[2]);
        if (!in) {
            std::cerr << "cannot read " << args[2] << "\n";
            return 1;
        }
        std::ostringstream buf;
        buf << in.rdbuf();
        harness::ProfileParseResult r = harness::parse_profile(buf.str());
        if (!r.profile.has_value()) {
            std::cerr << "invalid profile: " << r.error << "\n";
            return 1;
        }
        std::cout << "ok " << r.profile->profile_version << " ("
                  << r.profile->output_region.lo.size() << " actuators)\n";
        return 0;
    }

    return usage();
}
