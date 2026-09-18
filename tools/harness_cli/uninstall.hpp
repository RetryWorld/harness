// `rearguard uninstall` — removes an installation created by
// scripts/install.sh.
//
// This lives in the CLI rather than in a scripts/uninstall.sh because
// scripts/ is NOT part of the release (see CMakeLists.txt's install(TARGETS)
// and install(DIRECTORY) rules): a user who installed with `curl | sh` has
// rearguard on their PATH and no shell script to run. The uninstaller has
// to be the thing they already have.
//
// It undoes exactly what install.sh does and nothing else -- the four
// directories it unpacks, and the PATH block it appends to a shell rc file.
// The two must be read side by side; if install.sh changes what it creates,
// this changes with it.

#ifndef HARNESS_CLI_UNINSTALL_HPP
#define HARNESS_CLI_UNINSTALL_HPP

#include <string>

namespace harness::cli {

struct UninstallOptions {
    // Install root. Empty means "derive it from where this binary is running
    // from", which is what makes `rearguard uninstall` work with no
    // arguments regardless of where the user installed it.
    std::string prefix;
    bool dry_run = false;
    bool assume_yes = false;
    bool json = false;
};

int run_uninstall(const UninstallOptions& options);

}  // namespace harness::cli

#endif  // HARNESS_CLI_UNINSTALL_HPP
