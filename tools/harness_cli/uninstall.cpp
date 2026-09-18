#include "uninstall.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <system_error>
#include <vector>

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

namespace harness::cli {

namespace fs = std::filesystem;

namespace {

// The exact comment install.sh writes above the PATH line. Matched literally:
// a line this uninstaller did not put there is a line it must not remove.
constexpr const char* kRcMarker = "# added by harness-kernel install.sh";

// The four directories the tarball unpacks. Nothing outside this list is ever
// touched, so a wrong --prefix removes nothing rather than something
// catastrophic.
constexpr const char* kInstalledDirs[] = {"bin", "lib", "include", "share"};

std::string escape_json(const std::string& in) {
    std::string out;
    for (char c : in) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (c == '\n') {
            out += "\\n";
        } else {
            out += c;
        }
    }
    return out;
}

// Where this binary actually lives. Deriving the prefix from the running
// executable (rather than defaulting to ~/.harness) means uninstall removes
// the installation the user is actually running, including one installed with
// HARNESS_INSTALL_DIR set to somewhere else entirely.
std::string executable_path() {
#if defined(__APPLE__)
    std::uint32_t size = 0;
    _NSGetExecutablePath(nullptr, &size);
    std::string buf(size, '\0');
    if (_NSGetExecutablePath(buf.data(), &size) != 0) return {};
    buf.resize(std::char_traits<char>::length(buf.c_str()));
    std::error_code ec;
    fs::path resolved = fs::canonical(buf, ec);
    return ec ? buf : resolved.string();
#elif defined(__linux__)
    std::error_code ec;
    fs::path resolved = fs::canonical("/proc/self/exe", ec);
    return ec ? std::string{} : resolved.string();
#else
    return {};
#endif
}

// An install tree looks like <prefix>/{bin/rearguard, lib/libharness_kernel.*}.
// Requiring both means `uninstall --prefix ~` or a typo'd path is refused
// before anything is deleted, rather than after.
bool looks_like_install_root(const fs::path& prefix, std::string& why_not) {
    std::error_code ec;
    if (!fs::exists(prefix, ec)) {
        why_not = "no such directory — already uninstalled?";
        return false;
    }
    if (!fs::is_directory(prefix, ec)) {
        why_not = "not a directory";
        return false;
    }
    if (!fs::exists(prefix / "bin" / "rearguard", ec)) {
        why_not = "no bin/rearguard here";
        return false;
    }
    bool found_lib = false;
    for (fs::directory_iterator it(prefix / "lib", ec), end; !ec && it != end; it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (name.rfind("libharness_kernel", 0) == 0) {
            found_lib = true;
            break;
        }
    }
    if (!found_lib) {
        why_not = "no lib/libharness_kernel.* here";
        return false;
    }
    return true;
}

std::string home_dir() {
    const char* home = std::getenv("HOME");
    return (home != nullptr) ? std::string(home) : std::string();
}

// Does this export line name <prefix>/bin?
//
// A literal string compare is not enough: the prefix is canonicalised from the
// running binary's path, so an install under a symlinked directory (/tmp ->
// /private/tmp on macOS, or a symlinked $HOME) yields a prefix that never
// textually matches what install.sh wrote. Comparing the resolved paths is
// what makes the PATH cleanup actually fire in those cases instead of
// silently leaving a stale entry behind.
bool line_names_bin_dir(const std::string& line, const fs::path& bin_dir) {
    if (line.find(bin_dir.string()) != std::string::npos) return true;

    // export PATH="<dir>/bin:$PATH"  ->  <dir>/bin
    const std::string needle = "PATH=\"";
    const std::size_t start = line.find(needle);
    if (start == std::string::npos) return false;
    const std::size_t value_start = start + needle.size();
    const std::size_t quote = line.find('"', value_start);
    if (quote == std::string::npos) return false;
    std::string value = line.substr(value_start, quote - value_start);
    const std::size_t colon = value.find(':');
    if (colon != std::string::npos) value = value.substr(0, colon);
    if (value.empty()) return false;

    std::error_code ec;
    const fs::path resolved = fs::weakly_canonical(value, ec);
    if (ec) return false;
    const fs::path target = fs::weakly_canonical(bin_dir, ec);
    if (ec) return false;
    return resolved == target;
}

struct RcHit {
    fs::path file;
    std::string line;  // the PATH line itself, for reporting
};

// Finds the block install.sh appended: the marker comment followed by an
// export line naming <prefix>/bin. Both must be present and adjacent -- a
// stray marker with someone else's PATH line underneath is left alone.
std::vector<RcHit> find_rc_entries(const fs::path& prefix) {
    std::vector<RcHit> hits;
    const std::string home = home_dir();
    if (home.empty()) return hits;

    const std::string bin_dir = (prefix / "bin").string();
    for (const char* rc : {".zshrc", ".bashrc", ".profile", ".bash_profile"}) {
        const fs::path path = fs::path(home) / rc;
        std::ifstream in(path);
        if (!in) continue;
        std::string line;
        std::string previous;
        while (std::getline(in, line)) {
            if (previous == kRcMarker && line_names_bin_dir(line, bin_dir)) {
                hits.push_back({path, line});
            }
            previous = line;
        }
    }
    return hits;
}

// Rewrites `path` without the marker+export pair. Writes a sibling temp file
// and renames over the original, so an interrupted uninstall cannot leave a
// user with a truncated shell rc.
bool strip_rc_entry(const fs::path& path, const fs::path& prefix, std::string& error) {
    const fs::path bin_dir = prefix / "bin";

    std::ifstream in(path);
    if (!in) {
        error = "cannot read " + path.string();
        return false;
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) lines.push_back(line);
    in.close();

    std::vector<std::string> kept;
    kept.reserve(lines.size());
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const bool is_block = lines[i] == kRcMarker && i + 1 < lines.size() &&
                              line_names_bin_dir(lines[i + 1], bin_dir);
        if (is_block) {
            // install.sh prefixes the block with a blank line; drop it too so
            // repeated install/uninstall cycles don't accumulate blank lines.
            if (!kept.empty() && kept.back().empty()) kept.pop_back();
            ++i;  // skip the export line as well
            continue;
        }
        kept.push_back(lines[i]);
    }

    const fs::path tmp = path.string() + ".harness-uninstall.tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out) {
            error = "cannot write " + tmp.string();
            return false;
        }
        for (const auto& l : kept) out << l << '\n';
        if (!out) {
            error = "write failed for " + tmp.string();
            return false;
        }
    }

    std::error_code ec;
    fs::rename(tmp, path, ec);
    if (ec) {
        fs::remove(tmp, ec);
        error = "cannot replace " + path.string();
        return false;
    }
    return true;
}

std::uintmax_t count_entries(const fs::path& dir) {
    std::error_code ec;
    std::uintmax_t n = 0;
    for (fs::recursive_directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        if (!it->is_directory(ec)) n++;
    }
    return n;
}

}  // namespace

int run_uninstall(const UninstallOptions& options) {
    fs::path prefix;
    std::string prefix_source;

    if (!options.prefix.empty()) {
        std::error_code ec;
        fs::path given = fs::absolute(options.prefix, ec);
        prefix = ec ? fs::path(options.prefix) : given;
        prefix_source = "--prefix";
    } else {
        const std::string exe = executable_path();
        if (exe.empty()) {
            std::fprintf(stderr, "error: cannot determine this binary's location; "
                                 "pass --prefix <dir>\n");
            return 1;
        }
        // <prefix>/bin/rearguard -> <prefix>
        prefix = fs::path(exe).parent_path().parent_path();
        prefix_source = "derived from " + exe;
    }

    std::string why_not;
    if (!looks_like_install_root(prefix, why_not)) {
        std::fprintf(stderr,
                     "error: %s does not look like a harness installation (%s)\n"
                     "       refusing to delete anything.\n",
                     prefix.string().c_str(), why_not.c_str());
        if (options.prefix.empty()) {
            std::fprintf(stderr,
                         "       (this binary is running from a build tree, not an install "
                         "tree — pass --prefix <dir> to name one)\n");
        }
        return 1;
    }

    // Gather before removing, so --dry-run and the confirmation prompt report
    // exactly what the removal will act on.
    std::vector<fs::path> targets;
    std::uintmax_t file_count = 0;
    for (const char* name : kInstalledDirs) {
        const fs::path dir = prefix / name;
        std::error_code ec;
        if (fs::is_directory(dir, ec)) {
            targets.push_back(dir);
            file_count += count_entries(dir);
        }
    }
    const std::vector<RcHit> rc_hits = find_rc_entries(prefix);

    if (targets.empty() && rc_hits.empty()) {
        std::printf("nothing to remove: %s has no harness install directories\n",
                    prefix.string().c_str());
        return 0;
    }

    if (!options.json) {
        std::printf("harness installation at %s\n  (%s)\n\n", prefix.string().c_str(),
                    prefix_source.c_str());
        std::printf("would remove %llu file(s) in:\n",
                    static_cast<unsigned long long>(file_count));
        for (const auto& t : targets) std::printf("  %s\n", t.string().c_str());
        for (const auto& hit : rc_hits) {
            std::printf("  %s  (PATH entry: %s)\n", hit.file.string().c_str(),
                        hit.line.c_str());
        }
        std::printf("\n");
    }

    if (options.dry_run) {
        if (options.json) {
            std::printf("{\"prefix\":\"%s\",\"dry_run\":true,\"files\":%llu,\"removed\":false}\n",
                        escape_json(prefix.string()).c_str(),
                        static_cast<unsigned long long>(file_count));
        } else {
            std::printf("dry run: nothing was removed\n");
        }
        return 0;
    }

    if (!options.assume_yes) {
        // Never treat a non-interactive EOF as consent. A script that means to
        // uninstall says so with --yes.
        std::printf("Remove these? [y/N] ");
        std::fflush(stdout);
        std::string answer;
        if (!std::getline(std::cin >> std::ws, answer) || (answer != "y" && answer != "Y")) {
            std::printf("aborted; nothing was removed\n");
            return 1;
        }
    }

    int failures = 0;
    for (const auto& t : targets) {
        std::error_code ec;
        fs::remove_all(t, ec);
        if (ec) {
            std::fprintf(stderr, "error: failed to remove %s: %s\n", t.string().c_str(),
                         ec.message().c_str());
            failures++;
        }
    }

    for (const auto& hit : rc_hits) {
        std::string error;
        if (!strip_rc_entry(hit.file, prefix, error)) {
            std::fprintf(stderr, "error: %s (remove this line by hand: %s)\n", error.c_str(),
                         hit.line.c_str());
            failures++;
        }
    }

    // Only if it is now empty. The prefix may be a directory the user created
    // for other things too (~/.local, /usr/local), and removing it because we
    // put four directories inside it would be well beyond what was asked.
    std::error_code ec;
    if (fs::is_directory(prefix, ec) && fs::is_empty(prefix, ec) && !ec) {
        fs::remove(prefix, ec);
    }

    if (options.json) {
        std::printf("{\"prefix\":\"%s\",\"dry_run\":false,\"files\":%llu,\"removed\":%s,"
                    "\"failures\":%d}\n",
                    escape_json(prefix.string()).c_str(),
                    static_cast<unsigned long long>(file_count),
                    failures == 0 ? "true" : "false", failures);
        return failures == 0 ? 0 : 1;
    }

    if (failures > 0) {
        std::fprintf(stderr, "\nuninstall finished with %d error(s)\n", failures);
        return 1;
    }

    std::printf("removed harness installation from %s\n", prefix.string().c_str());
    if (!rc_hits.empty()) {
        std::printf("restart your shell (or open a new one) so PATH no longer points there\n");
    }
    return 0;
}

}  // namespace harness::cli
