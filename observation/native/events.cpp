#include "events.hpp"

#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/file.h>
#include <thread>
#include <unistd.h>

namespace harness::observation {
namespace {
volatile sig_atomic_t following = 1;
void stop_following(int) { following = 0; }

std::string timestamp(Ns value) {
  const auto seconds = static_cast<std::time_t>(value / 1000000000LL);
  std::tm tm{};
  gmtime_r(&seconds, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") << '.' << std::setw(3)
      << std::setfill('0') << ((value / 1000000LL) % 1000) << 'Z';
  return out.str();
}

void print_event(const Json &event, bool json) {
  if (json) {
    std::cout << event.dump() << '\n';
    return;
  }
  std::cout << timestamp(event.value("wall_ns", 0LL)) << ' '
            << (event.value("status", "failure") == "success" ? "SUCCESS" : "FAILURE")
            << ' ' << event.value("component", "harness") << '.'
            << event.value("action", "unknown");
  const auto &detail = event.value("detail", Json::object());
  if (detail.is_object()) {
    if (detail.contains("message") && detail["message"].is_string())
      std::cout << " — " << detail["message"].get<std::string>();
    else if (!detail.empty())
      std::cout << ' ' << detail.dump();
  }
  std::cout << '\n';
}
} // namespace

fs::path state_root() {
  if (const auto *value = std::getenv("REARGUARD_STATE_DIR"); value && *value)
    return fs::absolute(value);
  if (const auto *value = std::getenv("XDG_STATE_HOME"); value && *value)
    return fs::path(value) / "rearguard";
  const auto *home = std::getenv("HOME");
  require(home && *home, "HOME is not set; set REARGUARD_STATE_DIR");
  return fs::path(home) / ".local" / "state" / "rearguard";
}

void record_event(const std::string &component, const std::string &action,
                  const std::string &status, const Json &detail) {
  require(status == "success" || status == "failure",
          "event status must be success or failure");
  const auto root = state_root();
  fs::create_directories(root);
  const auto path = root / "events.jsonl";
  const int fd = ::open(path.c_str(), O_CREAT | O_APPEND | O_WRONLY, 0600);
  require(fd >= 0, "cannot open harness event log");
  const auto body = Json{{"schema_version", 1},
                         {"sequence_id", unique_id()},
                         {"wall_ns", wall_ns()},
                         {"pid", ::getpid()},
                         {"component", component},
                         {"action", action},
                         {"status", status},
                         {"detail", detail}}
                        .dump() + "\n";
  bool locked = ::flock(fd, LOCK_EX) == 0;
  std::size_t offset = 0;
  while (locked && offset < body.size()) {
    const auto n = ::write(fd, body.data() + offset, body.size() - offset);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) { locked = false; break; }
    offset += static_cast<std::size_t>(n);
  }
  if (locked) locked = ::fsync(fd) == 0;
  ::flock(fd, LOCK_UN);
  ::close(fd);
  require(locked, "cannot write harness event log");
}

Json read_events(Ns after) {
  Json result = Json::array();
  std::ifstream stream(state_root() / "events.jsonl");
  std::string line;
  Ns index = 0;
  while (std::getline(stream, line)) {
    ++index;
    if (index <= after || line.empty()) continue;
    try {
      auto event = Json::parse(line);
      event["index"] = index;
      result.push_back(std::move(event));
    } catch (const Json::exception &) {
      result.push_back({{"index", index}, {"wall_ns", 0}, {"component", "event_log"},
                        {"action", "decode"}, {"status", "failure"},
                        {"detail", {{"message", "invalid event record"}}}});
    }
  }
  return result;
}

int events_main(int argc, char **argv) {
  try {
    bool json = false, follow = false;
    bool seen_json = false, seen_follow = false, seen_after = false;
    Ns after = 0;
    for (int i = 0; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--json" && !seen_json) json = seen_json = true;
      else if (arg == "--follow" && !seen_follow) follow = seen_follow = true;
      else if (arg == "--after" && !seen_after && i + 1 < argc) {
        seen_after = true;
        const std::string value = argv[++i];
        std::size_t used = 0;
        after = std::stoll(value, &used);
        require(used == value.size() && after >= 0, "--after must be nonnegative");
      }
      else throw std::runtime_error("usage: rearguard events [--after N] [--follow] [--json]");
    }
    following = 1;
    const auto old_int = ::signal(SIGINT, stop_following);
    const auto old_term = ::signal(SIGTERM, stop_following);
    do {
      const auto events = read_events(after);
      for (const auto &event : events) {
        print_event(event, json);
        after = std::max(after, event.value("index", after));
      }
      std::cout.flush();
      if (follow && following) std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } while (follow && following);
    ::signal(SIGINT, old_int);
    ::signal(SIGTERM, old_term);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "harness events: " << error.what() << '\n';
    return 1;
  }
}

} // namespace harness::observation
