#include "connect.hpp"

#include <curl/curl.h>
#include <fcntl.h>
#include <openssl/rand.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "common.hpp"
#include "service_config.hpp"

namespace harness::cli {
namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;

volatile sig_atomic_t interrupted = 0;

void handle_interrupt(int) { interrupted = 1; }

bool supports_color() {
    const char* term = std::getenv("TERM");
    return ::isatty(STDOUT_FILENO) != 0 && std::getenv("NO_COLOR") == nullptr &&
           (term == nullptr || std::strcmp(term, "dumb") != 0);
}

bool is_interactive() { return ::isatty(STDOUT_FILENO) != 0; }

const char* paint(bool enabled, const char* code) { return enabled ? code : ""; }

std::string getenv_or(const char* name, const char* fallback) {
    const char* value = std::getenv(name);
    return value != nullptr && value[0] != '\0' ? value : fallback;
}

std::string default_device_name() {
    char hostname[256]{};
    if (::gethostname(hostname, sizeof(hostname) - 1) == 0 && hostname[0] != '\0') {
        return hostname;
    }
    return "robot-edge";
}

std::string random_secret() {
    unsigned char bytes[32]{};
    if (RAND_bytes(bytes, static_cast<int>(sizeof(bytes))) != 1) return {};
    static constexpr char hex[] = "0123456789abcdef";
    std::string result;
    result.reserve(sizeof(bytes) * 2);
    for (unsigned char byte : bytes) {
        result.push_back(hex[byte >> 4U]);
        result.push_back(hex[byte & 0x0fU]);
    }
    return result;
}

size_t append_response(char* data, size_t size, size_t count, void* output) {
    const size_t bytes = size * count;
    static_cast<std::string*>(output)->append(data, bytes);
    return bytes;
}

struct HttpResult {
    long status = 0;
    std::string body;
    std::string error;
};

HttpResult post_rpc(const std::string& api_url, const std::string& anon_key,
                    const std::string& function, const Json& payload) {
    HttpResult result;
    CURL* curl = curl_easy_init();
    if (curl == nullptr) {
        result.error = "could not initialize HTTPS client";
        return result;
    }

    char error_buffer[CURL_ERROR_SIZE]{};
    const std::string url = api_url + "/rest/v1/rpc/" + function;
    const std::string body = payload.dump();
    const std::string api_header = "apikey: " + anon_key;
    const std::string auth_header = "Authorization: Bearer " + anon_key;
    curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, api_header.c_str());
    headers = curl_slist_append(headers, auth_header.c_str());

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_response);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &result.body);
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error_buffer);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 20L);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "rearguard-cli/0.3");

    const CURLcode code = curl_easy_perform(curl);
    if (code != CURLE_OK) {
        result.error = error_buffer[0] != '\0' ? error_buffer : curl_easy_strerror(code);
    } else {
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &result.status);
    }
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return result;
}

std::string response_message(const HttpResult& result) {
    if (!result.error.empty()) return result.error;
    try {
        const Json parsed = Json::parse(result.body);
        if (parsed.contains("message") && parsed["message"].is_string()) {
            return parsed["message"].get<std::string>();
        }
    } catch (const Json::exception&) {
    }
    return "server returned HTTP " + std::to_string(result.status);
}

Json first_row(const HttpResult& result) {
    if (result.status < 200 || result.status >= 300) return {};
    try {
        Json parsed = Json::parse(result.body);
        if (parsed.is_array() && !parsed.empty() && parsed[0].is_object()) return parsed[0];
        if (parsed.is_object()) return parsed;
    } catch (const Json::exception&) {
    }
    return {};
}

bool report_scan_event(const std::string& api_url, const std::string& anon_key,
                       const std::string& device_id, const std::string& secret,
                       const std::string& scan_id, int sequence, const std::string& state,
                       int progress, const Json& payload, int domain_min = -1,
                       int domain_max = -1, const std::string& error = {}) {
    Json request{{"p_device_id", device_id},
                 {"p_device_secret", secret},
                 {"p_scan_id", scan_id},
                 {"p_sequence", sequence},
                 {"p_state", state},
                 {"p_progress", progress},
                 {"p_domain_min", domain_min < 0 ? Json(nullptr) : Json(domain_min)},
                 {"p_domain_max", domain_max < 0 ? Json(nullptr) : Json(domain_max)},
                 {"p_payload", payload},
                 {"p_error_message", error.empty() ? Json(nullptr) : Json(error)}};
    const auto result = post_rpc(api_url, anon_key, "report_device_setup_scan", request);
    if (result.status >= 200 && result.status < 300) return true;
    std::fprintf(stderr, "error: could not stream setup scan event: %s\n",
                 response_message(result).c_str());
    return false;
}

harness::observation::Options scan_options(int first, int last, int settle_ms,
                                           int parallelism) {
    std::vector<std::string> values{
        "setup", "--domain-min", std::to_string(first), "--domain-max",
        std::to_string(last), "--settle-ms", std::to_string(settle_ms),
        "--parallelism", std::to_string(parallelism)};
    std::vector<char*> argv;
    argv.reserve(values.size());
    for (auto& value : values) argv.push_back(value.data());
    return harness::observation::Options(static_cast<int>(argv.size()), argv.data());
}

int run_automatic_setup_scan(const ConnectOptions& options, const std::string& api_url,
                             const std::string& anon_key, const std::string& device_id,
                             const std::string& secret) {
    constexpr int kFirstDomain = 0;
    constexpr int kLastDomain = 232;
    constexpr int kDomainCount = kLastDomain - kFirstDomain + 1;
    const int batch_size = options.scan_parallelism;
    const std::string scan_id = harness::observation::unique_id();
    int sequence = 0;

    if (options.json) {
        std::printf("%s\n", Json{{"status", "scan_started"},
                                  {"scan_id", scan_id},
                                  {"domain_min", kFirstDomain},
                                  {"domain_max", kLastDomain}}
                                 .dump()
                                 .c_str());
    } else {
        const bool color = supports_color();
        std::printf("  %s›%s %s3.%s Discovering the ROS 2 graph automatically\n",
                    paint(color, "\033[1;36m"), paint(color, "\033[0m"),
                    paint(color, "\033[1m"), paint(color, "\033[0m"));
        std::printf("    Domains 0–232 · %d concurrent · %d ms discovery window\n",
                    batch_size, options.scan_settle_ms);
        std::fflush(stdout);
    }

    bool synced = report_scan_event(api_url, anon_key, device_id, secret, scan_id,
                                    sequence, "scanning", 0, nullptr);

    Json inventory{{"schema", "rearguard.ros_setup_inventory"},
                   {"schema_version", 2},
                   {"captured_wall_ns", 0},
                   {"scan", {{"domain_min", kFirstDomain},
                              {"domain_max", kLastDomain},
                              {"settle_ms", options.scan_settle_ms},
                              {"parallelism", batch_size}}},
                   {"available_domain_ids", Json::array()},
                   {"domains", Json::array()},
                   {"ros", nullptr},
                   {"device", nullptr}};
    try {
        for (int first = kFirstDomain; first <= kLastDomain; first += batch_size) {
            const int last = std::min(kLastDomain, first + batch_size - 1);
            if (!options.json && is_interactive()) {
                const bool color = supports_color();
                static constexpr char frames[] = {'|', '/', '-', '\\'};
                const int completed = first - kFirstDomain;
                const int before_progress = (completed * 100) / kDomainCount;
                constexpr int kBarWidth = 24;
                const int filled = (before_progress * kBarWidth) / 100;
                std::printf("\r\033[K    %s%c%s [", paint(color, "\033[1;36m"),
                            frames[static_cast<unsigned int>(sequence) % 4U],
                            paint(color, "\033[0m"));
                for (int cell = 0; cell < kBarWidth; ++cell)
                    std::printf("%c", cell < filled ? '=' : ' ');
                std::printf("] %3d%%  scanning domains %d–%d", before_progress, first, last);
                std::fflush(stdout);
            }
            auto batch_options = scan_options(first, last, options.scan_settle_ms, batch_size);
            auto batch = harness::observation::ros_discover(batch_options);
            if (inventory["ros"].is_null()) inventory["ros"] = batch["ros"];
            if (inventory["device"].is_null()) inventory["device"] = batch["device"];
            for (const auto& id : batch["available_domain_ids"])
                inventory["available_domain_ids"].push_back(id);
            for (const auto& domain : batch["domains"])
                inventory["domains"].push_back(domain);

            const int progress = ((last + 1) * 100) / kDomainCount;
            ++sequence;
            synced = report_scan_event(api_url, anon_key, device_id, secret, scan_id,
                                       sequence, "scanning", progress, batch, first, last) &&
                     synced;
            if (options.json) {
                std::printf("%s\n", Json{{"status", "scan_progress"},
                                          {"scan_id", scan_id},
                                          {"sequence", sequence},
                                          {"progress", progress},
                                          {"domain_min", first},
                                          {"domain_max", last},
                                          {"available_domain_ids", batch["available_domain_ids"]}}
                                         .dump()
                                         .c_str());
            } else if (is_interactive()) {
                constexpr int kBarWidth = 24;
                const int filled = (progress * kBarWidth) / 100;
                std::printf("\r\033[K    %s›%s [", paint(supports_color(), "\033[1;36m"),
                            paint(supports_color(), "\033[0m"));
                for (int cell = 0; cell < kBarWidth; ++cell)
                    std::printf("%c", cell < filled ? '=' : ' ');
                std::printf("] %3d%%  scanned domains %d–%d", progress, first, last);
                std::fflush(stdout);
            }
        }
        inventory["captured_wall_ns"] = harness::observation::wall_ns();
        inventory["inventory_hash"] = harness::observation::digest(inventory);
        ++sequence;
        synced = report_scan_event(api_url, anon_key, device_id, secret, scan_id,
                                   sequence, "completed", 100, inventory) &&
                 synced;
        if (options.json) {
            std::printf("%s\n", Json{{"status", "scan_completed"},
                                      {"scan_id", scan_id},
                                      {"sequence", sequence},
                                      {"synced", synced},
                                      {"inventory", inventory}}
                                     .dump()
                                     .c_str());
        } else {
            std::printf("%s  %s✓%s %s3.%s ROS 2 discovery complete · %zu active domain(s)\n",
                        is_interactive() ? "\r\033[K" : "",
                        paint(supports_color(), "\033[1;32m"),
                        paint(supports_color(), "\033[0m"),
                        paint(supports_color(), "\033[1m"),
                        paint(supports_color(), "\033[0m"), inventory["domains"].size());
            if (synced) {
                std::printf("  %s✓%s %s4.%s Discovery profile synced\n\n",
                            paint(supports_color(), "\033[1;32m"),
                            paint(supports_color(), "\033[0m"),
                            paint(supports_color(), "\033[1m"),
                            paint(supports_color(), "\033[0m"));
                std::printf("  %s✓ Device setup is ready.%s Continue in your browser.\n",
                            paint(supports_color(), "\033[1;32m"),
                            paint(supports_color(), "\033[0m"));
            } else {
                std::printf("  %s!%s %s4.%s Discovery finished, but cloud sync failed\n",
                            paint(supports_color(), "\033[1;33m"),
                            paint(supports_color(), "\033[0m"),
                            paint(supports_color(), "\033[1m"),
                            paint(supports_color(), "\033[0m"));
            }
        }
        return synced ? 0 : 1;
    } catch (const std::exception& cause) {
        std::string message = cause.what();
        if (message.size() > 1000) message.resize(1000);
        ++sequence;
        report_scan_event(api_url, anon_key, device_id, secret, scan_id, sequence,
                          "failed", 0, nullptr, -1, -1, message);
        if (options.json) {
            std::printf("%s\n", Json{{"status", "scan_failed"},
                                      {"scan_id", scan_id},
                                      {"error", message}}
                                     .dump()
                                     .c_str());
        } else {
            std::fprintf(stderr, "%s  ✗ 3. Automatic ROS 2 discovery failed: %s\n",
                         is_interactive() ? "\r\033[K" : "", message.c_str());
        }
        return 1;
    }
}

bool write_all(int fd, const std::string& contents) {
    size_t offset = 0;
    while (offset < contents.size()) {
        const ssize_t written = ::write(fd, contents.data() + offset, contents.size() - offset);
        if (written < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        offset += static_cast<size_t>(written);
    }
    return true;
}

bool save_credentials(const Json& credentials, std::string& path_out, std::string& error) {
    const char* home = std::getenv("HOME");
    if (home == nullptr || home[0] == '\0') {
        error = "HOME is not set; cannot choose a credential directory";
        return false;
    }
    const std::string config_home = getenv_or("XDG_CONFIG_HOME", (fs::path(home) / ".config").c_str());
    const fs::path directory = fs::path(config_home) / "rearguard";
    const fs::path path = directory / "device.json";
    const fs::path temporary = directory / (".device.json.tmp." + std::to_string(::getpid()));

    std::error_code ec;
    fs::create_directories(directory, ec);
    if (ec) {
        error = "could not create " + directory.string() + ": " + ec.message();
        return false;
    }
    if (::chmod(directory.c_str(), S_IRWXU) != 0) {
        error = "could not protect " + directory.string() + ": " + std::strerror(errno);
        return false;
    }

    const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
    if (fd < 0) {
        error = "could not create credential file: " + std::string(std::strerror(errno));
        return false;
    }
    const std::string contents = credentials.dump(2) + "\n";
    bool ok = write_all(fd, contents);
    if (ok && ::fsync(fd) != 0) ok = false;
    const int close_result = ::close(fd);
    if (close_result != 0) ok = false;
    if (!ok) {
        error = "could not write credential file: " + std::string(std::strerror(errno));
        fs::remove(temporary, ec);
        return false;
    }
    fs::rename(temporary, path, ec);
    if (ec) {
        error = "could not install credential file: " + ec.message();
        fs::remove(temporary, ec);
        return false;
    }
    path_out = path.string();
    return true;
}

}  // namespace

int run_connect(const ConnectOptions& options) {
    const std::string api_url = getenv_or("REARGUARD_SUPABASE_URL", kDefaultSupabaseUrl);
    const std::string anon_key =
        getenv_or("REARGUARD_SUPABASE_ANON_KEY", kDefaultSupabaseAnonKey);
    const std::string device_name = options.device_name.empty() ? default_device_name()
                                                                  : options.device_name;
    if (api_url.empty() || anon_key.empty()) {
        std::fprintf(stderr, "error: Rearguard cloud configuration is missing\n");
        return 1;
    }
    if (device_name.empty() || device_name.size() > 80) {
        std::fprintf(stderr, "error: device name must contain 1 to 80 characters\n");
        return 1;
    }

    if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {
        std::fprintf(stderr, "error: could not initialize HTTPS support\n");
        return 1;
    }
    struct CurlCleanup {
        ~CurlCleanup() { curl_global_cleanup(); }
    } curl_cleanup;

    const std::string secret = random_secret();
    if (secret.empty()) {
        std::fprintf(stderr, "error: operating-system random number generation failed\n");
        return 1;
    }

    const HttpResult started = post_rpc(api_url, anon_key, "begin_device_pairing",
                                        {{"p_device_name", device_name},
                                         {"p_device_secret", secret}});
    const Json start = first_row(started);
    if (!start.contains("pairing_id") || !start.contains("user_code")) {
        std::fprintf(stderr, "error: could not start device pairing: %s\n",
                     response_message(started).c_str());
        return 1;
    }

    const std::string pairing_id = start["pairing_id"].get<std::string>();
    const std::string compact_code = start["user_code"].get<std::string>();
    const std::string display_code = compact_code.substr(0, 4) + "-" + compact_code.substr(4);
    if (options.json) {
        std::printf("%s\n", Json{{"status", "pending"},
                                  {"user_code", display_code},
                                  {"verification_instruction",
                                   "Enter this code in the Connect your robot page already open in your browser."}}
                                 .dump()
                                 .c_str());
    } else {
        const bool color = supports_color();
        std::printf("%sRearguard%s  %sDevice pairing%s\n\n", paint(color, "\033[1;36m"),
                    paint(color, "\033[0m"), paint(color, "\033[2m"),
                    paint(color, "\033[0m"));
        std::printf("  %s1.%s In your signed-in browser, open %sConnect your robot%s\n",
                    paint(color, "\033[1;36m"), paint(color, "\033[0m"),
                    paint(color, "\033[1m"), paint(color, "\033[0m"));
        std::printf("  %s2.%s Enter this one-time code:\n\n", paint(color, "\033[1;36m"),
                    paint(color, "\033[0m"));
        std::printf("       %s%s%s\n\n", paint(color, "\033[1;97;44m"), display_code.c_str(),
                    paint(color, "\033[0m"));
        std::printf("  %s•%s This code expires in 10 minutes. Press Ctrl-C to cancel.\n",
                    paint(color, "\033[2m"), paint(color, "\033[0m"));
        std::fflush(stdout);
    }

    interrupted = 0;
    const auto previous_handler = ::signal(SIGINT, handle_interrupt);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::minutes(10);
    unsigned int spinner_frame = 0;
    while (!interrupted && std::chrono::steady_clock::now() < deadline) {
        if (!options.json) {
            const bool color = supports_color();
            static constexpr char frames[] = {'|', '/', '-', '\\'};
            std::printf("\r  %s%c%s Waiting for browser approval...", paint(color, "\033[1;36m"),
                        frames[spinner_frame++ % 4U], paint(color, "\033[0m"));
            std::fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
        if (interrupted) break;
        const HttpResult polled = post_rpc(api_url, anon_key, "poll_device_pairing",
                                           {{"p_pairing_id", pairing_id},
                                            {"p_device_secret", secret}});
        const Json poll = first_row(polled);
        if (!poll.contains("status")) {
            if (polled.status >= 400 || !polled.error.empty()) {
                std::fprintf(stderr, "error: pairing check failed: %s\n",
                             response_message(polled).c_str());
                ::signal(SIGINT, previous_handler);
                return 1;
            }
            continue;
        }
        const std::string status = poll["status"].get<std::string>();
        if (status == "pending") continue;
        if (status == "expired") break;
        if (status != "approved" || !poll.contains("device_id")) {
            std::fprintf(stderr, "error: server returned an invalid pairing result\n");
            ::signal(SIGINT, previous_handler);
            return 1;
        }

        const std::string device_id = poll["device_id"].get<std::string>();
        std::string credential_path;
        std::string save_error;
        const Json credentials{{"version", 1},
                               {"device_id", device_id},
                               {"device_name", device_name},
                               {"credential", secret},
                               {"api_url", api_url}};
        if (!save_credentials(credentials, credential_path, save_error)) {
            std::fprintf(stderr, "error: paired, but could not save credentials: %s\n",
                         save_error.c_str());
            ::signal(SIGINT, previous_handler);
            return 1;
        }
        ::signal(SIGINT, previous_handler);
        if (options.json) {
            std::printf("%s\n", Json{{"status", "connected"},
                                      {"device_id", device_id},
                                      {"credential_path", credential_path}}
                                     .dump()
                                     .c_str());
        } else {
            const bool color = supports_color();
            std::printf("%s  %s✓%s %s1.%s Connected as %s%s%s\n",
                        is_interactive() ? "\r\033[K" : "",
                        paint(color, "\033[1;32m"),
                        paint(color, "\033[0m"), paint(color, "\033[1m"),
                        paint(color, "\033[0m"), paint(color, "\033[1m"), device_name.c_str(),
                        paint(color, "\033[0m"));
            std::printf("  %s✓%s %s2.%s Device credential saved to %s\n",
                        paint(color, "\033[1;32m"), paint(color, "\033[0m"),
                        paint(color, "\033[1m"), paint(color, "\033[0m"),
                        credential_path.c_str());
        }
        return run_automatic_setup_scan(options, api_url, anon_key, device_id, secret);
    }
    ::signal(SIGINT, previous_handler);
    if (interrupted) {
        std::fprintf(stderr, "\nPairing cancelled.\n");
    } else {
        std::fprintf(stderr, "error: pairing code expired; run rearguard connect again\n");
    }
    return 1;
}

}  // namespace harness::cli
