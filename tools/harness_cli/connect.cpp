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
#include <string>
#include <thread>
#include <utility>

#include <nlohmann/json.hpp>

#include "service_config.hpp"

namespace harness::cli {
namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;

volatile sig_atomic_t interrupted = 0;

void handle_interrupt(int) { interrupted = 1; }

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
                                  {"verification_url", "https://rearguard.dev/device"}}
                                 .dump()
                                 .c_str());
    } else {
        std::printf("Rearguard device pairing\n\n");
        std::printf("Open https://rearguard.dev/device\n");
        std::printf("and enter this code:\n\n  %s\n\n", display_code.c_str());
        std::printf("Waiting for approval...\n");
        std::printf("This code expires in 10 minutes. Press Ctrl-C to cancel.\n");
        std::fflush(stdout);
    }

    interrupted = 0;
    const auto previous_handler = ::signal(SIGINT, handle_interrupt);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::minutes(10);
    while (!interrupted && std::chrono::steady_clock::now() < deadline) {
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
            std::printf("\nConnected as %s.\n", device_name.c_str());
            std::printf("Device credential saved to %s\n", credential_path.c_str());
        }
        return 0;
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
