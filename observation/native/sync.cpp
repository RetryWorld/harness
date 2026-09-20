#include "sync.hpp"

#include <curl/curl.h>
#include <signal.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <regex>
#include <thread>

namespace harness::observation {
namespace {

constexpr const char *default_backend_url = "https://info-32291--api.modal.run";
volatile sig_atomic_t sync_interrupted = 0;

void stop_sync(int) { sync_interrupted = 1; }

bool uuid_like(const std::string &value) {
  static const std::regex pattern(
      "^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-"
      "[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$");
  return std::regex_match(value, pattern);
}

std::string default_credentials_path() {
  const auto *home = std::getenv("HOME");
  require(home != nullptr && *home != '\0',
          "HOME is not set; pass --credentials explicitly");
  const auto *xdg = std::getenv("XDG_CONFIG_HOME");
  return (xdg && *xdg ? fs::path(xdg) : fs::path(home) / ".config") /
         "rearguard/device.json";
}

std::size_t append_response(char *data, std::size_t size, std::size_t count,
                            void *output) {
  const auto bytes = size * count;
  static_cast<std::string *>(output)->append(data, bytes);
  return bytes;
}

class CurlGlobal {
public:
  CurlGlobal() {
    require(curl_global_init(CURL_GLOBAL_DEFAULT) == CURLE_OK,
            "could not initialize HTTPS support");
  }
  ~CurlGlobal() { curl_global_cleanup(); }
};

class BackendTransport final : public WorkflowTransport {
public:
  explicit BackendTransport(DeviceCredentials credentials)
      : credentials_(std::move(credentials)) {}

  void upload(const std::string &robot_id, const Json &snapshot,
              const Json &receipts) override {
    request("POST", "/harness-profiles/edge/snapshot",
            {{"device_id", credentials_.device_id},
             {"robot_id", robot_id},
             {"snapshot", snapshot},
             {"receipts", receipts}});
  }

  Json pending(const std::string &profile_id) override {
    CURL *curl = curl_easy_init();
    require(curl != nullptr, "could not initialize HTTPS request");
    char *profile = curl_easy_escape(curl, profile_id.c_str(),
                                     static_cast<int>(profile_id.size()));
    char *device = curl_easy_escape(curl, credentials_.device_id.c_str(),
                                    static_cast<int>(credentials_.device_id.size()));
    require(profile != nullptr && device != nullptr,
            "could not encode workflow command request");
    const std::string path = "/harness-profiles/edge/" +
                             std::string(profile) + "/commands?device_id=" +
                             std::string(device);
    curl_free(profile);
    curl_free(device);
    curl_easy_cleanup(curl);
    auto result = request("GET", path, nullptr);
    require(result.is_array(), "backend returned a non-array command queue");
    return result;
  }

private:
  DeviceCredentials credentials_;

  Json request(const std::string &method, const std::string &path,
               const Json &payload) const {
    CURL *curl = curl_easy_init();
    require(curl != nullptr, "could not initialize HTTPS request");
    std::string response;
    char error[CURL_ERROR_SIZE]{};
    const std::string url = credentials_.backend_url + path;
    const std::string body = payload.is_null() ? "" : payload.dump();
    const std::string secret_header =
        "X-Rearguard-Device-Secret: " + credentials_.secret;
    curl_slist *headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, secret_header.c_str());
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_response);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "rearguard-workflow-sync/0.3");
    if (method == "POST") {
      curl_easy_setopt(curl, CURLOPT_POST, 1L);
      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
      curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE,
                       static_cast<long>(body.size()));
    }
    const auto code = curl_easy_perform(curl);
    long status = 0;
    if (code == CURLE_OK)
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
    const std::string failure =
        error[0] != '\0' ? error : curl_easy_strerror(code);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    require(code == CURLE_OK, "workflow backend request failed: " + failure);
    if (status < 200 || status >= 300) {
      std::string detail = response;
      try {
        const auto parsed = Json::parse(response);
        detail = parsed.value("detail", parsed.value("message", response));
      } catch (const Json::exception &) {
      }
      if (detail.size() > 1000) detail.resize(1000);
      throw std::runtime_error("workflow backend returned HTTP " +
                               std::to_string(status) + ": " + detail);
    }
    if (response.empty()) return Json::object();
    try {
      return Json::parse(response);
    } catch (const Json::exception &) {
      throw std::runtime_error("workflow backend returned invalid JSON");
    }
  }
};

} // namespace

WorkflowConnector::WorkflowConnector(Store store, std::string robot_id,
                                     WorkflowTransport &transport)
    : store_(std::move(store)), robot_id_(std::move(robot_id)),
      transport_(transport) {
  require(uuid_like(robot_id_), "--robot-id must be a UUID");
}

Json WorkflowConnector::cycle(bool heartbeat) {
  auto snapshot = store_.snapshot();
  auto receipts = store_.pending_sync_receipts();
  const auto revision = snapshot.at("revision").get<Ns>();
  const bool initial_upload = uploaded_revision_ < 0;
  const bool changed = revision != uploaded_revision_;
  const bool heartbeat_due =
      heartbeat &&
      (last_upload_monotonic_ns_ == 0 ||
       monotonic_ns() - last_upload_monotonic_ns_ >= 30000000000LL);
  if (changed || heartbeat_due || !receipts.empty()) {
    transport_.upload(robot_id_, snapshot, receipts);
    store_.acknowledge_sync_receipts(receipts);
    uploaded_revision_ = revision;
    last_upload_monotonic_ns_ = monotonic_ns();
  }

  const auto commands =
      transport_.pending(snapshot.at("profile_id").get<std::string>());
  for (const auto &command : commands) {
    require(command.is_object(), "workflow command must be an object");
    store_.apply_remote(command);
  }
  if (!commands.empty()) {
    snapshot = store_.snapshot();
    receipts = store_.pending_sync_receipts();
    require(!receipts.empty(), "workflow command did not produce a receipt");
    transport_.upload(robot_id_, snapshot, receipts);
    store_.acknowledge_sync_receipts(receipts);
    uploaded_revision_ = snapshot.at("revision").get<Ns>();
    last_upload_monotonic_ns_ = monotonic_ns();
  }
  return {{"profile_id", snapshot.at("profile_id")},
          {"revision", snapshot.at("revision")},
          {"initial_upload", initial_upload},
          {"snapshot_uploaded", changed || heartbeat_due || !receipts.empty() ||
                                    !commands.empty()},
          {"commands_received", commands.size()},
          {"receipts", receipts}};
}

DeviceCredentials load_device_credentials(
    const fs::path &path, const std::string &backend_override) {
  const auto credentials = read_json(path);
  DeviceCredentials result{
      required_text(credentials.at("device_id"), "device_id"),
      required_text(credentials.at("credential"), "credential"),
      backend_override.empty()
          ? credentials.value("backend_url", std::string(default_backend_url))
          : backend_override};
  require(result.secret.size() == 64 &&
              result.secret.find_first_not_of("0123456789abcdef") ==
                  std::string::npos,
          "device credential must be 64 lowercase hexadecimal characters");
  require(uuid_like(result.device_id), "paired device ID must be a UUID");
  while (!result.backend_url.empty() && result.backend_url.back() == '/')
    result.backend_url.pop_back();
  require(result.backend_url.starts_with("https://"),
          "workflow backend URL must use HTTPS");
  return result;
}

Json run_workflow_sync(const Options &options, bool once) {
  options.allow("--store --robot-id --credentials --backend-url --interval-ms");
  const auto credential_value = options.get("--credentials");
  const auto credentials_path = credential_value.empty()
                                    ? fs::path(default_credentials_path())
                                    : fs::path(credential_value);
  const auto credentials = load_device_credentials(
      credentials_path, options.get("--backend-url"));
  const auto interval_ms = options.integer("--interval-ms", 2000);
  require(interval_ms >= 250 && interval_ms <= 60000,
          "--interval-ms must be between 250 and 60000");
  CurlGlobal curl;
  BackendTransport transport(credentials);
  WorkflowConnector connector(Store(options.path("--store")),
                              options.need("--robot-id"), transport);
  if (once) return connector.cycle(true);

  sync_interrupted = 0;
  const auto previous_interrupt = ::signal(SIGINT, stop_sync);
  const auto previous_terminate = ::signal(SIGTERM, stop_sync);
  Ns failures = 0;
  while (!sync_interrupted) {
    try {
      const auto result = connector.cycle(true);
      failures = 0;
      if (result.value("snapshot_uploaded", false) ||
          result.value("commands_received", 0U) > 0)
        std::cout << result.dump() << std::endl;
    } catch (const std::exception &error) {
      ++failures;
      std::cerr << "workflow sync: " << error.what() << std::endl;
    }
    const auto backoff = std::min<Ns>(interval_ms * (1LL << std::min<Ns>(failures, 5)),
                                     60000);
    for (Ns waited = 0; waited < backoff && !sync_interrupted; waited += 100)
      std::this_thread::sleep_for(
          std::chrono::milliseconds(std::min<Ns>(100, backoff - waited)));
  }
  ::signal(SIGINT, previous_interrupt);
  ::signal(SIGTERM, previous_terminate);
  return {{"status", "stopped"}};
}

} // namespace harness::observation
