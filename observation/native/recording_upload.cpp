#include "recording_upload.hpp"
#include "events.hpp"

#include <algorithm>
#include <curl/curl.h>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <set>
#include <thread>

namespace harness::observation {
namespace {
volatile sig_atomic_t uploader_stopping = 0;

void uploader_stop_handler(int) { uploader_stopping = 1; }

fs::path default_credentials_path() {
  if (const auto *value = std::getenv("XDG_CONFIG_HOME"); value && *value)
    return fs::path(value) / "rearguard" / "device.json";
  const auto *home = std::getenv("HOME");
  require(home && *home, "HOME is not set; pass --credentials");
  return fs::path(home) / ".config" / "rearguard" / "device.json";
}

std::size_t append_response(char *data, std::size_t size, std::size_t count,
                            void *output) {
  const auto bytes = size * count;
  static_cast<std::string *>(output)->append(data, bytes);
  return bytes;
}

Json backend_request(const DeviceCredentials &credentials,
                     const std::string &path, const Json &payload) {
  CURL *curl = curl_easy_init();
  require(curl != nullptr, "could not initialize recording upload request");
  std::string response;
  char error[CURL_ERROR_SIZE]{};
  const auto body = payload.dump();
  const auto url = credentials.backend_url + path;
  const auto secret = "X-Rearguard-Device-Secret: " + credentials.secret;
  curl_slist *headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  headers = curl_slist_append(headers, secret.c_str());
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_response);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
  long status = 0;
  const auto result = curl_easy_perform(curl);
  if (result == CURLE_OK) curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  const auto failure = std::string(error[0] ? error : curl_easy_strerror(result));
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  require(result == CURLE_OK, "recording backend request failed: " + failure);
  require(status >= 200 && status < 300,
          "recording backend returned HTTP " + std::to_string(status) +
              ": " + response.substr(0, 1000));
  return response.empty() ? Json::object() : Json::parse(response);
}

void put_file(const fs::path &file, const Json &grant) {
  FILE *input = std::fopen(file.c_str(), "rb");
  require(input != nullptr, "cannot open MCAP for upload");
  CURL *curl = curl_easy_init();
  require(curl != nullptr, "could not initialize R2 upload");
  curl_slist *headers = nullptr;
  for (auto it = grant.at("required_headers").begin();
       it != grant.at("required_headers").end(); ++it) {
    const auto header = it.key() + ": " + it.value().get<std::string>();
    headers = curl_slist_append(headers, header.c_str());
  }
  std::string response;
  const auto upload_url = grant.at("upload_url").get<std::string>();
  char error[CURL_ERROR_SIZE]{};
  curl_easy_setopt(curl, CURLOPT_URL, upload_url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
  curl_easy_setopt(curl, CURLOPT_READDATA, input);
  curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE,
                   static_cast<curl_off_t>(fs::file_size(file)));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_response);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 600L);
  long status = 0;
  const auto result = curl_easy_perform(curl);
  if (result == CURLE_OK) curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  const auto failure = std::string(error[0] ? error : curl_easy_strerror(result));
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  std::fclose(input);
  require(result == CURLE_OK, "R2 MCAP upload failed: " + failure);
  // A retry may race with or follow a successful PUT whose completion call
  // was interrupted. If-None-Match protects the immutable object; 412 means
  // the exact content-addressed key already exists and can be verified.
  require((status >= 200 && status < 300) || status == 412,
          "R2 MCAP upload returned HTTP " + std::to_string(status));
}

fs::path mcap_file(const fs::path &segment) {
  std::vector<fs::path> files;
  if (fs::is_regular_file(segment) && segment.extension() == ".mcap")
    files.push_back(segment);
  else if (fs::is_directory(segment))
    for (const auto &entry : fs::recursive_directory_iterator(segment))
      if (entry.is_regular_file() && entry.path().extension() == ".mcap")
        files.push_back(entry.path());
  require(files.size() == 1, "closed recording segment must contain one MCAP");
  return files.front();
}

std::string modality_kind(const Json &topic) {
  const auto role = topic.value("role", "custom");
  if (role == "joint_state") return "joint_state";
  if (role == "camera") return "image";
  if (role == "action") return "trajectory";
  return "custom";
}

Json load_state(const fs::path &path) {
  if (!fs::is_regular_file(path))
    return {{"schema_version", 2}, {"operations", Json::object()}};
  auto state = read_json(path);
  if (state.value("schema_version", 1) == 1) {
    Json operations = Json::object();
    for (const auto &relative : state.value("uploaded", Json::array()))
      operations[relative.get<std::string>()] = {{"status", "submitted"}};
    state = {{"schema_version", 2}, {"operations", operations}};
  }
  if (!state.contains("operations") || !state["operations"].is_object())
    state["operations"] = Json::object();
  return state;
}

void journal(const fs::path &path, Json &state, const std::string &relative,
             const std::string &status, Json values = Json::object()) {
  auto &operation = state["operations"][relative];
  if (!operation.is_object()) operation = Json::object();
  operation["status"] = status;
  operation["updated_wall_ns"] = wall_ns();
  operation.update(values);
  atomic_json(path, state);
}

} // namespace

Json upload_closed_recordings(const fs::path &store, const fs::path &session,
                              const DeviceCredentials &credentials) {
  const auto recording_path = session / "recording.json";
  const auto session_path = session / "session.json";
  if (!fs::is_regular_file(recording_path) || !fs::is_regular_file(session_path))
    return {{"uploaded", 0}};
  const auto recording = read_json(recording_path);
  const auto session_meta = read_json(session_path);
  if (!session_meta.contains("workflow_store") || session_meta["workflow_store"].is_null() ||
      fs::weakly_canonical(session_meta["workflow_store"].get<std::string>()) !=
          fs::weakly_canonical(store))
    return {{"uploaded", 0}, {"skipped", "session belongs to another workflow store"}};
  const auto config = session_meta.at("config");
  const auto profile_id = Store(store).snapshot().at("profile_id").get<std::string>();
  const auto state_path = session / "cloud-recordings.json";
  auto state = load_state(state_path);
  std::set<std::string> uploaded;
  for (auto it = state["operations"].begin(); it != state["operations"].end(); ++it)
    if (it.value().value("status", "") == "submitted") uploaded.insert(it.key());
  unsigned count = 0;
  for (const auto &segment : recording.at("segments")) {
    const auto relative = segment.at("path").get<std::string>();
    if (uploaded.contains(relative)) continue;
    const auto file = mcap_file(session / relative);
    const auto sha = file_hash(file);
    const auto recording_id = digest({{"profile_id", profile_id},
                                      {"session_id", session_meta.at("session_id")},
                                      {"path", relative}, {"sha256", sha}});
    Json modalities = Json::array();
    const auto message_counts = segment.value("message_counts", Json::object());
    for (const auto &topic : config.at("topics"))
      modalities.push_back({
          {"modality_key", digest({topic.at("name"), topic.at("type")})},
          {"kind", modality_kind(topic)}, {"topic", topic.at("name")},
          {"ros2_type", topic.at("type")}, {"message_encoding", "cdr"},
          {"schema_encoding", "ros2msg"}, {"fields", Json::array()},
          {"entities", Json::array()}, {"metadata", {{"role", topic.value("role", "custom")}}},
          {"message_count", message_counts.value(topic.at("name").get<std::string>(), 0U)},
          {"start_time_ns", segment.at("start_ns")},
          {"end_time_ns", segment.at("end_ns")}});
    const Json payload = {
        {"device_id", credentials.device_id}, {"profile_id", profile_id},
        {"recording_id", recording_id}, {"sha256", sha},
        {"size_bytes", fs::file_size(file)}, {"start_time_ns", segment.at("start_ns")},
        {"end_time_ns", segment.at("end_ns")}, {"modalities", modalities},
        {"manifest", {{"schema_version", 1}, {"session_id", session_meta.at("session_id")},
                       {"segment", segment}}}};
    try {
      const auto previous_attempts = state["operations"][relative].value("attempts", 0U);
      journal(state_path, state, relative, "pending",
              {{"attempts", previous_attempts + 1U}, {"recording_id", recording_id},
               {"sha256", sha}, {"size_bytes", fs::file_size(file)},
               {"local_path", file.string()}, {"last_error", nullptr}});
      bool already_complete = false;
      try {
        // Completion-first repairs an upload whose acknowledgement was lost.
        backend_request(credentials, "/harness-profiles/edge/recordings/complete", payload);
        already_complete = true;
      } catch (const std::exception &) {
      }
      if (!already_complete) {
        const auto grant = backend_request(
            credentials, "/harness-profiles/edge/recordings/upload", payload);
        journal(state_path, state, relative, "uploading",
                {{"object_key", grant.at("object_key")},
                 {"grant_expires_at", grant.at("expires_at")}});
        put_file(file, grant);
        journal(state_path, state, relative, "uploaded_to_r2");
        backend_request(credentials, "/harness-profiles/edge/recordings/complete", payload);
      }
      journal(state_path, state, relative, "submitted");
    } catch (const std::exception &error) {
      journal(state_path, state, relative, "retrying",
              {{"last_error", error.what()}});
      throw;
    }
    uploaded.insert(relative);
    ++count;
  }
  return {{"uploaded", count}, {"segments_seen", recording.at("segments").size()}};
}

Json run_recording_uploader(const Options &options) {
  options.allow("--store --session --sessions-root --credentials --backend-url --interval-ms");
  const auto store = options.path("--store");
  require(options.values.contains("--session") != options.values.contains("--sessions-root"),
          "pass exactly one of --session or --sessions-root");
  const auto session = options.values.contains("--session")
                           ? options.path("--session") : fs::path();
  const auto sessions_root = options.values.contains("--sessions-root")
                                 ? options.path("--sessions-root") : fs::path();
  const auto credentials_path = options.values.contains("--credentials")
                                    ? options.path("--credentials")
                                    : default_credentials_path();
  const auto credentials = load_device_credentials(
      credentials_path, options.get("--backend-url", ""));
  const auto interval_ms = options.integer("--interval-ms", 2000);
  require(interval_ms >= 250 && interval_ms <= 60000,
          "interval-ms must be between 250 and 60000");
  uploader_stopping = 0;
  ::signal(SIGINT, uploader_stop_handler);
  ::signal(SIGTERM, uploader_stop_handler);
  while (!uploader_stopping) {
    std::vector<fs::path> sessions;
    if (!session.empty()) sessions.push_back(session);
    else if (fs::is_directory(sessions_root))
      for (const auto &entry : fs::directory_iterator(sessions_root))
        if (entry.is_directory()) sessions.push_back(entry.path());
    std::sort(sessions.begin(), sessions.end());
    for (const auto &candidate : sessions) {
      try {
        const auto result = upload_closed_recordings(store, candidate, credentials);
        if (result.value("uploaded", 0U) > 0)
          record_event("recording_uploader", "submitted", "success",
                       {{"session", candidate.string()}, {"result", result}});
      } catch (const std::exception &error) {
        record_event("recording_uploader", "retry", "failure",
                     {{"message", error.what()}, {"session", candidate.string()}});
      }
    }
    const auto slices = static_cast<unsigned>(interval_ms / 100);
    for (unsigned index = 0; index < slices && !uploader_stopping; ++index)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return {{"stopped", true},
          {"source", session.empty() ? sessions_root.string() : session.string()}};
}

} // namespace harness::observation
