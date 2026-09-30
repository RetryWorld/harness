#include "store.hpp"
#include "critic_features.hpp"
#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <sqlite3.h>
#include <unistd.h>

namespace harness::observation {
namespace {
class Database {
public:
  sqlite3 *db = nullptr;
  explicit Database(const fs::path &path) {
    const auto rc =
        sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READWRITE, nullptr);
    if (rc != SQLITE_OK) {
      const std::string error =
          db ? sqlite3_errmsg(db) : "SQLite allocation failed";
      sqlite3_close(db);
      throw std::runtime_error(error);
    }
    sqlite3_busy_timeout(db, 10000);
    exec("PRAGMA synchronous=FULL");
    exec("CREATE TABLE IF NOT EXISTS enforcement_outbox("
         "id TEXT PRIMARY KEY, body TEXT NOT NULL)");
  }
  ~Database() { sqlite3_close(db); }
  void exec(const char *sql) {
    require(sqlite3_exec(db, sql, nullptr, nullptr, nullptr) == SQLITE_OK,
            sqlite3_errmsg(db));
  }
};
class Statement {
  sqlite3_stmt *stmt_ = nullptr;
  sqlite3 *db_;

public:
  Statement(Database &db, const char *sql) : db_(db.db) {
    require(sqlite3_prepare_v2(db_, sql, -1, &stmt_, nullptr) == SQLITE_OK,
            sqlite3_errmsg(db_));
  }
  ~Statement() { sqlite3_finalize(stmt_); }
  void bind(int index, const std::string &value) {
    require(sqlite3_bind_text(stmt_, index, value.c_str(),
                              static_cast<int>(value.size()),
                              SQLITE_TRANSIENT) == SQLITE_OK,
            sqlite3_errmsg(db_));
  }
  void bind(int index, Ns value) {
    require(sqlite3_bind_int64(stmt_, index, value) == SQLITE_OK,
            sqlite3_errmsg(db_));
  }
  bool next() {
    const int rc = sqlite3_step(stmt_);
    require(rc == SQLITE_ROW || rc == SQLITE_DONE, sqlite3_errmsg(db_));
    return rc == SQLITE_ROW;
  }
  std::string text(int column) {
    const auto *p = sqlite3_column_text(stmt_, column);
    require(p != nullptr, "missing SQLite value");
    return reinterpret_cast<const char *>(p);
  }
  Ns integer(int column) { return sqlite3_column_int64(stmt_, column); }
};
Json snapshot(Database &db) {
  Statement q(db, "SELECT body FROM profile WHERE id=1");
  require(q.next(), "profile missing");
  return Json::parse(q.text(0));
}
const std::map<std::string, std::string> artifacts = {
    {"demonstration", "demonstration_mcap"},
    {"critic", "critic_model"},
    {"recovery", "recovery_model"},
    {"validation", "validation_report"}};
void range(const Json &v, double lower, double upper, const std::string &name) {
  require(v.is_number() && std::isfinite(v.get<double>()) &&
              v.get<double>() >= lower && v.get<double>() <= upper,
          "invalid " + name);
}
void refresh_critic_readiness(Json &state) {
  const bool ready = state.value("critic_context", Json(nullptr)).is_object() &&
                     state.value("propositions", Json::object()).is_object() &&
                     !state.value("propositions", Json::object()).empty();
  state["critic_profile_ready"] = ready;
  state["critic_runtime_status"] =
      ready ? "ready_for_compilation" : "awaiting_profile_setup";
}
void compile_critic(Json &state, const fs::path &root, Ns profile_revision) {
  refresh_critic_readiness(state);
  if (!state["critic_profile_ready"].get<bool>()) {
    state["critic_compilation"] = nullptr;
    state["critic_deployment"] = nullptr;
    if (state["placeholders"].is_array() &&
        std::find(state["placeholders"].begin(), state["placeholders"].end(),
                  Json("critic_inference")) == state["placeholders"].end())
      state["placeholders"].push_back("critic_inference");
    if (state.value("installed_deployment", Json(nullptr)).is_null())
      state["enforcement_status"] = "unavailable";
    return;
  }
  Json proposition_source = Json::object();
  for (auto it = state["propositions"].begin();
       it != state["propositions"].end(); ++it) {
    const auto &proposition = it.value();
    proposition_source[it.key()] = {
        {"id", proposition["id"]},
        {"description", proposition["description"]},
        {"polarity", proposition["polarity"]},
        {"scope", proposition["scope"]},
        {"candidate_threshold", proposition.value("candidate_threshold", 0.95)},
        {"snapshots", proposition["snapshots"]},
        {"recovery", proposition["recovery"]}};
  }
  Json source = {{"schema_version", 1},
                 {"compiler", "edge_profile_compiler_v1"},
                 {"encoder", critic_feature_encoder},
                 {"profile_id", state["profile_id"]},
                 {"binding_hash", state["binding_hash"]},
                 {"context", state["critic_context"]},
                 {"propositions", proposition_source}};
  const auto input_hash = digest(source);
  const auto previous = state.value("critic_compilation", Json(nullptr));
  if (previous.is_object() && previous.value("input_hash", "") == input_hash &&
      !(previous.value("compile_state", "") == "awaiting_ros_runtime" &&
        snapshot_compiler_available()))
    return;
  Json embedding_cache = Json::object();
  if (previous.is_object() &&
      previous.value("encoder", "") == critic_feature_encoder &&
      previous.value("heads", Json::array()).is_array())
    for (const auto &head : previous["heads"])
      for (const auto &record : head.value("snapshots", Json::array()))
        if (record.value("embedding", Json::object()).value("status", "") ==
            "ready")
          embedding_cache[digest(Json::array(
              {record["evidence_sha256"], record["window_hash"],
               head["required_roles_mask"]}))] = record["embedding"];

  Json heads = Json::array(), runtime_heads = Json::array(), classes = Json::array();
  bool missing_examples = false, compile_error = false, runtime_unavailable = false;
  for (auto it = state["propositions"].begin();
       it != state["propositions"].end(); ++it) {
    const auto &proposition = it.value();
    std::uint32_t required_roles = 0;
    for (const auto &role : proposition["scope"]["required_roles"]) {
      const auto bit = critic_role_bit(role.get<std::string>());
      compile_error = compile_error || bit == 0;
      required_roles |= bit;
    }
    Json snapshots = Json::array(), centers = Json::array(), center_counts = Json::array();
    std::vector<std::array<float, critic_embedding_dimensions>> sums;
    std::vector<std::size_t> counts;
    for (const auto &snapshot : proposition["snapshots"]) {
      Json record = {{"failure_id", snapshot["failure_id"]},
                     {"evidence_sha256", snapshot["evidence"]["sha256"]},
                     {"window_hash", digest(snapshot["window"])}};
      try {
        const auto cache_key = digest(Json::array(
            {record["evidence_sha256"], record["window_hash"], required_roles}));
        const auto embedded = embedding_cache.contains(cache_key)
                                  ? embedding_cache[cache_key]
                                  : compile_snapshot_embedding(
                                        root / "artifacts" /
                                            snapshot["evidence"]["sha256"]
                                                .get<std::string>(),
                                        state["binding"], snapshot["window"],
                                        required_roles);
        record["embedding"] = embedded;
        const auto status = embedded.value("status", "error");
        if (status == "runtime_unavailable") runtime_unavailable = true;
        if (status != "ready") compile_error = compile_error || status != "runtime_unavailable";
        if (status == "ready") {
          require(embedded["vector"].is_array() &&
                      embedded["vector"].size() == critic_embedding_dimensions,
                  "snapshot encoder returned an invalid vector");
          std::array<float, critic_embedding_dimensions> vector{};
          for (std::size_t d = 0; d < vector.size(); ++d)
            vector[d] = embedded["vector"][d].get<float>();
          std::size_t selected = sums.size();
          float best = -2;
          for (std::size_t k = 0; k < sums.size(); ++k) {
            double norm = 0, dot = 0;
            for (std::size_t d = 0; d < vector.size(); ++d) {
              dot += static_cast<double>(sums[k][d]) *
                     static_cast<double>(vector[d]);
              norm += static_cast<double>(sums[k][d]) *
                      static_cast<double>(sums[k][d]);
            }
            const auto similarity =
                static_cast<float>(dot / std::sqrt(std::max(norm, 1e-12)));
            if (similarity > best) { best = similarity; selected = k; }
          }
          if (sums.empty() || (sums.size() < 4 && best < 0.8F)) {
            sums.push_back(vector);
            counts.push_back(1);
          } else {
            for (std::size_t d = 0; d < vector.size(); ++d)
              sums[selected][d] += vector[d];
            ++counts[selected];
          }
        }
      } catch (const std::exception &error) {
        record["embedding"] = {{"status", "error"}, {"error", error.what()}};
        compile_error = true;
      }
      snapshots.push_back(std::move(record));
    }
    for (std::size_t k = 0; k < sums.size(); ++k) {
      double norm = 0;
      for (const auto value : sums[k])
        norm += static_cast<double>(value) * static_cast<double>(value);
      norm = std::sqrt(norm);
      Json center = Json::array();
      for (const auto value : sums[k])
        center.push_back(static_cast<double>(value) / norm);
      centers.push_back(std::move(center));
      center_counts.push_back(counts[k]);
    }
    missing_examples = missing_examples || snapshots.empty();
    const std::string head_status =
        snapshots.empty() ? "unknown_no_examples" :
        centers.empty() ? "embedding_failed" : "ready";
    state["propositions"][it.key()]["detector_status"] = head_status;
    heads.push_back({{"proposition_id", it.key()},
                     {"description", proposition["description"]},
                     {"scope", proposition["scope"]},
                     {"required_roles_mask", required_roles},
                     {"snapshots", snapshots},
                     {"centers", centers},
                     {"center_counts", center_counts},
                     {"candidate_threshold", proposition.value("candidate_threshold", 0.95)},
                     {"score_kind", "uncalibrated_cosine_similarity"},
                     {"recovery", proposition["recovery"]},
                     {"status", head_status}});
    runtime_heads.push_back({
        {"proposition_id", it.key()}, {"scope", proposition["scope"]},
        {"required_roles_mask", required_roles}, {"centers", centers},
        {"center_counts", center_counts},
        {"candidate_threshold", proposition.value("candidate_threshold", 0.95)},
        {"score_kind", "uncalibrated_cosine_similarity"}});
    classes.push_back(it.key());
  }
  const auto generation = previous.is_object()
                              ? previous.value("generation", 0) + 1
                              : 1;
  const auto context_hash = digest(state["critic_context"]);
  const std::string compile_state =
      missing_examples ? "awaiting_examples" :
      compile_error ? "compile_failed" :
      runtime_unavailable ? "awaiting_ros_runtime" : "ready_shadow";
  Json compiled = {{"schema_version", 1},
                   {"compiler", "edge_profile_compiler_v1"},
                   {"encoder", critic_feature_encoder},
                   {"generation", generation},
                   {"profile_revision", profile_revision},
                   {"input_hash", input_hash},
                   {"context_hash", context_hash},
                   {"heads", heads},
                   {"compile_state", compile_state}};
  compiled["model_hash"] = digest(compiled);
  state["critic_compilation"] = compiled;
  state["critic_deployment"] = {
      {"schema_version", 2},
      {"id", "critic-" + compiled["model_hash"].get<std::string>().substr(0, 32)},
      {"profile_id", state["profile_id"]},
      {"profile_revision", profile_revision},
      {"binding_hash", state["binding_hash"]},
      {"runtime", "proposition_bank_v1"},
      {"compiler", "edge_profile_compiler_v1"},
      {"model_hash", compiled["model_hash"]},
      {"context_hash", context_hash},
      {"classes", classes},
      {"heads", runtime_heads},
      {"encoder", critic_feature_encoder},
      {"score_kind", "uncalibrated_cosine_similarity"},
      {"mode", "shadow"},
      {"deployment_validated", false},
      {"executable", compile_state == "ready_shadow"},
      {"compile_state", compiled["compile_state"]}};
  state["critic_runtime_status"] = compiled["compile_state"];
  if (compile_state == "ready_shadow" && state["placeholders"].is_array()) {
    auto &placeholders = state["placeholders"];
    placeholders.erase(
        std::remove(placeholders.begin(), placeholders.end(),
                    Json("critic_inference")),
        placeholders.end());
  } else if (state["placeholders"].is_array() &&
             std::find(state["placeholders"].begin(),
                       state["placeholders"].end(),
                       Json("critic_inference")) ==
                 state["placeholders"].end()) {
    state["placeholders"].push_back("critic_inference");
  }
  if (state.value("installed_deployment", Json(nullptr)).is_null())
    state["enforcement_status"] =
        compile_state == "ready_shadow" ? "shadow" : "unavailable";
}
} // namespace
Store::Store(fs::path directory) : root(fs::absolute(std::move(directory))) {
  require(fs::is_regular_file(root / "workflow.sqlite3"),
          "workflow store missing; run workflow init first");
  Database db(root / "workflow.sqlite3");
  db.exec("CREATE TABLE IF NOT EXISTS sync_receipts(id TEXT PRIMARY KEY, "
          "body TEXT NOT NULL)");
  auto state = observation::snapshot(db);
  const auto before_compile = state;
  if (state.value("critic_runtime_status", "") == "awaiting_ros_runtime" &&
      snapshot_compiler_available())
    compile_critic(state, root, state.value<Ns>("revision", 0));
  if (state.contains("placeholders") && state["placeholders"].is_array()) {
    auto &placeholders = state["placeholders"];
    const auto original_size = placeholders.size();
    placeholders.erase(
        std::remove(placeholders.begin(), placeholders.end(), "database_sync"),
        placeholders.end());
    if (placeholders.size() != original_size || state != before_compile) {
      Statement update(db, "UPDATE profile SET body=? WHERE id=1");
      update.bind(1, encoded(state));
      update.next();
    }
  }
}
Store Store::create(const fs::path &directory, const fs::path &config) {
  auto binding = load_config(config);
  fs::create_directories(directory);
  const auto path = directory / "workflow.sqlite3";
  const int fd = ::open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0600);
  require(fd >= 0, "store already exists or cannot be created");
  ::close(fd);
  const auto profile_id = "profile-" + unique_id();
  const auto binding_hash = digest(binding);
  Json state = {{"schema_version", 1},
                {"profile_id", profile_id},
                {"revision", 0},
                {"binding", binding},
                {"binding_hash", binding_hash},
                {"candidates", Json::object()},
                {"failures", Json::object()},
                {"propositions", Json::object()},
                {"jobs", Json::object()},
                {"bundles", Json::object()},
                {"deployments", Json::object()},
                {"critic_deployment", nullptr},
                {"critic_compilation", nullptr},
                {"critic_context", nullptr},
                {"critic_profile_ready", false},
                {"critic_runtime_status", "awaiting_profile_setup"},
                {"installed_deployment", nullptr},
                {"enforcement_status", "unavailable"},
                {"placeholders", {"critic_inference", "recovery_controller"}}};
  compile_critic(state, directory, 0);
  Database db(path);
  db.exec(
      "PRAGMA journal_mode=WAL; CREATE TABLE profile(id INTEGER PRIMARY KEY "
      "CHECK(id=1), body TEXT NOT NULL); CREATE TABLE receipts(id TEXT PRIMARY "
      "KEY, request_hash TEXT NOT NULL, response TEXT NOT NULL); CREATE TABLE "
      "outbox(seq INTEGER PRIMARY KEY AUTOINCREMENT, body TEXT NOT NULL); "
      "CREATE TABLE sync_receipts(id TEXT PRIMARY KEY, body TEXT NOT NULL); "
      "CREATE TABLE IF NOT EXISTS enforcement_outbox(id TEXT PRIMARY KEY, body TEXT NOT NULL);");
  db.exec("BEGIN IMMEDIATE");
  Statement q(db, "INSERT INTO profile VALUES(1, ?)");
  q.bind(1, encoded(state));
  q.next();
  Json event = {{"schema_version", 1},
                {"event_id", unique_id()},
                {"profile_id", state["profile_id"]},
                {"revision", 0},
                {"request_id", nullptr},
                {"operation", "initialize"},
                {"actor", "local_cli"},
                {"wall_ns", wall_ns()},
                {"result", {{"binding_hash", state["binding_hash"]},
                            {"critic_runtime", "awaiting_profile_setup"}}}};
  Statement out(db, "INSERT INTO outbox(body) VALUES(?)");
  out.bind(1, encoded({{"event", event}, {"profile", state}}));
  out.next();
  db.exec("COMMIT");
  fs::create_directories(directory / "artifacts");
  return Store(directory);
}
Json Store::snapshot() const {
  Database db(root / "workflow.sqlite3");
  return observation::snapshot(db);
}
Json Store::outbox(Ns after) const {
  require(after >= 0, "after must be nonnegative");
  Database db(root / "workflow.sqlite3");
  Statement q(db,
              "SELECT seq,body FROM outbox WHERE seq>? ORDER BY seq LIMIT 100");
  q.bind(1, after);
  auto result = Json::array();
  while (q.next()) {
    auto row = Json::parse(q.text(1));
    row["seq"] = q.integer(0);
    result.push_back(row);
  }
  return result;
}
Json Store::import_artifact(const fs::path &source,
                            const std::string &kind) const {
  require(kind == "evidence_mcap" || kind == "demonstration_mcap" ||
              kind == "critic_model" || kind == "recovery_model" ||
              kind == "validation_report",
          "unknown artifact kind");
  require(fs::is_regular_file(source) && fs::file_size(source) > 0,
          "artifact must be a nonempty local file");
  auto temporary = root / "artifacts" / ("ingest-" + unique_id());
  try {
    fs::copy_file(source, temporary);
    const int fd = ::open(temporary.c_str(), O_RDONLY);
    require(fd >= 0, "cannot open copied artifact");
    const int result = ::fsync(fd);
    ::close(fd);
    require(result == 0, "artifact fsync failed");
    if (kind.ends_with("mcap")) {
      require(fs::file_size(temporary) >= 16, "incomplete MCAP artifact");
      std::ifstream stream(temporary, std::ios::binary);
      std::string first(8, '\0'), last(8, '\0');
      stream.read(first.data(), 8);
      stream.seekg(-8, std::ios::end);
      stream.read(last.data(), 8);
      require(first == std::string("\x89MCAP0\r\n", 8) && last == first,
              "MCAP artifact lacks magic/footer");
    }
    const auto sha = file_hash(temporary);
    const auto destination = root / "artifacts" / sha;
    if (fs::exists(destination)) {
      require(file_hash(destination) == sha, "stored artifact corruption");
      fs::remove(temporary);
    } else
      fs::rename(temporary, destination);
    return {
        {"sha256", sha}, {"size", fs::file_size(destination)}, {"kind", kind}};
  } catch (...) {
    fs::remove(temporary);
    throw;
  }
}
fs::path Store::verify_artifact(const Json &reference,
                                const std::string &kind) const {
  require(reference.is_object() && reference.value("kind", "") == kind,
          "requires " + kind + " artifact reference");
  const auto sha = reference.value("sha256", "");
  require(sha.size() == 64 &&
              sha.find_first_not_of("0123456789abcdef") == std::string::npos,
          "invalid artifact SHA256");
  const auto path = root / "artifacts" / sha;
  require(fs::is_regular_file(path) &&
              Json(fs::file_size(path)) == reference.at("size") &&
              file_hash(path) == sha,
          "artifact missing or hash/size mismatch");
  return path;
}
Json Store::apply(const Json &request) const {
  return apply_internal(request, false);
}
Json Store::apply_remote(const Json &request) const {
  const auto id = required_text(request.at("request_id"), "request_id");
  try {
    const auto response = apply_internal(request, true);
    return {{"request_id", id}, {"status", "applied"}, {"result", response}};
  } catch (const std::exception &error) {
    std::string message = error.what();
    if (message.size() > 2000) message.resize(2000);
    Json receipt =
        {{"request_id", id}, {"status", "rejected"}, {"error", message}};
    Database db(root / "workflow.sqlite3");
    Statement save(db, "INSERT INTO sync_receipts VALUES(?,?) ON CONFLICT(id) "
                       "DO UPDATE SET body=excluded.body");
    save.bind(1, id);
    save.bind(2, encoded(receipt));
    save.next();
    return receipt;
  }
}
Json Store::pending_sync_receipts() const {
  Database db(root / "workflow.sqlite3");
  Statement query(db, "SELECT body FROM sync_receipts ORDER BY rowid LIMIT 100");
  auto receipts = Json::array();
  while (query.next()) receipts.push_back(Json::parse(query.text(0)));
  return receipts;
}
void Store::acknowledge_sync_receipts(const Json &receipts) const {
  require(receipts.is_array(), "sync receipts must be an array");
  Database db(root / "workflow.sqlite3");
  db.exec("BEGIN IMMEDIATE");
  try {
    for (const auto &receipt : receipts) {
      Statement remove(db, "DELETE FROM sync_receipts WHERE id=?");
      remove.bind(1, required_text(receipt.at("request_id"), "request_id"));
      remove.next();
    }
    db.exec("COMMIT");
  } catch (...) {
    db.exec("ROLLBACK");
    throw;
  }
}
void Store::record_enforcement(const Json &record) const {
  require(record.is_object() && record.value("schema_version", 0) == 1,
          "enforcement record requires schema_version 1");
  const auto id = required_text(record.at("event_id"), "event_id");
  require(record.at("profile_id") == snapshot().at("profile_id"),
          "enforcement record profile mismatch");
  const auto mode = required_text(record.at("mode"), "mode");
  const auto outcome = required_text(record.at("outcome"), "outcome");
  require(mode == "shadow" || mode == "active", "invalid enforcement mode");
  require(outcome == "completed" || outcome == "fallback" ||
              outcome == "timed_out" || outcome == "aborted" ||
              outcome == "unknown",
          "invalid enforcement outcome");
  require(record.at("started_wall_ns").is_number_integer() &&
              record.at("started_wall_ns").get<Ns>() > 0,
          "invalid enforcement start time");
  Database db(root / "workflow.sqlite3");
  Statement save(db, "INSERT INTO enforcement_outbox VALUES(?,?) ON CONFLICT(id) "
                     "DO UPDATE SET body=excluded.body");
  save.bind(1, id);
  save.bind(2, encoded(record));
  save.next();
}
Json Store::pending_enforcements() const {
  Database db(root / "workflow.sqlite3");
  Statement query(db, "SELECT body FROM enforcement_outbox ORDER BY rowid LIMIT 100");
  auto records = Json::array();
  while (query.next()) records.push_back(Json::parse(query.text(0)));
  return records;
}
void Store::acknowledge_enforcements(const Json &records) const {
  require(records.is_array(), "enforcement acknowledgement must be an array");
  Database db(root / "workflow.sqlite3");
  db.exec("BEGIN IMMEDIATE");
  try {
    for (const auto &record : records) {
      // Delete exactly the uploaded version. A recovery may finish while its
      // start record is in flight; in that race the terminal replacement must
      // remain queued for the next cycle.
      Statement remove(db, "DELETE FROM enforcement_outbox WHERE id=? AND body=?");
      remove.bind(1, required_text(record.at("event_id"), "event_id"));
      remove.bind(2, encoded(record));
      remove.next();
    }
    db.exec("COMMIT");
  } catch (...) {
    db.exec("ROLLBACK");
    throw;
  }
}
Json Store::apply_internal(const Json &request, bool queue_sync_receipt) const {
  require(request.is_object() && request.value("schema_version", 0) == 1,
          "request requires schema_version 1");
  const auto id = required_text(request.at("request_id"), "request_id");
  const auto actor = required_text(request.at("actor"), "actor");
  require(request.at("expected_revision").is_number_integer() &&
              request.at("expected_revision").get<Ns>() >= 0,
          "expected_revision must be a nonnegative integer");
  const auto hash = digest(request);
  Database db(root / "workflow.sqlite3");
  db.exec("BEGIN IMMEDIATE");
  try {
    Statement receipt(db,
                      "SELECT request_hash,response FROM receipts WHERE id=?");
    receipt.bind(1, id);
    if (receipt.next()) {
      require(receipt.text(0) == hash,
              "request_id already used for different content");
      auto response = Json::parse(receipt.text(1));
      if (queue_sync_receipt) {
        const Json sync_receipt = {{"request_id", id},
                                   {"status", "applied"},
                                   {"result", response}};
        Statement sync(db, "INSERT INTO sync_receipts VALUES(?,?) ON "
                           "CONFLICT(id) DO UPDATE SET body=excluded.body");
        sync.bind(1, id);
        sync.bind(2, encoded(sync_receipt));
        sync.next();
      }
      db.exec("COMMIT");
      return response;
    }
    auto state = observation::snapshot(db);
    require(request.at("expected_revision") == state.at("revision"),
            "revision conflict: current revision is " +
                state.at("revision").dump());
    const auto op = request.at("operation").get<std::string>();
    auto result =
        transition(state, op, request.value("payload", Json::object()), actor);
    const auto next_revision = state.at("revision").get<Ns>() + 1;
    // Compilation is part of the same transaction as the profile mutation.
    // A crash can expose the old profile+model or the new profile+model, never
    // a new proposition set paired with an old critic generation.
    compile_critic(state, root, next_revision);
    state["revision"] = next_revision;
    Json event = {{"schema_version", 1},
                  {"event_id", unique_id()},
                  {"profile_id", state["profile_id"]},
                  {"revision", state["revision"]},
                  {"request_id", id},
                  {"operation", op},
                  {"actor", actor},
                  {"wall_ns", wall_ns()},
                  {"result", result}};
    Json response = {
        {"ok", true}, {"revision", state["revision"]}, {"result", result}};
    Statement update(db, "UPDATE profile SET body=? WHERE id=1");
    update.bind(1, encoded(state));
    update.next();
    Statement out(db, "INSERT INTO outbox(body) VALUES(?)");
    out.bind(1, encoded({{"event", event}, {"profile", state}}));
    out.next();
    Statement save(db, "INSERT INTO receipts VALUES(?,?,?)");
    save.bind(1, id);
    save.bind(2, hash);
    save.bind(3, encoded(response));
    save.next();
    if (queue_sync_receipt) {
      const Json sync_receipt = {{"request_id", id},
                                 {"status", "applied"},
                                 {"result", response}};
      Statement sync(db, "INSERT INTO sync_receipts VALUES(?,?)");
      sync.bind(1, id);
      sync.bind(2, encoded(sync_receipt));
      sync.next();
    }
    db.exec("COMMIT");
    return response;
  } catch (...) {
    db.exec("ROLLBACK");
    throw;
  }
}
Json Store::transition(Json &s, const std::string &op, const Json &p,
                       const std::string &actor) const {
  require(p.is_object(), "payload must be an object");
  auto item = [&](const char *collection, const char *key) -> Json & {
    require(p.contains(key) && p.at(key).is_string() &&
                s.at(collection).contains(p.at(key).get<std::string>()),
            std::string("unknown ") + key);
    return s.at(collection).at(p.at(key).get<std::string>());
  };
  if (op == "set_critic_context") {
    const auto &context = p.at("context");
    require(context.value("schema_version", 0) == 1,
            "critic_context requires schema_version 1");
    required_text(context.at("task"), "task");
    required_text(context.at("embodiment").at("class_id"), "embodiment class");
    require(context.at("embodiment").at("joint_names") == s["binding"]["joint_order"],
            "critic context joint names differ from binding");
    require(s["installed_deployment"].is_null(),
            "deactivate installed recovery before changing critic context");
    s["critic_context"] = context;
    if (s.contains("propositions"))
      for (auto &proposition : s["propositions"])
        proposition["detector_status"] = "needs_compile";
    refresh_critic_readiness(s);
    return context;
  }
  if (op == "add_proposition") {
    const auto id = required_text(p.at("proposition_id"), "proposition_id");
    const auto description = required_text(p.at("description"), "description");
    const auto &scope = p.at("scope");
    range(scope.at("required_hz"), 0.5, 2000, "required_hz");
    range(scope.at("max_input_age_ms"), 0.1, 10000, "max_input_age_ms");
    require(scope.at("required_roles").is_array() &&
                !scope.at("required_roles").empty(), "required_roles must be nonempty");
    for (const auto &role : scope.at("required_roles")) {
      const auto name = required_text(role, "required role");
      require(critic_role_bit(name) != 0,
              "unknown critic modality role: " + name);
      require(std::any_of(s["binding"]["topics"].begin(), s["binding"]["topics"].end(),
                         [&](const Json &topic) { return topic.value("role", "") == name; }),
              "proposition requires an unbound modality: " + name);
    }
    const auto candidate_threshold = p.value("candidate_threshold", 0.95);
    range(candidate_threshold, 0.5, 1.0, "candidate_threshold");
    if (!s.contains("propositions")) s["propositions"] = Json::object();
    require(s["propositions"].size() < 4096,
            "proposition bank capacity exceeded");
    require(!s["propositions"].contains(id), "proposition already exists");
    s["propositions"][id] = {
        {"id", id}, {"description", description}, {"polarity", "must_avoid"},
        {"scope", scope}, {"snapshots", Json::array()}, {"recovery", nullptr},
        {"candidate_threshold", candidate_threshold},
        {"detector_status", "unconfigured"}, {"created_by", actor}};
    refresh_critic_readiness(s);
    return s["propositions"][id];
  }
  if (op == "link_proposition_recovery") {
    auto &proposition = item("propositions", "proposition_id");
    const auto &bundle = item("bundles", "bundle_id");
    require(bundle["status"] == "approved" &&
                bundle["content_hash"] == p.at("content_hash"),
            "recovery link requires an exact approved bundle");
    const auto &failure = s["failures"].at(bundle["failure_id"].get<std::string>());
    require(failure.value("proposition_id", "") == proposition["id"].get<std::string>() &&
                failure["guidance_revision"] == bundle["guidance_revision"],
            "recovery bundle belongs to a different proposition or stale guidance");
    proposition["recovery"] = {{"bundle_id", bundle["id"]},
                                {"content_hash", bundle["content_hash"]}};
    return proposition;
  }
  if (op == "candidate") {
    const auto &w = p.at("window");
    require(w.value("status", "") == "ready" && w.contains("coverage") &&
                !w["coverage"].empty(),
            "candidate requires a completed evidence window");
    require(p.at("binding_hash") == s.at("binding_hash"),
            "candidate robot/topic binding mismatch");
    for (const auto &t : s["binding"]["topics"])
      if (t.value("required", true)) {
        const auto name = t.at("name").get<std::string>();
        require(w["coverage"].contains(name) &&
                    w["coverage"][name].value("valid_count", 0) >= 2,
                "candidate lacks required topic coverage");
      }
    const auto &d = p.at("detection");
    range(d.at("confidence"), 0, 1, "confidence");
    for (const auto *key : {"detector_id", "evidence_class", "source"})
      required_text(d.at(key), key);
    const auto session = required_text(p.at("session_id"), "session_id");
    const auto id =
        "c-" +
        digest(Json::array({session, w.at("epoch"), w.at("id")})).substr(0, 24);
    if (s["candidates"].contains(id))
      return s["candidates"][id];
    return s["candidates"][id] = {
        {"id", id}, {"session_id", session}, {"window", w},
        {"detection", d}, {"status", "pending_review"},
        {"approved_recovery_match", nullptr}, {"evidence", nullptr}};
  }
  if (op == "attach_evidence") {
    auto &c = item("candidates", "candidate_id");
    require(c["evidence"].is_null(),
            "candidate evidence is immutable once attached");
    verify_artifact(p.at("artifact"), "evidence_mcap");
    c["evidence"] = p["artifact"];
    // Routing happens only after immutable MCAP evidence is attached. An
    // unvalidated critic cannot reach this branch, even when a
    // random label happens to equal an approved evidence class.
    const auto &detection = c.at("detection");
    const auto deployed_critic = s.value("critic_deployment", Json(nullptr));
    const bool validated_deployment =
        detection.value("deployment_validated", false) && deployed_critic.is_object() &&
        deployed_critic.value("deployment_validated", false) &&
        detection.value("deployment_id", "") == deployed_critic.value("id", "");
    if (validated_deployment) {
      const auto evidence_class = detection.at("evidence_class").get<std::string>();
      for (auto it = s["bundles"].begin(); it != s["bundles"].end(); ++it) {
        const auto &bundle = it.value();
        if (bundle.value("status", "") != "approved") continue;
        const auto &contract = bundle.at("content").at("contract");
        bool semantic_match = contract.value("evidence_class", "") == evidence_class;
        if (!semantic_match && contract.contains("semantic_labels") &&
            contract["semantic_labels"].is_array())
          semantic_match = std::find(contract["semantic_labels"].begin(),
                                     contract["semantic_labels"].end(),
                                     Json(evidence_class)) !=
                           contract["semantic_labels"].end();
        if (!semantic_match) continue;
        c["status"] = "approved_recovery_match";
        c["approved_recovery_match"] = {
            {"bundle_id", it.key()}, {"recovery", bundle["content"]["recovery"]},
            {"match", contract.value("evidence_class", "") == evidence_class
                          ? "exact" : "approved_semantic_alias"}};
        break;
      }
    }
    return c;
  }
  if (op == "reject_candidate" || op == "accept_failure") {
    auto &c = item("candidates", "candidate_id");
    require(c["status"] == "pending_review", "candidate already reviewed");
    if (op == "reject_candidate") {
      c.update({{"status", "rejected"},
                {"reviewed_by", actor},
                {"reason", required_text(p.at("reason"), "reason")}});
      return c;
    }
    verify_artifact(c["evidence"], "evidence_mcap");
    const auto id = "f-" + unique_id().substr(0, 16);
    Json failure = {
        {"id", id},
        {"candidate_id", c["id"]},
        {"description", required_text(p.at("description"), "description")},
        {"guidance", required_text(p.at("guidance"), "guidance")},
        {"guidance_revision", 1},
        {"accepted_by", actor}};
    if (p.contains("proposition_id")) {
      auto &proposition = item("propositions", "proposition_id");
      failure["proposition_id"] = proposition["id"];
      // Keep exact immutable evidence and time bounds. A moment belongs to a
      // proposition only through this explicit acceptance, never label guessing.
      proposition["snapshots"].push_back({
          {"failure_id", id}, {"candidate_id", c["id"]},
          {"session_id", c["session_id"]}, {"window", c["window"]},
          {"evidence", c["evidence"]}, {"binding_hash", s["binding_hash"]},
          {"accepted_by", actor}});
      proposition["detector_status"] = "needs_compile";
    }
    s["failures"][id] = failure;
    c.update({{"status", "accepted_failure"},
              {"failure_id", id},
              {"reviewed_by", actor}});
    return failure;
  }
  if (op == "set_guidance") {
    auto &f = item("failures", "failure_id");
    if (!s["installed_deployment"].is_null())
      require(s["deployments"].at(
                  s["installed_deployment"].get<std::string>())["failure_id"] !=
                  f["id"],
              "deactivate installed recovery before revising its guidance");
    f.update({{"guidance", required_text(p.at("guidance"), "guidance")},
              {"guidance_revision", f["guidance_revision"].get<Ns>() + 1},
              {"edited_by", actor}});
    for (const auto *group : {"jobs", "bundles"})
      for (auto &entry : s[group])
        if (entry["failure_id"] == f["id"])
          entry["status"] = "superseded";
    return f;
  }
  if (op == "request_demonstration") {
    auto &f = item("failures", "failure_id");
    const auto &c = s["candidates"].at(f["candidate_id"].get<std::string>());
    verify_artifact(c["evidence"], "evidence_mcap");
    const auto id = "job-" + unique_id().substr(0, 16);
    Json j = {{"id", id},
              {"failure_id", f["id"]},
              {"guidance_revision", f["guidance_revision"]},
              {"guidance", f["guidance"]},
              {"description", f["description"]},
              {"evidence", c["evidence"]},
              {"window", c["window"]},
              {"binding", s["binding"]},
              {"binding_hash", s["binding_hash"]},
              {"requested_model",
               required_text(p.at("requested_model"), "requested_model")},
              {"status", "awaiting_external_generator"},
              {"requested_by", actor}};
    j["input_hash"] = digest(j);
    return s["jobs"][id] = j;
  }
  if (op == "submit_bundle") {
    auto &job = item("jobs", "job_id");
    auto &f = s["failures"].at(job["failure_id"].get<std::string>());
    require(job["status"] == "awaiting_external_generator" &&
                job["guidance_revision"] == f["guidance_revision"],
            "job is completed or superseded by new guidance");
    auto b = p.at("bundle");
    require(b.at("job_input_hash") == job["input_hash"] &&
                b.at("binding_hash") == s["binding_hash"],
            "bundle input or binding hash mismatch");
    for (const auto &[key, kind] : artifacts)
      verify_artifact(b.at(key), kind);
    const auto &c = b.at("contract");
    for (const auto *key :
         {"evidence_class", "entry_predicate", "completion_predicate",
          "fallback", "controller_id"})
      required_text(c.at(key), key);
    range(c.at("timeout_ms"), 0, 300000, "timeout_ms");
    require(c["timeout_ms"].get<double>() > 0, "timeout must be positive");
    range(c.at("confidence_min"), 0, 1, "confidence_min");
    range(c.at("retry_budget"), 0, 10, "retry_budget");
    require(c["retry_budget"].is_number_integer(),
            "retry_budget must be integer");
    const auto report =
        read_json(verify_artifact(b["validation"], "validation_report"));
    Json expected = {{"binding_hash", s["binding_hash"]},
                     {"job_input_hash", job["input_hash"]},
                     {"demonstration_sha256", b["demonstration"]["sha256"]},
                     {"critic_sha256", b["critic"]["sha256"]},
                     {"recovery_sha256", b["recovery"]["sha256"]},
                     {"contract_hash", digest(c)}};
    require(report.value("passed", Json()) == Json(true),
            "validation report must pass");
    for (auto it = expected.begin(); it != expected.end(); ++it)
      require(
          report.contains(it.key()) && report[it.key()] == it.value(),
          "validation report must bind every artifact and runtime contract");
    required_text(report.at("validator"), "validator");
    required_text(b.at("generator"), "generator");
    const auto hash = digest(b);
    const auto id = "b-" + hash;
    Json record = {{"id", id},
                   {"job_id", job["id"]},
                   {"failure_id", f["id"]},
                   {"guidance_revision", f["guidance_revision"]},
                   {"content", b},
                   {"content_hash", hash},
                   {"status", "pending_approval"},
                   {"submitted_by", actor}};
    job["status"] = "demonstration_submitted";
    return s["bundles"][id] = record;
  }
  if (op == "approve_bundle" || op == "reject_bundle") {
    auto &b = item("bundles", "bundle_id");
    require(b["status"] == "pending_approval",
            "bundle already reviewed or superseded");
    require(s["failures"].at(
                b["failure_id"].get<std::string>())["guidance_revision"] ==
                b["guidance_revision"],
            "bundle superseded by new guidance");
    require(p.at("content_hash") == b["content_hash"],
            "approval must name exact reviewed content_hash");
    b.update({{"status", op == "approve_bundle" ? "approved" : "rejected"},
              {"reviewed_by", actor}});
    return b;
  }
  if (op == "deploy") {
    auto &b = item("bundles", "bundle_id");
    auto &f = s["failures"].at(b["failure_id"].get<std::string>());
    require(b["status"] == "approved" &&
                f["guidance_revision"] == b["guidance_revision"],
            "only current approved bundles can be deployed");
    for (const auto &[key, kind] : artifacts)
      verify_artifact(b["content"].at(key), kind);
    const auto id = "d-" + unique_id().substr(0, 16);
    Json d = {{"id", id},
              {"bundle_id", b["id"]},
              {"failure_id", f["id"]},
              {"content_hash", b["content_hash"]},
              {"installed_by", actor},
              {"previous", s["installed_deployment"]},
              {"status", "installed_not_enforcing"},
              {"blockers", {"critic_inference", "recovery_controller"}},
              {"runtime_configuration",
               {{"binding_hash", s["binding_hash"]},
                {"critic", b["content"]["critic"]},
                {"recovery", b["content"]["recovery"]},
                {"contract", b["content"]["contract"]}}}};
    s["deployments"][id] = d;
    s["installed_deployment"] = id;
    s["enforcement_status"] = "installed_not_enforcing";
    return d;
  }
  if (op == "deactivate") {
    Json r = {{"previous", s["installed_deployment"]},
              {"reason", required_text(p.at("reason"), "reason")}};
    s["installed_deployment"] = nullptr;
    s["enforcement_status"] =
        s.value("critic_deployment", Json(nullptr)).is_object() ? "shadow" : "unavailable";
    refresh_critic_readiness(s);
    return r;
  }
  throw std::runtime_error("unknown workflow operation: " + op);
}
} // namespace harness::observation
