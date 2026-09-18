#include "store.hpp"
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
} // namespace
Store::Store(fs::path directory) : root(fs::absolute(std::move(directory))) {
  require(fs::is_regular_file(root / "workflow.sqlite3"),
          "workflow store missing; run workflow init first");
}
Store Store::create(const fs::path &directory, const fs::path &config) {
  auto binding = load_config(config);
  fs::create_directories(directory);
  const auto path = directory / "workflow.sqlite3";
  const int fd = ::open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0600);
  require(fd >= 0, "store already exists or cannot be created");
  ::close(fd);
  Json state = {{"schema_version", 1},
                {"profile_id", "profile-" + unique_id()},
                {"revision", 0},
                {"binding", binding},
                {"binding_hash", digest(binding)},
                {"candidates", Json::object()},
                {"failures", Json::object()},
                {"jobs", Json::object()},
                {"bundles", Json::object()},
                {"deployments", Json::object()},
                {"installed_deployment", nullptr},
                {"enforcement_status", "unavailable"},
                {"placeholders",
                 {"database_sync", "critic_inference", "recovery_controller"}}};
  Database db(path);
  db.exec(
      "PRAGMA journal_mode=WAL; CREATE TABLE profile(id INTEGER PRIMARY KEY "
      "CHECK(id=1), body TEXT NOT NULL); CREATE TABLE receipts(id TEXT PRIMARY "
      "KEY, request_hash TEXT NOT NULL, response TEXT NOT NULL); CREATE TABLE "
      "outbox(seq INTEGER PRIMARY KEY AUTOINCREMENT, body TEXT NOT NULL);");
  Statement q(db, "INSERT INTO profile VALUES(1, ?)");
  q.bind(1, encoded(state));
  q.next();
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
    state["revision"] = state.at("revision").get<Ns>() + 1;
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
    return s["candidates"][id] = {{"id", id},
                                  {"session_id", session},
                                  {"window", w},
                                  {"detection", d},
                                  {"status", "pending_review"},
                                  {"evidence", nullptr}};
  }
  if (op == "attach_evidence") {
    auto &c = item("candidates", "candidate_id");
    require(c["evidence"].is_null(),
            "candidate evidence is immutable once attached");
    verify_artifact(p.at("artifact"), "evidence_mcap");
    c["evidence"] = p["artifact"];
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
    return d;
  }
  if (op == "deactivate") {
    Json r = {{"previous", s["installed_deployment"]},
              {"reason", required_text(p.at("reason"), "reason")}};
    s["installed_deployment"] = nullptr;
    return r;
  }
  throw std::runtime_error("unknown workflow operation: " + op);
}
} // namespace harness::observation
