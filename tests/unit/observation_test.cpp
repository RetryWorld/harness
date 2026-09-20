#include "evidence.hpp"
#include "runtime.hpp"
#include "sync.hpp"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>

using namespace harness::observation;
namespace {
unsigned checks = 0;
void check(bool condition, const std::string &message) {
  ++checks;
  require(condition, "test failed: " + message);
}
template <class F> void rejects(F action, const std::string &fragment = "") {
  try {
    action();
  } catch (const std::exception &error) {
    check(std::string(error.what()).find(fragment) != std::string::npos,
          "unexpected error: " + std::string(error.what()));
    return;
  }
  throw std::runtime_error("expected rejection: " + fragment);
}
struct Fixture {
  fs::path root =
      fs::temp_directory_path() / ("harness-native-test-" + unique_id());
  Store store = Store::create(root / "store", HARNESS_OBSERVATION_CONFIG);
  Json binding = store.snapshot()["binding"];
  ~Fixture() { fs::remove_all(root); }
  Json apply(const std::string &operation, const Json &payload) {
    return store.apply({{"schema_version", 1},
                        {"request_id", unique_id()},
                        {"actor", "test-operator"},
                        {"expected_revision", store.snapshot()["revision"]},
                        {"operation", operation},
                        {"payload", payload}})["result"];
  }
  Json artifact(const std::string &kind, const Json &body = nullptr) {
    const auto path = root / unique_id();
    {
      std::ofstream stream(path, std::ios::binary);
      if (kind.ends_with("mcap"))
        stream << std::string("\x89MCAP0\r\n", 8)
               << "envelope-only test fixture, not a demonstration"
               << std::string("\x89MCAP0\r\n", 8);
      else
        stream << (body.is_null() ? "opaque test model" : body.dump());
    }
    return store.import_artifact(path, kind);
  }
  Json candidate(bool attach = true) {
    Json coverage = Json::object();
    for (const auto &t : binding["topics"])
      coverage[t["name"].get<std::string>()] = {{"valid_count", 5}};
    auto c = apply("candidate", {{"window",
                                  {{"id", unique_id()},
                                   {"epoch", 2},
                                   {"start_ns", 1},
                                   {"end_ns", 100},
                                   {"status", "ready"},
                                   {"coverage", coverage}}},
                                 {"session_id", "session"},
                                 {"binding_hash", digest(binding)},
                                 {"detection",
                                  {{"detector_id", "fixture"},
                                   {"source", "test_injection"},
                                   {"confidence", 0.8},
                                   {"evidence_class", "target_moved"}}}});
    if (attach)
      apply("attach_evidence", {{"candidate_id", c["id"]},
                                {"artifact", artifact("evidence_mcap")}});
    return c;
  }
  Json bundle() {
    const auto c = candidate();
    const auto f =
        apply("accept_failure", {{"candidate_id", c["id"]},
                                 {"description", "failed grasp"},
                                 {"guidance", "hold and reacquire"}});
    const auto j = apply(
        "request_demonstration",
        {{"failure_id", f["id"]}, {"requested_model", "external-fixture"}});
    Json content = {{"job_input_hash", j["input_hash"]},
                    {"binding_hash", digest(binding)},
                    {"generator", "external-fixture"},
                    {"demonstration", artifact("demonstration_mcap")},
                    {"critic", artifact("critic_model")},
                    {"recovery", artifact("recovery_model")},
                    {"contract",
                     {{"evidence_class", "target_moved"},
                      {"entry_predicate", "target_displaced"},
                      {"completion_predicate", "target_reacquired"},
                      {"fallback", "hold"},
                      {"controller_id", "so101"},
                      {"confidence_min", 0.7},
                      {"timeout_ms", 1000},
                      {"retry_budget", 1}}}};
    content["validation"] =
        artifact("validation_report",
                 {{"passed", true},
                  {"validator", "fixture"},
                  {"binding_hash", digest(binding)},
                  {"job_input_hash", j["input_hash"]},
                  {"contract_hash", digest(content["contract"])},
                  {"demonstration_sha256", content["demonstration"]["sha256"]},
                  {"critic_sha256", content["critic"]["sha256"]},
                  {"recovery_sha256", content["recovery"]["sha256"]}});
    return apply("submit_bundle", {{"job_id", j["id"]}, {"bundle", content}});
  }
  Json deploy() {
    const auto b = bundle();
    apply("approve_bundle",
          {{"bundle_id", b["id"]}, {"content_hash", b["content_hash"]}});
    return apply("deploy", {{"bundle_id", b["id"]}});
  }
};
struct FakeCritic final : CriticAdapter {
  std::optional<Detection> detection = Detection{"tiny", "target_moved", 0.9};
  bool ready() const override { return true; }
  bool load(const fs::path &, const Json &, const Json &) override {
    return true;
  }
  std::optional<Detection> evaluate(const Observations &) override {
    return detection;
  }
};
struct FakeController final : ControllerAdapter {
  std::string outcome = "running";
  bool hold_ack = true, begin_ack = true, resume_ack = true;
  bool ready() const override { return true; }
  bool prepare(const fs::path &, const Json &, const Json &) override {
    return true;
  }
  bool begin() override { return begin_ack; }
  std::string poll() override { return outcome; }
  bool resume() override { return resume_ack; }
  bool hold(const std::string &) override { return hold_ack; }
};
struct FakeWorkflowTransport final : WorkflowTransport {
  Json commands = Json::array();
  std::vector<Json> uploads;
  bool fail_upload = false;
  void upload(const std::string &robot_id, const Json &snapshot,
              const Json &receipts) override {
    if (fail_upload) throw std::runtime_error("offline");
    uploads.push_back({{"robot_id", robot_id},
                       {"snapshot", snapshot},
                       {"receipts", receipts}});
  }
  Json pending(const std::string &) override {
    const auto result = commands;
    commands = Json::array();
    return result;
  }
};
void evidence_tests() {
  const auto config = load_config(HARNESS_OBSERVATION_CONFIG);
  Evidence e(config, "test");
  e.tick(10000000000LL, 100);
  rejects([&] { e.check_live(100); }, "topic");
  const auto id = e.capture(5, 3, "cube displacement")["id"].get<std::string>();
  rejects([&] { e.promote(id, "failure", "operator"); }, "finish");
  for (const auto &t : config["topics"])
    for (Ns seconds : {6, 10, 12})
      e.sample(t["name"], seconds * 1000000000LL, nullptr, 100);
  e.check_live(100);
  e.tick(13000000000LL, 100);
  check(e.state["windows"][0]["status"] == "ready", "window completes");
  check(e.state["failures"].empty(), "no automatic failure confirmation");
  const auto f = e.promote(id, "failure", "operator");
  check(e.promote(id, "again", "operator") == f, "legacy promotion idempotent");
  check(e.state["proposals"].size() == 2 &&
            e.state["proposals"][0]["executable"] == false,
        "legacy templates nonexecutable");
  for (double invalid :
       {0.0, -1.0, 21.0, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()})
    rejects([&] { e.capture(invalid, 3, "bad"); });
  e.capture(1, 2, "reset");
  e.tick(1, 100);
  check(e.state["windows"].back()["status"] == "clock_reset",
        "reset invalidates collecting window");
  e.capture(1, 2, "stalled");
  rejects([&] { e.check_live(20000000100LL); }, "clock stopped");
  e.expire(20000000100LL);
  check(e.state["windows"].back()["status"] == "incomplete",
        "stalled post-window expires");
  Evidence bad(config, "bad");
  bad.tick(10000000000LL);
  const auto bad_id =
      bad.capture(5, 3, "invalid action")["id"].get<std::string>();
  for (const auto &t : config["topics"])
    for (Ns seconds : {6, 12})
      bad.sample(t["name"], seconds * 1000000000LL, nullptr, monotonic_ns(),
                 t["role"] != "action");
  bad.tick(13000000000LL);
  rejects([&] { bad.promote(bad_id, "failure", "operator"); }, "insufficient");
}
void workflow_tests() {
  {
    const auto config_path = fs::path(HARNESS_OBSERVATION_CONFIG);
    require(::setenv("REARGUARD_OBSERVATION_CONFIG_DIR",
                     config_path.parent_path().c_str(), 1) == 0,
            "could not set config path for test");
    check(load_config("so101")["schema_version"] == 1,
          "installed config resolves by logical name");
    ::unsetenv("REARGUARD_OBSERVATION_CONFIG_DIR");
  }
  {
    Fixture sync;
    FakeWorkflowTransport transport;
    WorkflowConnector connector(sync.store,
                                "00000000-0000-4000-8000-000000000100",
                                transport);
    connector.cycle();
    check(transport.uploads.size() == 1 &&
              transport.uploads.back()["snapshot"]["revision"] == 0,
          "connector uploads initial snapshot");
    connector.cycle();
    check(transport.uploads.size() == 1,
          "connector does not duplicate unchanged snapshots");
    transport.commands.push_back(
        {{"schema_version", 1},
         {"request_id", "00000000-0000-4000-8000-000000000001"},
         {"actor", "web-user"},
         {"expected_revision", 0},
         {"operation", "deactivate"},
         {"payload", {{"reason", "connector test"}}}});
    const auto applied = connector.cycle();
    check(applied["commands_received"] == 1 &&
              transport.uploads.size() == 2 &&
              transport.uploads.back()["snapshot"]["revision"] == 1 &&
              transport.uploads.back()["receipts"][0]["status"] == "applied",
          "connector applies command and uploads receipt with new snapshot");
    transport.commands.push_back(
        {{"schema_version", 1},
         {"request_id", "00000000-0000-4000-8000-000000000002"},
         {"actor", "web-user"},
         {"expected_revision", 0},
         {"operation", "deactivate"},
         {"payload", {{"reason", "stale command"}}}});
    connector.cycle();
    check(transport.uploads.size() == 3 &&
              transport.uploads.back()["snapshot"]["revision"] == 1 &&
              transport.uploads.back()["receipts"][0]["status"] == "rejected",
          "connector rejects stale command without changing profile");
    transport.commands.push_back(
        {{"schema_version", 1},
         {"request_id", "00000000-0000-4000-8000-000000000003"},
         {"actor", "web-user"},
         {"expected_revision", 1},
         {"operation", "deactivate"},
         {"payload", {{"reason", "offline receipt test"}}}});
    transport.fail_upload = true;
    rejects([&] { connector.cycle(); }, "offline");
    check(sync.store.snapshot()["revision"] == 2 &&
              sync.store.pending_sync_receipts().size() == 1,
          "applied command and receipt survive an upload failure");
    transport.fail_upload = false;
    connector.cycle();
    check(transport.uploads.size() == 4 &&
              transport.uploads.back()["snapshot"]["revision"] == 2 &&
              transport.uploads.back()["receipts"][0]["request_id"] ==
                  "00000000-0000-4000-8000-000000000003" &&
              sync.store.pending_sync_receipts().empty(),
          "durable command receipt uploads and clears after reconnecting");
  }
  {
    Fixture offline;
    FakeWorkflowTransport transport;
    transport.fail_upload = true;
    WorkflowConnector connector(offline.store,
                                "00000000-0000-4000-8000-000000000101",
                                transport);
    const auto local_profile = offline.store.snapshot();
    rejects([&] { connector.cycle(); }, "offline");
    check(offline.store.snapshot() == local_profile,
          "network failure leaves the local harness unchanged");
    transport.fail_upload = false;
    connector.cycle();
    check(transport.uploads.size() == 1 &&
              transport.uploads.back()["snapshot"] == local_profile,
          "connector uploads the retained harness after reconnecting");
  }
  Fixture f;
  Json retry = {{"schema_version", 1},       {"request_id", "retry"},
                {"actor", "operator"},       {"expected_revision", 0},
                {"operation", "deactivate"}, {"payload", {{"reason", "test"}}}};
  const auto first = f.store.apply(retry);
  check(Store(f.store.root).apply(retry) == first, "durable idempotency");
  check(f.store.outbox().size() == 2, "retry does not duplicate event");
  check(f.store.outbox().front()["event"]["operation"] == "initialize",
        "profile creation enters sync outbox");
  retry["payload"]["reason"] = "changed";
  rejects([&] { f.store.apply(retry); }, "different content");
  retry["request_id"] = "conflict";
  const auto before = f.store.snapshot();
  rejects([&] { f.store.apply(retry); }, "revision conflict");
  check(f.store.snapshot() == before, "conflict rolls back");
  rejects([&] { Store::create(f.store.root, HARNESS_OBSERVATION_CONFIG); },
          "already exists");
  rejects(
      [&] { f.apply("candidate", {{"window", {{"status", "collecting"}}}}); },
      "completed");
  auto c = f.candidate(false);
  rejects(
      [&] {
        f.apply("accept_failure", {{"candidate_id", c["id"]},
                                   {"description", "failure"},
                                   {"guidance", "hold"}});
      },
      "artifact");
  f.apply("reject_candidate",
          {{"candidate_id", c["id"]}, {"reason", "normal motion"}});
  rejects([&] { f.apply("accept_failure", {{"candidate_id", c["id"]}}); },
          "already reviewed");
  auto b = f.bundle();
  rejects([&] { f.apply("deploy", {{"bundle_id", b["id"]}}); }, "approved");
  rejects(
      [&] {
        f.apply("approve_bundle",
                {{"bundle_id", b["id"]}, {"content_hash", "wrong"}});
      },
      "exact");
  const auto new_job =
      f.apply("request_demonstration", {{"failure_id", b["failure_id"]},
                                        {"requested_model", "external"}});
  auto content = b["content"];
  content["job_input_hash"] = new_job["input_hash"];
  content["contract"]["timeout_ms"] = 1234;
  const auto snapshot = f.store.snapshot();
  rejects(
      [&] {
        f.apply("submit_bundle",
                {{"job_id", new_job["id"]}, {"bundle", content}});
      },
      "validation report");
  check(f.store.snapshot() == snapshot, "validation rollback");
  f.apply("set_guidance",
          {{"failure_id", b["failure_id"]}, {"guidance", "new guidance"}});
  rejects(
      [&] {
        f.apply("approve_bundle",
                {{"bundle_id", b["id"]}, {"content_hash", b["content_hash"]}});
      },
      "superseded");
  const auto d = f.deploy();
  check(Store(f.store.root).snapshot()["installed_deployment"] == d["id"],
        "deployment survives reopen");
  check(d["status"] == "installed_not_enforcing", "honest installed status");
  check(f.store.outbox().back()["profile"] == f.store.snapshot(),
        "outbox holds exact snapshot");
  rejects(
      [&] {
        f.apply("set_guidance", {{"failure_id", d["failure_id"]},
                                 {"guidance", "cannot change installed"}});
      },
      "deactivate");
  CriticInferencePlaceholder critic;
  RecoveryControllerPlaceholder controller;
  Runtime runtime(f.store, critic, controller);
  rejects([&] { runtime.activate(); }, "adapters");
  b = f.bundle();
  f.apply("approve_bundle",
          {{"bundle_id", b["id"]}, {"content_hash", b["content_hash"]}});
  {
    std::ofstream stream(f.store.root / "artifacts" /
                         b["content"]["critic"]["sha256"].get<std::string>());
    stream << "tampered";
  }
  rejects([&] { f.apply("deploy", {{"bundle_id", b["id"]}}); }, "hash/size");
}
void runtime_tests() {
  Fixture f;
  f.deploy();
  FakeCritic critic;
  FakeController controller;
  Runtime r(f.store, critic, controller);
  r.activate();
  check(r.step({}, 0)["kind"] == "recovery_started", "recovery begins");
  controller.outcome = "succeeded";
  check(r.step({}, 0.1)["kind"] == "recovery_completed",
        "completion acknowledged");
  check(r.step({}, 0.2).is_null(), "latched evidence does not retrigger");
  critic.detection.reset();
  r.step({}, 0.3);
  critic.detection = Detection{"tiny", "target_moved", 0.9};
  check(r.step({}, 0.4)["kind"] == "recovery_started",
        "new rising evidence triggers");
  check(r.step({}, 1.4)["kind"] == "fallback", "monotonic timeout");
  r.deactivate();
  r.activate();
  r.step({}, 2);
  controller.outcome = "failed";
  check(r.step({}, 2.1)["kind"] == "recovery_retry", "bounded retry");
  check(r.step({}, 2.2)["kind"] == "fallback", "retry exhaustion");
  r.deactivate();
  r.activate();
  check(r.step({}, 3, false)["hold_acknowledged"] == true,
        "stale observations fallback");
  r.deactivate();
  r.activate();
  controller.hold_ack = false;
  check(r.step({}, 4, false)["hold_acknowledged"] == false &&
            r.status == "fault_unconfirmed_hold",
        "no false hold claim");
  controller.hold_ack = true;
  r.deactivate();
  r.activate();
  f.apply("deactivate", {{"reason", "operator"}});
  check(r.step({}, 5)["kind"] == "fallback", "deployment change stops runtime");
}
} // namespace
int main() {
  try {
    check(encoded(Json::parse(
              "{\"z\":1e-5,\"a\":1e15,\"x\":1e16,\"unicode\":\"α\"}")) ==
              "{\"a\":1000000000000000.0,\"unicode\":\"\\u03b1\",\"x\":1e+16,"
              "\"z\":1e-05}",
          "legacy hash encoding");
    check(
        digest(Json::object()) ==
            "44136fa355b3678a1146ad16f7e8649e94fb4fc21fe77e8310c060f61caaff8a",
        "SHA256 known vector");
    evidence_tests();
    workflow_tests();
    runtime_tests();
    std::cout << checks << " native observation/workflow checks passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
