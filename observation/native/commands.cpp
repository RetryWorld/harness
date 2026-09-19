#include "common.hpp"
#include "store.hpp"
#include <iostream>
namespace harness::observation {
namespace {
Json workflow(const Options &o) {
  const auto root = o.path("--store");
  if (o.command == "init") {
    o.allow("--store --config");
    return Store::create(root, o.need("--config")).snapshot();
  }
  Store store(root);
  if (o.command == "show") {
    o.allow("--store");
    return store.snapshot();
  }
  if (o.command == "outbox") {
    o.allow("--store --after");
    return {{"connector", "placeholder"},
            {"delivery_status", "pending"},
            {"events", store.outbox(o.integer("--after", 0))}};
  }
  if (o.command == "artifact") {
    o.allow("--store --kind --file");
    const auto kind = o.need("--kind");
    require(kind != "evidence_mcap",
            "use export-window to attach recorded evidence");
    return store.import_artifact(o.need("--file"), kind);
  }
  if (o.command == "apply") {
    o.allow("--store --operation --payload --actor --expected-revision "
            "--request-id");
    o.need("--expected-revision");
    const auto op = o.need("--operation");
    require(
        op != "candidate" && op != "attach_evidence",
        "candidate and evidence registration require the observer/exporter");
    return store.apply(
        {{"schema_version", 1},
         {"request_id", o.get("--request-id", unique_id())},
         {"actor", o.need("--actor")},
         {"expected_revision", o.integer("--expected-revision", -1)},
         {"operation", op},
         {"payload", read_json(o.need("--payload"))}});
  }
  if (o.command == "generation-request") {
    o.allow("--store --job --out");
    const auto job = store.snapshot()["jobs"].at(o.need("--job"));
    require(job["status"] == "awaiting_external_generator",
            "generation job no longer pending");
    const auto artifact =
        store.verify_artifact(job["evidence"], "evidence_mcap");
    const auto out = o.path("--out");
    require(fs::create_directory(out), "output directory already exists");
    fs::copy_file(artifact, out / "evidence.mcap");
    atomic_json(out / "request.json", job);
    return {{"path", out.string()},
            {"status", "ready_for_external_generator"},
            {"input_hash", job["input_hash"]},
            {"network_request_sent", false}};
  }
  if (o.command == "export-window") {
    o.allow("--store --session --candidate");
    const auto state = store.snapshot();
    const auto id = o.need("--candidate");
    const auto &candidate = state["candidates"].at(id);
    const auto session = o.path("--session");
    const auto meta = read_json(session / "session.json");
    require(candidate["session_id"] == meta["session_id"],
            "candidate belongs to a different recording session");
    if (!candidate["evidence"].is_null()) {
      store.verify_artifact(candidate["evidence"], "evidence_mcap");
      return {{"artifact", candidate["evidence"]}, {"already_attached", true}};
    }
    const auto temp =
        fs::temp_directory_path() / ("harness-window-" + unique_id());
    try {
      const auto path = export_window(session, candidate["window"], temp);
      const auto artifact = store.import_artifact(path, "evidence_mcap");
      fs::remove_all(temp);
      return store.apply(
          {{"schema_version", 1},
           {"request_id", "evidence-" + id},
           {"actor", "edge_exporter"},
           {"expected_revision", state["revision"]},
           {"operation", "attach_evidence"},
           {"payload", {{"candidate_id", id}, {"artifact", artifact}}}});
    } catch (...) {
      fs::remove_all(temp);
      throw;
    }
  }
  throw std::runtime_error("unknown workflow command: " + o.command);
}
Json observe(const Options &o) {
  if (o.command == "start") {
    o.allow("--config --session --store --storage --domain-id --wall-timeout");
    return ros_start(o);
  }
  if (o.command == "profile" && o.values.contains("--config")) {
    o.allow("--config");
    return load_config(o.need("--config"));
  }
  const auto session = o.path("--session");
  const auto meta = read_json(session / "session.json");
  if (o.command == "profile") {
    o.allow("--session");
    return !meta.value("workflow_store", Json()).is_null()
               ? Store(meta["workflow_store"].get<std::string>()).snapshot()
               : meta["config"];
  }
  if (o.command == "status" || o.command == "windows" ||
      o.command == "proposals") {
    o.allow("--session");
    auto state = read_json(session / "state.json");
    if (o.command != "status")
      return state.at(o.command);
    state["heartbeat_age_wall_s"] =
        static_cast<double>(wall_ns() -
                            state.value<Ns>("heartbeat_wall_ns", 0)) /
        1e9;
    return state;
  }
  if (o.command == "capture") {
    o.allow("--session --before --after --label");
    o.need("--label");
  } else if (o.command == "candidate") {
    o.allow("--session --before --after --label --detector-id --evidence-class "
            "--confidence");
    o.need("--evidence-class");
    o.need("--confidence");
  } else if (o.command == "promote") {
    o.allow("--session --window --description --operator");
    o.need("--window");
    o.need("--description");
    o.need("--operator");
  } else if (o.command == "activate")
    o.allow("--session");
  else
    throw std::runtime_error("unknown observation command: " + o.command);
  return ros_request(o, meta);
}
Json scan(const Options &o) {
  if (o.command == "setup") {
    o.allow("--out --domain-min --domain-max --parallelism --settle-ms");
    require(o.integer("--settle-ms", 750) >= 0 &&
                o.integer("--settle-ms", 750) <= 10000,
            "settle-ms must be between 0 and 10000");
    auto result = ros_discover(o);
    if (o.values.contains("--out")) {
      const auto output = fs::absolute(o.need("--out"));
      require(output.filename() != ".", "--out must name a file");
      if (!output.parent_path().empty())
        fs::create_directories(output.parent_path());
      atomic_json(output, result);
      result["output"] = output.string();
    }
    return result;
  }
  if (o.command == "runtime") {
    o.allow("--config --session --store --storage --domain-id --wall-timeout");
    o.need("--domain-id");
    return ros_start(o);
  }
  throw std::runtime_error("unknown scan command: " + o.command);
}
} // namespace
int command_main(const std::string &group, int argc, char **argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--help") {
      const std::map<std::string, std::string> usage =
          group == "observe"
              ? std::map<
                    std::string,
                    std::string>{{"start",
                                  "--config FILE --session DIR [--store DIR] "
                                  "[--storage mcap|sqlite3] [--domain-id 71] "
                                  "[--wall-timeout 1800]"},
                                 {"profile", "--config FILE | --session DIR"},
                                 {"status", "--session DIR"},
                                 {"windows", "--session DIR"},
                                 {"proposals", "--session DIR"},
                                 {"activate", "--session DIR"},
                                 {"capture", "--session DIR --label TEXT "
                                             "[--before 5] [--after 3]"},
                                 {"candidate",
                                  "--session DIR --evidence-class TEXT "
                                  "--confidence NUMBER [--detector-id TEXT] "
                                  "[--before 5] [--after 3] [--label TEXT]"},
                                 {"promote",
                                  "--session DIR --window ID --description "
                                  "TEXT --operator TEXT"}}
              : group == "scan"
                    ? std::map<std::string, std::string>{
                          {"setup", "[--domain-min 0] [--domain-max 232] "
                                    "[--settle-ms 750] [--parallelism 16] "
                                    "[--out inventory.json]"},
                          {"runtime", "--config FILE --session DIR [--store "
                                      "DIR] [--storage mcap|sqlite3] "
                                      "--domain-id N [--wall-timeout 1800]"}}
                    : std::map<std::string, std::string>{
                    {"init", "--store DIR --config FILE"},
                    {"show", "--store DIR"},
                    {"outbox", "--store DIR [--after 0]"},
                    {"artifact", "--store DIR --kind "
                                 "demonstration_mcap|critic_model|recovery_"
                                 "model|validation_report --file FILE"},
                    {"apply",
                     "--store DIR --operation OP --payload FILE --actor TEXT "
                     "--expected-revision N [--request-id ID]"},
                    {"export-window",
                     "--store DIR --session DIR --candidate ID"},
                    {"generation-request", "--store DIR --job ID --out DIR"}};
      require(usage.contains(argv[0]), "unknown subcommand");
      std::cout << "rearguard " << group << ' ' << argv[0] << ' '
                << usage.at(argv[0]) << '\n';
      return 0;
    }
    if (argc == 0 || (argc == 1 && std::string(argv[0]) == "--help")) {
      std::cout
          << "rearguard " << group
          << (group == "observe" ? " <start|profile|status|windows|proposals|"
                                   "capture|candidate|promote|activate>\n"
              : group == "scan" ? " <setup|runtime>\n"
                                 : " <init|show|outbox|artifact|apply|export-"
                                   "window|generation-request>\n")
          << "See edge/observation/README.md and WORKFLOW.md for options.\n";
      return argc == 0 ? 1 : 0;
    }
    Options o(argc, argv);
    const auto result = group == "observe" ? observe(o)
                        : group == "scan"  ? scan(o)
                                            : workflow(o);
    if (!result.is_null())
      std::cout << result.dump(2) << std::endl;
    return result.is_object() && result.contains("ok") && result["ok"] == false
               ? 1
               : 0;
  } catch (const std::exception &error) {
    std::cerr << "harness " << group << ": " << error.what() << '\n';
    return 1;
  }
}
} // namespace harness::observation
