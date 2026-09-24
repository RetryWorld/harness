#pragma once
#include "common.hpp"
namespace harness::observation {
class Store {
public:
  fs::path root;
  explicit Store(fs::path directory);
  static Store create(const fs::path &directory, const fs::path &config);
  Json snapshot() const;
  Json outbox(Ns after = 0) const;
  Json import_artifact(const fs::path &source, const std::string &kind) const;
  fs::path verify_artifact(const Json &reference,
                           const std::string &kind) const;
  Json apply(const Json &request) const;
  Json apply_remote(const Json &request) const;
  Json pending_sync_receipts() const;
  void acknowledge_sync_receipts(const Json &receipts) const;
  void record_enforcement(const Json &record) const;
  Json pending_enforcements() const;
  void acknowledge_enforcements(const Json &records) const;

private:
  Json apply_internal(const Json &request, bool queue_sync_receipt) const;
  Json transition(Json &state, const std::string &operation,
                  const Json &payload, const std::string &actor) const;
};
} // namespace harness::observation
