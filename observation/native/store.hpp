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

private:
  Json transition(Json &state, const std::string &operation,
                  const Json &payload, const std::string &actor) const;
};
} // namespace harness::observation
