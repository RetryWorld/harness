#pragma once

#include "store.hpp"

namespace harness::observation {

struct DeviceCredentials {
  std::string device_id;
  std::string secret;
  std::string backend_url;
};

class WorkflowTransport {
public:
  virtual ~WorkflowTransport() = default;
  virtual void upload(const std::string &robot_id, const Json &snapshot,
                      const Json &receipts) = 0;
  virtual Json pending(const std::string &profile_id) = 0;
};

class WorkflowConnector {
public:
  WorkflowConnector(Store store, std::string robot_id,
                    WorkflowTransport &transport);
  Json cycle(bool heartbeat = false);

private:
  Store store_;
  std::string robot_id_;
  WorkflowTransport &transport_;
  Ns uploaded_revision_ = -1;
  Ns last_upload_monotonic_ns_ = 0;
};

DeviceCredentials load_device_credentials(const fs::path &path,
                                          const std::string &backend_override);
Json run_workflow_sync(const Options &options, bool once);

} // namespace harness::observation
