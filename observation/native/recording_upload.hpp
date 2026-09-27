#pragma once

#include "sync.hpp"

namespace harness::observation {

// Publishes every closed rosbag2 MCAP segment exactly once. A segment is
// marked submitted only after R2 acknowledges the bytes and the backend has
// accepted them for asynchronous verification/indexing.
Json upload_closed_recordings(const fs::path &store, const fs::path &session,
                              const DeviceCredentials &credentials);

// Long-running, independently supervised upload queue. Recording and the
// safety service continue even while this process is blocked on cloud I/O.
Json run_recording_uploader(const Options &options);

} // namespace harness::observation
