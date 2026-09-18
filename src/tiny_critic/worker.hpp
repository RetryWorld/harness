#pragma once
#include "critic.hpp"
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

namespace harness::tiny {
// For a single inference owner. At most one pending request; replace old work.
// The callback owns decoding/encoding outside the caller's observation loop.
// Jobs/results carry generation tokens so resets cannot publish old evidence.
class Worker {
public:
  using Job = std::function<Json()>;
private:
  std::mutex mutex_;
  std::condition_variable changed_;
  bool stopping_ = false;
  std::uint64_t generation_ = 0, dropped_ = 0;
  std::optional<Job> pending_;
  Json result_ = {{"status", "warming_up"}};
  std::thread thread_;
  void run();
public:
  Worker();
  ~Worker();
  Worker(const Worker &) = delete;
  Worker &operator=(const Worker &) = delete;
  void submit(Job);
  void reset();
  Json latest(Ns now, Ns epoch, Ns max_age);
};
}
