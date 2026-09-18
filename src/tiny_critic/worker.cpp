#include "worker.hpp"
namespace harness::tiny {
Worker::Worker() : thread_([this] { run(); }) {}
Worker::~Worker() {
  { std::lock_guard lock(mutex_); stopping_ = true; pending_.reset(); }
  changed_.notify_one();
  thread_.join();
}
void Worker::submit(Job job) {
  { std::lock_guard lock(mutex_); if (pending_) ++dropped_; pending_ = std::move(job); }
  changed_.notify_one();
}
void Worker::reset() {
  std::lock_guard lock(mutex_);
  ++generation_; pending_.reset(); result_ = {{"status", "warming_up"}};
}
Json Worker::latest(Ns now, Ns epoch, Ns max_age) {
  std::lock_guard lock(mutex_);
  auto result = result_;
  result["dropped_requests"] = dropped_;
  if (result.value("status", "") == "valid" &&
      (result.value("epoch", Ns{-1}) != epoch || result.value("t_ns", Ns{-1}) > now ||
       result.value("t_ns", Ns{-1}) < now - max_age)) {
    result["status"] = "stale";
    result["scores"] = Json::object();
  }
  return result;
}
void Worker::run() {
  for (;;) {
    Job job;
    std::uint64_t generation;
    {
      std::unique_lock lock(mutex_);
      changed_.wait(lock, [this] { return stopping_ || pending_.has_value(); });
      if (stopping_) return;
      generation = generation_; job = std::move(*pending_); pending_.reset();
    }
    Json result;
    try { result = job(); }
    catch (const std::exception &e) { result = {{"status", "unavailable"}, {"error", e.what()}}; }
    catch (...) { result = {{"status", "unavailable"}, {"error", "unknown inference error"}}; }
    std::lock_guard lock(mutex_);
    if (generation == generation_) result_ = std::move(result);
  }
}
}
