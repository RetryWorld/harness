#include "critic.hpp"
#include "worker.hpp"
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

using namespace harness::tiny;
using harness::observation::require;
int main() {
  try {
    Spec s = Spec::parse({{"steps",16},{"cameras",2},{"joints",32},
                         {"min_steps",4},{"sample_hz",2},{"max_age_s",0.75}});
    require(Frame(s).vision.size() == 2560, "frame storage dimensions");
    std::vector<std::uint8_t> rgb{255,0,0,99,99, 255,0,0,88,88};
    std::vector<float> output(3 * 224 * 224);
    preprocess(rgb,1,2,5,output);
    require(std::abs(output[0] - (1.F - 0.48145466F) / 0.26862954F) < 1e-5F, "red channel normalization");
    require(std::abs(output[224 * 224] + 0.4578275F / 0.26130258F) < 1e-5F, "green channel normalization");
    bool rejected = false;
    try { preprocess(rgb,0,2,5,output); } catch (const std::exception &) { rejected = true; }
    require(rejected, "zero width rejected");
    Worker worker;
    std::promise<void> started, release, done;
    auto released = release.get_future().share();
    worker.submit([&] { started.set_value(); released.wait(); return Json{{"status","valid"},{"t_ns",1},{"epoch",0}}; });
    started.get_future().wait();
    worker.reset();
    worker.submit([] { return Json{{"status","valid"},{"t_ns",2},{"epoch",0}}; });
    worker.submit([&] { done.set_value(); return Json{{"status","valid"},{"t_ns",3},{"epoch",1},{"scores",{{"failure",0.7}}}}; });
    release.set_value(); done.get_future().wait();
    Json result;
    for (int i = 0; i < 1000; ++i) {
      result = worker.latest(3,1,10);
      if (result.value("status", "") == "valid") break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(result.at("t_ns") == 3 && result.at("dropped_requests") == 1, "latest-only queue and generation reset");
    result = worker.latest(100,1,10);
    require(result.at("status") == "stale" && result.at("scores").empty(), "old scores cannot be consumed");
    require(worker.latest(3,2,10).at("status") == "stale", "epoch mismatch invalidates scores");
    std::cout << "tiny critic preprocessing, bounds, queue and freshness tests passed\n";
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
