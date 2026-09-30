#include "critic.hpp"
#include "worker.hpp"
#include "propositions.hpp"
#include "critic_features.hpp"
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

using namespace harness::tiny;
using harness::observation::require;
int main() {
  try {
    harness::observation::CriticFeatureAccumulator feature_accumulator;
    feature_accumulator.joint({0.1, -0.2}, {0.3, -0.4}, {0.5, -0.6});
    feature_accumulator.action({0.2, -0.1});
    const std::vector<std::uint8_t> pixels{255, 0, 0, 0, 255, 0};
    feature_accumulator.image(pixels, 2, 1, 6, 3);
    const auto feature_embedding = feature_accumulator.embedding(
        harness::observation::critic_joint_state |
        harness::observation::critic_action |
        harness::observation::critic_camera);
    double feature_norm = 0;
    for (const auto value : feature_embedding)
      feature_norm += static_cast<double>(value) * static_cast<double>(value);
    require(std::abs(feature_norm - 1) < 1e-5 &&
                feature_accumulator.roles() == 7,
            "role feature encoder emits a normalized deterministic embedding");
    const auto joint_only = feature_accumulator.embedding(
        harness::observation::critic_joint_state);
    require(joint_only[24] == 0 && joint_only[56] == 0,
            "proposition role mask excludes unrequested modalities");
    PropositionEmbedding embedding{};
    embedding[0] = 1;
    PropositionHead head;
    head.centers[0] = embedding;
    head.center_count = 1;
    head.required_hz = 200;
    head.max_input_age_ns = 10000000;
    head.required_modalities = 1;
    auto empty = head;
    empty.center_count = 0;
    PropositionBank bank({head, head, empty}, 200);
    std::array<PropositionScore, 3> scores;
    std::array<std::int64_t, 32> times{};
    require(bank.evaluate(embedding, 0, 0, 0, times, scores), "bank output dimensions");
    require(scores[0].similarity == 1 && scores[1].similarity == 1 &&
            scores[2].state == PropositionState::unknown,
            "independent overlapping propositions and empty guard abstention");
    bank.evaluate(embedding, 0, 6000000, 0, times, scores);
    require(scores[0].state == PropositionState::deadline_missed, "MUST rate gap is explicit");
    bank.evaluate(embedding, 0, 11000000, 0, times, scores);
    require(scores[0].state == PropositionState::insufficient_evidence, "stale modality abstains");
    bank.evaluate(embedding, 0, 0, 1, times, scores);
    require(scores[0].state == PropositionState::score, "epoch reset clears deadlines");
    bool rate_rejected = false;
    try { PropositionBank too_slow({head}, 100); }
    catch (const std::exception &) { rate_rejected = true; }
    require(rate_rejected, "unsatisfied MUST frequency rejected");
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
