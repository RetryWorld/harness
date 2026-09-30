#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

namespace harness::tiny {
// Scoring consumes the shared v2 encoder's normalized 64-D embedding. Bank
// construction/replacement happens off the inference thread. No MCAP parsing,
// history growth, allocation or string lookup occurs in evaluate().
inline constexpr std::size_t proposition_dimensions = 64;
using PropositionEmbedding = std::array<float, proposition_dimensions>;
struct PropositionHead {
  std::array<PropositionEmbedding, 4> centers{};
  std::size_t center_count = 0;
  double required_hz = 0;
  std::int64_t max_input_age_ns = 0;
  std::uint32_t required_modalities = 0;
};
enum class PropositionState { unknown, score, insufficient_evidence, deadline_missed };
struct PropositionScore {
  PropositionState state = PropositionState::unknown;
  float similarity = 0;
  // Similarity is not P(proposition). A separately fitted and validated
  // probability calibrator is required before exposing a probability.
};
class PropositionBank {
  std::vector<PropositionHead> heads_;
  std::vector<std::int64_t> last_evaluation_;
  std::int64_t epoch_ = -1;
public:
  explicit PropositionBank(std::vector<PropositionHead> heads, double evaluation_hz)
      : heads_(std::move(heads)), last_evaluation_(heads_.size(), -1) {
    if (heads_.size() > 4096 || !std::isfinite(evaluation_hz) || evaluation_hz <= 0)
      throw std::invalid_argument("invalid proposition bank capacity or evaluation rate");
    for (const auto &head : heads_) {
      if (head.center_count > head.centers.size() || !std::isfinite(head.required_hz) ||
          head.required_hz <= 0 || head.required_hz > evaluation_hz ||
          head.max_input_age_ns <= 0 || head.required_modalities == 0)
        throw std::invalid_argument("proposition scope cannot be served by this bank");
      for (std::size_t k = 0; k < head.center_count; ++k) {
        double norm = 0;
        for (const float value : head.centers[k]) norm += static_cast<double>(value) * static_cast<double>(value);
        if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-3)
          throw std::invalid_argument("prototype must be a normalized finite embedding");
      }
    }
  }
  std::size_t size() const { return heads_.size(); }
  // modality_times and embedding_time use the same epoch clock as now. A fast
  // scheduler cannot turn an old camera frame into fresh perceptual evidence.
  bool evaluate(const PropositionEmbedding &embedding, std::int64_t embedding_time,
                std::int64_t now, std::int64_t epoch,
                const std::array<std::int64_t, 32> &modality_times,
                std::span<PropositionScore> output) noexcept {
    if (output.size() != heads_.size()) return false;
    if (epoch != epoch_) {
      std::fill(last_evaluation_.begin(), last_evaluation_.end(), -1);
      epoch_ = epoch;
    }
    double norm = 0;
    for (const float value : embedding) norm += static_cast<double>(value) * static_cast<double>(value);
    const bool valid = std::isfinite(norm) && std::abs(norm - 1) <= 1e-3 &&
                       now >= 0 && embedding_time >= 0 && embedding_time <= now;
    for (std::size_t i = 0; i < heads_.size(); ++i) {
      const auto &head = heads_[i];
      auto &result = output[i];
      result = {};
      const auto previous = last_evaluation_[i];
      last_evaluation_[i] = now;
      if (previous >= 0 && (now < previous ||
          static_cast<double>(now - previous) * head.required_hz > 1e9)) {
        result.state = PropositionState::deadline_missed;
        continue;
      }
      bool fresh = valid && now - embedding_time <= head.max_input_age_ns;
      for (std::size_t modality = 0; modality < modality_times.size(); ++modality) {
        if ((head.required_modalities & (std::uint32_t{1} << modality)) == 0) continue;
        const auto time = modality_times[modality];
        fresh = fresh && time >= 0 && time <= now && now - time <= head.max_input_age_ns;
      }
      if (!fresh) { result.state = PropositionState::insufficient_evidence; continue; }
      if (head.center_count == 0) continue;
      result.state = PropositionState::score;
      result.similarity = -1;
      for (std::size_t k = 0; k < head.center_count; ++k) {
        float dot = 0;
        for (std::size_t d = 0; d < embedding.size(); ++d)
          dot += embedding[d] * head.centers[k][d];
        result.similarity = std::max(result.similarity, std::clamp(dot, -1.F, 1.F));
      }
    }
    return true;
  }
};
} // namespace harness::tiny
