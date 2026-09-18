// Reference critic: position delta below epsilon while commanded effort is
// above a bound. Proves multi-channel reasoning (build brief §7) — needs
// both position and effort from the same window to decide, unlike
// threshold_critic which only ever looks at one aux channel.

#include <cmath>
#include <cstring>
#include <new>

#include "critic/critic_params.hpp"
#include "harness/harness_critic.h"
#include "harness/harness_kernel.h"

struct hkc_handle {
    harness::critic::StallCriticParams params;
};

extern "C" {

uint32_t hkc_abi_version(void) noexcept { return HKC_ABI_VERSION; }

int32_t hkc_create(const uint8_t* params_pb, size_t len, hkc_handle** out) noexcept {
    if (out == nullptr) return HK_ERR_NULL_ARG;
    *out = nullptr;
    try {
        auto handle = new (std::nothrow) hkc_handle();
        if (handle == nullptr) return HK_ERR_INTERNAL;
        if (params_pb != nullptr && len == sizeof(harness::critic::StallCriticParams)) {
            std::memcpy(&handle->params, params_pb, len);
        }
        *out = handle;
        return HK_OK;
    } catch (...) {
        return HK_ERR_INTERNAL;
    }
}

void hkc_destroy(hkc_handle* handle) noexcept { delete handle; }

int32_t hkc_evaluate(hkc_handle* handle, const hkc_window* window, hkc_proposal* out) noexcept {
    if (handle == nullptr || window == nullptr || out == nullptr) return HK_ERR_NULL_ARG;

    *out = hkc_proposal{};
    out->evidence_class = handle->params.evidence_class_index;

    bool stalled = false;
    if (window->position != nullptr && window->effort != nullptr && window->n_samples >= 2 &&
        window->n_joints > 0) {
        std::uint32_t n = window->n_joints;
        std::uint32_t last = window->n_samples - 1;
        for (std::uint32_t j = 0; j < n && !stalled; ++j) {
            double p0 = window->position[j];
            double p1 = window->position[static_cast<std::size_t>(last) * n + j];
            double delta = std::fabs(p1 - p0);
            if (delta >= handle->params.position_epsilon) continue;

            bool effort_high_throughout = true;
            for (std::uint32_t s = 0; s < window->n_samples; ++s) {
                double e = window->effort[static_cast<std::size_t>(s) * n + j];
                if (std::fabs(e) < handle->params.effort_bound) {
                    effort_high_throughout = false;
                    break;
                }
            }
            if (effort_high_throughout) stalled = true;
        }
    }

    out->confidence = stalled ? 1.0F : 0.0F;
    out->suggested_mode = stalled ? 1U : 0U;
    return HK_OK;
}

uint64_t hkc_declared_wcet_us(hkc_handle* handle) noexcept {
    return handle == nullptr ? 0 : handle->params.declared_wcet_us;
}

}  // extern "C"
