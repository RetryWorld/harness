// Reference critic: an aux channel crosses a bound for N consecutive
// samples. Proves windowing (build brief §7).

#include <cstring>
#include <new>

#include "critic/critic_params.hpp"
#include "harness/harness_critic.h"
#include "harness/harness_kernel.h"

struct hkc_handle {
    harness::critic::ThresholdCriticParams params;
    std::uint32_t consecutive = 0;
};

extern "C" {

uint32_t hkc_abi_version(void) noexcept { return HKC_ABI_VERSION; }

int32_t hkc_create(const uint8_t* params_pb, size_t len, hkc_handle** out) noexcept {
    if (out == nullptr) return HK_ERR_NULL_ARG;
    *out = nullptr;
    try {
        auto handle = new (std::nothrow) hkc_handle();
        if (handle == nullptr) return HK_ERR_INTERNAL;
        if (params_pb != nullptr && len == sizeof(harness::critic::ThresholdCriticParams)) {
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

    bool crossing = false;
    if (window->aux != nullptr && handle->params.aux_index < window->n_aux) {
        double v = window->aux[handle->params.aux_index];
        crossing = handle->params.trigger_below ? (v < handle->params.bound)
                                                  : (v > handle->params.bound);
    }
    handle->consecutive = crossing ? (handle->consecutive + 1) : 0;

    bool fired = handle->consecutive >= handle->params.consecutive_required &&
                 handle->params.consecutive_required > 0;
    out->confidence = fired ? 1.0F : 0.0F;
    out->suggested_mode = fired ? 1U : 0U;
    return HK_OK;
}

uint64_t hkc_declared_wcet_us(hkc_handle* handle) noexcept {
    return handle == nullptr ? 0 : handle->params.declared_wcet_us;
}

}  // extern "C"
