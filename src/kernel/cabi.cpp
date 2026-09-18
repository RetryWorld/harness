// C ABI surface. Mirrors include/harness/harness_kernel.h exactly; every
// extern "C" function here is noexcept and wraps anything that can throw —
// an exception crossing this boundary is undefined behaviour, and this
// library is loaded into someone else's process.

#include <cstring>
#include <memory>
#include <new>

#include "harness/harness_kernel.h"
#include "harness/v1/profile.pb.h"
#include "kernel/kernel_gate.hpp"
#include "kernel/kernel_state.hpp"

extern "C" {

uint32_t hk_abi_version(void) noexcept { return HK_ABI_VERSION; }

hk_status hk_create(const uint8_t* profile_pb, size_t len, hk_handle** out) noexcept {
    if (profile_pb == nullptr || out == nullptr) return HK_ERR_NULL_ARG;
    *out = nullptr;
    try {
        harness::v1::HarnessProfile msg;
        if (!msg.ParseFromArray(profile_pb, static_cast<int>(len))) {
            return HK_ERR_INVALID_PROFILE;
        }
        if (msg.embodiment().dof() > kMaxJoints) {
            return HK_ERR_JOINT_MISMATCH;
        }
        auto runtime = harness::kernel::ProfileRuntime::build(msg);
        if (!runtime.has_value()) return HK_ERR_INVALID_PROFILE;

        auto handle = std::make_unique<hk_handle>();
        handle->profile = std::move(*runtime);
        *out = handle.release();
        return HK_OK;
    } catch (const std::bad_alloc&) {
        return HK_ERR_INTERNAL;
    } catch (...) {
        return HK_ERR_INTERNAL;
    }
}

void hk_destroy(hk_handle* handle) noexcept { delete handle; }

hk_status hk_gate(hk_handle* handle, const hk_command* cmd, hk_gated_command* out,
                   hk_event_sink* sink) noexcept {
    if (handle == nullptr || cmd == nullptr || out == nullptr) return HK_ERR_NULL_ARG;
    return harness::kernel::run_gate_cycle(*handle, *cmd, *out, sink);
}

hk_status hk_submit_proposal(hk_handle* handle, const hkc_proposal* proposal) noexcept {
    if (handle == nullptr || proposal == nullptr) return HK_ERR_NULL_ARG;
    return harness::kernel::submit_proposal(*handle, *proposal);
}

hk_status hk_report_budget(hk_handle* handle, uint32_t source_id, uint64_t elapsed_ns) noexcept {
    if (handle == nullptr) return HK_ERR_NULL_ARG;
    return harness::kernel::report_budget(*handle, source_id, elapsed_ns);
}

const char* hk_last_error(hk_handle* handle) noexcept {
    if (handle == nullptr) return "";
    return handle->last_error.c_str();
}

} // extern "C"
