// C ABI surface. The only interface any consumer sees.
//
// Mirrors include/harness_kernel.h, which is hand-written and committed. A
// generated header would let this side silently reshape the contract; here a
// contract change requires editing the header, which is a visible diff in review
// and a signal to bump HARNESS_ABI_VERSION.
//
// # Exception discipline
//
// Every extern "C" function is noexcept and wraps anything that can throw. An
// exception propagating across a C ABI boundary is undefined behaviour, and this
// library is loaded into someone else's process — our simulation rig, a ROS 2
// controller, a customer's runtime. A std::bad_alloc in profile parsing must
// become HARNESS_ERR_INVALID_PROFILE, not terminate their robot's control node.
//
// This is the discipline the language switch costs us: it was a compiler
// guarantee before and is now a review rule. The catch-all handlers below and
// the -fno-exceptions-free decision path are how it is paid for.

#include <cstring>
#include <memory>
#include <new>
#include <span>
#include <string>

#include "harness_kernel.h"
#include "kernel.hpp"
#include "profile.hpp"

// Opaque to C. Allocated once at load and never reallocated, so the decision
// path touches no allocator.
struct harness_profile {
    harness::HarnessProfile inner;
};

namespace {

harness_projection_result_t to_c(const harness::ProjectionRaw& raw) noexcept {
    harness_projection_result_t out{};
    out.verdict = static_cast<harness_verdict_t>(raw.verdict);
    out.failed = raw.failed;
    out.margin = raw.margin;
    out.binding_index = raw.binding_index;
    return out;
}

}  // namespace

extern "C" {

// ---------------------------------------------------------------- lifecycle

harness_status_t harness_profile_load(const char* json_utf8, size_t json_len,
                                      harness_profile_t** out_profile) noexcept {
    if (json_utf8 == nullptr || out_profile == nullptr) return HARNESS_ERR_NULL_ARG;
    *out_profile = nullptr;
    try {
        harness::ProfileParseResult r =
            harness::parse_profile(std::string_view(json_utf8, json_len));
        if (!r.profile.has_value()) return HARNESS_ERR_INVALID_PROFILE;
        auto owned = std::make_unique<harness_profile>();
        owned->inner = std::move(*r.profile);
        *out_profile = owned.release();
        return HARNESS_OK;
    } catch (const std::bad_alloc&) {
        return HARNESS_ERR_INTERNAL;
    } catch (...) {
        return HARNESS_ERR_INTERNAL;
    }
}

void harness_profile_free(harness_profile_t* profile) noexcept { delete profile; }

harness_status_t harness_profile_arity(const harness_profile_t* profile,
                                       size_t* out_arity) noexcept {
    if (profile == nullptr || out_arity == nullptr) return HARNESS_ERR_NULL_ARG;
    *out_arity = profile->inner.output_region.lo.size();
    return HARNESS_OK;
}

// ------------------------------------------------------------ decision path
//
// No try/catch here on purpose: project_into is noexcept and allocation-free by
// construction, so a handler would be dead code implying a risk that does not
// exist. The tests defend that property.

harness_status_t harness_project(const harness_profile_t* profile, const double* candidate,
                                 size_t candidate_len, double* emitted, size_t emitted_cap,
                                 harness_projection_result_t* out_result) noexcept {
    if (profile == nullptr || candidate == nullptr || emitted == nullptr ||
        out_result == nullptr) {
        return HARNESS_ERR_NULL_ARG;
    }
    const harness::OutputRegion& region = profile->inner.output_region;
    if (candidate_len != region.lo.size()) return HARNESS_ERR_ARITY;
    if (emitted_cap < candidate_len) return HARNESS_ERR_BUFFER_TOO_SMALL;

    const harness::ProjectionRaw raw = harness::project_into(
        std::span<const double>(candidate, candidate_len), region,
        std::span<double>(emitted, candidate_len));
    *out_result = to_c(raw);
    return raw.nonfinite_input ? HARNESS_ERR_NONFINITE : HARNESS_OK;
}

harness_status_t harness_project_batch(const harness_profile_t* profile, const double* candidates,
                                       size_t n, size_t arity, double* emitted,
                                       harness_projection_result_t* out_results) noexcept {
    if (profile == nullptr || candidates == nullptr || emitted == nullptr ||
        out_results == nullptr) {
        return HARNESS_ERR_NULL_ARG;
    }
    const harness::OutputRegion& region = profile->inner.output_region;
    if (arity != region.lo.size()) return HARNESS_ERR_ARITY;

    for (size_t i = 0; i < n; ++i) {
        const size_t off = i * arity;
        // Same predicate as the single-sample path. Not a fast path with its own
        // logic: a batch form that could disagree with the single form would
        // make the conformance check meaningless.
        const harness::ProjectionRaw raw = harness::project_into(
            std::span<const double>(candidates + off, arity), region,
            std::span<double>(emitted + off, arity));
        out_results[i] = to_c(raw);
    }
    return HARNESS_OK;
}

// -------------------------------------------------------------- provenance

uint32_t harness_abi_version(void) noexcept { return HARNESS_ABI_VERSION; }

const char* harness_kernel_version(void) noexcept { return harness::kKernelVersion; }
const char* harness_schema_revision(void) noexcept { return harness::kSchemaRevision; }
const char* harness_build_commit(void) noexcept { return harness::kBuildCommit; }

const char* harness_status_str(harness_status_t status) noexcept {
    switch (status) {
        case HARNESS_OK: return "ok";
        case HARNESS_ERR_NULL_ARG: return "null argument";
        case HARNESS_ERR_ARITY: return "arity mismatch between candidate and output region";
        case HARNESS_ERR_NONFINITE: return "non-finite value in input";
        case HARNESS_ERR_INVALID_PROFILE: return "profile failed to parse or validate";
        case HARNESS_ERR_BUFFER_TOO_SMALL: return "caller buffer too small";
        case HARNESS_ERR_ABI_MISMATCH: return "ABI version mismatch";
        default: return "internal error";
    }
}

}  // extern "C"
