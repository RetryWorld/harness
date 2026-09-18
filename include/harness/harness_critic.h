/* harness_critic.h — stable C ABI for a Harness critic.
 *
 * Critics observe and propose. They never actuate — no function on this
 * interface emits a command. This is the seam that lets a reasoning
 * adjudicator be replaced by the deterministic kernel, enforced by the type
 * system rather than by convention.
 *
 * A critic is a compiled artifact (a shared library) loaded independently of
 * the kernel. All allocation happens in hkc_create; hkc_evaluate is
 * alloc-free, and the caller times it and reports the elapsed time via
 * hk_report_budget.
 */

#ifndef HARNESS_CRITIC_H
#define HARNESS_CRITIC_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#define HKC_API __declspec(dllexport)
#else
#define HKC_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#define HKC_NOEXCEPT noexcept
#else
#define HKC_NOEXCEPT
#endif

#define HKC_ABI_VERSION 1

typedef struct hkc_handle hkc_handle;

typedef struct {
    uint64_t t_ns;
    uint32_t n_joints;
    uint32_t n_samples;
    const double* effort;   /* n_samples * n_joints, row-major */
    const double* position; /* n_samples * n_joints, row-major */
    const double* velocity; /* n_samples * n_joints, row-major */
    const double* aux;      /* n_aux values, most recent sample */
    uint32_t n_aux;
} hkc_window;

typedef struct {
    uint32_t evidence_class; /* index into the profile's declared evidence classes */
    float confidence;
    uint32_t suggested_mode; /* advisory only, never authoritative */
    uint8_t payload[64];
    uint32_t payload_len;
} hkc_proposal;

HKC_API uint32_t hkc_abi_version(void) HKC_NOEXCEPT;

/* Allocating; call OFF the control path, at init. `params_pb` is a serialised
 * harness.v1.CriticDecl. */
HKC_API int32_t hkc_create(const uint8_t* params_pb, size_t len, hkc_handle** out) HKC_NOEXCEPT;

HKC_API void hkc_destroy(hkc_handle*) HKC_NOEXCEPT;

/* Alloc-free. No locks, no I/O, no exceptions escaping. */
HKC_API int32_t hkc_evaluate(hkc_handle*, const hkc_window*, hkc_proposal* out) HKC_NOEXCEPT;

HKC_API uint64_t hkc_declared_wcet_us(hkc_handle*) HKC_NOEXCEPT;

#ifdef __cplusplus
} /* extern "C" */
#endif

#undef HKC_NOEXCEPT

#endif /* HARNESS_CRITIC_H */
