/* harness_kernel.h — stable C ABI for the Physical AI harness kernel (v2).
 *
 * This header IS the contract. Every consumer sees exactly this surface: a
 * simulation rig, a ROS 2 controller, a customer's runtime. There is no
 * privileged consumer.
 *
 * ABI rules (load-bearing, not style):
 *
 *  1. NO ALLOCATION CROSSES THE BOUNDARY on hk_gate. The caller owns every
 *     output buffer. A 1 kHz control loop cannot afford an allocator.
 *  2. NO EXCEPTIONS CROSS THE BOUNDARY. Every entry point is noexcept and
 *     wraps anything that can throw; errors are hk_status return codes.
 *  3. STRUCTS ARE APPEND-ONLY. Never reorder or remove a field. A deployed
 *     robot may run an older kernel than the tool reading its logs.
 *  4. hk_abi_version() MUST be checked before any other call.
 *  5. Dropping evidence must never block the gate — see hk_event_sink.
 */

#ifndef HARNESS_KERNEL_H
#define HARNESS_KERNEL_H

#include <stddef.h>
#include <stdint.h>

#include "harness/harness_critic.h"

#if defined(_WIN32)
#define HK_API __declspec(dllexport)
#else
#define HK_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#define HK_NOEXCEPT noexcept
#else
#define HK_NOEXCEPT
#endif

#define HK_ABI_VERSION 1

typedef struct hk_handle hk_handle;

typedef enum {
    HK_OK = 0,
    HK_ERR_INVALID_PROFILE = 1,
    HK_ERR_JOINT_MISMATCH = 2,
    HK_ERR_BUFFER_FULL = 3,
    HK_ERR_NULL_ARG = 4,
    HK_ERR_ABI_MISMATCH = 5,
    HK_ERR_INTERNAL = 99
} hk_status;

typedef enum { HK_ADMIT = 0, HK_CLAMP = 1, HK_HOLD_LAST = 2, HK_REJECT = 3 } hk_decision;
typedef enum { HK_BAND_NOMINAL = 0, HK_BAND_A = 1, HK_BAND_B = 2 } hk_band;

typedef struct {
    uint64_t t_ns;
    uint32_t n_joints;
    const double* effort;
} hk_command;

typedef struct {
    hk_decision decision;
    hk_band band;
    uint32_t n_joints;
    double* effort; /* caller-owned, at least n_joints long */
} hk_gated_command;

/* Caller-owned append buffer. `*len` is the current write offset, advanced by
 * the kernel; the caller drains it (copies out [0, *len)) and resets it to 0
 * between calls at whatever cadence it likes. Once `cap` is reached the
 * kernel stops writing and increments `*dropped` instead — it never blocks
 * and never wraps over undrained data. Each record is a varint length prefix
 * followed by a serialised harness.v1.EnforcementEvent. */
typedef struct {
    uint8_t* buf;
    size_t cap;
    size_t* len;
    uint32_t* dropped;
} hk_event_sink;

HK_API uint32_t hk_abi_version(void) HK_NOEXCEPT;

/* Allocating; call OFF the control path, at init. `profile_pb` is a
 * serialised, already-validated harness.v1.HarnessProfile — hk_create
 * re-validates defensively (fail closed) and returns HK_ERR_INVALID_PROFILE
 * on any structural violation (build brief §4.4, C1-C11). */
HK_API hk_status hk_create(const uint8_t* profile_pb, size_t len, hk_handle** out) HK_NOEXCEPT;

HK_API void hk_destroy(hk_handle*) HK_NOEXCEPT;

/* The one rule: no heap allocation, no locks, no I/O, no syscalls, no
 * exceptions escaping. Bounded work, proportional only to n_joints. Compiled
 * with -ffp-contract=off, no fast-math: determinism over speed. */
HK_API hk_status hk_gate(hk_handle*, const hk_command*, hk_gated_command*, hk_event_sink*) HK_NOEXCEPT;

/* Submit a critic's proposal for this cycle. Advisory evidence only — the
 * kernel decides whether it triggers Band A/B transfer via the profile's
 * declared recoveries and Band B triggers. Alloc-free. */
HK_API hk_status hk_submit_proposal(hk_handle*, const hkc_proposal*) HK_NOEXCEPT;

/* Report elapsed time for one budgeted source: source_id 0 is model
 * inference (Isolation.inference_wcet_ms); source_id (1 + critic index) is
 * the corresponding declared critic. Feeds IsolationEvent.compute_overrun and
 * Band B's overrun_count_max trigger. Alloc-free. */
HK_API hk_status hk_report_budget(hk_handle*, uint32_t source_id, uint64_t elapsed_ns) HK_NOEXCEPT;

HK_API const char* hk_last_error(hk_handle*) HK_NOEXCEPT;

#ifdef __cplusplus
} /* extern "C" */
#endif

#undef HK_NOEXCEPT

#endif /* HARNESS_KERNEL_H */
