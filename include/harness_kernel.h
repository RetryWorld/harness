/* harness_kernel.h — stable C ABI for the Physical AI harness kernel.
 *
 * This header IS the contract. Every consumer sees exactly this surface:
 * our simulation rig (via ctypes), a ROS 2 controller plugin (via dlopen),
 * and a customer's runtime in whatever language they run.
 *
 * There is no Python binding, no Rust-only path, and no privileged consumer.
 * If the sim needed something this header does not expose, the sim would be
 * exercising an interface no customer has, and "the thresholds were derived
 * with the code that enforces them" would stop being checkable.
 *
 * ---------------------------------------------------------------------------
 * ABI rules (these are load-bearing, not style)
 *
 *  1. NO ALLOCATION CROSSES THE BOUNDARY on the decision path. The caller owns
 *     every output buffer. A 1 kHz control loop cannot afford an allocator, and
 *     cross-language ownership of heap memory is the classic FFI foot-gun.
 *
 *  2. NO PANICS CROSS THE BOUNDARY. The decision path is written without
 *     indexing, unwrap, or division that can trap. Errors are return codes.
 *
 *  3. STRUCTS ARE #[repr(C)] AND APPEND-ONLY. Never reorder or remove a field.
 *     A deployed robot may run an older kernel than the tool reading its logs.
 *
 *  4. harness_abi_version() MUST be checked before any other call. A mismatch
 *     means the caller was compiled against a different contract, and silently
 *     proceeding would misread the envelope.
 * ---------------------------------------------------------------------------
 */

#ifndef HARNESS_KERNEL_H
#define HARNESS_KERNEL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
/* Every entry point is noexcept. This is part of the contract, not an
 * optimisation: an exception propagating across a C ABI boundary is undefined
 * behaviour, and this library is loaded into someone else's process — our
 * simulation rig, a ROS 2 controller, a customer's runtime. A C++ consumer sees
 * the guarantee; a C consumer sees a plain declaration. */
#define HARNESS_NOEXCEPT noexcept
#else
#define HARNESS_NOEXCEPT
#endif

/* Bumped on ANY change to this header. Not tied to the release version:
 * a kernel patch release must not force every consumer to recompile. */
#define HARNESS_ABI_VERSION 1

/* ---- status codes ------------------------------------------------------- */

typedef int32_t harness_status_t;

#define HARNESS_OK                    0
#define HARNESS_ERR_NULL_ARG         -1
#define HARNESS_ERR_ARITY            -2  /* candidate/region length mismatch */
#define HARNESS_ERR_NONFINITE        -3  /* NaN or Inf in an input */
#define HARNESS_ERR_INVALID_PROFILE  -4
#define HARNESS_ERR_BUFFER_TOO_SMALL -5
#define HARNESS_ERR_ABI_MISMATCH     -6
#define HARNESS_ERR_INTERNAL         -99 /* should be unreachable; report it */

/* ---- verdicts ----------------------------------------------------------- */

typedef int32_t harness_verdict_t;

#define HARNESS_VERDICT_ADMIT   1
#define HARNESS_VERDICT_DROP    2
#define HARNESS_VERDICT_REPLACE 3

/* Bitmask: several predicates can fail on one sample, and which ones failed is
 * what the annotation flow adjudicates. A single enum would lose that. */
typedef uint32_t harness_predicate_mask_t;

#define HARNESS_PRED_TYPE_COMPATIBILITY  (1u << 0)
#define HARNESS_PRED_TIMESTAMP_FRESHNESS (1u << 1)
#define HARNESS_PRED_VALUE_RANGE         (1u << 2)
#define HARNESS_PRED_TRAJECTORY_STABILITY (1u << 3)
#define HARNESS_PRED_SCHEDULABILITY      (1u << 4)
#define HARNESS_PRED_DELIVERY_FEASIBILITY (1u << 5)
#define HARNESS_PRED_LIFECYCLE_STATE     (1u << 6)

/* ---- projection --------------------------------------------------------- */

/* Result of one gate decision. #[repr(C)], append-only. */
typedef struct {
    harness_verdict_t         verdict;
    harness_predicate_mask_t  failed;
    /* Signed distance to the binding predicate. Negative == violated.
     * Defined as the WORST per-actuator violation, not an aggregate: an
     * aggregate hides a single breach behind well-behaved neighbours, and this
     * number is what thresholds are fitted to. */
    double                    margin;
    /* Index of the actuator that bound the decision, or -1. */
    int32_t                   binding_index;
} harness_projection_result_t;

/* Opaque handle to a loaded, validated Harness Profile. */
typedef struct harness_profile harness_profile_t;

/* --- lifecycle (allocating; call OFF the control path, at init) ---------- */

/* Parse and validate a profile from JSON. Fails closed: a profile that does not
 * validate does not load, because a robot running an incoherent envelope is
 * worse than one that refuses to start — the failure is silent. */
harness_status_t harness_profile_load(const char *json_utf8,
                                      size_t json_len,
                                      harness_profile_t **out_profile) HARNESS_NOEXCEPT;

void harness_profile_free(harness_profile_t *profile) HARNESS_NOEXCEPT;

/* Number of actuators the profile's output region covers. Callers size their
 * emitted[] buffer from this. */
harness_status_t harness_profile_arity(const harness_profile_t *profile,
                                       size_t *out_arity) HARNESS_NOEXCEPT;

/* --- decision path (NO allocation, NO panic, NO logging) ----------------- */

/* Evaluate the Projection gate against a candidate action.
 *
 * `emitted` is caller-owned and must hold at least `arity` doubles; on ADMIT it
 * receives the candidate unchanged, on REPLACE the clamped value, on DROP zeros.
 *
 * Determinism: no fused or order-dependent floating-point operation is used, so
 * the result is bit-identical across platforms for identical inputs. That is
 * what makes the simulation rig's conformance test meaningful rather than
 * approximate. */
harness_status_t harness_project(const harness_profile_t *profile,
                                 const double *candidate,
                                 size_t candidate_len,
                                 double *emitted,
                                 size_t emitted_cap,
                                 harness_projection_result_t *out_result) HARNESS_NOEXCEPT;

/* Batched form. Semantically identical to calling harness_project in a loop;
 * exists to amortise call overhead in the conformance test and in offline
 * replay. NOT a different code path — it calls the same predicate. */
harness_status_t harness_project_batch(const harness_profile_t *profile,
                                       const double *candidates, /* n * arity, row-major */
                                       size_t n,
                                       size_t arity,
                                       double *emitted,          /* n * arity */
                                       harness_projection_result_t *out_results) HARNESS_NOEXCEPT; /* n */

/* ---- provenance --------------------------------------------------------- */

/* Check this before anything else. */
uint32_t harness_abi_version(void) HARNESS_NOEXCEPT;

/* NUL-terminated, static lifetime, never freed by the caller. These are what
 * the simulation rig records into every replay tuple: a threshold with no
 * record of which kernel evaluated it has no provenance. */
const char *harness_kernel_version(void) HARNESS_NOEXCEPT;
const char *harness_schema_revision(void) HARNESS_NOEXCEPT;
const char *harness_build_commit(void) HARNESS_NOEXCEPT;

/* Human-readable text for a status code. Static lifetime. */
const char *harness_status_str(harness_status_t status) HARNESS_NOEXCEPT;

#ifdef __cplusplus
} /* extern "C" */
#endif

#undef HARNESS_NOEXCEPT

#endif /* HARNESS_KERNEL_H */
