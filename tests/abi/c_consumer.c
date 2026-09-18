/* Pure-C11 consumer of harness_kernel.h. Build brief §6 stability policy:
 * this TU must build and run regardless of what the C++ implementation
 * does, proving the header stays C-clean. Calls every function on the
 * surface at least once. Not a correctness test — kernel_gate_test.cpp
 * covers behaviour; this one covers "the header is still valid C". */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "harness/harness_kernel.h"

/* A minimal serialised harness.v1.HarnessProfile is not hand-buildable in
 * plain C without the generated bindings, so this TU exercises the failure
 * path (garbage bytes -> HK_ERR_INVALID_PROFILE) rather than a full gate
 * cycle. That is enough to prove every symbol resolves and every struct
 * layout is usable from C. */
int main(void) {
    uint32_t abi = hk_abi_version();
    if (abi != HK_ABI_VERSION) {
        fprintf(stderr, "abi version mismatch: got %u, expected %d\n", abi, HK_ABI_VERSION);
        return 1;
    }

    uint8_t garbage[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x02, 0x03};
    hk_handle* handle = NULL;
    hk_status st = hk_create(garbage, sizeof(garbage), &handle);
    if (st == HK_OK) {
        fprintf(stderr, "expected garbage bytes to be rejected\n");
        hk_destroy(handle);
        return 1;
    }

    /* handle is NULL here since creation failed; exercise the remaining
     * calls with NULL to prove they degrade to null-arg errors rather than
     * crashing -- also part of the ABI contract. */
    hk_command cmd;
    memset(&cmd, 0, sizeof(cmd));
    hk_gated_command out;
    memset(&out, 0, sizeof(out));
    st = hk_gate(handle, &cmd, &out, NULL);
    if (st != HK_ERR_NULL_ARG) {
        fprintf(stderr, "expected HK_ERR_NULL_ARG from hk_gate on a null handle\n");
        return 1;
    }

    hkc_proposal proposal;
    memset(&proposal, 0, sizeof(proposal));
    st = hk_submit_proposal(handle, &proposal);
    if (st != HK_ERR_NULL_ARG) {
        fprintf(stderr, "expected HK_ERR_NULL_ARG from hk_submit_proposal on a null handle\n");
        return 1;
    }

    st = hk_report_budget(handle, 0, 1000);
    if (st != HK_ERR_NULL_ARG) {
        fprintf(stderr, "expected HK_ERR_NULL_ARG from hk_report_budget on a null handle\n");
        return 1;
    }

    const char* err = hk_last_error(handle);
    (void)err; /* null handle -> "" by contract; not crashing is the test */

    hk_destroy(handle); /* destroying a null handle must be a no-op, not a crash */

    printf("c_consumer: all harness_kernel.h symbols resolved and behaved\n");
    return 0;
}
