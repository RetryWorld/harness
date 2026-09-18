// Structural validator for the Harness Profile. Build brief §4.4: these
// eleven constraints are the point of the schema and cannot be expressed in
// protobuf itself.
//
// Fail closed: `validate()` returns every violation it finds (not just the
// first), and the caller must treat any non-empty result as "do not load
// this profile."

#ifndef HARNESS_PROFILE_VALIDATOR_HPP
#define HARNESS_PROFILE_VALIDATOR_HPP

#include <string>
#include <vector>

#include "harness/v1/profile.pb.h"

namespace harness::profile {

enum class Constraint {
    C1_INELIGIBLE_ENGINE,
    C2_SIM_TIMING_AUTHORITATIVE,
    C3_RECOVERY_EVIDENCE_UNDECLARED,
    C4_CRITIC_BUDGET_EXCEEDED,
    C5_BAND_B_INCOMPLETE,
    C6_MISSING_REENTRY_PREDICATE,
    C7_JOINT_ARITY_MISMATCH,
    C8_JOINT_ORDER_HASH_MISMATCH,
    C9_PARENT_EMBODIMENT_MISMATCH,
    C10_DEPLOYED_CONFIDENCE_NOT_HARDWARE,
    C11_MISSING_PRESENCE,
};

struct ValidationError {
    Constraint constraint;
    std::string code;     // "C1".."C11", stable, machine-checkable
    std::string message;
};

// Returns an empty vector iff `profile` satisfies every constraint.
std::vector<ValidationError> validate(const harness::v1::HarnessProfile& profile);

// Canonical hash used by C8. Documented here because it is validator-internal
// state, not a wire-format guarantee: joins joint_names with '\n' and hashes
// with FNV-1a 64, rendered as lowercase hex. Exposed so profile authoring
// tools compute the same hash the validator checks against.
std::string canonical_joint_order_hash(
    const google::protobuf::RepeatedPtrField<std::string>& joint_names);

}  // namespace harness::profile

#endif  // HARNESS_PROFILE_VALIDATOR_HPP
