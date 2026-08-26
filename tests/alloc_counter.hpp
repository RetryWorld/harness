// Global operator new/delete replacement used to prove that the decision path
// allocates nothing.
//
// This lives in its own translation unit deliberately. Defining the replacement
// operators alongside the tests lets GCC inline both sides and then report
// -Wmismatched-new-delete: it sees memory from `malloc` handed to `free` and
// cannot tell that these ARE the global operators, so the pair is matched by
// definition. Replacing global new/delete with malloc/free is well-defined; the
// warning is a false positive.
//
// Splitting the TU is preferable to suppressing the diagnostic. -Werror is the
// compensating control for everything C++ does not guarantee statically, and
// punching a hole in it to silence one false positive is how that control
// erodes.

#ifndef HARNESS_TESTS_ALLOC_COUNTER_HPP
#define HARNESS_TESTS_ALLOC_COUNTER_HPP

#include <cstddef>

namespace harness_test {

// Incremented by every global allocation while tracking is enabled.
extern std::size_t alloc_count;
extern bool tracking;

class AllocationGuard {
public:
    AllocationGuard() { alloc_count = 0; tracking = true; }
    ~AllocationGuard() { tracking = false; }
    AllocationGuard(const AllocationGuard&) = delete;
    AllocationGuard& operator=(const AllocationGuard&) = delete;
    std::size_t count() const { return alloc_count; }
};

}  // namespace harness_test

#endif  // HARNESS_TESTS_ALLOC_COUNTER_HPP
