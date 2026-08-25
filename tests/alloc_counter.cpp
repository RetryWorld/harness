#include "alloc_counter.hpp"

#include <cstdlib>
#include <new>

namespace harness_test {
std::size_t alloc_count = 0;
bool tracking = false;
}  // namespace harness_test

void* operator new(std::size_t n) {
    if (harness_test::tracking) ++harness_test::alloc_count;
    void* p = std::malloc(n == 0 ? 1 : n);
    if (p == nullptr) throw std::bad_alloc();
    return p;
}

void* operator new[](std::size_t n) { return ::operator new(n); }

void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
