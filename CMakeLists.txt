# Harness kernel — C++ build.
#
# MUST configure and build standalone from a checkout of this directory alone.
# No dependency reaches outside edge/; no package manager, no network. That
# property is what makes the enforcement path independently auditable and edge/
# extractable, and it is tested by scripts/isolation_check.sh in CI.
#
# Deliberately zero third-party dependencies. The JSON parser is ours (src/json)
# for exactly this reason — see the comment at the top of that file.

cmake_minimum_required(VERSION 3.22)

project(harness_kernel
        VERSION 0.1.0
        DESCRIPTION "Physical AI harness kernel"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(HARNESS_BUILD_COMMIT "unknown" CACHE STRING "Commit or source hash recorded in the binary")

# ---------------------------------------------------------------------------
# Compile options.
#
# These are not style preferences. Two groups matter:
#
#  * The float flags. -ffp-contract=off stops the compiler fusing a*b+c into an
#    FMA, which would change the last bits of `margin` and break bit-identical
#    agreement with the simulation rig's shadow gate. -fno-fast-math keeps NaN
#    and infinity semantics, which the finiteness predicate depends on.
#    Removing either silently invalidates the conformance check.
#
#  * The warning set. C++ gives none of the guarantees the previous Rust
#    implementation did for free, so the compensating control is that warnings
#    are errors and the sanitizers run in CI. This is the cost of the language
#    choice, paid explicitly rather than assumed away.
# ---------------------------------------------------------------------------
add_library(harness_flags INTERFACE)
target_compile_options(harness_flags INTERFACE
  -ffp-contract=off
  -fno-fast-math
  -Wall -Wextra -Wpedantic -Werror
  -Wconversion -Wsign-conversion
  -Wshadow -Wnon-virtual-dtor -Wold-style-cast
  -Wcast-align -Wunused -Woverloaded-virtual
  -Wnull-dereference -Wdouble-promotion -Wformat=2
)

option(HARNESS_SANITIZE "Build with ASan+UBSan (tests and CI, never releases)" OFF)
if(HARNESS_SANITIZE)
  target_compile_options(harness_flags INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer -g)
  target_link_options(harness_flags INTERFACE -fsanitize=address,undefined)
endif()

# ---------------------------------------------------------------------------
# Core objects, compiled once and shared by the static and shared libraries so
# the two cannot diverge.
# ---------------------------------------------------------------------------
add_library(harness_core OBJECT
  src/kernel.cpp
  src/profile.cpp
  src/json.cpp
  src/cabi.cpp
)
target_include_directories(harness_core PUBLIC include PRIVATE src)
target_link_libraries(harness_core PRIVATE harness_flags)
target_compile_definitions(harness_core PRIVATE
  HARNESS_VERSION_STRING="${PROJECT_VERSION}"
  HARNESS_BUILD_COMMIT_STRING="${HARNESS_BUILD_COMMIT}"
)

# The artifact every consumer loads: our sim via ctypes, a ROS 2 controller via
# dlopen, a customer's runtime via ordinary FFI.
add_library(harness_kernel SHARED $<TARGET_OBJECTS:harness_core>)
target_include_directories(harness_kernel PUBLIC include)
target_link_libraries(harness_kernel PRIVATE harness_flags)
set_target_properties(harness_kernel PROPERTIES
  OUTPUT_NAME harness_kernel
  VERSION ${PROJECT_VERSION}
  SOVERSION 1                      # tracks HARNESS_ABI_VERSION, not the release
  CXX_VISIBILITY_PRESET hidden     # only the extern "C" surface is exported
  VISIBILITY_INLINES_HIDDEN ON
)

# For customers who link the kernel into their own binary.
add_library(harness_kernel_static STATIC $<TARGET_OBJECTS:harness_core>)
target_include_directories(harness_kernel_static PUBLIC include)
set_target_properties(harness_kernel_static PROPERTIES OUTPUT_NAME harness_kernel)

add_executable(harness-kernel src/main.cpp)
target_include_directories(harness-kernel PRIVATE src)
target_link_libraries(harness-kernel PRIVATE harness_kernel_static harness_flags)
target_compile_definitions(harness-kernel PRIVATE
  HARNESS_VERSION_STRING="${PROJECT_VERSION}"
  HARNESS_BUILD_COMMIT_STRING="${HARNESS_BUILD_COMMIT}"
)

# ---------------------------------------------------------------------------
# Tests. No gtest: a test dependency would break the offline-build property for
# the same reason a JSON library would.
# ---------------------------------------------------------------------------
option(HARNESS_BUILD_TESTS "Build tests" ON)
if(HARNESS_BUILD_TESTS)
  enable_testing()
  add_executable(harness_tests tests/test_kernel.cpp tests/alloc_counter.cpp)
  target_include_directories(harness_tests PRIVATE src tests)
  target_link_libraries(harness_tests PRIVATE harness_kernel_static harness_flags)
  add_test(NAME harness_tests COMMAND harness_tests)
endif()

include(GNUInstallDirs)
install(TARGETS harness_kernel harness_kernel_static harness-kernel
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(FILES include/harness_kernel.h DESTINATION include)
