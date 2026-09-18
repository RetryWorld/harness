// dlopen wrapper for a critic shared library. Real critics "arrive from the
// offline track as compiled artifacts" (build brief §7) — this is the
// loader that turns a path into live hkc_* function pointers, used by both
// the replay rig and (indirectly) the CLI.

#ifndef HARNESS_REPLAY_CRITIC_LOADER_HPP
#define HARNESS_REPLAY_CRITIC_LOADER_HPP

#include <cstdint>
#include <optional>
#include <string>

#include "harness/harness_critic.h"

namespace harness::replay {

class LoadedCritic {
public:
    ~LoadedCritic();
    LoadedCritic(LoadedCritic&& other) noexcept;
    LoadedCritic& operator=(LoadedCritic&& other) noexcept;
    LoadedCritic(const LoadedCritic&) = delete;
    LoadedCritic& operator=(const LoadedCritic&) = delete;

    static std::optional<LoadedCritic> load(const std::string& lib_path);

    int32_t create(const std::uint8_t* params, std::size_t len);
    void destroy();
    int32_t evaluate(const hkc_window& window, hkc_proposal& out) const;
    std::uint64_t declared_wcet_us() const;

private:
    LoadedCritic() = default;

    void* dl_handle_ = nullptr;
    hkc_handle* handle_ = nullptr;

    uint32_t (*abi_version_fn_)() = nullptr;
    int32_t (*create_fn_)(const uint8_t*, size_t, hkc_handle**) = nullptr;
    void (*destroy_fn_)(hkc_handle*) = nullptr;
    int32_t (*evaluate_fn_)(hkc_handle*, const hkc_window*, hkc_proposal*) = nullptr;
    uint64_t (*declared_wcet_fn_)(hkc_handle*) = nullptr;
};

}  // namespace harness::replay

#endif  // HARNESS_REPLAY_CRITIC_LOADER_HPP
