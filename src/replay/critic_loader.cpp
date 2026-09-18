#include "replay/critic_loader.hpp"

#include <dlfcn.h>

#include <utility>

namespace harness::replay {

std::optional<LoadedCritic> LoadedCritic::load(const std::string& lib_path) {
    void* h = dlopen(lib_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (h == nullptr) return std::nullopt;

    LoadedCritic lc;
    lc.dl_handle_ = h;
    lc.abi_version_fn_ = reinterpret_cast<uint32_t (*)()>(dlsym(h, "hkc_abi_version"));
    lc.create_fn_ =
        reinterpret_cast<int32_t (*)(const uint8_t*, size_t, hkc_handle**)>(dlsym(h, "hkc_create"));
    lc.destroy_fn_ = reinterpret_cast<void (*)(hkc_handle*)>(dlsym(h, "hkc_destroy"));
    lc.evaluate_fn_ = reinterpret_cast<int32_t (*)(hkc_handle*, const hkc_window*, hkc_proposal*)>(
        dlsym(h, "hkc_evaluate"));
    lc.declared_wcet_fn_ =
        reinterpret_cast<uint64_t (*)(hkc_handle*)>(dlsym(h, "hkc_declared_wcet_us"));

    if (lc.abi_version_fn_ == nullptr || lc.create_fn_ == nullptr || lc.destroy_fn_ == nullptr ||
        lc.evaluate_fn_ == nullptr || lc.declared_wcet_fn_ == nullptr) {
        dlclose(h);
        return std::nullopt;
    }
    if (lc.abi_version_fn_() != HKC_ABI_VERSION) {
        dlclose(h);
        return std::nullopt;
    }
    return lc;
}

int32_t LoadedCritic::create(const std::uint8_t* params, std::size_t len) {
    return create_fn_(params, len, &handle_);
}

void LoadedCritic::destroy() {
    if (handle_ != nullptr && destroy_fn_ != nullptr) {
        destroy_fn_(handle_);
        handle_ = nullptr;
    }
}

int32_t LoadedCritic::evaluate(const hkc_window& window, hkc_proposal& out) const {
    return evaluate_fn_(handle_, &window, &out);
}

std::uint64_t LoadedCritic::declared_wcet_us() const { return declared_wcet_fn_(handle_); }

LoadedCritic::LoadedCritic(LoadedCritic&& other) noexcept
    : dl_handle_(other.dl_handle_),
      handle_(other.handle_),
      abi_version_fn_(other.abi_version_fn_),
      create_fn_(other.create_fn_),
      destroy_fn_(other.destroy_fn_),
      evaluate_fn_(other.evaluate_fn_),
      declared_wcet_fn_(other.declared_wcet_fn_) {
    other.dl_handle_ = nullptr;
    other.handle_ = nullptr;
}

LoadedCritic& LoadedCritic::operator=(LoadedCritic&& other) noexcept {
    if (this != &other) {
        destroy();
        if (dl_handle_ != nullptr) dlclose(dl_handle_);
        dl_handle_ = other.dl_handle_;
        handle_ = other.handle_;
        abi_version_fn_ = other.abi_version_fn_;
        create_fn_ = other.create_fn_;
        destroy_fn_ = other.destroy_fn_;
        evaluate_fn_ = other.evaluate_fn_;
        declared_wcet_fn_ = other.declared_wcet_fn_;
        other.dl_handle_ = nullptr;
        other.handle_ = nullptr;
    }
    return *this;
}

LoadedCritic::~LoadedCritic() {
    destroy();
    if (dl_handle_ != nullptr) dlclose(dl_handle_);
}

}  // namespace harness::replay
