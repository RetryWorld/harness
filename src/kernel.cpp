#include "kernel.hpp"

#include <cmath>
#include <limits>

namespace harness {

namespace {

ProjectionRaw drop_with(Predicate p, std::span<double> emitted) noexcept {
    for (double& slot : emitted) slot = 0.0;
    ProjectionRaw r;
    r.verdict = Verdict::Drop;
    r.failed = static_cast<std::uint32_t>(p);
    r.margin = -std::numeric_limits<double>::infinity();
    r.binding_index = -1;
    r.nonfinite_input = (p == Predicate::ValueRange);
    return r;
}

}  // namespace

ProjectionRaw project_into(std::span<const double> candidate,
                           const OutputRegion& region,
                           std::span<double> emitted) noexcept {
    if (candidate.size() != region.lo.size() || candidate.size() != region.hi.size() ||
        emitted.size() < candidate.size()) {
        return drop_with(Predicate::TypeCompatibility, emitted);
    }

    double violation = -std::numeric_limits<double>::infinity();
    std::int32_t binding = -1;

    for (std::size_t i = 0; i < candidate.size(); ++i) {
        const double c = candidate[i];
        const double lo = region.lo[i];
        const double hi = region.hi[i];

        if (!std::isfinite(c)) {
            return drop_with(Predicate::ValueRange, emitted);
        }

        const double below = lo - c;
        const double above = c - hi;
        const double worst = (below > above) ? below : above;
        if (worst > violation) {
            violation = worst;
            binding = static_cast<std::int32_t>(i);
        }
        emitted[i] = (c < lo) ? lo : ((c > hi) ? hi : c);
    }

    const bool admitted = violation <= 0.0;
    ProjectionRaw r;
    r.verdict = admitted ? Verdict::Admit : Verdict::Replace;
    r.failed = admitted ? 0U : static_cast<std::uint32_t>(Predicate::ValueRange);
    r.margin = -violation;
    r.binding_index = binding;
    r.nonfinite_input = false;
    return r;
}

ProjectionOutcome project(std::span<const double> candidate, const OutputRegion& region) {
    ProjectionOutcome out;
    out.emitted.assign(region.lo.size(), 0.0);
    out.raw = project_into(candidate, region, out.emitted);
    return out;
}

const char* const kKernelVersion = HARNESS_VERSION_STRING;
const char* const kSchemaRevision = "harness.v1";
const char* const kBuildCommit = HARNESS_BUILD_COMMIT_STRING;

}  // namespace harness
