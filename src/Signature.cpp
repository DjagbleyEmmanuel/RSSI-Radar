#include "radar/Signature.hpp"

#include <algorithm>
#include <cmath>

namespace radar {

const char* signatureFeatureName(size_t i) {
    switch (i) {
        case 0: return "volatility";
        case 1: return "flatness";
        case 2: return "entropy";
        case 3: return "period";
        case 4: return "kurtosis";
        case 5: return "trend";
        case 6: return "outliers";
        case 7: return "spread";
        default: return "?";
    }
}

namespace {
// Normalisation bounds per feature, chosen so a typical reading lands mid-range
// and an extreme one saturates rather than dominating the distance. Without this
// the kurtosis term would swamp every other feature simply because its numeric
// range is wider.
constexpr double kLo[kSignatureDim] = {0.2, 0.0, 0.3, 0.0, 2.5, -3.0, 0.0, 0.2};
constexpr double kHi[kSignatureDim] = {4.0, 1.0, 1.0, 6.0, 12.0, 3.0, 12.0, 6.0};
}  // namespace

bool Signature::valid() const {
    for (double x : v)
        if (!std::isfinite(x)) return false;
    return true;
}

double Signature::distanceTo(const Signature& o) const {
    double acc = 0.0;
    for (size_t i = 0; i < kSignatureDim; ++i) {
        const double span = kHi[i] - kLo[i];
        const double a = (v[i] - kLo[i]) / span;
        const double b = (o.v[i] - kLo[i]) / span;
        const double d = a - b;
        acc += d * d;
    }
    return std::clamp(std::sqrt(acc / static_cast<double>(kSignatureDim)), 0.0, 1.0);
}

void SignatureStore::add(const Signature& s) {
    if (!s.valid()) return;
    entries_.push_back(s);
    while (entries_.size() > capacity_) entries_.pop_front();
}

bool SignatureStore::nearest(const Signature& s, Signature& match, double& distance) const {
    if (entries_.empty() || !s.valid()) return false;
    double best = 1e9;
    const Signature* bestSig = nullptr;
    for (const auto& e : entries_) {
        const double d = s.distanceTo(e);
        if (d < best) {
            best = d;
            bestSig = &e;
        }
    }
    if (!bestSig) return false;
    match = *bestSig;
    distance = best;
    return true;
}

}  // namespace radar