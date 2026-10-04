#include "radar/Anomaly.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "radar/Dsp.hpp"

#include <QString>

namespace radar {
namespace {

double meanOf(const std::vector<double>& x) {
    if (x.empty()) return 0.0;
    return std::accumulate(x.begin(), x.end(), 0.0) / static_cast<double>(x.size());
}

double varianceOf(const std::vector<double>& x, double mean) {
    if (x.size() < 2) return 0.0;
    double acc = 0.0;
    for (double v : x) acc += (v - mean) * (v - mean);
    return acc / static_cast<double>(x.size() - 1);
}

std::vector<double> diffOf(const std::vector<double>& x) {
    std::vector<double> d;
    if (x.size() < 2) return d;
    d.reserve(x.size() - 1);
    for (size_t i = 1; i < x.size(); ++i) d.push_back(x[i] - x[i - 1]);
    return d;
}

// Median absolute deviation, scaled to be a consistent estimator of sigma for
// Gaussian data. Used instead of the standard deviation wherever a single
// outlier would otherwise inflate the very quantity being tested.
double robustSigma(const std::vector<double>& x, double mean) {
    if (x.size() < 2) return 0.0;
    std::vector<double> dev(x.size());
    for (size_t i = 0; i < x.size(); ++i) dev[i] = std::fabs(x[i] - mean);
    const double med = meanOf(dev);
    return 1.4826 * med;
}

// Map a z-like statistic onto 0..1 with a smooth saturating curve, so scores
// are readable without pretending to be probabilities.
double squash(double z) { return 1.0 - std::exp(-0.5 * z * z); }

}  // namespace

const char* toString(AnomalyKind k) {
    switch (k) {
        case AnomalyKind::VarianceShift: return "VARIANCE";
        case AnomalyKind::Periodicity: return "PERIODIC";
        case AnomalyKind::ToneEmergence: return "TONE";
        case AnomalyKind::LevelShift: return "LEVEL";
        case AnomalyKind::Drift: return "DRIFT";
        case AnomalyKind::Impulse: return "IMPULSE";
        case AnomalyKind::Flatline: return "FLATLINE";
        case AnomalyKind::Collision: return "FADE";
    }
    return "UNKNOWN";
}

void AnomalyDetector::reset() {}

// ---------------------------------------------------------------- descriptors

double spectralFlatnessOf(const std::vector<double>& x) {
    // Wiener entropy: geometric mean over arithmetic mean of the power
    // spectrum. 1.0 for white noise, near 0 for a pure tone. It is the
    // standard measure of "how tonal is this", and it is bounded, which makes
    // it comparable across transmitters.
    const size_t n = x.size();
    if (n < 8) return 0.0;
    std::vector<std::complex<double>> buf(n);
    for (size_t i = 0; i < n; ++i) buf[i] = {x[i], 0.0};
    fft(buf, false);
    double logSum = 0.0;
    double sum = 0.0;
    size_t count = 0;
    for (size_t i = 1; i < n / 2; ++i) {  // skip DC
        const double p = std::norm(buf[i]);
        if (p <= 0.0) continue;
        logSum += std::log(p);
        sum += p;
        ++count;
    }
    if (count == 0 || sum <= 0.0) return 0.0;
    return std::exp(logSum / static_cast<double>(count)) / (sum / static_cast<double>(count));
}

double kurtosisOf(const std::vector<double>& x) {
    // Excess kurtosis: 0 for a Gaussian, positive for impulsive (heavy tailed)
    // signals such as interference bursts or a sudden occlusion.
    const size_t n = x.size();
    if (n < 8) return 3.0;
    const double m = meanOf(x);
    const double s = robustSigma(x, m);
    if (s <= 1e-9) return 3.0;
    double acc = 0.0;
    for (double v : x) {
        const double z = (v - m) / s;
        acc += z * z * z * z;
    }
    return acc / static_cast<double>(n);
}

double varianceRatioOf(const std::vector<double>& x) {
    // Engle-style variance ratio. For a white noise process this is ~1. Values
    // well above 1 mean volatility is clustering (bursty interference or motion);
    // well below 1 mean the series has become smoother than noise, which is
    // itself interesting because real RSSI is never that clean.
    const std::vector<double> d = diffOf(x);
    if (d.size() < 4) return 1.0;
    const double vx = varianceOf(x, meanOf(x));
    const double vd = varianceOf(d, meanOf(d));
    if (vx <= 1e-9) return 1.0;
    return vd / (2.0 * vx);
}

double shannonEntropyBits(const std::vector<double>& x) {
    // Entropy of the amplitude distribution over quantised bins. Near the
    // maximum means the level is spread evenly across its range (a busy,
    // varying channel); near zero means it is pinned to one value (idle or
    // stuck).
    if (x.size() < 8) return 0.0;
    double lo = *std::min_element(x.begin(), x.end());
    double hi = *std::max_element(x.begin(), x.end());
    if (hi - lo < 1e-9) return 0.0;
    const int bins = 16;
    std::vector<double> hist(bins, 0.0);
    for (double v : x) {
        int b = static_cast<int>((v - lo) / (hi - lo) * (bins - 1));
        b = std::clamp(b, 0, bins - 1);
        hist[static_cast<size_t>(b)] += 1.0;
    }
    double h = 0.0;
    for (double c : hist) {
        if (c <= 0.0) continue;
        const double p = c / static_cast<double>(x.size());
        h -= p * std::log2(p);
    }
    return h / std::log2(static_cast<double>(bins));  // normalised to 0..1
}

double dominantPeriod(const std::vector<double>& x, double sampleRateHz, double* strength) {
    // Autocorrelation peak beyond the first lag. A walking human, a fan or a
    // rotating camera all impose a periodicity on the envelope; noise does not.
    if (strength) *strength = 0.0;
    const size_t n = x.size();
    if (n < 32 || sampleRateHz <= 0.0) return 0.0;
    const double m = meanOf(x);
    std::vector<double> d(n);
    for (size_t i = 0; i < n; ++i) d[i] = x[i] - m;
    double c0 = 0.0;
    for (double v : d) c0 += v * v;
    if (c0 <= 1e-12) return 0.0;

    const size_t maxLag = std::min(n / 2, static_cast<size_t>(sampleRateHz * 30.0));
    double best = 0.0;
    size_t bestLag = 0;
    for (size_t lag = 2; lag <= maxLag; ++lag) {
        double acc = 0.0;
        for (size_t i = 0; i + lag < n; ++i) acc += d[i] * d[i + lag];
        const double r = acc / c0;
        if (r > best) {
            best = r;
            bestLag = lag;
        }
    }
    if (strength) *strength = best;
    return bestLag > 0 ? static_cast<double>(bestLag) / sampleRateHz : 0.0;
}

double mannKendall(const std::vector<double>& x, double* pValue) {
    // Rank-based trend test. Counts concordant minus discordant pairs; its
    // normal approximation gives a two-sided p-value. Unlike a least-squares
    // slope it is not dragged around by a few large excursions.
    const size_t n = x.size();
    if (pValue) *pValue = 1.0;
    if (n < 8) return 0.0;
    long long s = 0;
    for (size_t i = 0; i < n - 1; ++i)
        for (size_t j = i + 1; j < n; ++j) s += (x[j] > x[i]) - (x[j] < x[i]);
    // Variance of S under the null, with tie correction.
    std::vector<double> sorted = x;
    std::sort(sorted.begin(), sorted.end());
    double tieTerm = 0.0;
    size_t i = 0;
    while (i < n) {
        size_t j = i;
        while (j + 1 < n && sorted[j + 1] == sorted[i]) ++j;
        const double t = static_cast<double>(j - i + 1);
        tieTerm += t * (t - 1.0) * (2.0 * t + 5.0);
        i = j + 1;
    }
    const double nn = static_cast<double>(n);
    const double varS = (nn * (nn - 1.0) * (2.0 * nn + 5.0) - tieTerm) / 18.0;
    if (varS <= 0.0) return 0.0;
    const double z = (s - (s > 0 ? 1.0 : (s < 0 ? -1.0 : 0.0))) / std::sqrt(varS);
    if (pValue) *pValue = 2.0 * (1.0 - normalQuantile(0.5 * std::fabs(z) + 0.5));
    return z;
}

SegmentStats bestLevelShift(const std::vector<double>& x) {
    // Single best two-segment split, scored by a Welch t-statistic on the two
    // halves. This is the classic change-point test; it finds a step that the
    // whole-series mean and variance would smear out.
    SegmentStats best;
    const size_t n = x.size();
    if (n < 32) return best;
    const size_t minSeg = std::max<size_t>(8, n / 8);
    double bestT = 0.0;
    size_t bestAt = minSeg;
    for (size_t k = minSeg; k <= n - minSeg; ++k) {
        std::vector<double> a(x.begin(), x.begin() + static_cast<long>(k));
        std::vector<double> b(x.begin() + static_cast<long>(k), x.end());
        const double ma = meanOf(a), mb = meanOf(b);
        const double va = varianceOf(a, ma), vb = varianceOf(b, mb);
        const double se = std::sqrt(va / static_cast<double>(a.size()) +
                                    vb / static_cast<double>(b.size()));
        if (se <= 1e-9) continue;
        const double t = std::fabs(mb - ma) / se;
        if (t > bestT) {
            bestT = t;
            bestAt = k;
            best.meanBefore = ma;
            best.varBefore = va;
            best.meanAfter = mb;
            best.varAfter = vb;
        }
    }
    (void)bestAt;
    return best;
}

// ------------------------------------------------------------------- analysis

AnomalyReport AnomalyDetector::analyse(const std::vector<double>& x, double sampleRateHz) {
    AnomalyReport r;
    r.samples = x.size();
    if (x.size() < cfg_.minSamples) return r;  // enoughData stays false

    r.enoughData = true;
    r.meanDbm = meanOf(x);
    r.stdDbm = std::sqrt(varianceOf(x, r.meanDbm));
    r.varianceRatio = varianceRatioOf(x);
    r.spectralFlatness = spectralFlatnessOf(x);
    r.shannonEntropyBits = shannonEntropyBits(x);
    r.kurtosis = kurtosisOf(x);
    r.dominantPeriodS = dominantPeriod(x, sampleRateHz, &r.dominantPeriodStrength);

    double pDrift = 1.0;
    const double trendZ = mannKendall(x, &pDrift);
    const double spanS = sampleRateHz > 0 ? static_cast<double>(x.size()) / sampleRateHz : 0.0;
    r.trendPerMinute = spanS > 1.0 ? trendZ * r.stdDbm / (spanS / 60.0) : 0.0;

    // --- flatline: the series stopped moving at all.
    size_t unchanged = 0;
    for (size_t i = 1; i < x.size(); ++i)
        if (std::fabs(x[i] - x[i - 1]) < 0.01) ++unchanged;
    r.flatlineFraction = static_cast<double>(unchanged) / static_cast<double>(x.size() - 1);

    // --- robust outliers.
    const double rs = robustSigma(x, r.meanDbm);
    if (rs > 1e-9) {
        for (double v : x)
            if (std::fabs(v - r.meanDbm) > 4.0 * rs) ++r.outlierCount;
    }

    const auto add = [&](AnomalyKind kind, double score, double p, const std::string& head,
                         const std::string& det, const std::string& cav, double at) {
        Anomaly a;
        a.kind = kind;
        a.score = std::clamp(score, 0.0, 1.0);
        a.pValue = std::clamp(p, 0.0, 1.0);
        a.headline = head;
        a.detail = det;
        a.caveat = cav;
        a.atFraction = std::clamp(at, 0.0, 1.0);
        a.active = a.score >= cfg_.reportThreshold;
        r.anomalies.push_back(a);
    };

    // 1. Variance shift. Null: the level is fluctuating as it always has.
    {
        const double z = std::fabs(std::log(r.varianceRatio + 1e-9)) * 3.0;
        add(AnomalyKind::VarianceShift, squash(z), 1.0,
            r.varianceRatio > 1.0 ? "Volatility up" : "Volatility down",
            QString("var(ΔRSSI)/var(RSSI) = %1  (%2)")
                .arg(r.varianceRatio, 0, 'f', 2)
                .arg(r.varianceRatio > 1.0 ? "burstier than baseline"
                                            : "smoother than baseline")
                .toStdString(),
            "A change in fluctuation, not a change in position. Interference, a "
            "changing multipath environment and a moving object all produce this.",
            0.5);
    }

    // 2. Periodicity. Null: the envelope is noise with no repeating structure.
    {
        const double s = r.dominantPeriodStrength;
        const double p = s > 0 ? std::exp(-static_cast<double>(x.size()) * 0.5 * s * s) : 1.0;
        add(AnomalyKind::Periodicity, squash(std::max(0.0, s) * 6.0), p,
            r.dominantPeriodS > 0 ? "Periodic envelope" : "No periodicity",
            QString("dominant period %1 s, autocorrelation %2")
                .arg(r.dominantPeriodS, 0, 'f', 2)
                .arg(s, 0, 'f', 3)
                .toStdString(),
            "A repeating pattern in received power. Consistent with a periodic "
            "mover or a duty-cycled device; also consistent with access-point "
            "beacon traffic, which is periodic by nature.",
            0.5);
    }

    // 3. Tone emergence. Null: the envelope is noise-like.
    {
        // Flatness near 0 means tonal; near 1 means noise.
        const double score = squash(std::max(0.0, 0.45 - r.spectralFlatness) * 12.0);
        add(AnomalyKind::ToneEmergence, score, 1.0,
            r.spectralFlatness < 0.2 ? "Tonal envelope" : "Noise-like envelope",
            QString("spectral flatness %1 (1 = white, 0 = pure tone)")
                .arg(r.spectralFlatness, 0, 'f', 3)
                .toStdString(),
            "A narrowband component in the power envelope. Can be a rotating or "
            "repeating structure; a genuine Doppler shift would need phase, which "
            "this radio does not provide.",
            0.5);
    }

    // 4. Level shift (change point).
    {
        const SegmentStats seg = bestLevelShift(x);
        const double se = std::sqrt(seg.varBefore / std::max(1.0, x.size() / 2.0) +
                                    seg.varAfter / std::max(1.0, x.size() / 2.0));
        const double t = se > 1e-9 ? std::fabs(seg.meanAfter - seg.meanBefore) / se : 0.0;
        add(AnomalyKind::LevelShift, squash(t), 1.0,
            std::fabs(seg.meanAfter - seg.meanBefore) > r.stdDbm ? "Level step" : "No level step",
            QString("step %1 dB (Welch t = %2)")
                .arg(seg.meanAfter - seg.meanBefore, 0, 'f', 2)
                .arg(t, 0, 'f', 1)
                .toStdString(),
            "A sustained change in average power. Equally consistent with the "
            "target moving and with the access point changing transmit power or "
            "the radio switching rate; RSSI alone cannot separate these.",
            0.5);
    }

    // 5. Slow drift.
    {
        add(AnomalyKind::Drift, squash(std::fabs(trendZ) * 0.8), pDrift,
            std::fabs(trendZ) > 2.0 ? "Trending" : "Stable level",
            QString("Mann-Kendall z = %1, p = %2, ~%3 dB/min")
                .arg(trendZ, 0, 'f', 2)
                .arg(pDrift, 0, 'f', 4)
                .arg(r.trendPerMinute, 0, 'f', 2)
                .toStdString(),
            "A monotone trend across the window. Over minutes this is usually "
            "environmental, not a target: battery-save cycling and temperature "
            "drift both look like this.",
            0.5);
    }

    // 6. Impulsiveness.
    {
        const double score = squash(std::max(0.0, r.kurtosis - 3.5) * 0.35);
        add(AnomalyKind::Impulse, score, 1.0,
            r.kurtosis > 5.0 ? "Impulsive noise" : "Gaussian-ish",
            QString("excess kurtosis %1, %2 outliers beyond 4σ")
                .arg(r.kurtosis, 0, 'f', 2)
                .arg(r.outlierCount)
                .toStdString(),
            "Heavy tails mean occasional large excursions. Usually interference "
            "or an occluding object; it is not by itself evidence of a target.",
            0.5);
    }

    // 7. Flatline.
    {
        add(AnomalyKind::Flatline, squash(r.flatlineFraction * 6.0), 1.0,
            r.flatlineFraction > 0.6 ? "Signal frozen" : "Signal live",
            QString("%1% of consecutive samples unchanged")
                .arg(r.flatlineFraction * 100.0, 0, 'f', 0)
                .toStdString(),
            "An unchanging envelope usually means the capture has stopped, not "
            "that the scene is static. Treated as a fault, not a detection.",
            0.5);
    }

    // 8. Collision: an unusually deep fade against the robust spread.
    {
        const double deepest = *std::min_element(x.begin(), x.end());
        const double z = rs > 1e-9 ? std::fabs(deepest - r.meanDbm) / rs : 0.0;
        add(AnomalyKind::Collision, squash(z * 0.7), 1.0,
            z > 3.0 ? "Deep fade" : "No occlusion",
            QString("minimum %1 dBm, %2σ below mean")
                .arg(deepest, 0, 'f', 1)
                .arg(z, 0, 'f', 1)
                .toStdString(),
            "A sustained low-power episode. Consistent with an obstruction "
            "between the transmitter and this radio; equally consistent with "
            "the transmitter simply ceasing to transmit.",
            0.5);
    }

    std::sort(r.anomalies.begin(), r.anomalies.end(),
              [](const Anomaly& a, const Anomaly& b) { return a.score > b.score; });
    return r;
}

}  // namespace radar
