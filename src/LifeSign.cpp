#include "radar/LifeSign.hpp"

#include "radar/Dsp.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace radar {

void LifeSignDetector::configure(const Config& c) { cfg_ = c; }

void LifeSignDetector::reset() {
    samples_.clear();
    report_ = LifeSignReport{};
}

void LifeSignDetector::push(double rssiDbm, double wallTime) {
    if (rssiDbm <= -100.0) return;  // not a measurement
    samples_.emplace_back(wallTime, rssiDbm);

    // Retention is by time, not by sample count. The sample rate is whatever the
    // radio happens to deliver -- it depends on the channel's traffic -- so a
    // fixed-length buffer holds a different duration every run. Sizing it in
    // samples meant the buffer covered 12.8 s where 37.5 s was needed, and the
    // analysis then indexed past the end of it.
    const double want = std::max(20.0, cfg_.minCycles / std::max(1e-6, cfg_.minHz));
    while (samples_.size() > 2 && (samples_.back().first - samples_.front().first) > want * 1.25)
        samples_.pop_front();
}

LifeSignReport LifeSignDetector::evaluate(double sampleRateHz) {
    LifeSignReport r;
    r.samples = samples_.size();
    if (samples_.size() < 16) {
        r.reason = "collecting";
        return r;
    }

    const double span = samples_.back().first - samples_.front().first;
    r.windowSeconds = span;
    const double want = std::max(20.0, cfg_.minCycles / std::max(1e-6, cfg_.minHz));
    if (span < want * 0.9) {
        r.reason = "collecting";
        return r;
    }
    if (sampleRateHz < 0.5) {
        r.reason = "sample rate too low to resolve a 0.1 Hz cycle";
        return r;
    }

    // Resample onto a uniform grid. Samples arrive with the traffic, not on a
    // clock, and analysing an unevenly sampled series directly leaks energy
    // across the whole band.
    const int N = static_cast<int>(span * sampleRateHz);
    if (N < 64) {
        r.reason = "not enough samples for the window";
        return r;
    }
    std::vector<double> y(static_cast<size_t>(N), 0.0);
    size_t j = 0;
    const double t0 = samples_.front().first;
    for (int i = 0; i < N; ++i) {
        const double want_t = t0 + static_cast<double>(i) / sampleRateHz;
        while (j + 1 < samples_.size() && samples_[j + 1].first <= want_t) ++j;
        y[static_cast<size_t>(i)] = samples_[j].second;
    }

    // Remove mean, linear trend and curvature.
    //
    // A linear detrend alone left a 0.008 Hz drift component inside the search
    // band and it was reported as a 0.21 Hz respiration cycle. Quadratic is
    // enough to suppress the slow wander that dominates a real RSSI envelope
    // without eating into 0.1 Hz, which is the bottom of the band we care about.
    {
        // Least squares against [1, t, t^2] using the normal equations.
        const double n = static_cast<double>(N);
        double S0 = n, S1 = 0, S2 = 0, S3 = 0, S4 = 0;
        double T0 = 0, T1 = 0, T2 = 0;
        for (int i = 0; i < N; ++i) {
            const double t = static_cast<double>(i);
            const double v = y[static_cast<size_t>(i)];
            const double t2 = t * t;
            S1 += t;  S2 += t2;  S3 += t2 * t;  S4 += t2 * t2;
            T0 += v; T1 += t * v; T2 += t2 * v;
        }
        const double m[3][3] = {{S0, S1, S2}, {S1, S2, S3}, {S2, S3, S4}};
        double rhs[3] = {T0, T1, T2};
        // Gaussian elimination with partial pivoting.
        double a[3][4] = {{m[0][0], m[0][1], m[0][2], rhs[0]},
                          {m[1][0], m[1][1], m[1][2], rhs[1]},
                          {m[2][0], m[2][1], m[2][2], rhs[2]}};
        for (int c = 0; c < 3; ++c) {
            int piv = c;
            for (int r2 = c + 1; r2 < 3; ++r2)
                if (std::fabs(a[r2][c]) > std::fabs(a[piv][c])) piv = r2;
            if (std::fabs(a[piv][c]) < 1e-12) continue;  // singular: leave the fit alone
            for (int k = 0; k < 4; ++k) std::swap(a[c][k], a[piv][k]);
            for (int r2 = 0; r2 < 3; ++r2) {
                if (r2 == c) continue;
                const double f = a[r2][c] / a[c][c];
                for (int k = c; k < 4; ++k) a[r2][k] -= f * a[c][k];
            }
        }
        double coef[3] = {0, 0, 0};
        for (int c = 0; c < 3; ++c)
            if (std::fabs(a[c][c]) > 1e-12) coef[c] = a[c][3] / a[c][c];
        for (int i = 0; i < N; ++i) {
            const double t = static_cast<double>(i);
            y[static_cast<size_t>(i)] -= (coef[0] + coef[1] * t + coef[2] * t * t);
        }
    }

    double rms = 0.0;
    for (double v : y) rms += v * v;
    rms = std::sqrt(rms / static_cast<double>(N));
    // A floor, not an epsilon. Detrending a pure linear ramp leaves residue of
    // order 1e-8 dB, and the ratio test below reads a median of essentially zero
    // as "perfectly tonal", so a pure drift was being reported as respiration.
    // A real chest displacement moves the envelope by far more than this.
    // 0.15 dB RMS. A real chest displacement of a few millimetres moves a 5 GHz
    // envelope by a good fraction of a dB; the synthetic cases that slipped
    // through at 0.05 dB were drift residue spanning the whole window, which no
    // polynomial detrend can separate from a slow trend.
    constexpr double kMinRmsDb = 0.15;
    if (rms < kMinRmsDb) {
        r.rmsDb = rms;
        r.reason = "no variation to analyse";
        return r;
    }
    r.rmsDb = rms;

    // Hann-windowed FFT, zero-padded for resolution.
    //
    // Goertzel over a rectangular window was tried first and measured the rate
    // correctly but reported a peak strength of only 0.2 for a clean tone: a
    // rectangular window's sidelobes spread a single line across many bins, so
    // "share of total in-band power" collapsed regardless of how pure the tone
    // was. A Hann window concentrates a line properly, which is the whole point
    // of the ratio.
    int nfft = 1;
    while (nfft < N * 2) nfft <<= 1;  // zero-pad: finer bins, same data
    std::vector<std::complex<double>> buf(static_cast<size_t>(nfft), {0.0, 0.0});
    for (int i = 0; i < N; ++i)
        buf[static_cast<size_t>(i)] =
            {y[static_cast<size_t>(i)] * (0.5 - 0.5 * std::cos(2.0 * M_PI * i / (N - 1))), 0.0};
    fft(buf, false);

    const double binHz = sampleRateHz / static_cast<double>(nfft);
    const int kLo = std::max(1, static_cast<int>(std::ceil(cfg_.minHz / binHz)));
    const int kHi = std::min(nfft / 2 - 1, static_cast<int>(cfg_.maxHz / binHz));
    if (kHi <= kLo) {
        r.reason = "respiration band is narrower than one bin";
        return r;
    }

    std::vector<double> band;
    band.reserve(static_cast<size_t>(kHi - kLo + 1));
    for (int k = kLo; k <= kHi; ++k) band.push_back(std::norm(buf[static_cast<size_t>(k)]));
    const int peakAt = static_cast<int>(std::max_element(band.begin(), band.end()) - band.begin());
    r.rateHz = static_cast<double>(kLo + peakAt) * binHz;

    double total = 0.0;
    for (double v : band) total += v;
    if (total <= 0.0) {
        r.reason = "no spectral content";
        return r;
    }

    // Matched-band ratio.
    //
    // A single-bin peak cannot separate a tone from noise here, and the first
    // version of this detector tried exactly that and reported "respiration" for
    // pure 0.3 dB noise. The reason is geometric: the Rayleigh resolution of a
    // 37 s window is about 0.021 Hz, so a Hann-windowed line occupies roughly
    // four bins of the search band whatever its purity, and the largest of ~20
    // noise bins is of the same order. Both give peak/median near 4.
    //
    // What does separate them is where the power sits. A line puts nearly all of
    // its power inside its own main lobe; noise spreads power evenly across the
    // band. So integrate the lobe and compare it with the mean of everything
    // outside it. For noise the ratio is 1 by construction; for a line it is the
    // fraction of the band the lobe occupies, which is large.
    const double rayleighBins = sampleRateHz / static_cast<double>(N);
    const int halfWin =
        std::max(1, static_cast<int>(std::lround(2.0 * rayleighBins / binHz)) + 1);
    const int lo = std::max(0, peakAt - halfWin);
    const int hi = std::min(static_cast<int>(band.size()) - 1, peakAt + halfWin);

    double inWin = 0.0, outSum = 0.0;
    size_t outCount = 0;
    for (size_t i = 0; i < band.size(); ++i) {
        if (static_cast<int>(i) >= lo && static_cast<int>(i) <= hi)
            inWin += band[i];
        else {
            outSum += band[i];
            ++outCount;
        }
    }
    const double outMean = outCount > 0 ? outSum / static_cast<double>(outCount) : 0.0;
    const double winCount = static_cast<double>(hi - lo + 1);
    const double ratio = outMean > 0.0 ? inWin / (outMean * winCount) : 0.0;

    r.ratio = ratio;
    // Map the ratio to 0..1 for display and thresholding. Noise sits at 1; a
    // clean line lands near the inverse of its lobe's share of the band.
    r.strength = std::clamp(1.0 - 1.0 / std::max(1.0, ratio), 0.0, 1.0);
    // Both conditions must hold. A sharp line in a violently varying envelope is
    // a person walking past, not someone asleep, and reporting that as
    // respiration would be the worst possible failure for this feature.
    const bool lineStrong = r.strength >= cfg_.minStrength;
    const bool quietEnough = r.rmsDb <= cfg_.maxRmsDb;
    if (lineStrong && quietEnough) {
        r.detected = true;
        r.confidence =
            std::clamp((r.strength - cfg_.minStrength) / std::max(1e-6, 1.0 - cfg_.minStrength),
                       0.0, 1.0) *
            std::clamp(1.0 - (r.rmsDb - cfg_.maxRmsDb * 0.5) / std::max(1e-6, cfg_.maxRmsDb),
                       0.0, 1.0);
        r.reason = "periodic component consistent with respiration";
    } else if (!lineStrong) {
        r.reason = "no dominant period in the respiration band";
    } else {
        r.reason = "signal too active to be respiration";
    }
    report_ = r;
    return r;
}

}  // namespace radar