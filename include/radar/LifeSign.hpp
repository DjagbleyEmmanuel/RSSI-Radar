// Life-sign detection: looking for breathing in the received-power envelope.
//
// A resting person's chest moves the order of 5 mm at roughly 0.1-0.4 Hz, and a
// few millimetres of reflector is enough to change a 5 GHz signal measurably.
// This is the basis of camera-free "WiFi Sleep" sensing. It is a genuinely
// different measurement from gross motion: a person can be completely still and
// still be detected.
//
// The constraints here are severe and are enforced rather than glossed over:
//
//   * Separating 0.15 Hz breathing from slow drift needs several cycles, so the
//     window is long -- minutes, not seconds -- and the detector reports nothing
//     until it has that much history. A short window would produce a confident
//     number that is really just a trend.
//   * The series is resampled onto a uniform grid first. Samples arrive with the
//     traffic, not on a clock, and an FFT of an unevenly sampled series smears
//     energy across every bin.
//   * A linear trend is removed before analysis, for the same reason.
//   * Two conditions must both hold: a dominant line in the band, *and* a quiet
//     envelope. Either alone produces confident nonsense -- a sharp line in a
//     violently varying signal is a person walking past, not someone asleep.
//
// Even when this fires it reports "a periodic component consistent with
// respiration", never "a person is breathing". The same signature is produced by
// a fan, a curtain on a draught, or a machine on a timer.
#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace radar {

struct LifeSignReport {
    bool detected = false;
    double confidence = 0.0;
    // Dominant rate in the respiration band, Hz.
    double rateHz = 0.0;
    // 0..1: how much more concentrated the peak is than its surroundings.
    double strength = 0.0;
    // Raw matched-band ratio. 1.0 is exactly what broadband noise produces.
    double ratio = 1.0;
    // RMS of the detrended envelope in dB. Small for breathing, large for motion.
    double rmsDb = 0.0;
    double windowSeconds = 0.0;
    size_t samples = 0;
    // Why the detector is in its current state, shown verbatim in the UI.
    std::string reason;
};

class LifeSignDetector {
  public:
    struct Config {
        double minHz = 0.08;
        double maxHz = 0.5;
        // How many complete cycles of the slowest rate to wait for.
        //
        // This is a discrimination limit, not just a patience setting. The number
        // of independent frequency bins in the search band is roughly
        // (maxHz-minHz)*T, and a tone is detectable above the largest noise bin
        // only once that count is large. At T=37 s there are ~20 bins and a
        // clean tone scores about 5x the background while the largest of 20 noise
        // bins scores about 3x -- they overlap, and pure noise was reported as
        // respiration. At T=125 s there are ~50 bins, a tone scores ~12x and
        // noise ~4x, which separates cleanly.
        double minCycles = 10.0;
        size_t minSamples = 256;
        // Minimum share of in-band power in a single line.
        double minStrength = 0.55;
        // Envelope must be this quiet, in dB RMS, or the periodicity is motion.
        double maxRmsDb = 1.6;
    };

    void configure(const Config& c);
    void reset();

    // Feed one observation. Times need not be uniform.
    void push(double rssiDbm, double wallTime);

    LifeSignReport evaluate(double sampleRateHz);

    // Seconds of history currently retained.
    double heldSeconds() const {
        return samples_.size() < 2 ? 0.0 : samples_.back().first - samples_.front().first;
    }
    size_t filled() const { return samples_.size(); }

  private:
    Config cfg_;
    // Retained as (time, value) pairs and trimmed by age. Sizing the window in
    // samples does not work: the rate depends on the channel's traffic.
    std::deque<std::pair<double, double>> samples_;
    LifeSignReport report_;
};

}  // namespace radar