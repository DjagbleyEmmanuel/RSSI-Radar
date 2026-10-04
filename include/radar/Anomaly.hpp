// Statistical anomaly detection over an RSSI series.
//
// The premise of this module is narrow and worth stating plainly. With RSSI-only
// hardware there is no phase, no per-antenna geometry and no channel state
// information, so a great deal that a radar operator would want to know simply
// is not observable, and no amount of mathematics will manufacture it. What
// *is* observable is the statistical texture of the received power: how its
// variance changes, whether it became periodic, whether a tone appeared in the
// envelope, whether the mean is drifting, and where the outliers are.
//
// Every test here is a real statistical test with a stated null hypothesis, run
// against a baseline estimated from the signal's own history. Each result
// carries what it means and, just as importantly, what it does not prove. A
// flagged anomaly is evidence of a change in the channel, not proof of a person.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace radar {

enum class AnomalyKind {
    VarianceShift,   // the fluctuation energy changed: something started or stopped moving
    Periodicity,     // a repeating structure appeared in the envelope (cadence, duty cycle)
    ToneEmergence,   // the envelope became tonal rather than noise-like
    LevelShift,      // a step change in mean level (CUSUM)
    Drift,           // a slow monotone trend (Mann-Kendall)
    Impulse,         // heavy tails / spikes: kurtosis and robust outliers
    Flatline,        // the signal stopped moving, which is itself suspicious
    Collision        // an unusually deep, sustained fade
};

const char* toString(AnomalyKind k);

struct Anomaly {
    AnomalyKind kind = AnomalyKind::VarianceShift;
    // 0..1, how strongly the evidence fires. Comparable within a kind, not
    // across kinds -- these are deliberately not normalised into one scale
    // because they measure different things.
    double score = 0.0;
    // Two-sided approximate p-value where one is defined.
    double pValue = 1.0;
    std::string headline;   // short, for a list
    std::string detail;     // what was measured
    std::string caveat;     // what this does NOT establish
    // Where in the series it happened, 0..1 from the start.
    double atFraction = 0.0;
    bool active = false;
};

struct AnomalyReport {
    std::vector<Anomaly> anomalies;
    // Series descriptors, useful in their own right and shown in the UI so the
    // operator can see *why* something was flagged.
    double meanDbm = 0.0;
    double stdDbm = 0.0;
    double varianceRatio = 1.0;   // var(dx)/var(x), the classic ARCH-style ratio
    double spectralFlatness = 0.0;  // 0 = pure tone, 1 = white
    double shannonEntropyBits = 0.0;
    double dominantPeriodS = 0.0;
    double dominantPeriodStrength = 0.0;
    double trendPerMinute = 0.0;
    double kurtosis = 3.0;
    int outlierCount = 0;
    double flatlineFraction = 0.0;
    size_t samples = 0;
    bool enoughData = false;
};

class AnomalyDetector {
  public:
    struct Config {
        // Minimum samples before any verdict is offered. Below this the report
        // is returned with enoughData=false rather than guessing.
        size_t minSamples = 48;
        // Score above which an anomaly is reported as active.
        double reportThreshold = 0.5;
        // Rolling window of samples used as the baseline for "normal".
        size_t baselineWindow = 256;
    };

    void configure(const Config& c) { cfg_ = c; }
    void reset();

    // Analyse one transmitter's RSSI series (post-smoothing, oldest first).
    AnomalyReport analyse(const std::vector<double>& samplesDbm, double sampleRateHz);

  private:
    Config cfg_;
};

// Exposed for the self-test and for reuse by the UI's explanation text.
double spectralFlatnessOf(const std::vector<double>& x);
double kurtosisOf(const std::vector<double>& x);
double varianceRatioOf(const std::vector<double>& x);
double shannonEntropyBits(const std::vector<double>& x);
double dominantPeriod(const std::vector<double>& x, double sampleRateHz, double* strength);
// Mann-Kendall S statistic and its two-sided normal-approximation p-value.
double mannKendall(const std::vector<double>& x, double* pValue);
// Mean and variance of the first difference before and after `at`.
struct SegmentStats {
    double meanBefore = 0.0, varBefore = 0.0;
    double meanAfter = 0.0, varAfter = 0.0;
};
SegmentStats bestLevelShift(const std::vector<double>& x);

}  // namespace radar
