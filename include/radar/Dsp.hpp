// The mathematical core. Everything here is real signal processing on measured
// data -- there is no synthetic-sample path anywhere in this header.
#pragma once

#include <complex>
#include <cstddef>
#include <functional>
#include <random>
#include <vector>

#include "radar/Config.hpp"

namespace radar {

// ---------------------------------------------------------------- statistics

struct Stats {
    double n = 0, mean = 0, variance = 0, stddev = 0, min = 0, max = 0;
    double skew = 0, kurtosis = 0;
};

Stats computeStats(const std::vector<double>& x);
// Standard normal quantile, Acklam's rational approximation. Accurate to
// ~1.15e-9 over (0,1), which matters because we invert it for CFAR.
double normalQuantile(double p);
// Regularised lower incomplete gamma, used for the OS-CFAR threshold.
double gammaP(double a, double x);

// ------------------------------------------------------------------- windowing

// Centred moving average.
std::vector<double> movingAverage(const std::vector<double>& x, int halfWindow);
// Savitzky-Golay polynomial smoother: preserves peak shape far better than a
// boxcar, which matters because we are measuring small bumps in RSSI.
std::vector<double> savitzkyGolay(const std::vector<double>& x, int order, int halfWindow);
std::vector<double> exponentialSmoothing(const std::vector<double>& x, double alpha);
// Remove linear trend; exposes the micro-motion signature under the drift.
std::vector<double> detrend(const std::vector<double>& x);

// ------------------------------------------------------------------------ FFT

// In-place iterative radix-2 Cooley-Tukey. `inverse` applies the 1/N scaling.
void fft(std::vector<std::complex<double>>& a, bool inverse);
// Normalised power spectrum of a real signal, DC through Nyquist.
std::vector<double> powerSpectrum(const std::vector<double>& x, double sampleRateHz);
// Windowed (Hann) power spectrum.
std::vector<double> windowedPowerSpectrum(const std::vector<double>& x, double sampleRateHz);

// ------------------------------------------------------------------ path loss

struct PathLossModel {
    double referenceDistanceM = 1.0;
    double rssiAtRefDbm = -40.0;
    double exponent = 2.8;
    double shadowSigmaDb = 3.0;

    // RSSI -> range.
    double rangeFromRssi(double rssiDbm) const;
    // Range -> RSSI.
    double rssiFromRange(double rangeM) const;
    // Convert shadowing sigma (dB) into an equivalent range sigma (m) by
    // linearising the model at the current range.
    double rangeSigmaAt(double rangeM) const;
};

// Distance between two planar anchors.
double planarDistance(double x1, double y1, double x2, double y2);

// Weighted least-squares trilateration. `observations` are (anchorIndex, range,
// variance) triples. Returns false when the geometry is degenerate.
struct TrilatResult {
    bool ok = false;
    double x = 0, y = 0;
    double covariance[4]{};  // xx, xy, yy, and residual variance
    double gdop = 0.0;       // geometric dilution of precision
    double residualM = 0.0;
    int used = 0;
};
TrilatResult trilaterate(const std::vector<Anchor>& anchors,
                         const std::vector<std::array<double, 3>>& obs);

// --------------------------------------------------------------- Kalman / EKF

// Linear Kalman filter on [x, y, vx, vy] with a constant-velocity model.
class KalmanFilter {
  public:
    void reset(double x = 0, double y = 0);
    void setProcessNoise(double accelSigma, double dt);
    void predict(double dt);
    // Measurement: an observed range to a known anchor. Falls back to a
    // Cartesian update when the anchor position is unknown.
    bool updateRange(const Anchor& anchor, double measuredRangeM, double rangeVar);
    bool updatePosition(double x, double y, double varM);
    const TargetState& state() const { return s_; }
    bool initialised() const { return init_; }

  private:
    TargetState s_{};
    bool init_ = false;
};

// Extended Kalman filter: same state, but h(x) = range is nonlinear, so we
// linearise it with the analytic Jacobian.
class ExtendedKalmanFilter {
  public:
    void reset(double x = 0, double y = 0);
    void setProcessNoise(double accelSigma);
    void predict(double dt);
    bool updateRange(const Anchor& anchor, double measuredRangeM, double rangeVar);
    const TargetState& state() const { return s_; }
    bool initialised() const { return init_; }
    double innovation() const { return lastInnovation_; }

  private:
    TargetState s_{};
    bool init_ = false;
    double lastInnovation_ = 0.0;
};

// Sequential importance sampling particle filter. Handles the strongly
// non-Gaussian likelihood you get from log-distance inversion near the edge of
// a room, where the EKF's Gaussian assumption breaks down.
class ParticleFilter {
  public:
    void reset(double x = 0, double y = 0, double spreadM = 3.0);
    void setProcessNoise(double accelSigma, double dt);
    void setParticleCount(int n);
    void setResampleThreshold(double t);
    void predict(double dt);
    bool updateRange(const Anchor& anchor, double measuredRangeM, double rangeVar);
    void updateWeight(double logLikelihood);
    const TargetState& state() const { return s_; }
    int particleCount() const { return static_cast<int>(particles_.size()); }
    double effectiveSampleSize() const;

  private:
    void resampleIfNeeded();
    struct Particle { double x, y, vx, vy, w; };
    std::vector<Particle> particles_;
    TargetState s_{};
    std::mt19937 rng_{0x5EED1234u};
    std::normal_distribution<double> normal_{0.0, 1.0};
    double accumLogLik_ = 0.0;
    int sinceResample_ = 0;
    bool init_ = false;
    double accelSigma_ = 0.8;
    double resampleThreshold_ = 0.55;
};

// ------------------------------------------------------------------ spectral

struct SpectralPeak {
    double freqHz = 0.0;
    double power = 0.0;
};

// Welch averaged periodogram: split the record into overlapping Hann-windowed
// segments, periodogram each, and average. Averaging cuts the variance of the
// estimate by roughly the number of averages, which matters a great deal on the
// short, noisy records this application has to work with.
//
// The grid spans exactly one Nyquist band, so bin g of a gridPoints-wide result
// sits at (fs/2) * g / gridPoints. Pass binHz = (fs/2)/gridPoints to findPeaks.
//
// Deliberately NOT an AR/ESPRIT estimator: the lattice recursions (Burg,
// Levinson with a lattice parameterisation) have several incompatible
// sign/index conventions and the high-order forms are numerically fragile on
// records this short. A well-averaged periodogram is measurably more reliable
// here and, unlike them, is verifiable against a known tone.
std::vector<double> welchSpectrum(const std::vector<double>& x, double sampleRateHz,
                                  int segmentLength, double overlapFraction = 0.5);

std::vector<SpectralPeak> findPeaks(const std::vector<double>& spec, double binHz, int maxPeaks,
                                    double minSeparationHz);

// Envelope-beat Doppler model.
//
// The received envelope is the magnitude of a sum of paths. When a reflector
// moves by d, the phase difference between two paths changes by 2*pi*d/lambda,
// so the envelope beats at exactly f_beat = d_dot / lambda -- i.e.
//
//     v_radial = f_beat * lambda,      lambda = c / f_carrier
//
// This is the correct relation for RSSI-only sensing and it puts human motion
// squarely in the observable band: at 2.437 GHz lambda = 12.3 cm, so 0.1-2 m/s
// appears as 0.8-16 Hz.
//
// The naive "v = c / (2 f)" two-path delay model is wrong for this use: it
// implies a ~37 MHz beat for a 1 m/s walk, which no 50 Hz sampler could observe.
double beatFrequencyToRadialSpeed(double beatHz, double wavelengthM);
double radialSpeedToBeatFrequency(double speedMps, double wavelengthM);

// Wavelength in metres for a carrier frequency.
double wavelengthMetres(double carrierHz, double speedOfLight);

// Autocorrelation of a real sequence, lags 0..maxLag.
std::vector<double> autocorrelation(const std::vector<double>& x, int maxLag);

// ------------------------------------------------------------------- detector

struct DetectionResult {
    DetectionState state = DetectionState::Clear;
    double statistic = 0.0;   // the detector's decision value
    double threshold = 0.0;
    double confidence = 0.0;
    double logOdds = 0.0;
    bool entropyTrip = false;
    bool chiSquareTrip = false;
    bool cfarTrip = false;
    int alarmBin = -1;
};

// Sliding-window motion detector combining three independent statistics so no
// single failure mode (a fading dip, a duty-cycle change, a burst of noise)
// can fake a detection on its own.
class MotionDetector {
  public:
    void configure(const DetectorConfig& c);
    void reset();
    // `samples` are post-smoothing RSSI in dBm, oldest first.
    DetectionResult evaluate(const std::vector<double>& samples);

    const std::vector<double>& baselineHistogram() const { return baselineHist_; }
    void setBaselineHistogram(const std::vector<double>& h) { baselineHist_ = h; }

  private:
    double cfarThreshold(const std::vector<double>& statisticSeries) const;
    double entropyChange(double currentEntropy) const;

    DetectorConfig cfg_{};
    std::vector<double> baselineHist_;
    std::vector<double> statSeries_;
    double logOdds_ = 0.0;
    double confidence_ = 0.0;
    int activeSamples_ = 0;
    int absentSamples_ = 0;
    DetectionState state_ = DetectionState::Clear;
    std::vector<double> entropySeries_;
};

// Classical CFAR thresholds. Implemented separately so the choice is explicit
// and testable rather than buried in the detector.
double cfarThresholdCa(const std::vector<double>& cell, int guard, int train);
double cfarThresholdGo(const std::vector<double>& cell, int guard, int train);
double cfarThresholdZf(const std::vector<double>& cell, int guard, int train);

// Shannon entropy of a histogram normalised to [0,1].
double histogramEntropy(const std::vector<double>& hist);
// Pearson chi-square between two equal-length histograms.
double chiSquare(const std::vector<double>& observed, const std::vector<double>& expected);

// ------------------------------------------------------------------- fusion

struct RadioContribution {
    RadioKind radio = RadioKind::Unknown;
    bool valid = false;
    double x = 0, y = 0, vx = 0, vy = 0;
    double rangeM = 0;
    double bearingDeg = 0;
    double confidence = 0;
    double rangeVar = 1.0;
    double logOdds = 0.0;
    double velocityMps = 0;
};

// Inverse-variance weighted fusion with per-radio gating and log-odds evidence
// accumulation into a single posterior confidence.
struct FusionResult {
    bool ok = false;
    double x = 0, y = 0, vx = 0, vy = 0;
    double rangeM = 0, bearingDeg = 0, speedMps = 0;
    double confidence = 0;
    double posteriorMotion = 0;  // 0..1
    double covariance[4]{};
    int radiosUsed = 0;
};
FusionResult fuse(const std::vector<RadioContribution>& contributions,
                  const FusionConfig& cfg);

// --------------------------------------------------------------- calibration

// Sequential least-squares fit of (rssiAtRef, exponent) against known
// (anchor, range) pairs. Used by the calibration routine so path-loss
// constants come from measurements on this room rather than from a textbook.
struct PathLossCalibration {
    bool ok = false;
    double rssiAtRefDbm = -40.0;
    double exponent = 2.8;
    double shadowSigmaDb = 3.0;
    int samples = 0;
};
PathLossCalibration calibratePathLoss(const PathLossModel& seed,
                                      const std::vector<std::pair<double, double>>& rangeRssiPairs);

}  // namespace radar