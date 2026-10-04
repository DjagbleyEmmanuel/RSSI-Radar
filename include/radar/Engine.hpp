// The pipeline: observations -> tracks -> detection -> geometry -> fusion.
#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "radar/Anomaly.hpp"
#include "radar/Config.hpp"
#include "radar/Dsp.hpp"
#include "radar/Sensor.hpp"
#include "radar/Tracker.hpp"

namespace radar {

// One logged detection, carrying enough context that the log line is useful on
// its own without having to correlate it against the graphs.
struct RadarEvent {
    double wallTime = 0.0;
    DetectionState state = DetectionState::Clear;
    std::string text;
    std::string label;        // which transmitter
    double rangeM = 0.0;
    double rangeLoM = 0.0;
    double rangeHiM = 0.0;
    bool rangeValid = false;
    double rssiDbm = 0.0;
    double confidence = 0.0;
    double velocityMps = 0.0;
    bool velocityValid = false;
    double statistic = 0.0;
    uint64_t contactId = 0;
    // Set when the line describes a statistical anomaly rather than a trip, so
    // the UI can style it differently and avoid implying it is a detection.
    bool isAnomaly = false;
};

// One immutable picture of the radar's state, handed to the UI each frame.
struct Snapshot {
    TimePoint stamp{};

    DetectionResult detection;
    FusionResult fusion;
    TargetState ekf, particle;

    // Radial velocity from the two-path multipath beat frequency.
    double beatHz = 0.0;
    double radialVelocityMps = 0.0;
    bool velocityValid = false;
    std::vector<double> velocitySpectrum;
    double velocitySpectrumMaxHz = 0.0;
    double wavelengthM = 0.123;

    // The transmitter we are currently analysing most strongly.
    std::array<uint8_t, 6> primaryMac{};
    std::string primaryLabel;
    std::vector<double> primarySamples;
    std::vector<double> primaryTimes;
    std::vector<double> primarySmoothed;
    double primaryMeanDbm = 0.0;
    double primaryStdDbm = 0.0;
    double primaryRangeM = 0.0;

    // All live transmitters.
    std::vector<RssiTrack> tracks;

    // Tracked contacts with stable ids, ranges and disappearance timeouts.
    std::vector<Contact> contacts;
    uint64_t contactsCreated = 0;

    // Statistical analysis of the primary transmitter's envelope.
    AnomalyReport anomaly;
    // Per-transmitter reports, so every source is analysed and not just the
    // strongest one.
    std::vector<std::pair<std::string, AnomalyReport>> anomalyByLabel;

    std::vector<SensorStatus> sensors;

    // Diagnostics
    double effectiveSampleRateHz = 0.0;
    uint64_t totalObservations = 0;
    uint64_t totalDropped = 0;
    int activeSensorCount = 0;
    std::string primarySensorName;
    // True when a radio is running but delivering nothing. The UI shouts about
    // this rather than silently drawing a flat line.
    bool starved = false;
    std::string starveReason;

    // Trip history for the event log.
    std::vector<RadarEvent> events;
};

// Best-effort, signal-safe restore of any interface this process took into
// monitor mode. Safe to call from a signal handler: it only shells out and does
// not touch the engine's locks.
void restoreAllRadios();

class Engine {
  public:
    Engine();
    ~Engine();

    void setConfig(const Config& cfg);
    Config config() const;
    void applyPreset(const std::string& name);

    bool start(std::string* err);
    void stop();
    bool running() const { return running_.load(std::memory_order_relaxed); }

    // Drain sensor queues and run one pipeline iteration.
    void tick();

    Snapshot snapshot() const;

    HardwareSurvey hardware() const;
    std::vector<Preset> presets() const;
    // Calibration records (range, rssi) pairs seen this session for each
    // anchor, used by the calibration panel.
    PathLossCalibration calibration() const;

    void setAnchorPosition(size_t index, double x, double y);
    void addAnchor(const Anchor& a);
    void removeAnchor(size_t index);

    // Local LE advertising, so the operator has a controllable reference
    // transmitter to observe. Real over-the-air frames.
    bool startReferenceAdvert();

    void resetTracking();

  private:
    void ingest(const Observation& o);
    void selectPrimary();
    void runGeometry(double dt);
    void runVelocity();

    mutable std::mutex cfgMtx_;
    Config cfg_;
    std::string presetName_ = "Balanced Indoor";

    std::vector<std::unique_ptr<ISensor>> sensors_;
    std::atomic<bool> running_{false};
    mutable std::mutex snapMtx_;
    Snapshot snap_;
    std::vector<RadarEvent> events_;

    mutable std::mutex trackMtx_;
    std::map<uint64_t, RssiTrack> tracks_;  // keyed by macHash
    std::array<uint8_t, 6> primaryMac_{};
    bool havePrimary_ = false;

    // Contact tracking and statistical anomaly analysis.
    ContactTracker tracker_;
    AnomalyDetector anomaly_;

    // Estimators, one set per radio family.
    ExtendedKalmanFilter ekf_;
    ParticleFilter pf_;
    KalmanFilter kf_;
    MotionDetector detector_;
    PathLossModel pathLoss_;
    std::mutex estimatorMtx_;

    // Spectral scratch
    std::vector<double> spectrum_;
    std::vector<double> smoothed_;
    std::vector<double> detrended_;
    double sampleRate_ = 50.0;
    std::chrono::steady_clock::time_point lastTick_{};
    std::chrono::steady_clock::time_point lastObs_{};
    uint64_t totalObs_ = 0;
    uint64_t lastRateObs_ = 0;
    uint64_t totalDropped_ = 0;
    int decimationCounter_ = 0;
    bool velocityPeakLatched_ = false;
    double lastBeat_ = 0.0;
};

}  // namespace radar