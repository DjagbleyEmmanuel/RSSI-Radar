// Every tunable in one struct, JSON-serialisable, with a preset library.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "radar/Types.hpp"

namespace radar {

struct PathLossConfig {
    // Reference distance d0 and the RSSI a calibrated link reads at it.
    double referenceDistanceM = 1.0;
    double rssiAtRefDbm = -40.0;
    // Path-loss exponent. 2.0 is free space, 2.8-3.4 typical indoor clutter.
    double pathLossExponent = 2.8;
    // Shadowing standard deviation, in dB. Sets the measurement noise floor.
    double shadowSigmaDb = 3.0;
    // Ignore observations weaker than this; below roughly -100 dBm an 802.11
    // receiver is reading its own noise floor. Measured beacons from a phone
    // hotspot in this environment sit near -96 dBm, so this must not be tighter.
    double minRssiDbm = -100.0;
    double maxRssiDbm = -20.0;
};

struct WindowConfig {
    // Sliding window length in seconds for statistics and detection.
    double windowSeconds = 4.0;
    // Decimation: keep one sample per this many received frames. Trades
    // latency against variance.
    int decimation = 1;
    // Exponential smoothing factor for the RSSI estimate, 0 < a <= 1.
    double emaAlpha = 0.35;
    // Savitzky-Golay smoothing: odd polynomial order and half-window.
    int sgOrder = 3;
    int sgHalfWindow = 6;
};

struct EstimatorConfig {
    // Process noise: std-dev of acceleration, m/s^2. Higher tracks more.
    double processNoiseAccel = 0.8;
    // Measurement noise on range, metres. Tied to pathLossConfig.shadowSigmaDb
    // unless explicitly overridden.
    double rangeNoiseM = 1.5;
    double initialPositionSigmaM = 4.0;
    double initialVelocitySigmaMps = 1.0;

    // Particle filter
    int particleCount = 2000;
    double resampleThreshold = 0.55;
    int resampleEvery = 12;

    // Weighted-least-squares anchors actually used for trilateration.
    bool useAnchorTrilateration = true;
    int minAnchorsForFix = 3;
    double anchorWeightFloor = 0.05;
};

struct SpectralConfig {
    // Welch averaged periodogram. segmentLength sets the frequency resolution
    // (fs / segmentLength); overlap trades variance against edge effects.
    int welchSegment = 128;
    double welchOverlap = 0.5;
    // Carrier frequency of the link under observation; sets the wavelength that
    // converts an envelope beat frequency into a radial velocity.
    double carrierFrequencyHz = 2437e6;
    double speedOfLight = 299792458.0;
    // Band of envelope-beat frequencies that correspond to plausible human
    // motion at the above carrier (0.1-2 m/s at 2.4 GHz -> ~0.8-16 Hz).
    double fMinHz = 0.05;
    double fMaxHz = 20.0;
    bool enableVelocity = true;
};

struct DetectorConfig {
    // Detection statistic is the z-score of the windowed mean-abs-difference.
    double baseThresholdSigma = 3.0;
    // CFAR guard/training cells in the time-frequency plane.
    int cfarGuardCells = 3;
    int cfarTrainCells = 12;
    // falseAlarmRate in (0,1); converted to a threshold via the OS-CFAR
    // formula so the operator thinks in the units they actually care about.
    double falseAlarmRate = 0.02;
    // Entropy change detector on the binned RSSI histogram.
    bool useEntropy = true;
    double entropyThreshold = 0.18;
    int histogramBins = 16;
    // Chi-square goodness of fit against the baseline distribution.
    bool useChiSquare = true;
    double chiSquareThreshold = 15.0;
    // Hold/drop hysteresis, in seconds.
    double holdSeconds = 1.5;
    double departSeconds = 2.5;
};



struct FusionConfig {
    // Inverse-variance weighting across radios.
    bool inverseVarianceWeighting = true;
    // Per-radio prior weights; WiFi is denser than BT so it leads by default.
    double wifiWeight = 1.0;
    double bluetoothWeight = 0.65;
    // Drop a radio's contribution when its own confidence falls below this.
    double radioGateConfidence = 0.15;
    // Log-odds evidence contributed per confirming frame.
    double motionLogOdds = 0.55;
    double stillLogOdds = -0.28;
    double priorLogOddsMotion = -2.2;  // prior odds ~ 0.10
    // Confidence smoothing.
    double confidenceEmaAlpha = 0.12;
};

struct RadioConfig {
    bool enableWifi = true;
    bool enableBluetooth = true;
    bool enableCsi = true;
    bool autoMonitorMode = true;
    // Stop the NetworkManager *service* while sensing, and start it again on
    // exit.
    //
    // This is not optional politeness. On iwlwifi the card only accepts a
    // channel change (SIOCSIWCHAN) while NetworkManager is not holding the
    // device; with NM running the ioctl fails with EBUSY, the card stays on its
    // power-on default of channel 1, and if the access point is anywhere else
    // the monitor interface receives absolutely nothing. Measured here:
    //   NM running   -> set channel refused, 0 frames
    //   NM stopped   -> set channel 8 accepted, 40/40 frames with radiotap
    // Restore is unconditional, including from the signal handler.
    bool stopNetworkManager = true;
    std::string wifiInterface = "auto";
    std::string bluetoothDevice = "hci0";
    int wifiChannel = 0;      // 0 = follow the regulatory default
    bool requestRootForMonitor = true;
};

struct UiConfig {
    double refreshHz = 20.0;
    double historySeconds = 30.0;
    int waterfallRows = 220;
    bool showGrid = true;
    bool showTrails = true;
    bool showVelocitySpectrum = true;
    double rangeRingMetres = 12.0;
    int theme = 0;
};

struct Config {

    // Life-sign detection. The window is derived from minCycles and minHz, so a
    // slower rate needs proportionally more history before anything is reported.
        bool lifeSignEnabled = true;
        double lifeSignMinHz = 0.08;
        double lifeSignMaxHz = 0.5;
        double lifeSignMinCycles = 3.0;
        double lifeSignMinStrength = 0.42;
        // Envelope RMS ceiling, in dB. Above this the periodicity is motion.
        double lifeSignMaxRmsDb = 1.6;
        // Minimum change, in dB, before a transmitter contributes to a direction.
        double directionMinChangeDb = 1.2;
        // How many signatures to keep for comparison.
        size_t signatureCapacity = 32;
    PathLossConfig pathLoss;
    WindowConfig window;
    EstimatorConfig estimator;
    SpectralConfig spectral;
    DetectorConfig detector;
    FusionConfig fusion;
    RadioConfig radio;
    UiConfig ui;

    std::vector<Anchor> anchors;
    std::string presetName = "Balanced Indoor";

    nlohmann::json toJson() const;
    static Config fromJson(const nlohmann::json& j);

    bool save(const std::string& path, std::string* err = nullptr) const;
    static Config load(const std::string& path, std::string* err = nullptr);

    // Sanitise: clamp everything into a numerically safe range.
    Config validated() const;
};

struct Preset {
    std::string name;
    std::string description;
    Config config;
};

// Built-in library. Each one is a genuinely different operating point, not a
// relabelled default.
std::vector<Preset> builtinPresets();
const Preset* findPreset(const std::vector<Preset>& list, const std::string& name);
// Map a preset onto an existing config, preserving the user's anchor list.
Config applyPreset(const Config& base, const Config& presetCfg);

}  // namespace radar