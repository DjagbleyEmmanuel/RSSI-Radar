// Core value types shared by every layer of the radar.
#pragma once

#include <array>
#include <chrono>
#include <complex>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace radar {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

// Which physical radio produced an observation. Kept open so a plugged-in
// CSI-capable dongle shows up without touching any other code.
enum class RadioKind { Wifi, WifiCsi, Bluetooth, Unknown };

enum class FrameClass { Management, Control, Data, Extension, Unknown };

enum class DetectionState {
    Clear,       // nothing above threshold
    Present,     // a target is held in the track
    Departed,    // track is decaying, confidence falling
    Unavailable  // the owning radio is not producing data
};

const char* toString(RadioKind k);
const char* toString(FrameClass c);
const char* toString(DetectionState s);

// One received over-the-air observation. Filled by an ISensor, consumed by the
// estimator. No field here is ever synthesised: if a radio cannot report it,
// the corresponding flag stays false and the value is undefined.
struct Observation {
    RadioKind radio = RadioKind::Unknown;
    FrameClass frameClass = FrameClass::Unknown;

    std::array<uint8_t, 6> transmitter{};
    std::array<uint8_t, 6> receiver{};

    double rssiDbm = -100.0;
    double snrDb = 0.0;
    double frequencyHz = 0.0;
    double channel = 0.0;
    double bandwidthHz = 20e6;

    // Per-antenna RSSI when the firmware supplies it (radiotap XChannel).
    std::array<int8_t, 4> antennaRssi{};
    int antennaCount = 0;

    // hostTime is our own steady_clock stamp taken on the capture thread.
    // ktimeNs is the kernel SOF_TIMESTAMPING value, which is what makes
    // inter-radio time-of-flight and Doppler estimation actually meaningful.
    TimePoint hostTime{};
    double ktimeNs = 0.0;

    // Channel State Information. Only ever populated by a provider that has
    // real phase-amplitude taps. Intel iwlwifi exposes none, so on this machine
    // hasCsi is permanently false -- see CsiProvider for the auto-detection.
    bool hasCsi = false;
    std::vector<std::complex<float>> csi;
};

// A known transmitter location, used to invert the path-loss model.
struct Anchor {
    std::string id;
    double x = 0.0;
    double y = 0.0;
    RadioKind radio = RadioKind::Wifi;
    // Per-anchor calibration: measured RSSI at the reference distance d0.
    double rssiAtRefDbm = -40.0;
    bool calibrated = false;
};

// State vector is [x, y, vx, vy] in metres / metres-per-second.
struct TargetState {
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;

    // 4x4 covariance, row-major.
    std::array<double, 16> covariance{};

    double rangeM = 0.0;
    double bearingDeg = 0.0;
    double speedMps = 0.0;
    double confidence = 0.0;
    bool valid = false;
    TimePoint stamp{};
};

// Per-radio health, surfaced verbatim in the UI so the operator can always see
// what is physically real right now.
struct SensorStatus {
    std::string name;
    RadioKind kind = RadioKind::Unknown;
    bool available = false;
    bool active = false;
    bool requiresRoot = false;
    std::string detail;
    std::string device;
    std::string driver;
    double rateHz = 0.0;
    uint64_t framesSeen = 0;
    uint64_t framesDropped = 0;
    // The three numbers that matter when diagnosing a capture:
    //   withRadiotapAndSignal -- usable radiometric observations
    //   withRadiotapNoSignal  -- radiotap present but no DBM_ANTSIGNAL field,
    //                            i.e. the driver is not reporting received power
    //   withoutRadiotap       -- no radiotap header at all
    uint64_t framesWithRadiotap = 0;
    uint64_t framesWithRadiotapNoSignal = 0;
    uint64_t framesWithoutRadiotap = 0;
    int lastRssiDbm = -100;
    std::vector<std::string> capabilities;
};

// A live per-transmitter RSSI history, the core signal the DSP works on.
struct RssiTrack {
    std::array<uint8_t, 6> mac{};
    RadioKind radio = RadioKind::Unknown;
    std::vector<double> samplesDbm;
    std::vector<double> timesS;
    std::string label;

    double mean() const;
    double variance() const;
    double stddev() const;
    double min() const;
    double max() const;
    // Mean absolute first difference: the cheapest honest motion proxy there is.
    double meanAbsDiff() const;
    size_t count() const { return samplesDbm.size(); }
};

std::string macToString(const std::array<uint8_t, 6>& mac);

// FNV-1a, used for stable colour assignment per MAC in the UI.
uint64_t macHash(const std::array<uint8_t, 6>& mac);

}  // namespace radar