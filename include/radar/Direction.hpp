// Direction from multiple access points.
//
// With one receiver, bearing cannot be measured -- there is no phase and no
// baseline, and no amount of signal processing substitutes for either. What a
// multi-transmitter environment does give is something weaker and still useful:
// *which* transmitters the change is moving toward and which away from.
//
// Each access point in range is effectively a sensor at a different physical
// location. If one AP's received power rises while another's falls, the body
// moved toward the first and away from the second. That is genuine directional
// information, derived from RSSI differences between independent sources.
//
// It is only available with two or more access points. With one, the estimator
// reports unavailable rather than producing a number, because a direction from a
// single receiver is not a weak measurement -- it is not a measurement at all.
//
// The result is deliberately qualitative. Turning "toward AP A, away from AP B"
// into a compass bearing requires those access points' positions to be surveyed
// and entered as anchors. Without that, any heading printed would be invented.
#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "radar/Types.hpp"

namespace radar {

struct DirectionReport {
    bool available = false;
    // Transmitter whose power rose most.
    std::string risenLabel;
    std::string fallenLabel;
    double risenDb = 0.0;
    double fallenDb = 0.0;
    // Difference between the two extremes: the evidence for any direction.
    double spreadDb = 0.0;
    double quality = 0.0;   // 0..1
    size_t contributors = 0;
    double rangeM = 0.0;
    // Never a compass bearing without surveyed anchor positions.
    double bearingDeg = 0.0;
    bool bearingValid = false;
    std::string reason;
    std::string caveat;
};

class DirectionEstimator {
  public:
    struct Config {
        // Alpha for the tracked level of each transmitter.
        double emaAlpha = 0.15;
        // How fast the baseline follows the current level, so it tracks slow
        // environmental drift but not a body moving through.
        double baselineAlpha = 0.002;
        size_t baselineFrames = 24;
        // Ignore changes smaller than this: thermal drift and rate adaptation.
        double minChangeDb = 1.2;
        double holdSeconds = 12.0;
    };

    void configure(const Config& c);
    void reset();

    void update(const std::vector<RssiTrack>& tracks, double wallTime);
    DirectionReport estimate(double rangeM, double bodyDbm) const;

    // How many independent transmitters are currently being tracked.
    size_t transmitterCount() const { return state_.size(); }

  private:
    struct State {
        std::string label;
        double ema = 0.0;
        double baseline = 0.0;
        bool baselineReady = false;
        double lastSeen = 0.0;
        uint64_t frames = 0;
        std::deque<double> baselineSamples;
    };
    struct Entry {
        std::string label;
        double level;
        double changeDb;
        double lastSeen;
    };

    Config cfg_;
    std::map<uint64_t, State> state_;
};

}  // namespace radar