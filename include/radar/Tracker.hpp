// Multi-target tracking over received power.
//
// A "track" in this program is a per-transmitter RSSI history. A "contact" is
// something an operator can point at: a stable identity, how long it has been
// present, where it is (as far as RSSI alone honestly allows), and whether it
// has just stopped being heard.
//
// The distinction matters because RSSI is a very weak observable. Inverting the
// path-loss model gives a range estimate whose uncertainty is dominated by
// shadowing, not by measurement noise -- a 3 dB shadowing sigma at 20 m is
// roughly a factor of two in range. So every contact carries an explicit range
// *interval* rather than a single confident number, and bearing is only ever
// populated when the fusion stage genuinely resolved it from anchor geometry.
// Nothing here invents a position that the physics did not supply.
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "radar/Dsp.hpp"
#include "radar/Types.hpp"

namespace radar {

enum class ContactState {
    Active,   // heard within the last update interval
    Fading,   // not heard recently, presence decaying, still drawn
    Lost      // past the hold time, about to be dropped
};

const char* toString(ContactState s);

struct Contact {
    uint64_t id = 0;                  // stable for the life of the contact
    std::array<uint8_t, 6> mac{};
    RadioKind radio = RadioKind::Unknown;
    std::string label;

    // Timing, all in seconds.
    double firstSeen = 0.0;
    double lastSeen = 0.0;            // wall clock of the most recent frame
    double lastHeard = 0.0;           // monotonic, for the timeout
    double dwellSeconds = 0.0;
    double silenceSeconds = 0.0;      // how long since it was last heard
    uint64_t updates = 0;
    uint64_t framesMissed = 0;

    // Level, alpha-tracked so a single deep fade cannot erase the estimate.
    double levelDbm = -100.0;
    double levelSigmaDb = 0.0;
    double peakDbm = -100.0;
    double troughDbm = -100.0;

    // Range from the inverted path-loss model, with the interval that the
    // shadowing sigma actually implies.
    double rangeM = 0.0;
    double rangeLoM = 0.0;
    double rangeHiM = 0.0;
    bool rangeValid = false;

    // Radial speed from the filtered level derivative. Noisy by nature, so it
    // is heavily low-passed and flagged low confidence.
    double velocityMps = 0.0;
    bool velocityValid = false;

    double bearingDeg = 0.0;
    bool bearingValid = false;

    // 0..1 presence, decays once the contact stops being heard.
    double presence = 0.0;
    // 0..1 quality of the estimate, from sample count and stability.
    double confidence = 0.0;
    ContactState state = ContactState::Active;
};

struct TrackerConfig {
    // How long a contact is kept alive after its last frame. Past this it is
    // dropped, which is what produces a clean "disappeared" event rather than an
    // indefinite smear of ghosts.
    double holdSeconds = 6.0;
    // Alpha for the level tracker. Low enough to reject a single-frame fade.
    double levelAlpha = 0.18;
    // Cutoff for the velocity low-pass, Hz.
    double velocityCutoffHz = 0.15;
    // Contacts need this many frames before a range is quoted at all.
    size_t minSamplesForRange = 5;
    // Cap on retained contacts, oldest-and-silentest dropped first.
    size_t maxContacts = 24;
};

class ContactTracker {
  public:
    void configure(const TrackerConfig& c) { cfg_ = c; }
    const TrackerConfig& config() const { return cfg_; }
    void reset();

    // Fold the current per-transmitter histories into contacts.
    //   tracks    live RSSI histories, one per transmitter
    //   now       monotonic seconds, used only for timeouts
    //   wall      wall-clock seconds, used for display timestamps
    //   pl        calibrated path-loss model for the range inversion
    //   bearings  optional resolved bearings, keyed by mac hash
    void update(const std::vector<RssiTrack>& tracks, double now, double wall,
                const PathLossModel& pl,
                const std::map<uint64_t, double>* bearings = nullptr);

    // Contacts that have just been dropped for good, so the caller can raise an
    // event. Cleared by each update.
    std::vector<uint64_t> takeVanished();

    std::vector<Contact> contacts() const;
    size_t size() const { return order_.size(); }
    // Total contacts created this session, including dropped ones.
    uint64_t created() const { return nextId_; }

  private:
    TrackerConfig cfg_;
    std::map<uint64_t, Contact> live_;   // keyed by macHash
    std::vector<uint64_t> order_;        // most recently heard first
    std::vector<uint64_t> vanished_;
    uint64_t nextId_ = 1;

    // Per-contact velocity filter state.
    std::map<uint64_t, double> velState_;
    std::map<uint64_t, double> lastLevel_;
};

}  // namespace radar
