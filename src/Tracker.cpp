#include "radar/Tracker.hpp"

#include <algorithm>
#include <cmath>

namespace radar {

const char* toString(ContactState s) {
    switch (s) {
        case ContactState::Active: return "ACTIVE";
        case ContactState::Fading: return "FADING";
        case ContactState::Lost: return "LOST";
    }
    return "UNKNOWN";
}

void ContactTracker::reset() {
    live_.clear();
    order_.clear();
    vanished_.clear();
    velState_.clear();
    lastLevel_.clear();
    nextId_ = 1;
}

std::vector<uint64_t> ContactTracker::takeVanished() {
    std::vector<uint64_t> out;
    out.swap(vanished_);
    return out;
}

std::vector<Contact> ContactTracker::contacts() const {
    // Recency order, so the UI's primary contact is simply the first element
    // and no caller has to re-sort.
    std::vector<Contact> out;
    out.reserve(order_.size());
    for (uint64_t key : order_) {
        const auto it = live_.find(key);
        if (it != live_.end()) out.push_back(it->second);
    }
    return out;
}


void ContactTracker::update(const std::vector<RssiTrack>& tracks, double now, double wall,
                            const PathLossModel& pl,
                            const std::map<uint64_t, double>* bearings) {
    vanished_.clear();

    // Mark everything as not-yet-heard this pass; anything still silent at the
    // end of the pass has genuinely gone quiet.
    for (auto& kv : live_)
        kv.second.framesMissed++;

    for (const auto& t : tracks) {
        const uint64_t key = macHash(t.mac);
        if (t.count() == 0) continue;

        // Reject a transmitter that has only been seen once or twice: a single
        // frame produces a range with no error bar at all.
        const double mean = t.mean();
        const double sd = t.stddev();

        auto it = live_.find(key);
        if (it == live_.end()) {
            Contact c;
            c.id = nextId_++;
            c.mac = t.mac;
            c.radio = t.radio;
            c.label = t.label.empty() ? macToString(t.mac) : t.label;
            c.firstSeen = wall;
            c.levelDbm = mean;
            c.peakDbm = mean;
            c.troughDbm = mean;
            c.levelSigmaDb = sd;
            c.updates = 1;
            c.framesMissed = 0;
            c.presence = 0.5;
            it = live_.emplace(key, c).first;
        }

        Contact& c = it->second;
        c.framesMissed = 0;
        c.lastSeen = wall;
        c.lastHeard = now;
        c.updates++;
        c.dwellSeconds = std::max(0.0, wall - c.firstSeen);

        // Alpha-tracked level. Using the instantaneous mean rather than the most
        // recent sample keeps one deep fade from dominating the estimate.
        const double prev = c.levelDbm;
        c.levelDbm = prev + cfg_.levelAlpha * (mean - prev);
        // Slow max/min so a contact's dynamic range is visible over its life.
        c.peakDbm = std::max(c.peakDbm, mean);
        c.troughDbm = std::min(c.troughDbm, mean);
        // Running spread of the level, a decent proxy for link instability.
        c.levelSigmaDb = std::sqrt(0.9 * c.levelSigmaDb * c.levelSigmaDb + 0.1 * sd * sd);

        // --- range, with the interval shadowing actually implies.
        if (t.count() >= cfg_.minSamplesForRange) {
            c.rangeM = pl.rangeFromRssi(c.levelDbm);
            // A dB error maps to a range ratio of 10^(dB/(10n)). With n around
            // 2.8 that is a factor of ~1.3 per dB, which is why the interval is
            // wide and is reported rather than hidden.
            const double k = std::pow(10.0, pl.shadowSigmaDb / (10.0 * std::max(0.5, pl.exponent)));
            c.rangeLoM = c.rangeM / k;
            c.rangeHiM = c.rangeM * k;
            c.rangeValid = true;
        }

        // --- radial speed from the filtered level derivative.
        // d/dt of RSSI = -10n/ln(10) * (1/d) * d', so d' = -d * (dRSSI/dt) * ln10/(10n).
        // Enormously noisy, so it is low-passed hard and only trusted once the
        // level itself is stable.
        const auto prevLevel = lastLevel_.find(key);
        if (prevLevel != lastLevel_.end() && t.timesS.size() >= 2) {
            const double dt = t.timesS.back() - t.timesS[t.timesS.size() - 2];
            if (dt > 1e-3) {
                const double slope = (c.levelDbm - prevLevel->second) / dt;
                auto vs = velState_.find(key);
                if (vs == velState_.end())
                    vs = velState_.emplace(key, slope).first;
                else
                    vs->second += 0.12 * (slope - vs->second);
                const double d = std::max(0.3, c.rangeM);
                c.velocityMps = -d * vs->second * 2.302585 / (10.0 * std::max(0.5, pl.exponent));
                // Only claim a velocity when the level is quiet enough for the
                // derivative to mean anything.
                c.velocityValid = c.levelSigmaDb < 4.0 && c.updates > 8;
            }
        }
        lastLevel_[key] = c.levelDbm;

        if (bearings) {
            const auto b = bearings->find(key);
            if (b != bearings->end()) {
                c.bearingDeg = b->second;
                c.bearingValid = true;
            } else {
                c.bearingValid = false;
            }
        }

        // Confidence from evidence: more frames is better, a quiet link is
        // better, and a settled range beats a swinging one.
        const double n = std::min(1.0, static_cast<double>(c.updates) / 40.0);
        const double stab = std::exp(-c.levelSigmaDb / 5.0);
        c.confidence = std::clamp(0.25 * n + 0.45 * stab + (c.rangeValid ? 0.30 : 0.0), 0.0, 1.0);
        c.presence = 1.0;
        c.state = ContactState::Active;
    }

    // --- decay and expire.
    std::vector<uint64_t> drop;
    for (auto& kv : live_) {
        Contact& c = kv.second;
        if (c.framesMissed == 0) continue;
        c.silenceSeconds = std::max(0.0, now - c.lastHeard);
        const double hold = std::max(0.5, cfg_.holdSeconds);

        if (c.silenceSeconds >= hold) {
            drop.push_back(kv.first);
            continue;
        }
        // Linear decay over the hold window, so the fade is predictable rather
        // than an abrupt disappearance.
        c.presence = std::clamp(1.0 - c.silenceSeconds / hold, 0.0, 1.0);
        c.state = c.presence > 0.35 ? ContactState::Fading : ContactState::Lost;
    }

    for (uint64_t key : drop) {
        vanished_.push_back(live_[key].id);
        live_.erase(key);
        velState_.erase(key);
        lastLevel_.erase(key);
    }

    // --- recency ordering, most recently heard first.
    order_.clear();
    order_.reserve(live_.size());
    for (const auto& kv : live_) order_.push_back(kv.first);
    std::sort(order_.begin(), order_.end(), [&](uint64_t a, uint64_t b) {
        return live_.at(a).lastSeen > live_.at(b).lastSeen;
    });

    // --- bound the population.
    if (live_.size() > cfg_.maxContacts) {
        // Drop the least recently heard, which are also the least useful.
        const size_t excess = live_.size() - cfg_.maxContacts;
        for (size_t i = 0; i < excess && i < order_.size(); ++i) {
            const uint64_t key = order_.back();
            order_.pop_back();
            vanished_.push_back(live_[key].id);
            live_.erase(key);
            velState_.erase(key);
            lastLevel_.erase(key);
        }
    }
}

}  // namespace radar
