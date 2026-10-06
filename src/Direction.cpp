#include "radar/Direction.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace radar {

void DirectionEstimator::configure(const Config& c) { cfg_ = c; }


void DirectionEstimator::reset() { state_.clear(); }

void DirectionEstimator::update(const std::vector<RssiTrack>& tracks, double wallTime) {
    // Each transmitter is a sensor at a different physical location. What
    // matters is not any one RSSI value but how each one *changed* relative to
    // the others: a body moving toward one access point and away from another is
    // what gives a direction.
    //
    // This is only possible with two or more independent transmitters. With one
    // there is nothing to compare against, and no amount of processing invents a
    // direction from a single receiver -- so the estimator reports unavailable
    // rather than guessing.
    for (const auto& t : tracks) {
        if (t.count() < 4) continue;
        const uint64_t key = macHash(t.mac);
        const double mean = t.mean();
        const double sigma = std::max(0.4, t.stddev());
        const double dBm = mean - 3.0 * sigma;

        auto it = state_.find(key);
        if (it == state_.end()) {
            State st;
            st.label = t.label.empty() ? macToString(t.mac) : t.label;
            st.ema = mean;
            st.baseline = mean;
            st.baselineReady = false;
            st.lastSeen = wallTime;
            it = state_.emplace(key, st).first;
        }
        State& st = it->second;
        st.ema += cfg_.emaAlpha * (mean - st.ema);
        st.lastSeen = wallTime;
        st.frames++;

        if (!st.baselineReady) {
            st.baselineSamples.push_back(mean);
            if (st.baselineSamples.size() >= static_cast<size_t>(cfg_.baselineFrames)) {
                st.baseline = std::accumulate(st.baselineSamples.begin(),
                                              st.baselineSamples.end(), 0.0) /
                              static_cast<double>(st.baselineSamples.size());
                st.baselineReady = true;
            }
        } else {
            // Walk the baseline slowly toward the current level so it tracks slow
            // environmental drift but not a body moving through.
            st.baseline += cfg_.baselineAlpha * (mean - st.baseline);
        }
    }

    // Expire transmitters that have gone quiet, otherwise a stale entry keeps
    // contributing a "change" forever.
    for (auto it = state_.begin(); it != state_.end();) {
        if (wallTime - it->second.lastSeen > cfg_.holdSeconds)
            it = state_.erase(it);
        else
            ++it;
    }
}

DirectionReport DirectionEstimator::estimate(double rangeM, double bodyDbm) const {
    DirectionReport rep;
    rep.rangeM = rangeM;

    if (state_.size() < 2) {
        rep.available = false;
        rep.reason = state_.empty() ? "no transmitters heard yet"
                                    : "only one transmitter: nothing to compare it against";
        return rep;
    }

    // Each transmitter's change relative to its own baseline, expressed in dB and
    // then normalised by distance so a far but strong change is not over-weighted.
    std::vector<Entry> es;
    es.reserve(state_.size());
    for (const auto& kv : state_) {
        const State& st = kv.second;
        if (!st.baselineReady) continue;
        const double change = st.ema - st.baseline;
        // Only a change worth mentioning: below this it is thermal and rate
        // adaptation noise.
        if (std::fabs(change) < cfg_.minChangeDb) continue;
        es.push_back({st.label, st.ema, change, st.lastSeen});
    }
    if (es.empty()) {
        rep.available = false;
        rep.reason = "no transmitter has changed enough to give a direction";
        return rep;
    }

    // Identify the most-risen and most-fallen transmitters. The direction of
    // travel points from the fallen one toward the risen one.
    const auto byChange = [&es](const Entry& a, const Entry& b) { return a.changeDb > b.changeDb; };
    std::sort(es.begin(), es.end(), byChange);
    const Entry& up = es.front();
    const Entry& down = es.back();

    rep.available = true;
    rep.risenLabel = up.label;
    rep.risenDb = up.changeDb;
    rep.fallenLabel = down.label;
    rep.fallenDb = down.changeDb;
    rep.bearingDeg = 0.0;  // the true bearing needs anchor geometry; see caveat
    rep.bearingValid = false;

    // Quality: how separated the two extremes are, and how many transmitters
    // agree. One transmitter moving on its own is much weaker evidence than four
    // all changing consistently.
    const double spread = std::fabs(up.changeDb - down.changeDb);
    const double spreadScore = std::clamp(spread / std::max(1e-6, cfg_.minChangeDb * 4.0), 0.0, 1.0);
    const double agree = static_cast<double>(es.size()) /
                         static_cast<double>(std::max<size_t>(1, state_.size()));
    rep.quality = std::clamp(0.65 * spreadScore + 0.35 * agree, 0.0, 1.0);
    rep.spreadDb = spread;
    rep.contributors = es.size();

    if (es.size() < 2) {
        rep.reason = "only one transmitter changed; not enough for a direction";
    } else {
        rep.reason = "direction relative to " + up.label + " and " + down.label;
    }

    // Bearing is deliberately not produced. Which compass direction "toward
    // AP A and away from AP B" corresponds to requires the anchor positions to be
    // known and surveyed. Without them any number printed would be an invention,
    // so the qualitative finding is what gets reported.
    rep.caveat =
        "This says which access points the change is toward, not a compass bearing. "
        "A bearing needs the access point positions surveyed and entered as anchors; "
        "with one access point there is no direction information at all.";
    return rep;
}

}  // namespace radar