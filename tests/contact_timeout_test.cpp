// Regression test: a contact must stop being plotted once its transmitter goes
// quiet, and must come back when the transmitter resumes.
//
// This is a real bug that shipped, not a hypothetical. RssiTrack::count() is
// samplesDbm.size(), which is cumulative and is never pruned, so a track that
// has gone silent still reports the sample count it died on. The tracker used
// "count() > 0" as its test for "this transmitter was just heard", which is true
// forever for every track ever seen. That reset the silence counter on every
// tick, so the decay and expiry paths were unreachable: contacts never faded,
// never expired, and each sat on the scope forever holding the range it had
// when it was last alive. A band that went quiet filled up with stale blips
// instead of emptying, and the AP count grew without bound.

#include "radar/Tracker.hpp"

#include <cstdio>
#include <vector>

using namespace radar;

namespace {

// RssiTrack's accessors and the MAC helpers live in the sensor translation unit,
// which pulls in Qt, so they are provided here directly. Only what ContactTracker
// calls is implemented.

int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  [%s] %s\n", ok ? "pass" : "FAIL", what.c_str());
    if (!ok) ++failures;
}

}  // namespace

namespace radar {

uint64_t macHash(const std::array<uint8_t, 6>& m) {
    uint64_t h = 1469598103934665603ULL;
    for (uint8_t b : m) {
        h ^= b;
        h *= 1099511628211ULL;
    }
    return h;
}

std::string macToString(const std::array<uint8_t, 6>& m) {
    char b[32];
    std::snprintf(b, sizeof b, "%02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3],
                  m[4], m[5]);
    return b;
}

double RssiTrack::mean() const {
    double s = 0;
    for (double v : samplesDbm) s += v;
    return samplesDbm.empty() ? 0.0 : s / static_cast<double>(samplesDbm.size());
}
double RssiTrack::variance() const {
    const double m = mean();
    double s = 0;
    for (double v : samplesDbm) s += (v - m) * (v - m);
    return samplesDbm.size() < 2 ? 0.0 : s / static_cast<double>(samplesDbm.size());
}
double RssiTrack::stddev() const {
    return variance() <= 0 ? 0.0 : std::sqrt(variance());
}

}  // namespace radar

namespace {

RssiTrack makeTrack() {
    RssiTrack t;
    t.mac = {0xa0, 0xe6, 0xe0, 0x6e, 0x9a, 0x02};
    t.radio = RadioKind::Wifi;
    return t;
}

}  // namespace

int main() {
    TrackerConfig cfg;
    cfg.holdSeconds = 1.0;  // the "Fast Intrusion" preset value
    cfg.minSamplesForRange = 5;
    cfg.maxContacts = 24;

    ContactTracker tr;
    tr.configure(cfg);

    const PathLossModel pl;
    RssiTrack t = makeTrack();
    double now = 0.0;
    int step = 0;

    const auto tick = [&] {
        tr.update({t}, now, now, pl, nullptr);
        now += 0.1;
        ++step;
    };
    const auto heard = [&] {
        t.samplesDbm.push_back(-50.0 + (step % 3) * 0.5);
        t.timesS.push_back(now);
    };

    std::puts("contact timeout");

    // Jitter must not be mistaken for disappearance.
    //
    // At ~20 Hz frames against a comparable engine tick rate, most ticks carry
    // no new sample. Counting each of those as the start of a disappearance made
    // a continuously-heard strong AP flip between ACTIVE and FADING many times a
    // second, which pulsed the ageing halo and changed the label's width on
    // every flip, so a single blip at the origin read as a fast flicker.
    {
        ContactTracker j;
        j.configure(cfg);
        RssiTrack jt = makeTrack();
        double jn = 0.0;
        int flickers = 0;
        ContactState prev = ContactState::Active;
        // 20 Hz frames, 40 Hz ticks: every other tick sees nothing new.
        for (int i = 0; i < 400; ++i) {
            if (i % 2 == 0) {
                jt.samplesDbm.push_back(-24.0 + (i % 5) * 0.4);
                jt.timesS.push_back(jn);
            }
            j.update({jt}, jn, jn, pl, nullptr);
            const auto cs = j.contacts();
            if (!cs.empty()) {
                if (cs[0].state != prev) ++flickers;
                prev = cs[0].state;
            }
            jn += 0.025;  // 40 Hz tick
        }
        check(j.contacts().size() == 1, "contact survives 20 s of frame jitter");
        check(flickers == 0,
              "state stays steady through jitter (no flicker); saw " + std::to_string(flickers) +
                  " transitions");
        check(!j.contacts().empty() && j.contacts()[0].presence == 1.0,
              "presence stays 1.0 while frames keep arriving");
    }

    // Being heard every tick: one contact, and it must stay ACTIVE.
    for (int i = 0; i < 20; ++i) {
        heard();
        tick();
    }
    check(tr.contacts().size() == 1, "exactly one contact while the transmitter is live");
    check(!tr.contacts().empty() && tr.contacts()[0].state == ContactState::Active,
          "contact stays ACTIVE while frames keep arriving");

    // Now it goes silent. No further samples are added -- exactly what happens
    // when the band goes quiet, or the pinned channel changes, or the AP moves.
    const int before = step;
    for (int i = 0; i < 400; ++i) tick();
    const double silentFor = (step - before) * 0.1;
    check(tr.contacts().empty(),
          "contact expires once the transmitter is silent past the hold window");
    std::printf("         (silent for %.1f s with holdSeconds=%.1f)\n", silentFor,
                cfg.holdSeconds);

    // And it must come back when the transmitter resumes, rather than being
    // permanently discarded because it once went quiet.
    for (int i = 0; i < 20; ++i) {
        heard();
        tick();
    }
    check(tr.contacts().size() == 1, "contact returns when the transmitter resumes");
    check(!tr.contacts().empty() && tr.contacts()[0].state == ContactState::Active,
          "reappearing contact is ACTIVE again");

    // A burst of transmitters that all fall silent must not leave debris behind.
    {
        std::vector<RssiTrack> burst;
        for (int k = 0; k < 12; ++k) {
            RssiTrack b = makeTrack();
            b.mac[5] = static_cast<uint8_t>(0x20 + k);
            for (int i = 0; i < 10; ++i) {
                b.samplesDbm.push_back(-60.0 + k);
                b.timesS.push_back(now + i * 0.1);
            }
            burst.push_back(b);
        }
        tr.update(burst, now, now, pl, nullptr);
        const int peak = static_cast<int>(tr.contacts().size());
        for (int i = 0; i < 400; ++i) {
            now += 0.1;
            tr.update(burst, now, now, pl, nullptr);
        }
        std::printf("         (burst of %d transmitters)\n", peak);
        check(tr.contacts().empty(), "a burst of silent transmitters leaves no contacts");
    }

    std::printf("%s: %d failure(s)\n", failures ? "FAILED" : "OK", failures);
    return failures ? 1 : 0;
}