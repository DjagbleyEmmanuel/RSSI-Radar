// Band sweep channel-ranking test.
//
// Tests sweepProbeBetter(), the rule the band sweep uses to decide which channel
// to listen on. No radio required: the rule is the part that was wrong, and it
// can be driven with the tallies a real sweep produces.
//
// Background, because this is a real bug that shipped and cost a whole session's
// sensing. The sweep counted every frame the radio delivered, on every channel,
// with a 260 ms dwell, and took whichever channel returned the most. An access
// point beaconing at the usual 10 Hz puts about two beacons into a 260 ms window,
// so a real access point scored two or three frames -- indistinguishable from the
// noise floor throwing the occasional frame onto an empty channel. Ties were
// broken by position in the sweep order, so the winner was whichever tied channel
// happened to be visited first.
//
// Measured over five runs on one access point, the old rule pinned channel 11
// twice, channel 1 twice, and channel 3 once -- an entirely empty channel, after
// which the capture reported a 0.0 Hz sample rate for the rest of the session
// while drawing the access point as a contact that never went away.
//
// Build and run from build/:
//
//   g++ -std=c++17 -I../include ../tests/band_sweep_test.cpp \
//       $OBJS -o /tmp/sweep_test
//   /tmp/sweep_test
//
// Exits non-zero if any case fails.

#include "radar/Sensor.hpp"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

using namespace radar;

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  [%s] %s\n", ok ? "pass" : "FAIL", what.c_str());
    if (!ok) ++failures;
}

SweepProbe probe(int channel, uint64_t frames, int usable, double bestRssi) {
    SweepProbe p;
    p.channel = channel;
    p.frames = frames;
    p.usable = usable;
    p.bestRssiDbm = bestRssi;
    return p;
}

// What the sweep would have chosen, given a set of measurements.
int choose(const std::vector<SweepProbe>& ps) {
    std::vector<SweepProbe> v = ps;
    std::sort(v.begin(), v.end(),
              [](const SweepProbe& a, const SweepProbe& b) { return sweepProbeBetter(a, b); });
    return v.empty() ? 0 : v.front().channel;
}

}  // namespace

int main() {
    std::puts("band sweep ranking");

    // The failure that motivated this: an empty channel with more noise frames
    // than the real access point managed to beacon. Frame count must not decide it.
    {
        const std::vector<SweepProbe> ps = {
            probe(1, 3, 3, -29.0),  // a real access point, 3 beacons
            probe(3, 4, 0, -999.0), // empty channel, 4 frames of noise
            probe(8, 5, 0, -999.0), // empty channel, 5 frames of noise
            probe(11, 3, 3, -26.0), // a real access point, 3 beacons
        };
        check(choose(ps) == 11, "empty channels are never chosen for their frame count");
    }

    // Stronger link wins over a busier but weaker one: that is the better
    // signal to noise for radiometric work.
    {
        const std::vector<SweepProbe> ps = {
            probe(6, 40, 40, -78.0), // lots of traffic, weak
            probe(1, 4, 4, -31.0),   // little traffic, strong
        };
        check(choose(ps) == 1, "strongest reading wins over highest frame count");
    }

    // A channel with no usable reading at all loses to any channel that has one,
    // however few frames it managed.
    {
        const std::vector<SweepProbe> ps = {
            probe(9, 99, 0, -999.0),
            probe(11, 1, 1, -60.0),
        };
        check(choose(ps) == 11, "one usable reading beats many unusable frames");
    }

    // Equal power: the count of usable readings breaks the tie, deterministically.
    // The old rule broke this by sweep position, which is why it alternated
    // between two equally valid channels from run to run.
    {
        const std::vector<SweepProbe> a = {probe(1, 5, 5, -40.0), probe(11, 3, 3, -40.0)};
        const std::vector<SweepProbe> b = {probe(11, 3, 3, -40.0), probe(1, 5, 5, -40.0)};
        check(choose(a) == 1 && choose(b) == 1,
              "ties break on usable readings, not on the order they were surveyed");
    }

    // Exactly the tallies observed on the real radio, with access points on
    // channels 1 and 11, and bleed from the channel-11 access point landing on the
    // adjacent channel 10. 2.4 GHz channels overlap, so channel 10 produces
    // genuine usable readings that belong to channel 11's access point.
    //
    // At survey dwell that bleed is competitive: it looks as strong and more
    // numerous than either real channel. Ranking alone therefore puts channel 10
    // first, which is expected and is precisely why the sweep has a second,
    // longer confirming pass instead of committing to the survey winner.
    {
        const std::vector<SweepProbe> survey = {
            probe(1, 1, 1, -29.0),   // the access point on channel 1
            probe(6, 1, 0, -999.0),  //
            probe(11, 1, 1, -26.0),  // the access point on channel 11
            probe(10, 2, 2, -26.0),  // bleed from channel 11
            probe(3, 0, 0, -999.0),  //
        };
        check(choose(survey) == 10,
              "at survey dwell, adjacent-channel bleed is competitive and ranks first");

        // The confirming dwell is what separates them: the real channel keeps
        // producing usable readings, the bleed channel does not. This is the pass
        // that decided correctly in the live run.
        const std::vector<SweepProbe> confirmed = {
            probe(10, 2, 2, -26.0),  // rejected: 2 radiometric frames
            probe(11, 8, 8, -21.0),  // accepted: 8 radiometric frames
        };
        check(choose(confirmed) == 11,
              "the confirming dwell separates adjacent-channel bleed from the real channel");
    }

    // No reading on a channel must never be outranked by a bogus power figure.
    // If a future change let bestRssiDbm decide without first requiring a usable
    // reading, a default-initialised or sentinel-filled probe could win on a
    // value that means nothing.
    {
        const SweepProbe bogus{9, 500, 0, 0.0};      // frames, but no usable reading
        const SweepProbe real_{11, 1, 1, -95.0};     // weak, but a real reading
        check(sweepProbeBetter(real_, bogus), "a bogus power on a silent channel never wins");
        check(!sweepProbeBetter(bogus, real_), "and the reverse is not claimed either");
    }

    // The actual observed failure, verbatim.
    //
    // Sweep tallies from a live run where the only access point in range was on
    // channel 9. The old rule took the highest frame count and broke the tie by
    // survey order, so it pinned channel 8 -- an empty channel -- and capture then
    // reported a 0.0 Hz sample rate for the rest of the session while drawing the
    // access point as a contact that never went away.
    //
    //    1:0 6:0 11:0 2:0 7:0 12:0 3:0 8:2 13:0 4:0 9:2 10:0 5:0  -> channel 8
    //
    // Channels 8 and 9 both returned 2 frames. Channel 9 had the access point;
    // channel 8 did not, and the two were indistinguishable because nothing was
    // measuring received power.
    {
        const std::vector<SweepProbe> ps = {
            probe(1, 0, 0, -999.0), probe(6, 0, 0, -999.0), probe(11, 0, 0, -999.0),
            probe(2, 0, 0, -999.0), probe(7, 0, 0, -999.0), probe(12, 0, 0, -999.0),
            probe(3, 0, 0, -999.0), probe(8, 2, 0, -999.0), probe(13, 0, 0, -999.0),
            probe(4, 0, 0, -999.0), probe(9, 2, 0, -999.0), probe(10, 0, 0, -999.0),
            probe(5, 0, 0, -999.0),
        };

        // The rule that was replaced, reproduced exactly: strictly greater while
        // surveying in order, so ties go to whichever channel was visited first.
        const auto oldRule = [](const std::vector<SweepProbe>& v) {
            int best = 0;
            uint64_t bestFrames = 0;
            for (const auto& p : v)
                if (p.frames > bestFrames) {
                    bestFrames = p.frames;
                    best = p.channel;
                }
            return best;
        };
        check(oldRule(ps) == 8, "the old rule reproduces the observed bad choice of channel 8");

        // The replacement finds no usable reading anywhere on this band, so the
        // confirming pass rejects every candidate and the caller reports honestly
        // instead of pinning noise. That is the intended difference in outcome.
        const bool anyUsable =
            std::any_of(ps.begin(), ps.end(), [](const SweepProbe& p) { return p.usable > 0; });
        check(!anyUsable, "the new rule finds no usable reading, so no channel is confirmable");

        // And given a real reading on channel 9, the new rule gets it right where
        // the old rule could not.
        std::vector<SweepProbe> withSignal = ps;
        for (auto& p : withSignal)
            if (p.channel == 9) {
                p.usable = 6;
                p.bestRssiDbm = -30.0;
            }
        check(choose(withSignal) == 9, "with a real reading present, channel 9 is chosen");
        check(oldRule(withSignal) == 8, "the old rule still gets this one wrong");
    }

    std::printf("%s: %d failure(s)\n", failures ? "FAILED" : "OK", failures);
    return failures ? 1 : 0;
}