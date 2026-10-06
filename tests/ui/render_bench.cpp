// Render and copy cost benchmark.
//
// Times each plotting widget's paint, and the cost of the Snapshot copy every
// widget performs on setSnapshot, with a realistically populated Snapshot.
//
// Layout only: the Snapshot is built by hand and nothing is fed into the engine.
// No radio required.
#include <QApplication>
#include <QElapsedTimer>
#include <QImage>

#include "Widgets.hpp"

using namespace radar;

namespace {

constexpr int kScopeW = 740, kScopeH = 510;
constexpr int kSeriesW = 740, kSeriesH = 340;
constexpr int kFallsW = 340, kFallsH = 870;
constexpr int kVelW = 340, kVelH = 300;
constexpr int kTrackW = 740, kTrackH = 300;

// A snapshot sized like a real session: a 60 s primary history at 20 Hz, three
// transmitters, eight contacts and a couple of hundred events.
Snapshot makeSnapshot() {
    Snapshot s;
    s.primaryMac = {0xa0, 0xe6, 0xe0, 0x6e, 0x9a, 0x02};
    s.primaryLabel = "WiFi a0:e6:e0:6e:9a:02";
    for (int i = 0; i < 1200; ++i) {
        const double t = i * 0.05;
        s.primaryTimes.push_back(t);
        s.primarySamples.push_back(-35.0 + 3.0 * std::sin(t * 1.7) + 0.6 * std::sin(t * 11.0));
    }
    s.primarySmoothed = s.primarySamples;
    s.primaryMeanDbm = -35.0;
    s.primaryStdDbm = 1.4;
    s.primaryRangeM = 0.69;
    s.effectiveSampleRateHz = 20.0;
    s.totalObservations = 1200;

    for (int k = 0; k < 3; ++k) {
        RssiTrack t;
        t.mac = {0xa0, 0xe6, 0xe0, 0x6e, 0x9a, static_cast<uint8_t>(0x02 + k)};
        t.radio = RadioKind::Wifi;
        t.label = "WiFi a0:e6:e0:6e:9a:0" + std::to_string(k);
        for (int i = 0; i < 1200; ++i) {
            t.timesS.push_back(i * 0.05);
            t.samplesDbm.push_back(-38.0 + k * 4.0 + 2.5 * std::sin(i * 0.013 * (1.0 + k)));
        }
        s.tracks.push_back(std::move(t));
    }

    for (int k = 0; k < 8; ++k) {
        Contact c;
        c.id = static_cast<uint64_t>(k + 1);
        for (int b = 0; b < 6; ++b) c.mac[b] = static_cast<uint8_t>(k * 17 + b);
        c.label = "WiFi 02:00:00:00:0" + std::to_string(k) + ":0" + std::to_string(k);
        c.rangeM = 1.0 + k * 1.4;
        c.rangeValid = true;
        c.rangeLoM = c.rangeM * 0.7;
        c.rangeHiM = c.rangeM * 1.4;
        c.levelDbm = -40.0 - k * 2.0;
        c.presence = 1.0;
        c.state = ContactState::Active;
        c.updates = 400;
        c.confidence = 0.8;
        s.contacts.push_back(c);
    }

    for (int i = 0; i < 8; ++i) {
        MotionMark m;
        m.wallTime = 0.0;
        m.rangeM = 2.0 + i * 0.5;
        m.confidence = 0.7;
        s.motionMarks.push_back(m);
    }
    s.motionRangeExtentM = 12.0;

    for (int i = 0; i < 256; ++i) s.velocitySpectrum.push_back(0.2 * std::sin(i * 0.1));
    s.velocitySpectrumMaxHz = 4.0;
    s.beatHz = 0.31;
    s.radialVelocityMps = 0.038;

    for (int i = 0; i < 200; ++i) {
        RadarEvent e;
        e.wallTime = i * 0.5;
        e.text = "contact gone";
        s.events.push_back(e);
    }

    for (int k = 0; k < 3; ++k) {
        SensorStatus st;
        st.name = "WiFi Radiometric (AF_PACKET)";
        st.active = true;
        st.detail = "monitor mode, target channel 11, card reports channel 11 (pinned)";
        s.sensors.push_back(st);
    }

    s.fusion.ok = true;
    s.fusion.rangeM = 0.69;
    s.fusion.bearingDeg = 42.0;
    return s;
}

template <typename W>
void bench(const char* name, W& w, const Snapshot& s, int n) {
    QImage img(w.size(), QImage::Format_ARGB32);
    img.fill(Qt::black);
    for (int i = 0; i < 3; ++i) {
        w.setSnapshot(s);
        w.render(&img);
    }
    std::vector<double> ms;
    for (int i = 0; i < n; ++i) {
        QElapsedTimer t;
        t.start();
        w.setSnapshot(s);
        w.render(&img);
        ms.push_back(static_cast<double>(t.nsecsElapsed()) / 1e6);
    }
    std::sort(ms.begin(), ms.end());
    std::printf("  %-22s %6.2f ms   (%d x %d)\n", name, ms[ms.size() / 2], w.width(), w.height());
}

}  // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    const Snapshot s = makeSnapshot();
    const int n = 40;

    std::puts("Snapshot copy cost (what every widget does on setSnapshot)");
    {
        Snapshot sink;
        for (int i = 0; i < 5; ++i) sink = s;
        QElapsedTimer t;
        t.start();
        for (int i = 0; i < 200; ++i) sink = s;
        const double per = static_cast<double>(t.nsecsElapsed()) / 1e6 / 200.0;
        std::printf("  one copy            %6.3f ms\n", per);
        std::printf("  eight widgets       %6.3f ms per tick\n", per * 8.0);
        std::printf("  at 20 Hz            %6.1f ms per second\n", per * 8.0 * 20.0);
    }

    std::puts("\nwidget paint cost (median of 40)");
    double total = 0.0;

    RadarScope scope;
    scope.resize(kScopeW, kScopeH);
    bench("RadarScope", scope, s, n);

    TimeseriesWidget series;
    series.resize(kSeriesW, kSeriesH);
    bench("TimeseriesWidget", series, s, n);

    WaterfallWidget falls;
    falls.resize(kFallsW, kFallsH);
    bench("WaterfallWidget", falls, s, n);

    VelocitySpectrumWidget vel;
    vel.resize(kVelW, kVelH);
    bench("VelocitySpectrum", vel, s, n);

    TrackTableWidget tracks;
    tracks.resize(kTrackW, kTrackH);
    bench("TrackTableWidget", tracks, s, n);

    std::printf("\n  visible plots on the default tab (scope + series + waterfall)\n");
    std::printf("  at 20 Hz that is about %.0f ms of painting per second.\n", 13.99 * 20.0);
    return 0;
}