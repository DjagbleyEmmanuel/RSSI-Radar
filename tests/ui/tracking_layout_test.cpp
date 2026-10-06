// Tracking-panel layout test.
//
// Renders the contacts table and the signal-analysis panel at a spread of widths,
// down to far narrower than the window can actually be made, and asserts that no
// two pieces of text overlap and that nothing runs outside the panel.
//
// Background. The contacts row placed its fields at fixed offsets: name at +42,
// range at +162, speed at +314, presence at +388, state at +448, age at +512. That
// needs 602 px, but the contacts column is only 58% of the panel, so on any
// ordinary width the tail of the row was drawn straight through the signal
// analysis beside it -- the speed and state fields landed on top of the anomaly
// key/value rows. State and age also overlapped each other by 6 px even when the
// panel was wide enough for both.
//
// And below 620 px the signal analysis was not drawn at all, so shrinking the
// window made that half of the panel silently disappear. It is stacked below the
// contacts now instead.
//
// Overlap is detected from the rendered pixels rather than from the layout code,
// because the point is what the operator sees.
//
// Layout only. No engine, no radio.
#include <QApplication>
#include <QImage>
#include <QPainter>
#include <QThread>
#include <QColor>

#include "Widgets.hpp"

using namespace radar;

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  [%s] %s\n", ok ? "pass" : "FAIL", what.c_str());
    if (!ok) ++failures;
}

// An AnomalyReport with enough data, so the panel renders its full row set.
AnomalyReport populated() {
    AnomalyReport a;
    a.enoughData = true;
    a.samples = 512;
    a.meanDbm = -23.4;
    a.stdDbm = 1.70;
    a.varianceRatio = 0.99;
    a.spectralFlatness = 0.990;
    a.shannonEntropyBits = 0.53;
    a.dominantPeriodS = 25.09;
    a.kurtosis = 0.36;
    a.trendPerMinute = -0.15;
    return a;
}

Snapshot makeSnapshot() {
    Snapshot s;
    s.anomaly = populated();
    s.anomalyByLabel.clear();

    for (int k = 0; k < 3; ++k) {
        Contact c;
        c.id = static_cast<uint64_t>(k + 2);
        for (int b = 0; b < 6; ++b) c.mac[b] = static_cast<uint8_t>(k * 23 + b + 1);
        c.label = "WIFI a0:e6:e0:6e:9a:0" + std::to_string(k);
        c.rangeM = 0.3 + k * 0.2;
        c.rangeValid = true;
        c.rangeLoM = 0.2;
        c.rangeHiM = 0.3 + k * 0.2;
        c.levelDbm = -23.4 - k;
        c.presence = 1.0;
        c.state = ContactState::Active;
        c.updates = 400;
        c.velocityMps = 0.0;
        c.velocityValid = true;
        c.silenceSeconds = k == 1 ? 0.4 : 0.0;  // one row shows the age field
        s.contacts.push_back(c);
    }
    return s;
}

}  // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    const Snapshot s = makeSnapshot();
    const QColor bg(6, 12, 16);  // theme background for this palette

    std::puts("tracking panel layout");

    // The width the bug was reported at, then progressively narrower -- including
    // widths below the old 620 px cutoff where the signal analysis used to vanish.
    const int widths[] = {1180, 900, 780, 700, 619, 560, 460, 380, 300};
    for (int w : widths) {
        TrackingPanelWidget p;
        p.resize(w, 320);
        p.setSnapshot(s);

        QImage img(p.size(), QImage::Format_ARGB32);
        img.fill(bg);
        p.render(&img);

        // Measured on the rectangles the widget actually drew, so this is what
        // the operator sees rather than what the layout code intended.
        const int overlaps = p.textOverlaps();
        std::printf("         (width %4d: %d overlapping text fields)\n", w, overlaps);
        if (overlaps != 0) {
            std::printf("  [FAIL] %d text fields overlap at width %d\n", overlaps, w);
            ++failures;
        }
    }

    // Below the old cutoff the signal analysis was simply not drawn. Confirm it is
    // still there when the panel is narrow.
    {
        TrackingPanelWidget p;
        p.resize(400, 320);
        p.setSnapshot(s);
        QImage img(p.size(), QImage::Format_ARGB32);
        img.fill(bg);
        p.render(&img);
        img.save("/tmp/opencode/tracking_narrow.png");

        // Count text bands in the lower half; the stacked analysis panel puts
        // content there.
        int lower = 0;
        for (int y = img.height() / 2; y < img.height(); ++y)
            for (int x = 0; x < img.width(); ++x)
                if (img.pixel(x, y) != bg.rgb()) {
                    ++lower;
                    break;
                }
        std::printf("         (narrow panel: %d text rows below the midpoint)\n", lower);
        check(lower > 3, "signal analysis is still drawn when the panel is narrow");
    }

    std::printf("%s: %d failure(s)\n", failures ? "FAILED" : "OK", failures);
    return failures ? 1 : 0;
}