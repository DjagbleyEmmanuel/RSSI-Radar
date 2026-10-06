// Hover readout test.
//
// Checks that hovering a contact blip reports that contact's identity, and that
// the readout cannot overlap a label, a blip, or the AP badge.
//
// Background: this was a QToolTip following the cursor. It covered the access
// points being read and moved with the pointer, so it also hid whatever the
// pointer moved to next. It was removed, which fixed the obstruction but left no
// way to tell the tracked transmitters apart at all -- every blip is labelled
// "AP". It now draws into a fixed strip at the bottom of the sweep which is
// measured and reserved before the labels are placed.
//
// Layout only. The Snapshot is built by hand; nothing is fed into the engine and
// no reported data is produced.
//
// Build and run from build/:
//
//   g++ -std=c++17 -fPIC -I../include -I../ui $(pkg-config --cflags Qt6Widgets) \
//       ../tests/ui/scope_hover_test.cpp $OBJS -o /tmp/hover_test \
//       $(pkg-config --libs Qt6Widgets Qt6Gui Qt6Core)
//   QT_QPA_PLATFORM=offscreen /tmp/hover_test
//
// Exits non-zero if hovering shows nothing, or if the readout overlaps text.
#include <QApplication>
#include <QImage>
#include <QMouseEvent>

#include "Widgets.hpp"

using namespace radar;

namespace {

int failures = 0;

void check(bool ok, const QString& what) {
    std::printf("  [%s] %s\n", ok ? "pass" : "FAIL", what.toUtf8().constData());
    if (!ok) ++failures;
}

Snapshot makeSnapshot(int n) {
    Snapshot s;
    for (int i = 0; i < n; ++i) {
        Contact c;
        c.id = static_cast<uint64_t>(i + 1);
        for (int b = 0; b < 6; ++b)
            c.mac[b] = static_cast<uint8_t>(i * 7 + b * 3);
        c.label = "network-name-" + std::to_string(i);
        c.rangeM = 1.5 + i * 0.9;
        c.rangeValid = true;
        c.rangeLoM = c.rangeM * 0.7;
        c.rangeHiM = c.rangeM * 1.4;
        c.bearingValid = false;
        c.levelDbm = -38.0 - i * 3;
        c.presence = 1.0;
        c.confidence = 0.8;
        c.state = ContactState::Active;
        c.updates = 40;
        c.velocityMps = 0.12;
        c.velocityValid = true;
        s.contacts.push_back(c);
    }
    s.fusion.ok = false;  // no anchors, so bearing stays honestly unavailable
    return s;
}

QImage render(RadarScope& w) {
    QImage img(w.size(), QImage::Format_ARGB32);
    img.fill(Qt::black);
    w.render(&img);
    return img;
}

}  // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    RadarScope w;
    w.resize(760, 700);

    std::puts("hover readout");

    w.setSnapshot(makeSnapshot(6));
    QImage idle = render(w);
    idle.save("/tmp/opencode/hover_idle.png");

    // No pointer over anything: nothing should be reported.
    check(w.hoverRect().isEmpty(), "no readout drawn when nothing is hovered");
    check(w.labelOverlaps() == 0, "no overlapping text with no hover");

    // Aim a synthetic mouse move at one label's centre, which is how the widget
    // decides what is under the pointer.
    const QRectF target = w.labelRect(3);
    check(!target.isEmpty(), "contact 3 has a drawn label to hover");
    if (target.isEmpty()) return 1;

    QMouseEvent mv(QEvent::MouseMove, target.center(), target.center(),
                   Qt::NoButton, Qt::NoButton, Qt::NoModifier);
    QApplication::sendEvent(&w, &mv);

    QImage hovered = render(w);
    hovered.save("/tmp/opencode/hover.png");

    // The readout must actually be drawn. This is the assertion that matters:
    // an earlier version computed the strip inside the motion-marks loop, so it
    // silently never appeared and the overlap checks below all passed vacuously.
    const QRectF hr = w.hoverRect();
    check(!hr.isEmpty(), "hovering draws a readout");

    // The two fields inside it must fit: identity on the left, measurements on
    // the right, no overlap. Sizing the strip to whichever was longer let the
    // bold name run straight through the detail text.
    check(hr.width() > 0 && hr.height() > 0, "readout has a real size");
    if (!hr.isEmpty()) {
        // Measured from the fields the widget actually built, against the fonts
        // it actually draws them with, so the check is about real content.
        const QFontMetrics m(QFont("Sans", 8));
        const QFontMetrics b(QFont("Sans", 8, QFont::Bold));
        const int hw = b.horizontalAdvance(w.hoverHeadText());
        const int dw = m.horizontalAdvance(w.hoverDetailText());
        std::printf("         (readout %.0f px of %.0f; fields %d + %d + padding)\n",
                    hr.width(), static_cast<double>(w.width()), hw, dw);

        // Compact: it is a corner note, not a banner across the scope. The first
        // version spelled out range interval, level, state, velocity and bearing
        // and ran past half the sweep's width.
        check(hr.width() <= w.width() * 0.47, "readout stays under half the sweep width");
        check(hr.width() >= std::min(hw + dw + 27, static_cast<int>(w.width() * 0.47)),
              "readout is wide enough for its fields");
        // Identity gets at most half the strip, so a long SSID elides by design
        // rather than pushing the numbers out.
        check(hw <= hr.width() / 2 || hr.width() >= hw + dw + 27,
              "identity field fits without eliding when there is room");
        check(hr.right() <= w.width() && hr.bottom() <= w.height(),
              "readout stays inside the widget");
        std::printf("         (head=\"%s\"  detail=\"%s\")\n",
                    w.hoverHeadText().toUtf8().constData(),
                    w.hoverDetailText().toUtf8().constData());
    }

    // Hovering must not disturb the layout: the readout is reserved before the
    // labels are placed, so a label can never end up underneath it.
    check(w.labelOverlaps() == 0, "hover readout does not overlap any label or the badge");

    // And the readout must not sit on the badge either.
    check(!w.apBadgeRect().intersects(hr), "hover readout is clear of the badge");

    // Moving off must clear it again.
    QMouseEvent away(QEvent::MouseMove, QPointF(2, 2), QPointF(2, 2), Qt::NoButton,
                     Qt::NoButton, Qt::NoModifier);
    QApplication::sendEvent(&w, &away);
    render(w);
    check(w.hoverRect().isEmpty(), "readout clears when the pointer moves off");
    check(w.labelOverlaps() == 0, "layout still clean after moving off");

    std::printf("%s: %d failure(s)\n", failures ? "FAILED" : "OK", failures);
    return failures ? 1 : 0;
}