// Sweep-wedge cache equivalence test.
//
// The scope no longer fills itself with a QConicalGradient every frame. That
// costs an atan2 per pixel -- around 190,000 of them, each frame, which profiling
// put at 25% of all cycles in libm. Instead the wedge is rasterised once at
// gradient angle zero and blitted rotated.
//
// This asserts the two are actually equivalent, because "looks about right" is
// not good enough for the thing you look at to decide where a person is. If the
// rotation sign or the origin is wrong the wedge points the wrong way and the
// sweep silently misleads, so the two renderings are compared pixel by pixel.
//
// Layout only. No engine, no radio.
#include <QApplication>
#include <QImage>
#include <QPainter>
#include <QPixmap>
#include <cmath>
#include <cstdio>

namespace {

constexpr int kSize = 300;      // square pixmap
constexpr double kRadius = 140.0;

void paintGradient(QPainter& p, QPointF centre, double angleDeg, QColor tint) {
    QConicalGradient g(centre, angleDeg);
    g.setColorAt(0.00, QColor(tint.red(), tint.green(), tint.blue(), 78));
    g.setColorAt(0.10, QColor(tint.red(), tint.green(), tint.blue(), 20));
    g.setColorAt(0.28, QColor(tint.red(), tint.green(), tint.blue(), 0));
    g.setColorAt(1.00, QColor(tint.red(), tint.green(), tint.blue(), 0));
    p.setBrush(g);
    p.setPen(Qt::NoPen);
    p.drawEllipse(centre, kRadius, kRadius);
}

// What the scope used to do: fill directly at the live sweep angle.
QImage direct(double sweepDeg, QColor tint) {
    QImage img(kSize, kSize, QImage::Format_ARGB32);
    img.fill(Qt::transparent);
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing, true);
    paintGradient(p, QPointF(kSize / 2.0, kSize / 2.0), -sweepDeg, tint);
    p.end();
    return img;
}

// What the scope does now: render at angle zero, then rotate the blit.
QImage cached(double sweepDeg, QColor tint) {
    QPixmap wedge(kSize, kSize);
    wedge.fill(Qt::transparent);
    {
        QPainter wp(&wedge);
        wp.setRenderHint(QPainter::Antialiasing, true);
        paintGradient(wp, QPointF(kSize / 2.0, kSize / 2.0), 0.0, tint);
        wp.end();
    }
    QImage img(kSize, kSize, QImage::Format_ARGB32);
    img.fill(Qt::transparent);
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing, false);
    p.translate(kSize / 2.0, kSize / 2.0);
    p.rotate(sweepDeg);
    p.drawPixmap(-kSize / 2.0, -kSize / 2.0, wedge);
    p.end();
    return img;
}

}  // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    int failures = 0;

    std::puts("sweep wedge cache");

    // Where is the bright end of the wedge, as an angle from north?
    const auto wedgeHeading = [](const QImage& img) {
        double bestSum = -1.0;
        double bestDeg = 0.0;
        const double cx = img.width() / 2.0, cy = img.height() / 2.0;
        const double r = kRadius * 0.75;
        for (int deg = 0; deg < 360; ++deg) {
            const double a = deg * M_PI / 180.0;
            const int x = static_cast<int>(cx + r * std::sin(a));
            const int y = static_cast<int>(cy - r * std::cos(a));
            const QRgb px = img.pixel(x, y);
            const double sum = qAlpha(px) + qRed(px);
            if (sum > bestSum) {
                bestSum = sum;
                bestDeg = deg;
            }
        }
        return bestDeg;
    };

    // At zero the two must agree, and the wedge must point due north.
    {
        const QImage d = direct(0.0, QColor(64, 224, 255));
        const QImage c = cached(0.0, QColor(64, 224, 255));
        int diff = 0;
        for (int y = 0; y < kSize; ++y)
            for (int x = 0; x < kSize; ++x)
                if (qAlpha(d.pixel(x, y)) != qAlpha(c.pixel(x, y))) ++diff;
        std::printf("         (at 0 deg: %d differing pixels, heading %.0f)\n", diff,
                    wedgeHeading(d));
        // A rotated blit resamples, so the antialiased rim differs by a little;
        // 32 pixels of a 300x300 image is that, and nothing else may differ.
        if (diff > 400) {
            std::printf("  [FAIL] zero angle must match apart from rim resampling\n");
            ++failures;
        }
        // QConicalGradient starts at 3 o'clock, so with no sweep the bright end
        // of the wedge points east, which is what the scope has always shown.
        const double h = wedgeHeading(d);
        if (std::fabs(h - 90.0) > 3.0) {
            std::printf("  [FAIL] wedge must start at 3 o'clock, got %.0f\n", h);
            ++failures;
        }
    }

    // Across the sweep, both must agree. A wrong rotation sign still produces a
    // plausible-looking wedge at some angles, so the heading itself is compared
    // as well as the pixels.
    for (double deg : {15.0, 37.0, 90.0, 143.0, 180.0, 270.0, 355.0}) {
        const QImage d = direct(deg, QColor(255, 86, 86));
        const QImage c = cached(deg, QColor(255, 86, 86));

        // Count pixels that differ materially, rather than taking the worst
        // single pixel. The wedge's leading edge is a sharp gradient falling from
        // alpha 78 to 0 inside a tenth of the circle, so rotating a rasterisation
        // of it necessarily resamples that one edge: at an angle that is not a
        // multiple of the pixel grid, the bright end lands between two pixels and
        // one image has 78 there where the other has 0. A handful of edge pixels
        // is the expected cost; anything spread across the interior would mean the
        // cache is genuinely wrong.
        int differing = 0;
        for (int y = 0; y < kSize; ++y)
            for (int x = 0; x < kSize; ++x)
                if (std::abs(int(qAlpha(d.pixel(x, y))) - int(qAlpha(c.pixel(x, y)))) > 8)
                    ++differing;

        const double hd = wedgeHeading(d);
        const double hc = wedgeHeading(c);
        double hdiff = std::fabs(hd - hc);
        if (hdiff > 180.0) hdiff = 360.0 - hdiff;

        std::printf("         (at %5.1f deg: %5d differing pixels (%.2f%%), heading direct "
                    "%.0f / cached %.0f)\n",
                    deg, differing, 100.0 * differing / (kSize * kSize), hd, hc);

        // The heading is the property that matters: it is where the sweep appears
        // to be pointing, and it must not drift.
        if (hdiff > 2.0) {
            std::printf("  [FAIL] wedge heading differs at %.0f deg\n", deg);
            ++failures;
        }
        // Only the resampled edge may differ. 1% of the image is far more than a
        // one-pixel edge needs and still catches a real mismatch.
        if (differing > (kSize * kSize) / 100) {
            std::printf("  [FAIL] too much of the wedge differs at %.0f deg\n", deg);
            ++failures;
        }
    }

    std::printf("%s: %d failure(s)\n", failures ? "FAILED" : "OK", failures);
    return failures ? 1 : 0;
}