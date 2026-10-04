// Shared plotting primitives for the instrument views.
//
// Everything here is anti-aliased and resolution independent: smooth
// Catmull-Rom-to-bezier curves, gradient fills, glow, "nice number" axis ticks
// and a hover crosshair. The four tab views are built from these so they look
// and behave like one instrument rather than four separate sketches.
#pragma once

#include <QColor>
#include <QPainter>
#include <QPainterPath>
#include <QPointF>
#include <QRectF>

#include <algorithm>
#include <cmath>
#include <vector>

namespace radar {
namespace plot {

// Multiplier applied to every trace width. Exposed so the operator can thin or
// thicken the instrument traces without a rebuild.
inline qreal& traceScaleRef() {
    static qreal scale = 0.55;
    return scale;
}
inline void setTraceScale(qreal v) {
    traceScaleRef() = std::clamp(v, 0.25, 2.0);
}
inline qreal traceScale() { return traceScaleRef(); }

// Round to 1, 2 or 5 times a power of ten, so axis labels read 20 / 50 / 100
// rather than 23.7 / 47.4 / 94.8.
inline double niceStep(double rough) {
    if (!(rough > 0)) return 1.0;
    const double mag = std::pow(10.0, std::floor(std::log10(rough)));
    const double n = rough / mag;
    if (n <= 1.0) return mag;
    if (n <= 2.0) return 2.0 * mag;
    if (n <= 5.0) return 5.0 * mag;
    return 10.0 * mag;
}

inline std::vector<double> niceTicks(double lo, double hi, int target) {
    std::vector<double> out;
    if (!(hi > lo)) return {lo};
    const double step = niceStep((hi - lo) / std::max(1, target));
    const double first = std::ceil(lo / step) * step;
    for (double v = first; v <= hi + step * 1e-9; v += step) {
        // Guard against accumulating floating point drift on the label.
        out.push_back(std::round(v / step) * step);
        if (out.size() > 64) break;
    }
    return out;
}

// Map a data value to a y coordinate inside `area`, with `hi` at the top.
inline qreal mapY(const QRectF& area, double lo, double hi, double v) {
    if (!(hi > lo)) return area.bottom();
    const double f = (v - lo) / (hi - lo);
    return area.bottom() - static_cast<qreal>(std::clamp(f, -0.25, 1.25)) * area.height();
}

inline qreal mapX(const QRectF& area, int index, int count) {
    if (count <= 1) return area.left();
    return area.left() + area.width() * index / static_cast<qreal>(count - 1);
}

// Catmull-Rom spline converted to cubic Beziers. Gives a curve that passes
// exactly through every sample (so peaks are not rounded away) while still
// reading as smooth -- a naive lineTo looks jagged, and a low-order polynomial
// fit invents peaks that are not in the data.
inline QPainterPath smoothPath(const QRectF& area, const std::vector<double>& v, double lo,
                               double hi) {
    QPainterPath path;
    const int n = static_cast<int>(v.size());
    if (n == 0) return path;
    if (n == 1) {
        path.moveTo(area.left(), mapY(area, lo, hi, v[0]));
        path.lineTo(area.left() + 1.0, mapY(area, lo, hi, v[0]));
        return path;
    }
    path.moveTo(mapX(area, 0, n), mapY(area, lo, hi, v[0]));
    for (int i = 0; i < n - 1; ++i) {
        const int i0 = std::max(0, i - 1);
        const int i1 = i;
        const int i2 = i + 1;
        const int i3 = std::min(n - 1, i + 2);
        const QPointF p0(mapX(area, i0, n), mapY(area, lo, hi, v[static_cast<size_t>(i0)]));
        const QPointF p1(mapX(area, i1, n), mapY(area, lo, hi, v[static_cast<size_t>(i1)]));
        const QPointF p2(mapX(area, i2, n), mapY(area, lo, hi, v[static_cast<size_t>(i2)]));
        const QPointF p3(mapX(area, i3, n), mapY(area, lo, hi, v[static_cast<size_t>(i3)]));
        const QPointF c1 = p1 + (p2 - p0) / 6.0;
        const QPointF c2 = p2 - (p3 - p1) / 6.0;
        path.cubicTo(c1, c2, p2);
    }
    return path;
}

inline void gradientFill(QPainter& p, const QPainterPath& curve, const QRectF& area,
                         const QColor& c, double topAlpha = 150.0) {
    if (curve.isEmpty()) return;
    QPainterPath fill = curve;
    fill.lineTo(area.right(), area.bottom());
    fill.lineTo(area.left(), area.bottom());
    fill.closeSubpath();
    QLinearGradient g(area.topLeft(), area.bottomLeft());
    g.setColorAt(0.0, QColor(c.red(), c.green(), c.blue(), static_cast<int>(topAlpha)));
    g.setColorAt(0.55, QColor(c.red(), c.green(), c.blue(), static_cast<int>(topAlpha * 0.28)));
    g.setColorAt(1.0, QColor(c.red(), c.green(), c.blue(), 0));
    p.fillPath(fill, g);
}

// Draw a curve with a soft outer glow, by stroking progressively wider, more
// transparent copies underneath the crisp line.
//
// Two things here were wrong before and both showed up as "the glow is too
// thick":
//
//  1. The halo spread was a fixed `width + i * 1.9` pixels, completely
//     independent of the trace width. So thinning the trace shrank the bright
//     core while leaving an ~8px band of glow around it, and because the halo is
//     stroked along the whole accumulated curve it became more obtrusive the
//     longer the trace grew. The spread is now a multiple of the core width.
//
//  2. The layer alpha profile peaked at the *innermost* halo layer and fell to
//     zero at the outermost, so the glow was effectively invisible -- trading
//     one visual defect for the opposite one. Each layer now contributes, with
//     the composite brightening toward the core.
//
// The operator's trace-weight control is applied here rather than at each call
// site, because it had been plumbed all the way to the slider and then never
// actually multiplied anything.
inline void glowLine(QPainter& p, const QPainterPath& curve, const QColor& c, qreal width = 1.8,
                     int layers = 3) {
    if (curve.isEmpty()) return;
    p.save();
    p.setBrush(Qt::NoBrush);
    p.setRenderHint(QPainter::Antialiasing, true);

    const qreal core = std::max<qreal>(0.8, width * traceScale());
    // Outer edge of the halo, as a multiple of the core width.
    const qreal reach = std::max<qreal>(0.6, core * 1.5);

    // Widest first, so the narrower, brighter layers composite on top.
    for (int i = layers; i >= 1; --i) {
        const qreal f = static_cast<qreal>(i) / layers;  // 1 = outermost
        const qreal w = core + reach * f;
        const qreal a = c.alpha() * 0.17 * (1.0 - 0.72 * f);
        QColor g = c;
        g.setAlpha(std::clamp(static_cast<int>(a), 0, 255));
        p.setPen(QPen(g, w, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.drawPath(curve);
    }
    p.setPen(QPen(c, core, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPath(curve);
    p.restore();
}

// Framed plot background with grid and labelled axes. Returns the inner plot
// rectangle that data should be drawn into.
inline QRectF axes(QPainter& p, const QRectF& outer, double yLo, double yHi, int yTicks,
                   const QString& yUnit = {}, const QString& xUnit = {}, int decimals = 0) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);

    const QRectF area = outer.adjusted(46, 12, -12, -24);

    // Frame
    p.setBrush(QColor(10, 16, 22));
    p.setPen(QPen(QColor(28, 48, 58), 1));
    p.drawRoundedRect(area.adjusted(-4, -4, 4, 4), 5, 5);

    // Horizontal grid + labels
    const auto yt = niceTicks(yLo, yHi, yTicks);
    for (double v : yt) {
        const qreal y = mapY(area, yLo, yHi, v);
        if (y < area.top() - 1 || y > area.bottom() + 1) continue;
        p.setPen(QPen(QColor(24, 40, 48), 1));
        p.drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
        p.setPen(QColor(104, 140, 152));
        const QString label = decimals > 0 ? QString::number(v, 'f', decimals)
                                            : QString::number(qRound(v));
        const QRectF box(area.left() - 44, y - 8, 40, 16);
        p.drawText(box, Qt::AlignRight | Qt::AlignVCenter, label);
    }

    // Axis captions, kept outside the plot so they can never cover data.
    p.setPen(QColor(88, 122, 134));
    if (!yUnit.isEmpty()) {
        p.save();
        p.translate(outer.left() + 11, area.center().y());
        p.rotate(-90);
        p.drawText(QRectF(-60, -10, 120, 20), Qt::AlignCenter, yUnit);
        p.restore();
    }
    if (!xUnit.isEmpty())
        p.drawText(QRectF(area.left(), area.bottom() + 5, area.width(), 16),
                   Qt::AlignRight | Qt::AlignVCenter, xUnit);

    p.restore();
    return area;
}

// Vertical separator grid, used by the time-series views.
inline void timeGrid(QPainter& p, const QRectF& area, double seconds, int columns) {
    p.save();
    p.setPen(QPen(QColor(22, 36, 43), 1));
    for (int i = 0; i <= columns; ++i) {
        const qreal x = area.left() + area.width() * i / static_cast<qreal>(columns);
        p.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()));
    }
    p.setPen(QColor(72, 100, 112));
    int lastRight = -1000;
    for (int i = 0; i <= columns; ++i) {
        const qreal x = area.left() + area.width() * i / static_cast<qreal>(columns);
        const double t = seconds * (1.0 - static_cast<double>(i) / columns);
        const QString label = QString::number(t, 'f', t < 10 ? 1 : 0) + "s";
        const int halfW = label.length() * 4;
        const int leftEdge = static_cast<int>(x) - halfW;
        const int rightEdge = static_cast<int>(x) + halfW;
        // Skip a label rather than let it collide with the previous one, and
        // never draw outside the plot frame.
        if (leftEdge <= lastRight + 4) continue;
        if (rightEdge > static_cast<int>(area.right()) + 2) continue;
        lastRight = rightEdge;
        p.drawText(QRectF(x - halfW, area.bottom() + 4, halfW * 2, 15), Qt::AlignCenter, label);
    }
    p.restore();
}

// Hover crosshair with a value callout.
inline void crosshair(QPainter& p, const QRectF& area, qreal mx, const QString& readout,
                      const QColor& c, const std::vector<double>& v, double lo, double hi) {
    if (mx < area.left() || mx > area.right() || v.empty()) return;
    p.save();
    p.setPen(QPen(QColor(c.red(), c.green(), c.blue(), 120), 1, Qt::DashLine));
    p.drawLine(QPointF(mx, area.top()), QPointF(mx, area.bottom()));

    const int idx = static_cast<int>(std::lround((mx - area.left()) / area.width() *
                                                 static_cast<qreal>(v.size() - 1)));
    if (idx < 0 || idx >= static_cast<int>(v.size())) {
        p.restore();
        return;
    }
    const qreal y = mapY(area, lo, hi, v[static_cast<size_t>(idx)]);
    p.setPen(QPen(c, 1));
    p.setBrush(QColor(6, 10, 14));
    p.drawEllipse(QPointF(mx, y), 3.6, 3.6);

    // Callout box, flipped to whichever side has room so it never clips.
    QFontMetrics fm(p.font());
    const int w = fm.horizontalAdvance(readout) + 14;
    const int h = fm.height() + 8;
    const int bx = std::clamp(static_cast<int>(mx) + 10, static_cast<int>(area.left()) + 2,
                              static_cast<int>(area.right()) - w - 2);
    const int by = std::clamp(static_cast<int>(y) - h - 10, static_cast<int>(area.top()) + 2,
                              static_cast<int>(area.bottom()) - h - 2);
    p.setBrush(QColor(8, 14, 19, 235));
    p.setPen(QPen(QColor(c.red(), c.green(), c.blue(), 150), 1));
    p.drawRoundedRect(QRect(bx, by, w, h), 4, 4);
    p.setPen(QColor(220, 240, 246));
    p.drawText(QRect(bx, by, w, h), Qt::AlignCenter, readout);
    p.restore();
}

// Perceptually ordered colour ramp for the spectrogram: deep blue -> teal ->
// amber -> red. Monotonic in lightness so intensity reads correctly even in
// greyscale or for a red-green colourblind reader.
inline QColor heat(double f) {
    f = std::clamp(f, 0.0, 1.0);
    static const double stops[] = {0.00, 0.22, 0.45, 0.68, 0.85, 1.00};
    static const int rgb[][3] = {{8, 16, 34},   {16, 78, 122},  {22, 150, 150},
                                 {190, 190, 70}, {245, 140, 50}, {255, 70, 70}};
    for (int i = 0; i < 5; ++i) {
        if (f <= stops[i + 1]) {
            const double t = (f - stops[i]) / (stops[i + 1] - stops[i]);
            return QColor(static_cast<int>(rgb[i][0] + t * (rgb[i + 1][0] - rgb[i][0])),
                          static_cast<int>(rgb[i][1] + t * (rgb[i + 1][1] - rgb[i][1])),
                          static_cast<int>(rgb[i][2] + t * (rgb[i + 1][2] - rgb[i][2])));
        }
    }
    return QColor(255, 70, 70);
}

}  // namespace plot
}  // namespace radar