#include "Widgets.hpp"

#include <QFontMetrics>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QMouseEvent>
#include <QPainterPath>
#include <QTimerEvent>

#include <algorithm>
#include <cmath>

#include "Plot.hpp"
#include "Theme.hpp"

namespace radar {
namespace {
constexpr double kPi = 3.14159265358979323846;

// Fallback path-loss model used purely for drawing, when the engine has not
// supplied a calibration. Mirrors Config::validated()'s defaults.
const PathLossModel& drawModel() {
    static PathLossModel m{1.0, -40.0, 2.8, 3.0};
    return m;
}

inline double rangeFromDbm(double dbm) { return drawModel().rangeFromRssi(dbm); }

// Centred status text that can never spill outside the plot frame.
void centredText(QPainter& p, const QRectF& area, const QString& text, const QColor& c) {
    p.save();
    p.setPen(c);
    const QFontMetrics fm(p.font());
    p.drawText(area, Qt::AlignCenter,
               fm.elidedText(text, Qt::ElideRight, static_cast<int>(area.width()) - 10));
    p.restore();
}

void drawGrid(QPainter& p, const QRect& r, int cols, int rows) {
    p.setPen(QPen(theme::grid(), 1));
    for (int i = 1; i < cols; ++i) {
        const int x = r.left() + r.width() * i / cols;
        p.drawLine(x, r.top(), x, r.bottom());
    }
    for (int i = 1; i < rows; ++i) {
        const int y = r.top() + r.height() * i / rows;
        p.drawLine(r.left(), y, r.right(), y);
    }
}
}  // namespace

// ============================================================ RadarScope

RadarScope::RadarScope(QWidget* parent) : QWidget(parent) {
    setMinimumSize(360, 360);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void RadarScope::setRangeMetres(double r) {
    rangeM_ = std::clamp(r, 0.5, 500.0);
    update();
}

void RadarScope::setSnapshot(const Snapshot& s) {
    snap_ = s;
    history_.push_back(s);
    while (history_.size() > 90) history_.pop_front();
    if (snap_.detection.state == DetectionState::Present && snap_.fusion.ok) {
        Blip b;
        b.pos = QPointF(snap_.fusion.rangeM, snap_.fusion.bearingDeg);
        b.confidence = snap_.detection.confidence;
        b.mac = macHash(snap_.primaryMac);
        trails_.push_back({b});
        if (trails_.size() > 40) trails_.erase(trails_.begin());
    }
    update();
}

void RadarScope::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), theme::bg());

    const int side = std::min(width(), height());
    const QPointF centre(width() / 2.0, height() / 2.0);
    const double radius = side / 2.0 - 26.0;
    if (radius < 20) return;

    const auto toScreen = [&](double range, double bearingDeg) {
        const double a = (bearingDeg - 90.0) * kPi / 180.0;
        const double rr = std::clamp(range / rangeM_, 0.0, 1.0) * radius;
        return QPointF(centre.x() + rr * std::cos(a), centre.y() + rr * std::sin(a));
    };

    // --- range rings
    //
    // The ring scale follows the data when there is data. A fixed 12 m range was
    // the other reason the scope looked broken: an access point at -31 dBm
    // inverts to about 2 m, so its blip sat almost exactly on the centre and the
    // outer rings were wasted on empty space. Rings are only stretched to fit
    // once something is actually out there, and never shrunk below the operator's
    // setting.
    double displayRange = rangeM_;
    {
        double extent = snap_.motionRangeExtentM;
        for (const auto& c : snap_.contacts) {
            const double r = c.rangeValid ? c.rangeM : rangeFromDbm(c.levelDbm);
            if (r > extent) extent = r;
        }
        if (extent > 0.05) {
            // Fit the rings to the data in *both* directions. They previously only
            // ever expanded, which left a 0.3 m contact as a dot on the centre of a
            // 12 m scope -- the other half of why the scope looked dead. The slider
            // becomes the maximum range shown rather than a fixed scale, with a
            // floor so a near-field contact cannot blow the rings up to nothing.
            const double fitted = std::max(1.0, std::ceil(extent * 1.8));
            displayRange = std::clamp(fitted, 1.0, std::max(1.0, rangeM_));
        }
    }
    const double ringRange = displayRange;
    const int rings = 4;
    for (int i = 1; i <= rings; ++i) {
        const double f = i / static_cast<double>(rings);
        p.setPen(QPen(f == 1 ? theme::gridBright() : theme::grid(), 1, Qt::SolidLine));
        p.drawEllipse(centre, radius * f, radius * f);
        if (showGrid_) {
            p.setPen(theme::grid());
            const double label = ringRange * f;
            p.drawText(QPointF(centre.x() + 3, centre.y() - radius * f - 2),
                       QString::number(label, 'g', 3) + " m");
        }
    }
    // Whether anything is sitting on or beyond the outer ring, so a near-field
    // contact is visibly clamped rather than silently off-scale.
    double widest = snap_.motionRangeExtentM;
    for (const auto& c : snap_.contacts) {
        const double r = c.rangeValid ? c.rangeM : rangeFromDbm(c.levelDbm);
        if (r > widest) widest = r;
    }
    const bool clamped = widest > ringRange * 0.99;

    // --- bearing spokes
    if (showGrid_) {
        p.setPen(QPen(theme::grid(), 1));
        for (int i = 0; i < 12; ++i) {
            const double a = i * 30.0 * kPi / 180.0 - kPi / 2.0;
            p.drawLine(centre, QPointF(centre.x() + radius * std::cos(a),
                                       centre.y() + radius * std::sin(a)));
        }
        p.setPen(theme::textDim());
        p.drawText(QPointF(centre.x() - 10, centre.y() - radius - 12), "N");
        p.drawText(QPointF(centre.x() - 6, centre.y() + radius + 18), "S");
        p.drawText(QPointF(centre.x() + radius + 8, centre.y() + 4), "E");
        p.drawText(QPointF(centre.x() - radius - 20, centre.y() + 4), "W");
    }

    // --- rotating sweep with a phosphor-like wedge.
    // Advance from elapsed time, not per repaint: at an uneven frame rate a
    // per-paint increment makes the sweep visibly stutter or stall.
    {
        const auto now = std::chrono::steady_clock::now();
        if (lastSweep_ != now) {
            if (lastSweep_.time_since_epoch().count() != 0) {
                const double dt = std::chrono::duration<double>(now - lastSweep_).count();
                sweepAngle_ = std::fmod(sweepAngle_ + dt * 72.0, 360.0);
            }
            lastSweep_ = now;
        }
    }
    const bool present = snap_.detection.state == DetectionState::Present;
    const QColor sweepTint = present ? QColor(255, 86, 86) : QColor(64, 224, 255);
    QConicalGradient sweepGrad(centre, -sweepAngle_);
    sweepGrad.setColorAt(0.00, QColor(sweepTint.red(), sweepTint.green(), sweepTint.blue(), 78));
    sweepGrad.setColorAt(0.10, QColor(sweepTint.red(), sweepTint.green(), sweepTint.blue(), 20));
    sweepGrad.setColorAt(0.28, QColor(sweepTint.red(), sweepTint.green(), sweepTint.blue(), 0));
    sweepGrad.setColorAt(1.00, QColor(sweepTint.red(), sweepTint.green(), sweepTint.blue(), 0));
    p.setBrush(sweepGrad);
    p.setPen(Qt::NoPen);
    p.drawEllipse(centre, radius, radius);

    // --- trails
    if (showTrails_) {
        for (const auto& trail : trails_) {
            if (trail.size() < 2) continue;
            for (size_t i = 1; i < trail.size(); ++i) {
                const double age = static_cast<double>(i) / static_cast<double>(trail.size());
                p.setPen(QPen(QColor(64, 224, 255, static_cast<int>(90 * age)), 1.4));
                p.drawLine(toScreen(trail[i - 1].pos.x(), trail[i - 1].pos.y()),
                           toScreen(trail[i].pos.x(), trail[i].pos.y()));
            }
        }
    }

    // --- confidence history ring
    if (!history_.empty()) {
        for (size_t i = 0; i < history_.size(); ++i) {
            const auto& h = history_[i];
            if (h.detection.confidence <= 0.0) continue;
            const double frac = static_cast<double>(i) / static_cast<double>(history_.size());
            p.setPen(QPen(QColor(255, 186, 66, static_cast<int>(200 * frac)), 1.4));
            p.drawArc(QRectF(centre.x() - radius - 7, centre.y() - radius - 7,
                              (radius + 7) * 2, (radius + 7) * 2),
                      static_cast<int>(frac * -90 * 16), static_cast<int>(0.06 * -90 * 16));
        }
    }

    // --- motion marks.
    //
    // A contact is a transmitter: one exists per access point in range and it
    // does not move. With a single access point that is a single stationary dot,
    // which is exactly what the scope used to show and read as broken. A mark is
    // an *event* -- the detector tripped -- and these are the transient, fading
    // indicators of where activity has been detected.
    //
    // Bearing is only drawn when anchors resolved one. With a single access point
    // there is no direction to measure, so the marks are laid out along a range
    // axis at a clearly-marked placeholder bearing and the caption says so. A
    // confident-looking arrow in an arbitrary direction would be a fabrication.
    {
        const double nowWall =
            std::chrono::duration<double>(snap_.stamp.time_since_epoch()).count();
        const double life = 14.0;
        const bool anyBearing = snap_.fusion.ok;

        for (const auto& m : snap_.motionMarks) {
            const double age = nowWall - m.wallTime;
            if (age < 0.0 || age > life) continue;
            const double fade = 1.0 - age / life;
            const double conf = std::clamp(m.confidence, 0.0, 1.0);

            // Without a measured bearing, spread along a horizontal range axis at
            // a documented placeholder rather than inventing a direction.
            const double bearing = m.bearingValid ? m.bearingDeg : 0.0;
            QPointF pos;
            if (m.bearingValid) {
                pos = toScreen(m.rangeM, bearing);
            } else {
                const double f = std::clamp(m.rangeM / ringRange, 0.0, 1.0);
                pos = QPointF(centre.x() + f * radius, centre.y());
            }

            // Fading halo, growing and dimming as the mark ages: a recent
            // detection is tight and bright, an old one is wide and faint.
            const int alpha = static_cast<int>(200 * fade * (0.35 + 0.65 * conf));
            if (alpha <= 3) continue;
            for (int k = 1; k <= 3; ++k) {
                const double rr = 4.0 + k * (3.5 + 9.0 * (1.0 - fade));
                p.setPen(QPen(QColor(255, 150, 90, std::max(6, alpha / (5 * k))), 1,
                              Qt::DotLine));
                p.setBrush(Qt::NoBrush);
                p.drawEllipse(pos, rr, rr);
            }

            // The mark itself: a hollow dotted diamond, deliberately unlike the
            // solid contact blip so the two are never confused.
            const double sz = 6.0;
            p.setPen(QPen(QColor(255, 140, 80, alpha), 1.6));
            p.setBrush(QColor(20, 10, 6, std::min(160, alpha)));
            QPolygonF diamond;
            diamond << QPointF(pos.x(), pos.y() - sz) << QPointF(pos.x() + sz, pos.y())
                    << QPointF(pos.x(), pos.y() + sz) << QPointF(pos.x() - sz, pos.y());
            p.drawPolygon(diamond);

            // Age and range on a leader, shown only while it still means
            // something: a screen full of numbers ages out of usefulness fast.
            if (fade > 0.25) {
                p.setFont(QFont(font().family(), 8));
                p.setPen(QColor(255, 190, 150, static_cast<int>(220 * fade)));
                p.drawText(pos + QPointF(sz + 4, -sz),
                           QStringLiteral("%1m  %2s ago")
                               .arg(m.rangeM, 0, 'f', 1)
                               .arg(age, 0, 'f', 0));
                p.setFont(font());
            }
        }

        // A compact badge at the left edge rather than a sentence across the
        // middle of the scope. The earlier caption spanned the full width of the
        // sweep and sat right where the marks are, which is the worst possible
        // place for a piece of text that exists to explain them.
        if (!snap_.motionMarks.empty() && !anyBearing) {
            const int apCount = std::max(1, static_cast<int>(snap_.directionTransmitters));
            const QString badge = apCount > 1 ? QStringLiteral("BEARING N/A")
                                             : QStringLiteral("1 AP · BEARING N/A");

            p.setFont(QFont(font().family(), 8, QFont::Bold));
            const QFontMetrics fm(p.font());
            const int tw = fm.horizontalAdvance(badge) + 30;
            const int th = 18;
            const QRect box(10, 10, tw, th);

            p.setPen(QPen(QColor(255, 186, 66, 120), 1));
            p.setBrush(QColor(14, 11, 6, 215));
            p.drawRoundedRect(box, 9, 9);

            // Miniature compass with no needle: reads as "no direction" at a
            // glance without needing to be read at all.
            const QPoint cc(box.left() + 13, box.center().y());
            p.setPen(QPen(QColor(255, 186, 66, 150), 1, Qt::DotLine));
            p.setBrush(Qt::NoBrush);
            p.drawEllipse(cc, 5, 5);
            p.setPen(QPen(QColor(255, 186, 66, 190), 1.2));
            p.drawLine(cc + QPoint(-3, 3), cc + QPoint(3, -3));

            p.setPen(QColor(255, 206, 140, 225));
            p.drawText(QRect(box.left() + 23, box.top(), tw - 26, th),
                       Qt::AlignLeft | Qt::AlignVCenter, badge);
            p.setFont(font());
        }
    }

    // --- tracked contacts on the sweep.
    //
    // Drawn from the tracker's contacts rather than the raw RSSI tracks, which
    // is what gives them a stable id, a live range and a disappearance timeout.
    //
    // Placement is decluttered. Without it every contact whose bearing is
    // unknown is placed at a hash of its MAC address, so a room with a handful
    // of transmitters puts several blips on the same spot and they become one
    // unreadable dot. Contacts are now placed in priority order and pushed off
    // each other, with a leader line drawn from the displaced blip back to where
    // the contact actually is, so nothing is implied about position that the
    // measurement does not support.
    {
        struct Placed {
            QPointF truePos;
            QPointF shownPos;
            const Contact* c;
            bool displaced = false;
        };
        std::vector<Placed> placed;

        // Priority: the primary first, then whoever is being heard most strongly
        // and most recently, so the blips that matter keep their true position
        // and the marginal ones move.
        std::vector<const Contact*> order;
        for (const auto& c : snap_.contacts) {
            if (c.presence <= 0.01) continue;  // fully faded out: not drawn at all
            order.push_back(&c);
        }
        std::stable_sort(order.begin(), order.end(), [&](const Contact* a, const Contact* b) {
            const bool ap = a->id == (snap_.contacts.empty() ? 0 : snap_.contacts.front().id);
            const bool bp = b->id == (snap_.contacts.empty() ? 0 : snap_.contacts.front().id);
            if (ap != bp) return ap;
            if (a->levelDbm != b->levelDbm) return a->levelDbm > b->levelDbm;
            return a->lastSeen > b->lastSeen;
        });

        const qreal minSep = 15.0;   // px between two blips
        constexpr int kMaxShown = 14;  // beyond this the scope is unreadable anyway

        int shown = 0;
        int hidden = 0;
        for (const Contact* c : order) {
            if (++shown > kMaxShown) {
                ++hidden;
                continue;
            }
            const double range = c->rangeValid ? c->rangeM : rangeFromDbm(c->levelDbm);
            // Bearing only when something actually measured one. Otherwise the
            // blip is placed by a stable hash so it does not jump frame to frame,
            // and the label is marked so the angle is not read as a direction.
            const double bearing =
                c->bearingValid ? c->bearingDeg : static_cast<double>(macHash(c->mac) % 360);
            const QPointF truePos = toScreen(range, bearing);

            // Declutter: try the true position, then rotate around the ring, then
            // step outward radially.
            QPointF pos = truePos;
            bool displaced = false;
            for (int attempt = 0; attempt < 24; ++attempt) {
                bool clash = false;
                for (const auto& q : placed) {
                    const qreal dx = pos.x() - q.shownPos.x();
                    const qreal dy = pos.y() - q.shownPos.y();
                    if (dx * dx + dy * dy < minSep * minSep) {
                        clash = true;
                        break;
                    }
                }
                if (!clash) break;
                displaced = true;
                if (attempt < 12) {
                    // Rotate 30 degrees about the centre.
                    const double a2 = (bearing + 30.0 * (attempt + 1)) * kPi / 180.0 - kPi / 2.0;
                    const qreal rr = std::clamp(range / rangeM_, 0.0, 1.0) * radius;
                    pos = QPointF(centre.x() + rr * std::cos(a2), centre.y() + rr * std::sin(a2));
                } else {
                    // Push outward a little further each time.
                    const qreal rr = std::clamp(range / rangeM_, 0.0, 1.0) * radius +
                                     7.0 * (attempt - 11);
                    pos = toScreen(rangeM_ * rr / std::max(1e-6, radius), bearing);
                }
            }

            const QColor base = theme::forMac(macHash(c->mac));
            const int alpha = static_cast<int>(255 * std::clamp(c->presence, 0.0, 1.0));
            const bool primary = !snap_.contacts.empty() && c->id == snap_.contacts.front().id;

            // Leader from where the contact really is to where it is being drawn.
            if (displaced) {
                p.setPen(QPen(QColor(base.red(), base.green(), base.blue(),
                                     std::max(30, alpha / 4)),
                              1, Qt::DotLine));
                p.drawLine(truePos, pos);
                p.setBrush(Qt::NoBrush);
                p.setPen(QPen(QColor(base.red(), base.green(), base.blue(),
                                     std::max(30, alpha / 3)),
                              1));
                p.drawEllipse(truePos, 2.5, 2.5);
            }

            // Ageing halo: bright and tight when just heard, wide and dim as the
            // contact ages out of the hold window.
            if (c->silenceSeconds > 0.15) {
                const double fade = std::clamp(1.0 - c->silenceSeconds / 6.0, 0.0, 1.0);
                for (int k = 1; k <= 3; ++k) {
                    const double rr = 5.0 + k * 4.0 * (1.0 - fade) + (primary ? 3.0 : 0.0);
                    p.setPen(QPen(QColor(base.red(), base.green(), base.blue(),
                                         static_cast<int>(46 * fade / k)),
                                  1));
                    p.setBrush(Qt::NoBrush);
                    p.drawEllipse(pos, rr, rr);
                }
            }

            p.setBrush(QColor(base.red(), base.green(), base.blue(), alpha));
            p.setPen(QPen(QColor(base.red(), base.green(), base.blue(), std::min(255, alpha)),
                          primary ? 2 : 1));
            const double sz = primary ? 6.5 : 4.0;
            p.drawEllipse(pos, sz, sz);

            placed.push_back({truePos, pos, c, displaced});
        }

        // Labels, stacked so they never overlap each other or a blip. Each is
        // tied to its blip by a short leader, which is what keeps the mapping
        // readable once several have been decluttered.
        p.setFont(QFont(font().family(), 8));
        const QFontMetrics fm(p.font());
        std::vector<QRectF> taken;
        for (const auto& pl : placed) {
            const Contact* c = pl.c;
            const double range = c->rangeValid ? c->rangeM : rangeFromDbm(c->levelDbm);
            const QColor base = theme::forMac(macHash(c->mac));
            const int alpha = static_cast<int>(255 * std::clamp(c->presence, 0.0, 1.0));

            QString tag = QStringLiteral("#%1").arg(c->id);
            if (pl.displaced || rangeM_ <= range)
                tag += QStringLiteral(" %1m").arg(range, 0, 'f', 1);
            if (c->state != ContactState::Active) tag += QStringLiteral(" %1").arg(toString(c->state));
            if (!c->bearingValid) tag += QStringLiteral("·");  // bearing unknown

            const qreal tw = fm.horizontalAdvance(tag) + 6;
            const qreal th = fm.height() + 2;
            QRectF box(pl.shownPos.x() + 7, pl.shownPos.y() - th - 2, tw, th);

            // Nudge down until it finds clear space, then sideways if needed.
            for (int guard = 0; guard < 40; ++guard) {
                bool clash = false;
                for (const auto& r : taken)
                    if (r.intersects(box)) {
                        clash = true;
                        break;
                    }
                if (!clash)
                    for (const auto& q : placed) {
                        const QRectF br(q.shownPos.x() - 8, q.shownPos.y() - 8, 16, 16);
                        if (br.intersects(box)) {
                            clash = true;
                            break;
                        }
                    }
                if (!clash) break;
                box.moveTop(box.top() + th - 1);
                if (box.bottom() > height() - 4) {
                    box.moveTop(4);
                    box.moveLeft(box.left() + tw + 4);
                }
            }
            taken.push_back(box);

            p.setPen(QPen(QColor(base.red(), base.green(), base.blue(),
                                 std::max(40, alpha / 3)),
                          1));
            p.drawLine(QPointF(pl.shownPos.x() + 1, pl.shownPos.y()), box.topLeft());

            p.setPen(QColor(base.red(), base.green(), base.blue(), alpha));
            p.fillRect(box, QColor(6, 12, 16, 190));
            p.drawRect(box);
            p.setPen(QColor(226, 242, 248, alpha));
            p.drawText(box, Qt::AlignCenter, tag);
        }
        p.setFont(font());

        if (hidden > 0) {
            p.setFont(QFont(font().family(), 8));
            p.setPen(theme::textDim());
            p.drawText(QRect(centre.x() - 60, centre.y() - 8, 120, 16), Qt::AlignCenter,
                       QStringLiteral("+%1 more").arg(hidden));
            p.setFont(font());
        }
    }

    // --- fused track estimate
    if (snap_.fusion.ok) {
        const QPointF pos = toScreen(snap_.fusion.rangeM, snap_.fusion.bearingDeg);
        const bool present = snap_.detection.state == DetectionState::Present;
        const QColor c = present ? theme::red() : theme::amber();
        p.setPen(QPen(c, 2));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(pos, 10, 10);
        p.drawLine(pos + QPointF(-14, 0), pos + QPointF(14, 0));
        p.drawLine(pos + QPointF(0, -14), pos + QPointF(0, 14));
        p.setPen(c);
        p.drawText(pos + QPointF(16, 14),
                   QString("v=%1 m/s").arg(snap_.fusion.speedMps, 0, 'f', 2));
    }

    // --- radar-waterfall decay strip along the bottom of the scope
    if (!history_.empty()) {
        const int h = std::min(28, height() / 8);
        const QRect strip(0, height() - h, width(), h);
        for (int i = 0; i < strip.width(); ++i) {
            const size_t idx = static_cast<size_t>(i) * history_.size() /
                               static_cast<size_t>(std::max(1, strip.width()));
            if (idx >= history_.size()) break;
            const double c = history_[idx].detection.confidence;
            if (c <= 0.01) continue;
            const QColor col = QColor(64, 224, 255, static_cast<int>(200 * std::clamp(c, 0.0, 1.0)));
            const int barH = std::max(
                1, strip.height() - 2 -
                       static_cast<int>((1.0 - c) * (strip.height() - 2)));
            p.fillRect(QRect(strip.left() + i, strip.top() + 1, 1, barH), QBrush(c >= 0.01 ? theme::cyan() : theme::cyanDim()));
            (void)col;
        }
    }
}

// ========================================================= TimeseriesWidget

TimeseriesWidget::TimeseriesWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(150);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void TimeseriesWidget::setSnapshot(const Snapshot& s) {
    snap_ = s;
    update();
}

void TimeseriesWidget::setRange(float lo, float hi) {
    lo_ = lo;
    hi_ = hi;
    update();
}

void TimeseriesWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), theme::bg());

    // Explicit bands, because every label on this widget used to be positioned
    // by hand against a single plot rectangle and three of them collided: the
    // "dBm" caption sat on top of the highest axis tick, and the mean readout
    // was painted straight over the traces with no background behind it.
    const qreal capH = 15.0;    // "dBm" + range caption, above the plot
    const qreal footH = 17.0;   // statistics footer, below the plot
    const qreal gutterL = 42.0;
    const qreal gutterR = 10.0;

    const QRectF plot(rect().left() + gutterL, rect().top() + capH + 2,
                      std::max(10.0, rect().width() - gutterL - gutterR),
                      std::max(10.0, rect().height() - capH - footH - 10.0));
    const QRect area(plot.toRect());
    drawGrid(p, area, 8, 5);

    // Caption band: axis name on the left, live range on the right, so neither
    // can reach the tick labels.
    p.setFont(QFont(font().family(), 8, QFont::Bold));
    p.setPen(theme::textDim());
    p.drawText(QRectF(rect().left() + 4, rect().top(), 40, capH),
               Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("dBm"));

    if (snap_.primarySamples.size() >= 3) {
        p.setFont(font());
        p.setPen(QColor(70, 100, 114));
        p.drawText(QRectF(plot.left(), rect().top(), plot.width(), capH),
                   Qt::AlignRight | Qt::AlignVCenter,
                   QStringLiteral("%1 … %2")
                       .arg(lo_, 0, 'f', 0)
                       .arg(hi_, 0, 'f', 0));
    }
    p.setFont(font());

    // dB axis ticks
    p.setPen(theme::textDim());
    for (int i = 0; i <= 4; ++i) {
        const double v = hi_ - (hi_ - lo_) * i / 4.0;
        const int y = area.top() + area.height() * i / 4;
        p.drawLine(QPoint(area.left() - 4, y), QPoint(area.left(), y));
        p.drawText(QPoint(2, y + 4), QString::number(v, 'f', 0));
    }

    if (snap_.primarySamples.size() < 3) {
        p.setPen(theme::textDim());
        p.drawText(plot, Qt::AlignCenter, QStringLiteral("NO DATA"));
        return;
    }

    const auto toY = [&](double dbm) {
        const double f = (dbm - lo_) / (hi_ - lo_);
        return area.bottom() - std::clamp(f, 0.0, 1.0) * area.height();
    };
    const int n = static_cast<int>(snap_.primarySamples.size());
    const auto toX = [&](int i) {
        return area.left() + area.width() * i / std::max(1, n - 1);
    };

    // Threshold band: the configured detection floor, so you can see how close
    // the signal is to tripping.
    if (snap_.detection.threshold > 0.0) {
        p.setPen(QPen(QColor(255, 186, 66, 90), 1, Qt::DashLine));
        const int y = toY(lo_ + (hi_ - lo_) * 0.0 + snap_.detection.statistic);
        p.drawLine(area.left(), y, area.right(), y);
    }

    // Raw trace
    p.setPen(QPen(QColor(104, 140, 152, 160), 0.8));
    QPainterPath raw;
    for (int i = 0; i < n; ++i) {
        const QPointF pt = QPointF(toX(i), toY(snap_.primarySamples[static_cast<size_t>(i)]));
        if (i == 0) raw.moveTo(pt);
        else raw.lineTo(pt);
    }
    p.drawPath(raw);

    // Smoothed trace, coloured by whether we are detecting
    if (snap_.primarySmoothed.size() >= 3) {
        const bool present = snap_.detection.state == DetectionState::Present;
        p.setPen(QPen(present ? theme::red() : theme::cyan(), 1.5));
        QPainterPath sm;
        const int m = static_cast<int>(snap_.primarySmoothed.size());
        for (int i = 0; i < m; ++i) {
            const int x = area.right() - area.width() * (m - 1 - i) / std::max(1, m - 1);
            const QPointF pt = QPointF(x, toY(snap_.primarySmoothed[static_cast<size_t>(i)]));
            if (i == 0) sm.moveTo(pt);
            else sm.lineTo(pt);
        }
        p.drawPath(sm);
    }

    // Mean line, with its readout in an opaque chip that is clamped inside the
    // plot. Drawn bare, the text was unreadable wherever it crossed a trace and
    // could climb out through the top of the frame on a strong signal.
    {
        const int my = static_cast<int>(toY(snap_.primaryMeanDbm));
        p.setPen(QPen(QColor(96, 255, 168, 110), 0.9, Qt::DashLine));
        p.drawLine(area.left(), my, area.right(), my);

        p.setFont(QFont(font().family(), 8));
        const QString txt = QStringLiteral("mean %1 dBm").arg(snap_.primaryMeanDbm, 0, 'f', 2);
        const QFontMetrics fm(p.font());
        const int tw = fm.horizontalAdvance(txt) + 10;
        const int th = fm.height() + 2;
        // Prefer just above the line; drop below it when there is no room.
        int by = my - th - 3;
        if (by < area.top() + 1) by = std::min(my + 4, area.bottom() - th - 1);
        const int bx = std::clamp(area.left() + 4, area.left() + 1,
                                  std::max(area.left() + 1, area.right() - tw - 1));
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(6, 12, 16, 225));
        p.drawRoundedRect(QRect(bx, by, tw, th), 3, 3);
        p.setPen(theme::green());
        p.drawText(QRect(bx, by, tw, th), Qt::AlignCenter, txt);
        p.setFont(font());
    }

    // Footer band. Elided, because the source label is an SSID or MAC of
    // arbitrary length and used to be clipped mid-word at the right edge.
    {
        const QString txt =
            QStringLiteral("%1  ·  %2 dB RMS  ·  %3 Hz  ·  %4")
                .arg(QString::fromStdString(snap_.primaryLabel))
                .arg(snap_.primaryStdDbm, 0, 'f', 2)
                .arg(snap_.effectiveSampleRateHz, 0, 'f', 1)
                .arg(snap_.detection.statistic > 0
                         ? QStringLiteral("stat %1 dB").arg(snap_.detection.statistic, 0, 'f', 3)
                         : QStringLiteral("idle"));
        const QFontMetrics fm(font());
        const QRectF band(rect().left() + 4, rect().bottom() - footH + 1,
                          rect().width() - 12, footH - 2);
        p.setPen(theme::textDim());
        p.drawText(band, Qt::AlignLeft | Qt::AlignVCenter,
                   fm.elidedText(txt, Qt::ElideRight, static_cast<int>(band.width())));
    }
}

// ========================================================= WaterfallWidget

WaterfallWidget::WaterfallWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(200);
    setMouseTracking(true);
}

void WaterfallWidget::pushColumn(double statistic, double entropy) {
    cols_.push_back(static_cast<float>(statistic));
    ent_.push_back(static_cast<float>(entropy));
    while (static_cast<int>(cols_.size()) > rows_) {
        cols_.pop_front();
        ent_.pop_front();
    }
}

void WaterfallWidget::setSnapshot(const Snapshot& s) {
    snap_ = s;
    // Track the running maximum so the colour scale stays usable as the
    // statistic grows, but decay slowly so the image does not flicker.
    // The threshold participates in the same scale, otherwise the rule and the
    // columns drift apart and the rule ends up in the wrong place.
    scale_ = std::max(scale_ * 0.998f,
                      std::max(std::fabs(static_cast<float>(s.detection.statistic)),
                               s.detection.threshold > 0
                                   ? static_cast<float>(s.detection.threshold)
                                   : 0.0f));
    pushColumn(scale_ > 1e-6f ? s.detection.statistic / scale_ : 0.0f, s.detection.confidence);
    mouseX_ = -1;
    update();
}

void WaterfallWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), theme::bg());

    const QRectF area = plot::axes(p, QRectF(rect()), 0.0, 1.0, 5, QStringLiteral("intensity"),
                                  QStringLiteral("time →"), 1);

    const int n = static_cast<int>(cols_.size());
    if (n < 4) {
        centredText(p, area, QStringLiteral("NO DATA"), theme::textDim());
        return;
    }

    // Spectrogram: one vertical stripe per sample, brightest at the top of the
    // column scaled by intensity.
    // stripe width is derived per column below
    for (int i = 0; i < n; ++i) {
        const double x = area.left() + area.width() * i / static_cast<qreal>(n - 1);
        const double amp = std::clamp(static_cast<double>(cols_[static_cast<size_t>(i)]), 0.0, 1.0);
        const QColor c = plot::heat(amp);
        const qreal h = area.height() * static_cast<qreal>(amp);
        p.fillRect(QRectF(x, area.bottom() - h, std::max(1.0, area.width() / n + 1.0), h), c);
    }

    // Entropy / confidence contour drawn over the top so both are readable.
    std::vector<double> conf(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
        conf[static_cast<size_t>(i)] = static_cast<double>(ent_[static_cast<size_t>(i)]);
    if (n >= 4) {
        const QPainterPath cp =
            plot::smoothPath(area, conf, 0.0, 1.0);
        plot::glowLine(p, cp, theme::cyan(), 1.3);
    }

    // Adaptive threshold, on the same scale as the columns.
    if (snap_.detection.threshold > 0 && scale_ > 1e-6f) {
        const double frac =
            std::clamp(static_cast<double>(snap_.detection.threshold / scale_), 0.0, 1.0);
        const qreal y = area.bottom() - area.height() * frac;
        p.setPen(QPen(theme::amber(), 1, Qt::DashLine));
        p.drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
        // Label sits just above the rule and is right-aligned so it cannot run
        // into the y-axis ticks or the legend.
        p.setPen(theme::amber());
        p.drawText(QRectF(area.left() + 8, y - 16, area.width() - 16, 15),
                   Qt::AlignRight | Qt::AlignVCenter,
                   QStringLiteral("adaptive threshold  %1 dB")
                       .arg(snap_.detection.threshold, 0, 'f', 3));
    }

    // Legend, bottom-right inside the plot, clear of the data's baseline.
    {
        const QRectF leg(area.right() - 128, area.bottom() - 40, 120, 32);
        p.setPen(theme::cyan());
        p.drawText(QRectF(leg.left(), leg.top(), leg.width(), 15),
                   Qt::AlignRight | Qt::AlignVCenter, QStringLiteral("— confidence"));
        p.setPen(theme::amber());
        p.drawText(QRectF(leg.left(), leg.top() + 15, leg.width(), 15),
                   Qt::AlignRight | Qt::AlignVCenter, QStringLiteral("- - CFAR threshold"));
    }

    // Hover readout
    if (mouseX_ >= area.left() && mouseX_ <= area.right()) {
        const int idx = static_cast<int>((mouseX_ - area.left()) / area.width() * (n - 1));
        if (idx >= 0 && idx < n) {
            p.setPen(QPen(QColor(64, 224, 255, 110), 1, Qt::DashLine));
            p.drawLine(QPointF(mouseX_, area.top()), QPointF(mouseX_, area.bottom()));
            const QString txt =
                QStringLiteral("stat %1  conf %2")
                    .arg(static_cast<double>(cols_[static_cast<size_t>(idx)]), 0, 'f', 3)
                    .arg(static_cast<double>(ent_[static_cast<size_t>(idx)]), 0, 'f', 3);
            plot::crosshair(p, area, mouseX_, txt, theme::cyan(), conf, 0.0, 1.0);
        }
    }
}

// ================================================== VelocitySpectrumWidget

VelocitySpectrumWidget::VelocitySpectrumWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(190);
    setMouseTracking(true);
}

void VelocitySpectrumWidget::setSnapshot(const Snapshot& s) {
    snap_ = s;
    norm_.clear();
    if (!s.velocitySpectrum.empty()) {
        double peak = 1e-30;
        for (double v : s.velocitySpectrum) peak = std::max(peak, v);
        norm_.reserve(s.velocitySpectrum.size());
        for (double v : s.velocitySpectrum)
            norm_.push_back(std::pow(std::clamp(v / peak, 0.0, 1.0), 0.45));  // perceptual
    }
    update();
}

void VelocitySpectrumWidget::mouseMoveEvent(QMouseEvent* e) {
    mouseX_ = e->position().x();
    update();
}
void VelocitySpectrumWidget::leaveEvent(QEvent*) {
    mouseX_ = -1;
    update();
}

void VelocitySpectrumWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), theme::bg());

    const double fMax = std::max(snap_.velocitySpectrumMaxHz, 1e-6);
    const double lambda = snap_.wavelengthM > 0 ? snap_.wavelengthM : 0.123;
    const QRectF area = plot::axes(p, QRectF(rect()), 0.0, 1.0, 5,
                                  QStringLiteral("relative power"), QStringLiteral(""), 1);

    if (norm_.empty()) {
        centredText(p, area, QStringLiteral("beat spectrum unavailable — need ≥64 samples"), theme::textDim());
        return;
    }

    // Filled smooth curve.
    const QPainterPath curve = plot::smoothPath(area, norm_, 0.0, 1.0);
    plot::gradientFill(p, curve, area, theme::cyan(), 170.0);
    plot::glowLine(p, curve, theme::cyan(), 1.4);

    // Peak marker and callout.
    if (snap_.velocityValid && snap_.velocitySpectrumMaxHz > 0) {
        const double frac = std::clamp(snap_.beatHz / fMax, 0.0, 1.0);
        const qreal x = area.left() + area.width() * frac;
        p.setPen(QPen(theme::red(), 1.4, Qt::DashLine));
        p.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()));
        const QRectF box(x + 8, area.top() + 6, 168, 34);
        p.setBrush(QColor(10, 16, 22, 235));
        p.setPen(QPen(theme::red(), 1));
        p.drawRoundedRect(box, 5, 5);
        p.setPen(QColor(226, 242, 248));
        p.drawText(QRectF(box.left(), box.top() + 3, box.width(), 15), Qt::AlignCenter,
                   QStringLiteral("%1 Hz").arg(snap_.beatHz, 0, 'f', 3));
        p.setPen(theme::red());
        p.drawText(QRectF(box.left(), box.top() + 18, box.width(), 14), Qt::AlignCenter,
                   QStringLiteral("→ %1 m/s").arg(snap_.radialVelocityMps, 0, 'f', 3));
    }

    // Velocity scale along the bottom: the same axis expressed in m/s.
    p.save();
    p.setPen(QColor(104, 140, 152));
    for (int i = 0; i <= 4; ++i) {
        const double f = i / 4.0;
        const double hz = fMax * (1.0 - f);
        const double v = beatFrequencyToRadialSpeed(hz, lambda);
        p.drawText(QRectF(area.left() + area.width() * f - 40, area.bottom() + 4, 80, 15),
                   Qt::AlignCenter, QStringLiteral("%1 m/s").arg(v, 0, 'f', v < 10 ? 2 : 1));
    }
    p.restore();

    p.setPen(theme::textDim());
    p.drawText(QRectF(area.left(), height() - 2, area.width(), 14), Qt::AlignCenter,
               QStringLiteral("envelope beat frequency   ·   λ = %1 cm at %2 GHz")
                   .arg(lambda * 100.0, 0, 'f', 1)
                   .arg(snap_.wavelengthM > 0 ? 2437.0 / (lambda * 1000.0) : 2437.0, 2437.0 / (lambda * 1000.0), 'f', 2));

    if (mouseX_ >= area.left() && mouseX_ <= area.right() && !norm_.empty()) {
        const int idx = static_cast<int>((mouseX_ - area.left()) / area.width() * (static_cast<int>(norm_.size()) - 1));
        if (idx >= 0 && idx < static_cast<int>(norm_.size())) {
            const double hz = snap_.velocitySpectrumMaxHz * idx / (norm_.size() - 1);
            p.setPen(QPen(QColor(64, 224, 255, 110), 1, Qt::DashLine));
            p.drawLine(QPointF(mouseX_, area.top()), QPointF(mouseX_, area.bottom()));
            plot::crosshair(p, area, mouseX_,
                            QStringLiteral("%1 Hz → %2 m/s")
                                .arg(hz, 0, 'f', 2)
                                .arg(beatFrequencyToRadialSpeed(hz, lambda), 0, 'f', 3),
                            theme::cyan(), norm_, 0.0, 1.0);
        }
    }
}

// ======================================================== TrackTableWidget

TrackTableWidget::TrackTableWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(210);
    setMouseTracking(true);
}

void TrackTableWidget::setSnapshot(const Snapshot& s) {
    snap_ = s;
    // Keep a per-MAC rolling window so the chart shows history, not just a level.
    constexpr size_t kMax = 240;
    for (const auto& t : s.tracks) {
        if (t.count() < 2) continue;
        auto& v = series_[macHash(t.mac)];
        v.push_back(t.mean());
        while (v.size() > kMax) v.erase(v.begin());
    }
    // Drop transmitters that have gone quiet.
    for (auto it = series_.begin(); it != series_.end();) {
        bool alive = false;
        for (const auto& t : s.tracks)
            if (macHash(t.mac) == it->first && t.count() >= 2) alive = true;
        if (alive) {
            ++it;
        } else {
            it = series_.erase(it);
        }
    }
    mouseX_ = -1;
    update();
}

void TrackTableWidget::mouseMoveEvent(QMouseEvent* e) {
    mouseX_ = e->position().x();
    update();
}
void TrackTableWidget::leaveEvent(QEvent*) {
    mouseX_ = -1;
    update();
}

void TrackTableWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), theme::bg());

    // One lane per transmitter. A single shared dBm axis is honest but useless
    // here: a marginal link 40 dB down flattens the strong one into a straight
    // line. Stacked lanes keep full resolution on every source at once, which
    // is what a spectrum analyser does.
    struct Src {
        uint64_t hash = 0;
        std::array<uint8_t, 6> mac{};
        QColor colour;
        std::vector<double> values;
        double last = 0.0;
        double sd = 0.0;
        bool primary = false;
    };
    std::vector<Src> srcs;
    for (const auto& t : snap_.tracks) {
        const uint64_t h = macHash(t.mac);
        auto it = series_.find(h);
        if (it == series_.end() || it->second.size() < 4) continue;
        Src s;
        s.hash = h;
        s.mac = t.mac;
        s.colour = theme::forMac(h);
        s.values = it->second;
        s.last = t.mean();
        s.sd = t.stddev();
        s.primary = (t.mac == snap_.primaryMac);
        srcs.push_back(std::move(s));
    }
    if (srcs.empty()) {
        const QRectF empty = plot::axes(p, QRectF(rect()), -95.0, -25.0, 5,
                                        QStringLiteral("RSSI dBm"), QString(), 0);
        centredText(p, empty, QStringLiteral("no transmitters observed yet"),
                    theme::textDim());
        return;
    }
    std::sort(srcs.begin(), srcs.end(),
              [](const Src& a, const Src& b) { return (int)a.primary > (int)b.primary; });
    constexpr size_t kMaxLanes = 4;
    if (srcs.size() > kMaxLanes) srcs.resize(kMaxLanes);

    // Shared frame and time axis; lanes stack inside it.
    const int marginL = 62, marginR = 12, marginT = 10, marginB = 22;
    const QRectF outer = QRectF(rect());
    const QRectF plotArea(outer.left() + marginL, outer.top() + marginT, outer.width() - marginL - marginR,
                          outer.height() - marginT - marginB);
    p.setBrush(QColor(10, 16, 22));
    p.setPen(QPen(QColor(28, 48, 58), 1));
    p.drawRoundedRect(plotArea.adjusted(-4, -4, 4, 4), 5, 5);

    const double seconds = 30.0;
    plot::timeGrid(p, plotArea, seconds, 6);

    const int lanes = static_cast<int>(srcs.size());
    const qreal laneH = plotArea.height() / lanes;

    for (int i = 0; i < lanes; ++i) {
        Src& s = srcs[static_cast<size_t>(i)];
        const QRectF lane(plotArea.left(), plotArea.top() + laneH * i, plotArea.width(),
                          laneH - (i + 1 < lanes ? 4.0 : 0.0));

        // Per-lane range from the data in that lane only.
        double lo = 1e9, hi = -1e9;
        for (double v : s.values) {
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
        const double pad = std::max(0.8, (hi - lo) * 0.22);
        lo -= pad;
        hi += pad;

        if (i > 0) {
            p.setPen(QPen(QColor(24, 40, 48), 1));
            p.drawLine(QPointF(lane.left(), lane.top()), QPointF(lane.right(), lane.top()));
        }

        // Lane scale, right-aligned into the left gutter.
        p.setFont(QFont(font().family(), 8));
        p.setPen(QColor(120, 156, 168));
        p.drawText(QRectF(plotArea.left() - 58, lane.top() + 1, 52, 12),
                   Qt::AlignRight | Qt::AlignVCenter, QString::number(hi, 'f', 1));
        p.drawText(QRectF(plotArea.left() - 58, lane.bottom() - 13, 52, 12),
                   Qt::AlignRight | Qt::AlignVCenter, QString::number(lo, 'f', 1));

        const QPainterPath path = plot::smoothPath(lane, s.values, lo, hi);
        if (s.primary) {
            plot::gradientFill(p, path, lane, s.colour, 90.0);
            plot::glowLine(p, path, s.colour, 1.6);
        } else {
            plot::glowLine(p, path, QColor(s.colour.red(), s.colour.green(), s.colour.blue(), 165),
                           1.1, 2);
        }

        // Lane title, inside the lane so it can never overlap another lane.
        const QString title = QStringLiteral("%1%2  %3 dBm  σ%4")
                                 .arg(s.primary ? QStringLiteral("▸ ") : QString())
                                 .arg(QString::fromStdString(macToString(s.mac)))
                                 .arg(s.last, 0, 'f', 1)
                                 .arg(s.sd, 0, 'f', 2);
        p.setPen(s.primary ? s.colour.lighter(140) : s.colour);
        p.drawText(QRectF(lane.left() + 6, lane.top() + 2,
                          std::min(lane.width() - 12.0, 300.0), 13),
                   Qt::AlignLeft | Qt::AlignVCenter, title);

        if (s.primary && mouseX_ >= lane.left() && mouseX_ <= lane.right() &&
            s.values.size() >= 2) {
            p.setPen(QPen(QColor(64, 224, 255, 110), 1, Qt::DashLine));
            p.drawLine(QPointF(mouseX_, lane.top()), QPointF(mouseX_, lane.bottom()));
        }
    }
    p.setFont(font());

    // Y-axis caption for the whole stack.
    p.save();
    p.setPen(QColor(88, 122, 134));
    p.translate(outer.left() + 11, plotArea.center().y());
    p.rotate(-90);
    p.drawText(QRectF(-70, -10, 140, 20), Qt::AlignCenter, QStringLiteral("RSSI dBm per source"));
    p.restore();

    if (static_cast<int>(series_.size()) > lanes) {
        p.setPen(theme::textDim());
        p.drawText(QRectF(plotArea.left() + 6, plotArea.top() + 14, 200, 14),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("+%1 more transmitters not shown")
                       .arg(static_cast<int>(series_.size()) - lanes));
    }
}

// ======================================================= SignaturePanelWidget

SignaturePanelWidget::SignaturePanelWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(190);
}

void SignaturePanelWidget::setSnapshot(const Snapshot& s) {
    snap_ = s;
    update();
}

void SignaturePanelWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), theme::bg());

    const int W = width();
    const int left = 12;
    // Three side by side when there is room, otherwise stacked. An earlier
    // version simply stopped after the first column when narrow, which meant the
    // Direction and Signature readings were never shown at all -- and the tab is
    // narrow in the default layout, so that was the normal case rather than an
    // edge case.
    const bool wide = W > 760;
    const int colW = wide ? (W - 2 * left) / 3 : W - 2 * left;
    // When stacked, every section spans the full width from the same left margin.
    // Offsetting them horizontally as well pushed the second and third off the
    // right edge, which is how this first went in.
    const int x1 = left;
    const int x2 = wide ? left + colW : left;
    const int x3 = wide ? left + 2 * colW : left;
    // When stacked, each section gets a third of the height and its own band.
    const int bandH = wide ? height() : std::max(120, height() / 3);
    const int y2off = wide ? 0 : bandH;
    const int y3off = wide ? 0 : 2 * bandH;

    p.setFont(QFont(font().family(), 8, QFont::Bold));
    p.setPen(theme::violet());

    // ---------------- column 1: life sign
    p.drawText(QPoint(x1, 14), QStringLiteral("LIFE SIGN"));
    p.setFont(font());
    {
        const auto& ls = snap_.lifeSign;
        int y = 32;

        // Progress toward a usable window. Reporting nothing at all until the
        // window is full is deliberate, so this bar is the honest explanation for
        // an otherwise blank panel.
        const QRectF bar(x1, y - 9, colW - 16, 6);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(22, 36, 43));
        p.drawRoundedRect(bar, 3, 3);
        p.setBrush(theme::violet());
        p.drawRoundedRect(
            QRectF(bar.left(), bar.top(), bar.width() * std::clamp(snap_.lifeSignProgress, 0.0, 1.0),
                   bar.height()),
            3, 3);
        p.setBrush(Qt::NoBrush);

        if (ls.samples == 0) {
            p.setPen(theme::textDim());
            p.drawText(QRect(x1, y + 4, colW - 12, 18), Qt::AlignLeft | Qt::AlignVCenter,
                       QStringLiteral("NO DATA"));
        } else if (ls.reason == std::string("collecting")) {
            p.setPen(theme::textDim());
            p.drawText(QRect(x1, y + 4, colW - 12, 18), Qt::AlignLeft | Qt::AlignVCenter,
                       QStringLiteral("collecting %1%").arg(snap_.lifeSignProgress * 100.0, 0, 'f', 0));
            p.drawText(QRect(x1, y + 22, colW - 12, 32), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
                       QStringLiteral("needs a long window — respiration is 0.1–0.5 Hz, so "
                                      "several minutes of history"));
        } else {
            const QColor c = ls.detected ? theme::green() : theme::textDim();
            p.setPen(c);
            p.setFont(QFont(font().family(), 15));
            p.drawText(QRect(x1, y + 2, colW - 12, 22),
                       Qt::AlignLeft | Qt::AlignVCenter,
                       ls.detected ? QStringLiteral("PRESENT")
                                   : QStringLiteral("no clear cycle"));
            p.setFont(font());
            y += 28;
            p.setPen(QColor(180, 206, 216));
            p.drawText(QRect(x1, y, colW - 12, 15), Qt::AlignLeft | Qt::AlignVCenter,
                       QStringLiteral("rate %1 Hz · strength %2")
                           .arg(ls.rateHz, 0, 'f', 3)
                           .arg(ls.strength * 100.0, 0, 'f', 0)
                           + QStringLiteral("%"));
            y += 15;
            p.setPen(QColor(150, 180, 192));
            p.drawText(QRect(x1, y, colW - 12, 15), Qt::AlignLeft | Qt::AlignVCenter,
                       QStringLiteral("envelope %1 dB RMS · %2 s window")
                           .arg(ls.rmsDb, 0, 'f', 2)
                           .arg(ls.windowSeconds, 0, 'f', 0));
            y += 15;
            p.setPen(QColor(120, 150, 164));
            p.setFont(QFont(font().family(), 8));
            QFontMetrics fm(p.font());
            p.drawText(QRect(x1, y, colW - 12, 30), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
                       fm.elidedText(QString::fromStdString(ls.reason), Qt::ElideRight,
                                     colW - 12) +
                           QStringLiteral("  (not proof of a person — a fan or a "
                                          "curtain looks identical)"));
            p.setFont(font());
        }
    }

    if (!wide) {
        p.setPen(QPen(QColor(28, 48, 58), 1));
        p.drawLine(left, bandH - 6, W - left, bandH - 6);
        p.drawLine(left, 2 * bandH - 6, W - left, 2 * bandH - 6);
    }

    // ---------------- column 2: direction
    p.setFont(QFont(font().family(), 8, QFont::Bold));
    p.setPen(theme::amber());
    p.drawText(QPoint(x2, 14 + y2off), QStringLiteral("DIRECTION"));
    p.setFont(font());
    {
        const auto& d = snap_.direction;
        int y = 32 + y2off;
        p.setPen(theme::textDim());
        p.drawText(QRect(x2, y, colW - 12, 15), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("%1 transmitters tracked")
                       .arg(snap_.directionTransmitters));

        y += 18;
        if (!d.available) {
            p.setPen(QColor(150, 180, 192));
            QFontMetrics fm(font());
            p.drawText(QRect(x2, y, colW - 12, 34), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
                       fm.elidedText(QString::fromStdString(d.reason), Qt::ElideRight,
                                     colW - 12));
        } else {
            p.setFont(QFont(font().family(), 10));
            p.setPen(theme::amber());
            p.drawText(QRect(x2, y, colW - 12, 18), Qt::AlignLeft | Qt::AlignVCenter,
                       QStringLiteral("→ %1").arg(QString::fromStdString(d.risenLabel)));
            y += 18;
            p.setPen(theme::cyan());
            p.drawText(QRect(x2, y, colW - 12, 18), Qt::AlignLeft | Qt::AlignVCenter,
                       QStringLiteral("← %1").arg(QString::fromStdString(d.fallenLabel)));
            y += 20;
            p.setFont(font());
            p.setPen(QColor(180, 206, 216));
            p.drawText(QRect(x2, y, colW - 12, 15), Qt::AlignLeft | Qt::AlignVCenter,
                       QStringLiteral("%1 / %2 dB · spread %3 dB")
                           .arg(d.risenDb, 0, 'f', 1)
                           .arg(d.fallenDb, 0, 'f', 1)
                           .arg(d.spreadDb, 0, 'f', 1));
            y += 15;
            p.setPen(QColor(150, 180, 192));
            p.drawText(QRect(x2, y, colW - 12, 15), Qt::AlignLeft | Qt::AlignVCenter,
                       QStringLiteral("quality %1% · %2 changed")
                           .arg(d.quality * 100.0, 0, 'f', 0)
                           .arg(d.contributors));
        }
        p.setFont(QFont(font().family(), 8));
        p.setPen(QColor(120, 150, 164));
        QFontMetrics fm(p.font());
        p.drawText(QRect(x2, (wide ? height() - 34 : y2off + bandH - 30), colW - 12, 28),
                   Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
                   fm.elidedText(QString::fromStdString(d.caveat), Qt::ElideRight, colW - 12));
        p.setFont(font());
    }

    // ---------------- column 3: signature
    p.setFont(QFont(font().family(), 8, QFont::Bold));
    p.setPen(theme::green());
    p.drawText(QPoint(x3, 14 + y3off), QStringLiteral("SIGNATURE"));
    p.setFont(font());
    {
        int y = 32 + y3off;
        if (!snap_.signatureValid) {
            p.setPen(theme::textDim());
            p.drawText(QRect(x3, y, colW - 12, 18), Qt::AlignLeft | Qt::AlignVCenter,
                       QStringLiteral("recorded on each detection"));
        } else {
            const Signature& g = snap_.signature;
            const double bounds[8][2] = {{0.2, 4.0}, {0.0, 1.0}, {0.3, 1.0}, {0.0, 6.0},
                                         {2.5, 12.0}, {-3.0, 3.0}, {0.0, 12.0}, {0.2, 6.0}};
            p.setFont(QFont(font().family(), 8));
            for (int i = 0; i < 8; ++i) {
                const QRectF bar(x3 + 76, y - 8, colW - 150, 5);
                p.setPen(Qt::NoPen);
                p.setBrush(QColor(22, 36, 43));
                p.drawRoundedRect(bar, 2, 2);
                const double f = std::clamp((g.v[i] - bounds[i][0]) /
                                                (bounds[i][1] - bounds[i][0]),
                                            0.0, 1.0);
                p.setBrush(theme::green());
                p.drawRoundedRect(QRectF(bar.left(), bar.top(), bar.width() * f, bar.height()),
                                  2, 2);
                p.setBrush(Qt::NoBrush);
                p.setPen(QColor(150, 180, 192));
                p.drawText(QRect(x3, y - 11, 74, 12), Qt::AlignLeft | Qt::AlignVCenter,
                           QString::fromLatin1(signatureFeatureName(static_cast<size_t>(i))));
                y += 13;
            }
            p.setFont(font());
            y += 6;
            p.setPen(QColor(180, 206, 216));
            p.drawText(QRect(x3, y, colW - 12, 15), Qt::AlignLeft | Qt::AlignVCenter,
                       QStringLiteral("%1 stored").arg(snap_.signatureCount));
            y += 16;
            if (snap_.nearestValid) {
                const bool close = snap_.nearestDistance < 0.18;
                p.setPen(close ? theme::green() : theme::textDim());
                p.drawText(QRect(x3, y, colW - 12, 15), Qt::AlignLeft | Qt::AlignVCenter,
                           QStringLiteral("closest match %1")
                               .arg(snap_.nearestDistance, 0, 'f', 3));
                y += 14;
                p.setPen(QColor(150, 180, 192));
                p.drawText(QRect(x3, y, colW - 12, 15), Qt::AlignLeft | Qt::AlignVCenter,
                           QStringLiteral("%1")
                               .arg(close ? QStringLiteral("resembles a previous event")
                                          : QStringLiteral("unlike anything recorded")));
            }
        }
    }
}

// ======================================================= TrackingPanelWidget

TrackingPanelWidget::TrackingPanelWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(150);
}

void TrackingPanelWidget::setSnapshot(const Snapshot& s) {
    snap_ = s;
    update();
}

void TrackingPanelWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), theme::bg());

    const int w = width();
    const int left = 10;
    const int right = w - 10;
    // Contacts take two thirds, anomalies the rest; below that width a single
    // column is more readable than two squeezed ones.
    const bool split = w > 620;
    const int listW = split ? static_cast<int>((right - left) * 0.60) : right - left;
    const int anomX = left + listW + 14;

    p.setFont(QFont(font().family(), 8, QFont::Bold));
    p.setPen(theme::cyan());
    p.drawText(QPoint(left, 14), QStringLiteral("CONTACTS"));
    p.setFont(font());
    p.setPen(QColor(88, 122, 134));
    p.drawText(QPoint(left, 26), QStringLiteral("id · source · range · speed · presence"));

    if (snap_.contacts.empty()) {
        p.setFont(QFont(font().family(), 10));
        p.setPen(theme::textDim());
        p.drawText(QRect(left, 30, listW, 22), Qt::AlignCenter | Qt::AlignVCenter,
                   QStringLiteral("NO TARGET"));
        p.setFont(font());
    } else {
        int y = 40;
        for (const auto& c : snap_.contacts) {
            if (y > height() - 8) break;
            const QColor base = theme::forMac(macHash(c.mac));
            const int alpha = static_cast<int>(255 * std::clamp(c.presence, 0.0, 1.0));

            p.setPen(QPen(base, 2));
            p.drawLine(left, y - 4, left + 8, y - 4);

            p.setPen(QColor(96, 255, 168, alpha));
            p.drawText(QRectF(left + 14, y - 12, 26, 16), Qt::AlignLeft | Qt::AlignVCenter,
                       QStringLiteral("#%1").arg(c.id));

            p.setPen(QColor(210, 232, 240, alpha));
            QString name = QString::fromStdString(c.label);
            name = name.length() > 16 ? name.left(15) + QChar(0x2026) : name;
            p.drawText(QRectF(left + 42, y - 12, 120, 16), Qt::AlignLeft | Qt::AlignVCenter, name);

            // Range with the interval shadowing implies. A bare number here
            // would overstate the precision by a factor of about two.
            if (c.rangeValid) {
                p.setPen(QColor(180, 206, 216, alpha));
                p.drawText(QRectF(left + 162, y - 12, 150, 16),
                           Qt::AlignLeft | Qt::AlignVCenter,
                           QStringLiteral("%1 m  [%2-%3]")
                               .arg(c.rangeM, 0, 'f', 1)
                               .arg(c.rangeLoM, 0, 'f', 1)
                               .arg(c.rangeHiM, 0, 'f', 1));
            } else {
                p.setPen(QColor(94, 132, 150, alpha));
                p.drawText(QRectF(left + 162, y - 12, 150, 16), Qt::AlignLeft | Qt::AlignVCenter,
                           QStringLiteral("range n/a"));
            }

            p.setPen(QColor(150, 180, 192, alpha));
            p.drawText(QRectF(left + 314, y - 12, 70, 16), Qt::AlignLeft | Qt::AlignVCenter,
                       c.velocityValid ? QStringLiteral("%1 m/s").arg(c.velocityMps, 0, 'f', 3)
                                       : QStringLiteral("--"));

            // Presence bar: the visual that makes a fading contact obvious.
            const QRectF bar(left + 388, y - 9, 54, 6);
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(22, 36, 43));
            p.drawRoundedRect(bar, 3, 3);
            p.setBrush(QColor(base.red(), base.green(), base.blue(), alpha));
            p.drawRoundedRect(QRectF(bar.left(), bar.top(), bar.width() * c.presence, bar.height()),
                              3, 3);

            p.setPen(c.state == ContactState::Active ? theme::green()
                                                      : theme::amber());
            p.drawText(QRectF(left + 448, y - 12, 70, 16), Qt::AlignLeft | Qt::AlignVCenter,
                       toString(c.state));

            if (c.silenceSeconds > 0.2) {
                p.setPen(QColor(94, 132, 150, alpha));
                p.drawText(QRectF(left + 512, y - 12, 90, 16), Qt::AlignLeft | Qt::AlignVCenter,
                           QStringLiteral("-%1s").arg(c.silenceSeconds, 0, 'f', 1));
            }
            y += 19;
        }
    }

    if (!split) return;

    // --- anomaly panel
    p.setFont(QFont(font().family(), 8, QFont::Bold));
    p.setPen(theme::violet());
    p.drawText(QPoint(anomX, 14), QStringLiteral("SIGNAL ANALYSIS"));
    p.setFont(font());

    const AnomalyReport& a = snap_.anomaly;
    if (!a.enoughData) {
        p.setPen(theme::textDim());
        p.drawText(QRect(anomX, 24, right - anomX, 40),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("%1/%2 samples\nneed more data")
                       .arg(a.samples)
                       .arg(size_t(48)));
        return;
    }

    int y = 28;
    const int avail = right - anomX;
    p.setFont(QFont(font().family(), 8));
    struct Row { QString k; QString v; };
    const std::vector<Row> rows = {
        {QStringLiteral("mean"), QStringLiteral("%1 dBm  ±%2").arg(a.meanDbm, 0, 'f', 1).arg(a.stdDbm, 0, 'f', 2)},
        {QStringLiteral("volatility"), QStringLiteral("%1").arg(a.varianceRatio, 0, 'f', 2)},
        {QStringLiteral("flatness"), QStringLiteral("%1").arg(a.spectralFlatness, 0, 'f', 3)},
        {QStringLiteral("entropy"), QStringLiteral("%1 / 1").arg(a.shannonEntropyBits, 0, 'f', 2)},
        {QStringLiteral("period"), a.dominantPeriodS > 0.0
                                   ? QStringLiteral("%1 s").arg(a.dominantPeriodS, 0, 'f', 2)
                                   : QStringLiteral("none")},
        {QStringLiteral("kurtosis"), QStringLiteral("%1").arg(a.kurtosis, 0, 'f', 2)},
        {QStringLiteral("trend"), QStringLiteral("%1 dB/min").arg(a.trendPerMinute, 0, 'f', 2)},
    };
    for (const auto& r : rows) {
        p.setPen(QColor(94, 132, 150));
        p.drawText(QRectF(anomX, y, 70, 14), Qt::AlignLeft | Qt::AlignVCenter, r.k);
        p.setPen(QColor(200, 224, 234));
        p.drawText(QRectF(anomX + 70, y, avail - 70, 14), Qt::AlignLeft | Qt::AlignVCenter, r.v);
        y += 14;
    }

    // Flagged anomalies, most confident first, each with what it does not prove.
    y += 4;
    int shown = 0;
    for (const auto& an : a.anomalies) {
        if (!an.active || shown >= 2) break;
        if (y > height() - 26) break;
        p.setPen(theme::violet());
        p.drawText(QRectF(anomX, y, avail, 14), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("%1  %2")
                       .arg(QString::fromLatin1(toString(an.kind)))
                       .arg(QString::fromStdString(an.headline)));
        y += 13;
        p.setPen(QColor(120, 150, 164));
        // Elide rather than let the explanation run off the panel.
        QFontMetrics fm(p.font());
        p.drawText(QRectF(anomX, y, avail, 13), Qt::AlignLeft | Qt::AlignVCenter,
                   fm.elidedText(QString::fromStdString(an.detail), Qt::ElideRight, avail));
        y += 15;
        ++shown;
    }
    if (shown == 0) {
        p.setPen(QColor(94, 132, 150));
        p.drawText(QRect(anomX, y, avail, 14), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("nothing anomalous"));
    }
}

// ======================================================= EventTimelineWidget

EventTimelineWidget::EventTimelineWidget(QWidget* parent) : QWidget(parent) {
    // Two stacked regions plus an axis band; anything less than this and the
    // layout has to start dropping rows.
    setMinimumHeight(300);
}

void EventTimelineWidget::clearMarks() {
    marks_.clear();
    clearedThrough_ = snap_.events.empty() ? 0 : snap_.events.back().wallTime;
    update();
}

void EventTimelineWidget::setSnapshot(const Snapshot& s) {
    snap_ = s;
    const double now = std::chrono::duration<double>(s.stamp.time_since_epoch()).count();
    // Only events we have not already recorded, so the ribbon does not
    // re-append the whole ring on every tick.
    for (const auto& e : s.events) {
        if (e.wallTime <= clearedThrough_) continue;
        if (lastMarked_ != 0.0 && e.wallTime <= lastMarked_) continue;
        lastMarked_ = e.wallTime;
        marks_.emplace_back(e.wallTime, 0.0);
    }
    // Confidence ribbon: one mark per tick, so the timeline shows the detection
    // history as a shape rather than only as discrete log lines.
    marks_.emplace_back(now, s.detection.confidence);
    while (marks_.size() > 600) marks_.pop_front();
    update();
}

void EventTimelineWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), theme::bg());

    const double now = std::chrono::duration<double>(snap_.stamp.time_since_epoch()).count();
    const double window = 60.0;
    const double t0 = now - window;

    // ------------------------------------------------------------------ layout
    //
    // Every band below is derived from the widget's real height instead of a
    // hardcoded bottom margin. The fixed -140 that used to be here was larger
    // than this widget's own minimum height, so at small sizes the ribbon
    // collapsed to zero height and the axis labels, the caption and the centred
    // empty-state text were all painted into the same few pixels.
    const QRectF full(rect());
    const qreal gutter = 46.0;
    const qreal padR = -12.0;
    const qreal w = full.width() + padR - gutter;
    if (w < 40.0 || full.height() < 70.0) return;  // too small to lay out

    const qreal axisH = 15.0;    // time-axis labels under the ribbon
    const qreal headerH = 17.0;  // caption inside the log box
    const qreal lineH = 15.0;
    const int maxLines = 5;
    const qreal minLog = 8.0 + headerH + lineH * maxLines + 8.0;

    const qreal top = 8.0;
    const qreal avail = full.height() - top - 6.0;
    qreal ribbonH = std::max(46.0, avail * 0.50);
    qreal logH = avail - ribbonH - axisH;
    if (logH < minLog) {
        logH = minLog;
        ribbonH = std::max(40.0, avail - logH - axisH);
    }

    const QRectF ribbon(gutter, top, w, ribbonH);
    const QRectF logArea(gutter, ribbon.bottom() + axisH + 5.0, w, logH);

    // ------------------------------------------------------------ ribbon frame
    p.setBrush(QColor(10, 16, 22));
    p.setPen(QPen(QColor(28, 48, 58), 1));
    p.drawRoundedRect(ribbon.adjusted(-4, -4, 4, 4), 5, 5);

    // Grid plus the time axis, drawn in its own band directly beneath the frame.
    for (int i = 0; i <= 6; ++i) {
        const double t = t0 + window * i / 6.0;
        const qreal x = ribbon.left() + ribbon.width() * i / 6.0;
        p.setPen(QPen(QColor(24, 40, 48), 1));
        p.drawLine(QPointF(x, ribbon.top()), QPointF(x, ribbon.bottom()));
        p.setPen(QColor(104, 140, 152));
        p.setFont(QFont(font().family(), 8));
        p.drawText(QRectF(x - 26, ribbon.bottom() + 1, 52, axisH - 2),
                   Qt::AlignCenter, QStringLiteral("-%1s").arg(static_cast<int>(now - t)));
        p.setFont(font());
    }

    // Confidence ribbon
    std::vector<double> conf, ts;
    for (const auto& m : marks_) {
        // Event marks carry a zero confidence and are drawn separately below;
        // mixing them in would drag the ribbon to zero at every event.
        if (m.first < t0 || m.second <= 0.0) continue;
        conf.push_back(m.second);
        ts.push_back(m.first);
    }

    // Caption sits in a reserved strip at the top of the ribbon so it can never
    // sit underneath the centred empty-state text.
    const qreal capH = 15.0;
    const QRectF curveArea = ribbon.adjusted(0, capH, 0, 0);
    bool anyMotion = false;
    for (double c : conf)
        if (c > 0.005) anyMotion = true;

    p.setFont(QFont(font().family(), 8));
    p.setPen(theme::red());
    p.drawText(QRectF(ribbon.left() + 6, ribbon.top() + 1, ribbon.width() - 12, capH - 2),
               Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("detection confidence"));
    p.setFont(font());

    if (conf.size() >= 3) {
        std::vector<double> onGrid(conf.size());
        for (size_t i = 0; i < ts.size(); ++i) {
            const double f = std::clamp((ts[i] - t0) / window, 0.0, 1.0);
            const int slot = static_cast<int>(f * (conf.size() - 1));
            onGrid[static_cast<size_t>(slot)] =
                std::max(onGrid[static_cast<size_t>(slot)], conf[i]);
        }
        const QPainterPath path = plot::smoothPath(curveArea, onGrid, 0.0, 1.0);
        plot::gradientFill(p, path, curveArea, theme::red(), 130.0);
        plot::glowLine(p, path, theme::red(), 1.3);
    }

    // Empty-state text occupies only the curve area, never the caption strip.
    if (conf.size() < 3 || !anyMotion) {
        p.setPen(theme::textDim());
        p.drawText(curveArea, Qt::AlignCenter,
                   conf.size() < 3 ? QStringLiteral("START TO RECORD")
                                    : QStringLiteral("NO MOTION"));
    }

    // Event marks, clipped to the ribbon so they cannot escape into the axis.
    p.save();
    p.setClipRect(ribbon);
    for (const auto& e : snap_.events) {
        if (e.wallTime < t0) continue;
        const qreal x = ribbon.left() + ribbon.width() * (e.wallTime - t0) / window;
        const QColor mc = e.isAnomaly ? theme::violet() : theme::amber();
        p.setPen(QPen(mc, 1, Qt::DashLine));
        p.drawLine(QPointF(x, ribbon.top()), QPointF(x, ribbon.bottom()));
        p.setBrush(mc);
        const double r = e.isAnomaly ? 3.5 : 3.0;
        p.drawEllipse(QPointF(x, ribbon.top() + capH * 0.5), r, r);
    }
    p.restore();

    // -------------------------------------------------------------- recent log
    p.setBrush(QColor(10, 16, 22));
    p.setPen(QPen(QColor(28, 48, 58), 1));
    p.drawRoundedRect(logArea.adjusted(-4, -4, 4, 4), 5, 5);

    // One header, in its own strip. Previously two different captions were drawn
    // at the same y, directly on top of each other.
    p.setFont(QFont(font().family(), 8, QFont::Bold));
    p.setPen(theme::amber());
    p.drawText(QRectF(logArea.left() + 6, logArea.top() + 1, logArea.width() - 12, headerH - 2),
               Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("RECENT"));
    p.setFont(QFont(font().family(), 8));
    p.setPen(QColor(94, 132, 150));
    p.drawText(QRectF(logArea.left() + 6, logArea.top() + 1, logArea.width() - 12, headerH - 2),
               Qt::AlignRight | Qt::AlignVCenter,
               QStringLiteral("full history below"));

    const int fits =
        std::max(0, static_cast<int>((logArea.height() - headerH - 8.0) / lineH));
    int line = 0;
    for (auto it = snap_.events.rbegin(); it != snap_.events.rend() && line < fits; ++it, ++line) {
        // Deliberately short: this strip has room for a fragment, and a long
        // line here used to be clipped mid-word.
        QString txt = QString::fromStdString(it->text);
        if (txt.startsWith(QStringLiteral("PRESENT"))) txt = QStringLiteral("PRESENT");
        else if (txt.startsWith(QStringLiteral("DEPARTED"))) txt = QStringLiteral("DEPARTED");
        else if (txt.startsWith(QStringLiteral("contact gone"))) txt = QStringLiteral("CONTACT LOST");
        if (it->isAnomaly) {
            const int colon = txt.indexOf(QLatin1Char(':'));
            txt = colon > 0 ? QStringLiteral("ANOMALY ") + txt.mid(1, colon) : QStringLiteral("ANOMALY");
        }
        if (it->rangeValid && it->rangeM > 0.0)
            txt += QStringLiteral("  %1m").arg(it->rangeM, 0, 'f', 1);
        QColor c = theme::text();
        if (it->isAnomaly) c = theme::violet();
        else if (it->state == DetectionState::Present) c = theme::red();
        p.setPen(c);
        const QFontMetrics fm(p.font());
        p.drawText(QRectF(logArea.left() + 8, logArea.top() + headerH + line * lineH,
                          logArea.width() - 16, lineH - 1),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   fm.elidedText(txt, Qt::ElideRight, static_cast<int>(logArea.width()) - 24));
    }
    if (snap_.events.empty()) {
        p.setPen(theme::textDim());
        p.drawText(QRectF(logArea.left(), logArea.top() + headerH, logArea.width(),
                          logArea.height() - headerH),
                   Qt::AlignCenter, QStringLiteral("NO EVENTS"));
    }
    p.setFont(font());
}

// ============================================================ NeonButton

NeonButton::NeonButton(const QString& text, QWidget* parent) : QPushButton(text, parent) {
    accent_ = theme::cyan();
    setFocusPolicy(Qt::StrongFocus);
    setCursor(Qt::PointingHandCursor);
    setAttribute(Qt::WA_Hover, true);
    // Fully custom painting: strip the stylesheet background and border.
    setFlat(true);
    setStyleSheet("QPushButton{background:transparent;border:none;}");
    animTimer_ = startTimer(16);  // ~60 Hz easing
}

void NeonButton::setAccent(const QColor& c) {
    accent_ = c;
    update();
}
void NeonButton::setPrimary(bool on) {
    primary_ = on;
    update();
}
void NeonButton::setDanger(bool on) {
    danger_ = on;
    update();
}

QSize NeonButton::sizeHint() const {
    QFontMetrics fm(font());
    return QSize(std::max(120, fm.horizontalAdvance(text()) + 34),
                 std::max(34, fm.height() + 16));
}

void NeonButton::setPressValue(qreal v) {
    press_ = v;
    update();
}
void NeonButton::setHoverValue(qreal v) {
    hover_ = v;
    update();
}
void NeonButton::setFlashValue(qreal v) {
    flash_ = v;
    update();
}

void NeonButton::bump(int delta, int ms) {
    pressTarget_ = delta;
    Q_UNUSED(ms);
}

// Exponential approach with a snappy rate: fast enough to feel instant, slow
// enough that the travel is visible rather than a jump cut.
void NeonButton::timerEvent(QTimerEvent*) {
    bool dirty = false;
    const qreal rate = 0.28;
    const qreal newPress = press_ + (pressTarget_ - press_) * rate;
    if (std::fabs(newPress - press_) > 1e-4) {
        press_ = newPress;
        dirty = true;
    } else if (press_ != pressTarget_) {
        press_ = pressTarget_;
        dirty = true;
    }
    const qreal newHover = hover_ + (hoverTarget_ - hover_) * 0.2;
    if (std::fabs(newHover - hover_) > 1e-4) {
        hover_ = newHover;
        dirty = true;
    }
    if (flash_ > 0.0) {
        flash_ = std::max(0.0, flash_ - 0.045);
        dirty = true;
    }
    if (dirty) update();
}

void NeonButton::enterEvent(QEnterEvent*) {
    hoverTarget_ = 1.0;
    update();
}
void NeonButton::leaveEvent(QEvent*) {
    hoverTarget_ = 0.0;
    pressTarget_ = 0.0;
    update();
}
void NeonButton::mousePressEvent(QMouseEvent* e) {
    if (isEnabled()) pressTarget_ = 1.0;
    QPushButton::mousePressEvent(e);
}
void NeonButton::mouseReleaseEvent(QMouseEvent* e) {
    pressTarget_ = 0.0;
    startFlash();
    QPushButton::mouseReleaseEvent(e);
}
void NeonButton::keyPressEvent(QKeyEvent* e) {
    if (isEnabled() && (e->key() == Qt::Key_Space || e->key() == Qt::Key_Return ||
                        e->key() == Qt::Key_Enter))
        pressTarget_ = 1.0;
    QPushButton::keyPressEvent(e);
}
void NeonButton::keyReleaseEvent(QKeyEvent* e) {
    if (e->key() == Qt::Key_Space || e->key() == Qt::Key_Return || e->key() == Qt::Key_Enter) {
        pressTarget_ = 0.0;
        startFlash();
    }
    QPushButton::keyReleaseEvent(e);
}

void NeonButton::startFlash() { flash_ = 1.0; update(); }

void NeonButton::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    const QRectF area = QRectF(rect()).adjusted(1.5, 1.5, -1.5, -1.5);

    const bool on = isEnabled();
    QColor base = accent_.darker(on ? 320 : 520);
    if (danger_)
        base = QColor(120, 26, 26).darker(on ? 0 : 200);
    if (!on)
        base = QColor(18, 24, 30);
    QColor edge = accent_;
    if (!on)
        edge = QColor(46, 62, 72);
    if (danger_)
        edge = theme::red();

    // 3D travel: the top face slides down by `travel` into a darker base that
    // stays put, which is what reads as a physical button being pushed.
    const qreal travel = 4.0 * static_cast<double>(press_);
    const QRectF baseRect = area;
    const QRectF faceRect = area.adjusted(0, travel, 0, travel * 2.0);

    // Base (the visible "side" of the button while pressed).
    p.setPen(Qt::NoPen);
    p.setBrush(base.darker(150));
    p.drawRoundedRect(baseRect, 6, 6);

    // Top face.
    QLinearGradient g(faceRect.topLeft(), faceRect.bottomLeft());
    QColor top = base.lighter(on ? 100 + int(120 * hover_) : 0);
    QColor bot = base.darker(on ? 100 + int(60 * hover_) : 40);
    if (primary_ && on) {
        top = accent_.darker(150 + int(90 * hover_));
        bot = accent_.darker(280);
    }
    g.setColorAt(0.0, top);
    g.setColorAt(1.0, bot);
    p.setBrush(g);
    p.setPen(QPen(edge, on ? 1.0 : 1.0));
    p.drawRoundedRect(faceRect, 6, 6);

    // Hover glow inside the face.
    if (on && hover_ > 0.01) {
        QColor glow = edge;
        glow.setAlpha(int(70 * hover_));
        p.setPen(Qt::NoPen);
        p.setBrush(glow);
        p.drawRoundedRect(faceRect.adjusted(1, 1, -1, -1), 5, 5);
    }

    // Click flash: a bright wash that decays.
    if (flash_ > 0.01 && on) {
        QColor f = edge;
        f.setAlpha(int(150 * flash_));
        p.setPen(Qt::NoPen);
        p.setBrush(f);
        p.drawRoundedRect(faceRect.adjusted(1, 1, -1, -1), 5, 5);
    }

    // Focus ring, so keyboard users can see where they are.
    if (hasFocus()) {
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(accent_.lighter(160), 1.5, Qt::DashLine));
        p.drawRoundedRect(area.adjusted(-2, -2, 2, 2), 8, 8);
    }

    // Label.
    QColor fg = on ? (danger_ ? theme::red().lighter(150) : accent_.lighter(180))
                   : theme::textDim();
    if (primary_ && on)
        fg = theme::cyan().lighter(190);
    p.setPen(fg);
    p.setFont(font());
    QFontMetrics fm(font());
    const QRectF textRect = faceRect.adjusted(0, -travel * 0.5, 0, 0);
    p.drawText(textRect, Qt::AlignCenter, fm.elidedText(text(), Qt::ElideRight,
                                                          static_cast<int>(faceRect.width()) - 12));
}

}  // namespace radar
