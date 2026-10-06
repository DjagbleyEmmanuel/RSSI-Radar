// Custom-painted instrument widgets.
#pragma once

#include <QColor>
#include <QString>
#include <QPushButton>
#include <QWidget>

#include <chrono>
#include <deque>
#include <map>
#include <vector>

#include "radar/Engine.hpp"

class QPropertyAnimation;

namespace radar {

// A push button with a real 3D press: the top face travels down into a bevel
// behind it, animated rather than swapped, plus a hover glow and a click flash.
// Everything is painted by hand, so the application stylesheet must not style
// this class.
class NeonButton : public QPushButton {
    Q_OBJECT
    // Depth of the pressed face, 0 = at rest, 1 = fully depressed.
    Q_PROPERTY(qreal press READ pressValue WRITE setPressValue)
    // Hover glow strength, 0..1.
    Q_PROPERTY(qreal hover READ hoverValue WRITE setHoverValue)
    // Decaying flash after a click, 0..1.
    Q_PROPERTY(qreal flash READ flashValue WRITE setFlashValue)

  public:
    explicit NeonButton(const QString& text, QWidget* parent = nullptr);

    void setAccent(const QColor& c);
    void setPrimary(bool on);
    // Destructive/stand-down styling, used by the stop control.
    void setDanger(bool on);
    void setEnabledLook(bool on);

    qreal pressValue() const { return press_; }
    qreal hoverValue() const { return hover_; }
    qreal flashValue() const { return flash_; }

    QSize sizeHint() const override;

  protected:
    void paintEvent(QPaintEvent*) override;
    void enterEvent(QEnterEvent*) override;
    void leaveEvent(QEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void keyPressEvent(QKeyEvent*) override;
    void keyReleaseEvent(QKeyEvent*) override;
    void timerEvent(QTimerEvent*) override;

  private:
    void setPressValue(qreal v);
    void setHoverValue(qreal v);
    void setFlashValue(qreal v);
    void startFlash();
    void bump(int delta, int ms);

    QColor accent_;
    bool primary_ = false;
    bool danger_ = false;

    // Animated values are driven by a short internal timer rather than
    // QPropertyAnimation so they keep easing even with no event loop repaints
    // scheduled by the owner.
    qreal press_ = 0.0;
    qreal pressTarget_ = 0.0;
    qreal hover_ = 0.0;
    qreal hoverTarget_ = 0.0;
    qreal flash_ = 0.0;
    int animTimer_ = -1;
};

// PPI scope: range rings, rotating sweep, target blips with trails.
class RadarScope : public QWidget {
    Q_OBJECT
  public:
    explicit RadarScope(QWidget* parent = nullptr);
    void setSnapshot(const Snapshot& s);
    void setRangeMetres(double r);
    void setShowGrid(bool g) { showGrid_ = g; }
    void setShowTrails(bool t) { showTrails_ = t; }

    // Test hook: number of pairs of drawn contact labels that overlap. Zero is
    // the requirement; eyeballing a screenshot is not a sufficient check.
    //
    // The AP badge counts too: it is fixed text at the top-left of the sweep and
    // is seeded into the label declutter as occupied space, so a label landing on
    // it would be exactly the overlap this is here to prevent. Those collisions
    // are returned as extra counts, which is why the total is not necessarily
    // even.
    int labelOverlaps() const {
        int n = 0;
        // Compared on the drawn geometry, not the padded hover rectangle.
        for (size_t i = 0; i < hit_.size(); ++i) {
            for (size_t j = i + 1; j < hit_.size(); ++j)
                if (hit_[i].box.intersects(hit_[j].box)) ++n;
            if (!badgeRect_.isEmpty() && badgeRect_.intersects(hit_[i].box)) ++n;
        }
        return n;
    }

    // Test hook: the AP badge rectangle exactly as drawn.
    QRectF apBadgeRect() const { return badgeRect_; }

    // Test hook: the hover readout rectangle exactly as drawn. Empty when
    // nothing is hovered, which is what makes "hovering shows something" a
    // checkable claim rather than an assumption.
    QRectF hoverRect() const { return hoverStrip_; }

    // Test hooks: the two fields of the readout, so a test can measure what they
    // actually need rather than guessing at a hardcoded string.
    QString hoverHeadText() const { return hoverHead_; }
    QString hoverDetailText() const { return hoverDetail_; }

    // Test hook: the drawn label rectangle for one contact, so a test can aim a
    // synthetic mouse move at it and exercise the hover path.
    QRectF labelRect(uint64_t id) const {
        for (const auto& h : hit_)
            if (h.id == id) return h.box;
        return QRectF();
    }

  protected:
    void paintEvent(QPaintEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void leaveEvent(QEvent*) override;

  private:
    // Where each contact ended up on screen, so hovering can name it.
    struct Hit {
        uint64_t id = 0;
        QRectF rect;   // padded, for hover tolerance
        QRectF box;    // the label rectangle exactly as drawn
    };
    std::vector<Hit> hit_;
    uint64_t hoverId_ = 0;
    // Fixed text at the top-left of the sweep. Kept as a member rather than a
    // local so the declutter pass below it and the layout test can both see it.
    QRectF badgeRect_;
    // The hover readout along the bottom edge, same reasoning.
    QRectF hoverStrip_;
    QString hoverHead_;
    QString hoverDetail_;
    // Channel currently being listened to, shown in the hover readout.

    struct Blip {
        QPointF pos;
        double confidence = 0;
        double age = 0;
        quint64 mac = 0;
    };
    Snapshot snap_;
    double rangeM_ = 12.0;
    bool showGrid_ = true;
    bool showTrails_ = true;
    double sweepAngle_ = 0.0;
    std::chrono::steady_clock::time_point lastSweep_{};
    std::vector<std::vector<Blip>> trails_;
    std::deque<Snapshot> history_;
};

// RSSI time series with the smoothed curve overlaid.
class TimeseriesWidget : public QWidget {
    Q_OBJECT
  public:
    explicit TimeseriesWidget(QWidget* parent = nullptr);
    void setSnapshot(const Snapshot& s);
    void setRange(float lo, float hi);

  protected:
    void paintEvent(QPaintEvent*) override;

  private:
    Snapshot snap_;
    float lo_ = -95.0f;
    float hi_ = -25.0f;
};

// Smoothly scrolling spectrogram of the detection statistic, with the entropy
// trace overlaid and the adaptive threshold drawn across it.
class WaterfallWidget : public QWidget {
    Q_OBJECT
  public:
    explicit WaterfallWidget(QWidget* parent = nullptr);
    void setSnapshot(const Snapshot& s);

  protected:
    void paintEvent(QPaintEvent*) override;

  private:
    void pushColumn(double statistic, double entropy);

    Snapshot snap_;
    std::deque<float> cols_;   // normalised amplitude, oldest first
    std::deque<float> ent_;     // confidence, parallel to cols_
    int rows_ = 200;
    // Single shared vertical scale. Two independent running maxima meant the
    // threshold rule and the spectrogram columns could not line up.
    float scale_ = 1.0f;
    qreal mouseX_ = -1;
};

// Beat-frequency pseudo-spectrum drawn as a filled smooth curve, with the peak
// called out and the velocity scale on the right.
class VelocitySpectrumWidget : public QWidget {
    Q_OBJECT
  public:
    explicit VelocitySpectrumWidget(QWidget* parent = nullptr);
    void setSnapshot(const Snapshot& s);

  protected:
    void paintEvent(QPaintEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void leaveEvent(QEvent*) override;

  private:
    Snapshot snap_;
    std::vector<double> norm_;   // peak-normalised spectrum for plotting
    qreal mouseX_ = -1;
};

// Every observed transmitter as a live multi-series chart, plus a level bar
// column, replacing the old fixed-width text table.
class TrackTableWidget : public QWidget {
    Q_OBJECT
  public:
    explicit TrackTableWidget(QWidget* parent = nullptr);
    void setSnapshot(const Snapshot& s);

  protected:
    void paintEvent(QPaintEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void leaveEvent(QEvent*) override;

  private:
    Snapshot snap_;
    // Per-MAC rolling RSSI history, oldest first.
    std::map<uint64_t, std::vector<double>> series_;
    qreal mouseX_ = -1;
};

// Tracked contacts with their range intervals and disappearance state, plus the
// statistical analysis of the primary envelope. One panel because the two answer
// the same question from different directions: what is out there, and does the
// signal from it look like anything other than noise.
class TrackingPanelWidget : public QWidget {
    Q_OBJECT
  public:
    explicit TrackingPanelWidget(QWidget* parent = nullptr);
    void setSnapshot(const Snapshot& s);

  protected:
    void paintEvent(QPaintEvent*) override;

  private:
    Snapshot snap_;
};

// Life sign, direction and the channel signature, shown together because they
// are three readings of the same question asked three ways: is something here,
// which way did it come from, and does this look like anything seen before.
class SignaturePanelWidget : public QWidget {
    Q_OBJECT
  public:
    explicit SignaturePanelWidget(QWidget* parent = nullptr);
    void setSnapshot(const Snapshot& s);

  protected:
    void paintEvent(QPaintEvent*) override;

  private:
    Snapshot snap_;
};

// Detection timeline: a ribbon of events along a rolling time axis plus the
// log, so a detection is visible as a mark and not only as a line of text.
class EventTimelineWidget : public QWidget {
    Q_OBJECT
  public:
    explicit EventTimelineWidget(QWidget* parent = nullptr);
    void setSnapshot(const Snapshot& s);
    // Drop the ribbon and stop redrawing marks for events already cleared.
    void clearMarks();

  protected:
    void paintEvent(QPaintEvent*) override;

  private:
    Snapshot snap_;
    std::deque<std::pair<double, double>> marks_;  // wall-time, confidence
    double lastMarked_ = 0.0;        // avoids re-appending the whole ring
    double clearedThrough_ = 0.0;     // operator pressed Clear
};

}  // namespace radar
