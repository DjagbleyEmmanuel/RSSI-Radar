#pragma once

#include <QMainWindow>
#include <QTimer>

#include <atomic>
#include <memory>
#include <thread>

#include "radar/Engine.hpp"

class QComboBox;
class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QTableWidget;
class QTabWidget;   // Qt global

namespace radar {

class NeonButton;   // radar::NeonButton, defined in Widgets.hpp
class RadarScope;
class TimeseriesWidget;
class WaterfallWidget;
class VelocitySpectrumWidget;
class TrackTableWidget;
class TrackingPanelWidget;
class SignaturePanelWidget;
class EventTimelineWidget;

class MainWindow : public QMainWindow {
    Q_OBJECT
  public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

  private slots:
    void onTick();
    void onStart();
    void onStop();
    void onClearEvents();
    void onPresetChanged(int index);
    void onRangeChanged(int value);
    void onTuneChanged();
    void onSaveConfig();
    void onLoadConfig();
    void onAddAnchor();
    void onReferenceAdvert();
    void onSelftest();
  public slots:
    // Used by --screenshot to capture a specific tab and status string.
    void selectTab(int index);
    void setStatusForTest(const QString& text, const QString& colour);
    void startSensingForTest();

  private:
    QWidget* buildLeftColumn();
    QWidget* buildRightColumn();
    QWidget* buildSensorPanel();
    QWidget* buildTelemetryPanel();
    QWidget* buildControlPanel();
    QWidget* buildPresetPanel();
    QWidget* buildAnchorPanel();
    void refreshSensorPanel(const Snapshot& s);
    void refreshTelemetry(const Snapshot& s);
    void pushEvent(const QString& text, const QString& colour = QString());
    void applyTheme();

    Engine engine_;

    RadarScope* scope_ = nullptr;
    TimeseriesWidget* timeseries_ = nullptr;
    WaterfallWidget* waterfall_ = nullptr;
    VelocitySpectrumWidget* velocity_ = nullptr;
    TrackTableWidget* tracks_ = nullptr;
    EventTimelineWidget* timeline_ = nullptr;
    TrackingPanelWidget* tracking_ = nullptr;
    SignaturePanelWidget* signature_ = nullptr;

    QLabel* statusDot_ = nullptr;
    QLabel* statusText_ = nullptr;
    QLabel* verdict_ = nullptr;
    QPlainTextEdit* eventLog_ = nullptr;
    QLabel* eventCount_ = nullptr;
    NeonButton* clearEventsButton_ = nullptr;
    size_t eventsShown_ = 0;
    QTableWidget* sensorTable_ = nullptr;
    QPlainTextEdit* notes_ = nullptr;

    QLabel* mRange_ = nullptr;
    QLabel* mBearing_ = nullptr;
    QLabel* mSpeed_ = nullptr;
    QLabel* mRssi_ = nullptr;
    QLabel* mJitter_ = nullptr;
    QLabel* mConfidence_ = nullptr;
    QLabel* mRate_ = nullptr;
    QLabel* mVelocity_ = nullptr;
    QLabel* mStat_ = nullptr;
    QLabel* mThreshold_ = nullptr;

    QComboBox* presetBox_ = nullptr;
    QTabWidget* tabs_ = nullptr;
    NeonButton* startBtn_ = nullptr;
    NeonButton* stopBtn_ = nullptr;
    QSlider* rangeSlider_ = nullptr;
    QSlider* traceSlider_ = nullptr;
    QSlider* windowSlider_ = nullptr;
    QSlider* thresholdSlider_ = nullptr;
    QSlider* decaySlider_ = nullptr;
    QDoubleSpinBox* exponentSpin_ = nullptr;
    QDoubleSpinBox* shadowSpin_ = nullptr;
    QSpinBox* particlesSpin_ = nullptr;
    QCheckBox* gridChk_ = nullptr;
    QCheckBox* trailsChk_ = nullptr;
    QCheckBox* entropyChk_ = nullptr;
    QCheckBox* chiChk_ = nullptr;
    QComboBox* anchorBox_ = nullptr;

    // Two independent clocks. `animTimer_` runs unconditionally so the scope
    // keeps sweeping and the buttons keep easing even when no capture is
    // running; `pipelineTimer_` only drives the estimator while sensing.
    QTimer animTimer_;
    QTimer pipelineTimer_;
    QTimer startPoll_;
    // Signature of the last sensor-panel content, so it is only rebuilt when it
    // actually differs.
    QString sensorPanelSig_;
    bool tuning_ = false;

    // Starting the sensors is slow and unavoidably blocking: releasing
    // NetworkManager, converting the interface to monitor mode and surveying
    // the band all happen inside Engine::start(). That used to run on the GUI
    // thread, so pressing Start sensing froze the window for as long as it took
    // -- tens of seconds. It now runs on a worker and the result is delivered
    // back on the GUI thread.
    std::thread startThread_;
    std::atomic<bool> startBusy_{false};
    std::atomic<bool> startCancel_{false};
    // Set when the worker finishes, read by pollStartFinished() on the GUI
    // thread, which is also what reapplies the button states.
    bool startDone_ = false;
    bool startOk_ = false;
    std::string startErr_;
    void pollStartFinished();
    void finishStart(bool ok, std::string err);
    // Joins a worker that is still running. Called from the destructor and from
    // Stop, so a start in flight can never outlive the engine it is starting.
    void joinStartWorker();
};

}  // namespace radar