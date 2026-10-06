#include "MainWindow.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QTime>
#include <QDateTime>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QTabBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QVBoxLayout>

#include <cmath>

#include "Plot.hpp"
#include "Theme.hpp"
#include "Widgets.hpp"

namespace radar {
namespace {

QWidget* metricTile(const QString& caption, QLabel** out) {
    auto* box = new QWidget;
    auto* v = new QVBoxLayout(box);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(1);
    auto* val = new QLabel("--");
    val->setProperty("role", "metric");
    auto* cap = new QLabel(caption);
    cap->setProperty("role", "metricUnit");
    v->addWidget(val);
    v->addWidget(cap);
    *out = val;
    return box;
}

inline QString qs(const std::string& s) { return QString::fromStdString(s); }

QSlider* slider(int lo, int hi, int val) {
    auto* s = new QSlider(Qt::Horizontal);
    s->setRange(lo, hi);
    s->setValue(val);
    return s;
}

}  // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle("RSSI RADAR — WiFi / Bluetooth radiometric motion sensing");
    resize(1560, 960);
    applyTheme();

    auto* central = new QWidget;
    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(10, 8, 10, 8);
    root->setSpacing(8);

    // ------------------------------------------------------------- header
    auto* header = new QHBoxLayout;
    auto* title = new QLabel("RSSI RADAR");
    title->setProperty("role", "h1");
    auto* sub = new QLabel("PASSIVE RADIOMETRIC SENSING");
    sub->setProperty("role", "h2");
    statusDot_ = new QLabel("●");
    statusDot_->setFixedWidth(16);
    statusDot_->setStyleSheet("color:#688C98; font-size:17px;");
    statusText_ = new QLabel("STOPPED");
    statusText_->setProperty("role", "h2");
    statusText_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // The status lives in a fixed-width container with a right-aligned label.
    // Without the reserved width the row re-flows every time the text changes
    // between "STOPPED" and "MOTION DETECTED", and the two labels collide.
    auto* statusBox = new QWidget;
    statusBox->setFixedWidth(240);
    auto* sb = new QHBoxLayout(statusBox);
    sb->setContentsMargins(0, 0, 0, 0);
    sb->setSpacing(8);
    sb->addStretch(1);
    statusText_->setMinimumWidth(196);
    sb->addWidget(statusDot_);
    sb->addWidget(statusText_);

    header->addWidget(title);
    header->addSpacing(12);
    header->addWidget(sub);
    header->addStretch(1);
    header->addWidget(statusBox);
    root->addLayout(header);

    verdict_ = new QLabel;
    verdict_->setWordWrap(true);
    verdict_->setStyleSheet("color:#FFBA42; padding:4px 6px;");
    root->addWidget(verdict_);

    // ------------------------------------------------------------- body
    auto* split = new QSplitter(Qt::Horizontal);

    auto* left = new QWidget;
    auto* lv = new QVBoxLayout(left);
    lv->setContentsMargins(0, 0, 0, 0);
    lv->setSpacing(8);

    scope_ = new RadarScope;
    auto* scopeBox = new QGroupBox("POLAR RANGE / BEARING");
    auto* sbl = new QVBoxLayout(scopeBox);
    sbl->addWidget(scope_);
    lv->addWidget(scopeBox, 3);

    timeseries_ = new TimeseriesWidget;
    auto* tsBox = new QGroupBox("RECEIVED POWER");
    auto* tbl = new QVBoxLayout(tsBox);
    tbl->addWidget(timeseries_);
    lv->addWidget(tsBox, 2);

    split->addWidget(left);

    auto* tabs = new QTabWidget;
    waterfall_ = new WaterfallWidget;
    auto* wfBox = new QGroupBox("DETECTION WATERFALL");
    auto* wbl = new QVBoxLayout(wfBox);
    wbl->addWidget(waterfall_);
    tabs->addTab(wfBox, QStringLiteral("Waterfall"));

    velocity_ = new VelocitySpectrumWidget;
    auto* vBox = new QGroupBox(QStringLiteral("ENVELOPE-BEAT SPECTRUM \u2192 RADIAL VELOCITY"));
    auto* vbl = new QVBoxLayout(vBox);
    vbl->addWidget(velocity_);
    tabs->addTab(vBox, QStringLiteral("Velocity"));

    tracks_ = new TrackTableWidget;
    auto* tBox = new QGroupBox(QStringLiteral("TRANSMITTERS \u2014 received power per source"));
    auto* tbb = new QVBoxLayout(tBox);
    tbb->addWidget(tracks_);
    tabs->addTab(tBox, QStringLiteral("Sources"));

    // Contacts and the statistical analysis live on their own tab: they are the
    // "what is actually out there" view, distinct from the raw per-source
    // levels, and squeezing them onto Sources made both unreadable.
    tracking_ = new TrackingPanelWidget;
    tracking_->setMinimumHeight(230);
    tabs->addTab(tracking_, QStringLiteral("Tracking"));

    // Life sign, direction and the channel signature.
    signature_ = new SignaturePanelWidget;
    tabs->addTab(signature_, QStringLiteral("Signature"));

    auto* eventsBox = new QWidget;
    auto* evl = new QVBoxLayout(eventsBox);
    evl->setContentsMargins(6, 6, 6, 6);
    evl->setSpacing(8);

    timeline_ = new EventTimelineWidget;
    timeline_->setMinimumHeight(300);
    evl->addWidget(timeline_, 2);

    // Log header with a Clear control. Previously the log was built into a
    // QGroupBox that was never added to any layout, so it existed as an orphan
    // top-level window with no scrollbar reachable from the main UI -- which is
    // why the text appeared to spill over the controls and could not be read.
    auto* logHead = new QHBoxLayout;
    auto* logTitle = new QLabel(QStringLiteral("EVENT LOG"));
    logTitle->setStyleSheet("color:#5E8496; font-size:10px; letter-spacing:1px;");
    logHead->addWidget(logTitle);
    logHead->addStretch(1);

    eventCount_ = new QLabel(QStringLiteral("0"));
    eventCount_->setStyleSheet("color:#5E8496; font-size:10px;");
    logHead->addWidget(eventCount_);

    clearEventsButton_ = new NeonButton(QStringLiteral("CLEAR"));
    clearEventsButton_->setToolTip(QStringLiteral("Clear the event history and timeline"));
    clearEventsButton_->setFixedHeight(24);
    clearEventsButton_->setMinimumWidth(72);
    connect(clearEventsButton_, &QPushButton::clicked, this, &MainWindow::onClearEvents);
    logHead->addWidget(clearEventsButton_);
    evl->addLayout(logHead);

    eventLog_ = new QPlainTextEdit;
    eventLog_->setReadOnly(true);
    eventLog_->setMaximumBlockCount(600);
    // Scrollable, and the wrap keeps long anomaly lines readable instead of
    // pushing a horizontal scrollbar across the panel.
    eventLog_->setLineWrapMode(QPlainTextEdit::WidgetWidth);
    eventLog_->setWordWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
    eventLog_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    eventLog_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    eventLog_->setFont(QFont("monospace", 9));
    evl->addWidget(eventLog_, 3);

    tabs->addTab(eventsBox, QStringLiteral("Events"));
    tabs_ = tabs;

    split->addWidget(tabs);

    auto* rightScroll = new QScrollArea;
    rightScroll->setWidgetResizable(true);
    rightScroll->setMinimumWidth(430);
    rightScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    rightScroll->setFrameShape(QFrame::NoFrame);
    auto* right = new QWidget;
    auto* rv = new QVBoxLayout(right);
    rv->setContentsMargins(0, 0, 10, 0);
    rv->setSpacing(8);
    // Everything in this panel wraps rather than being clipped; a clipped label
    // is worse than a taller one.
    right->setMinimumWidth(400);
    rv->addWidget(buildTelemetryPanel());
    rv->addWidget(buildSensorPanel());
    rv->addWidget(buildPresetPanel());
    rv->addWidget(buildControlPanel());
    rv->addWidget(buildAnchorPanel());
    notes_ = new QPlainTextEdit;
    notes_->setReadOnly(true);
    rv->addWidget(notes_);
    rv->addStretch(1);
    rightScroll->setWidget(right);
    split->addWidget(rightScroll);

    split->setStretchFactor(0, 3);
    split->setStretchFactor(1, 2);
    split->setStretchFactor(2, 0);
    root->addWidget(split, 1);

    setCentralWidget(central);

    // Hardware survey at startup: tell the truth immediately, before any
    // capture is attempted.
    const HardwareSurvey survey = engine_.hardware();
    QStringList notes;
    notes << "HARDWARE SURVEY";
    notes << "---------------";
    notes << QString("wifi device  : %1").arg(qs(survey.wifiDevice.empty() ? "(none)" : survey.wifiDevice));
    notes << QString("wifi driver  : %1").arg(qs(survey.wifiDriver));
    notes << QString("firmware     : %1").arg(qs(survey.wifiFirmware));
    notes << QString("reg domain   : %1%2")
                 .arg(qs(survey.regDomainCountry),
                      survey.regDomainPassiveScan ? QString("  [PASSIVE-SCAN]") : QString());
    notes << QString("monitor mode : %1").arg(survey.monitorCapable ? "supported" : "no");
    notes << QString("injection    : %1").arg(survey.injectionSupported ? "supported" : "no");
    notes << QString("userspace CSI: %1")
                 .arg(survey.csiCapable ? QString("vendor commands present")
                                        : QString("NO - no vendor commands"));
    notes << QString("BT long-range: %1").arg(survey.btLongRangePhy ? "yes" : "no (BT 4.0 controller)");
    notes << QString("BT AoA/AoD   : %1").arg(survey.btDirectionFinding ? "yes" : "no");
    for (const auto& b : survey.bluetooth)
        if (!b.detail.empty()) notes << QString("BT            : %1").arg(qs(b.detail));
    notes << QString("");
    notes << QString("NOTES");
    for (const auto& n : survey.notes) notes << QString("\u2022 ") + qs(n);
    notes_->setPlainText(notes.join("\n"));

    // The animation clock never stops: without it a QWidget only repaints when
    // something dirties it, so the radar scope sits frozen until the window is
    // moved. 30 fps is plenty for a 72 deg/s sweep and costs almost nothing.
    connect(&animTimer_, &QTimer::timeout, this, [this] {
        scope_->update();
        if (engine_.running()) onTick();
    });
    animTimer_.start(33);

    connect(&pipelineTimer_, &QTimer::timeout, this, &MainWindow::onTick);
    connect(startBtn_, &NeonButton::clicked, this, &MainWindow::onStart);
    connect(stopBtn_, &NeonButton::clicked, this, &MainWindow::onStop);
}

MainWindow::~MainWindow() {
    animTimer_.stop();
    pipelineTimer_.stop();
    engine_.stop();
}

void MainWindow::applyTheme() { qApp->setStyleSheet(theme::styleSheet()); }

QWidget* MainWindow::buildTelemetryPanel() {
    auto* box = new QGroupBox("TELEMETRY");
    auto* grid = new QGridLayout(box);
    grid->setSpacing(6);
    int r = 0, c = 0;
    const auto add = [&](QLabel** slot, const QString& cap) {
        grid->addWidget(metricTile(cap, slot), r, c);
        // Two columns instead of three: at a 430 px panel width three columns
        // clipped the captions ("TRACK SPEED", "CONFIDENCE").
        if (++c == 2) {
            c = 0;
            ++r;
        }
    };
    add(&mRange_, QStringLiteral("RANGE (m)"));
    add(&mBearing_, QStringLiteral("BEARING"));
    add(&mSpeed_, QStringLiteral("TRACK SPEED"));
    add(&mRssi_, QStringLiteral("RSSI (dBm)"));
    add(&mJitter_, QStringLiteral("JITTER RMS"));
    add(&mConfidence_, QStringLiteral("CONFIDENCE"));
    add(&mRate_, QStringLiteral("SAMPLE RATE"));
    add(&mVelocity_, QStringLiteral("RADIAL v"));
    add(&mStat_, QStringLiteral("STATISTIC"));
    grid->addWidget(metricTile(QStringLiteral("THRESHOLD"), &mThreshold_), r, c);
    return box;
}

QWidget* MainWindow::buildSensorPanel() {
    auto* box = new QGroupBox("SENSORS");
    auto* v = new QVBoxLayout(box);
    sensorTable_ = new QTableWidget(0, 4);
    sensorTable_->setHorizontalHeaderLabels({"SENSOR", "STATE", "RATE", "DETAIL"});
    sensorTable_->horizontalHeader()->setStretchLastSection(true);
    sensorTable_->verticalHeader()->setVisible(false);
    sensorTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    sensorTable_->setSelectionMode(QAbstractItemView::NoSelection);
    sensorTable_->setFocusPolicy(Qt::NoFocus);
    sensorTable_->setMinimumHeight(170);
    sensorTable_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    sensorTable_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    sensorTable_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    sensorTable_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
    sensorTable_->setWordWrap(true);
    sensorTable_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    v->addWidget(sensorTable_);
    return box;
}

QWidget* MainWindow::buildPresetPanel() {
    auto* box = new QGroupBox("PRESETS");
    auto* v = new QVBoxLayout(box);
    presetBox_ = new QComboBox;
    const auto presets = engine_.presets();
    for (const auto& p : presets) presetBox_->addItem(QString::fromStdString(p.name));
    connect(presetBox_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &MainWindow::onPresetChanged);
    v->addWidget(presetBox_);

    auto* desc = new QLabel(QString::fromStdString(presets.empty() ? "" : presets[0].description));
    desc->setWordWrap(true);
    desc->setMinimumWidth(340);
    desc->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    desc->setTextInteractionFlags(Qt::TextSelectableByMouse);
    desc->setStyleSheet("color:#688C98; font-size:10px;");
    desc->setObjectName("presetDesc");
    v->addWidget(desc);
    connect(presetBox_, &QComboBox::currentTextChanged, this,
            [this, presets](const QString& name) {
                for (const auto& p : presets)
                    if (QString::fromStdString(p.name) == name)
                        if (auto* d = findChild<QLabel*>("presetDesc"))
                            d->setText(QString::fromStdString(p.description));
            });

    auto* row = new QHBoxLayout;
    auto* save = new NeonButton("Save config");
    save->setAccent(theme::cyan());
    auto* load = new NeonButton("Load config");
    load->setAccent(theme::cyan());
    connect(save, &QPushButton::clicked, this, &MainWindow::onSaveConfig);
    connect(load, &QPushButton::clicked, this, &MainWindow::onLoadConfig);
    row->addWidget(save);
    row->addWidget(load);
    v->addLayout(row);
    return box;
}

QWidget* MainWindow::buildControlPanel() {
    auto* box = new QGroupBox("CONTROL");
    auto* v = new QVBoxLayout(box);
    auto* form = new QFormLayout;

    auto* topRow = new QHBoxLayout;
    startBtn_ = new NeonButton("START SENSING");
    startBtn_->setPrimary(true);
    stopBtn_ = new NeonButton("STOP");
    stopBtn_->setDanger(true);
    stopBtn_->setEnabled(false);  // nothing to stop yet
    topRow->addWidget(startBtn_);
    topRow->addWidget(stopBtn_);

    auto* selftest = new NeonButton("Hardware probe");
    selftest->setAccent(theme::violet());
    connect(selftest, &NeonButton::clicked, this, &MainWindow::onSelftest);
    topRow->addWidget(selftest);
    v->addLayout(topRow);

    rangeSlider_ = slider(2, 60, 12);
    rangeSlider_->setToolTip("PPI full-scale range, metres");
    connect(rangeSlider_, &QSlider::valueChanged, this, &MainWindow::onRangeChanged);
    form->addRow("Scope range (m)", rangeSlider_);

    // Trace weight: scales every graph line and its glow together.
    traceSlider_ = slider(25, 150, static_cast<int>(plot::traceScale() * 100.0));
    traceSlider_->setToolTip("Thickness of every trace on the graph views");
    connect(traceSlider_, &QSlider::valueChanged, this, [this](int v) {
        plot::setTraceScale(v / 100.0);
        if (scope_) scope_->update();
        if (timeseries_) timeseries_->update();
        if (waterfall_) waterfall_->update();
        if (velocity_) velocity_->update();
        if (tracks_) tracks_->update();
        if (timeline_) timeline_->update();
    });
    form->addRow("Trace weight", traceSlider_);

    windowSlider_ = slider(1, 60, 4);
    connect(windowSlider_, &QSlider::valueChanged, this, &MainWindow::onTuneChanged);
    form->addRow("Window (s)", windowSlider_);

    thresholdSlider_ = slider(5, 120, 30);
    connect(thresholdSlider_, &QSlider::valueChanged, this, &MainWindow::onTuneChanged);
    form->addRow("Threshold (σ/10)", thresholdSlider_);

    decaySlider_ = slider(2, 200, 28);
    connect(decaySlider_, &QSlider::valueChanged, this, &MainWindow::onTuneChanged);
    form->addRow("Path-loss exponent", decaySlider_);

    exponentSpin_ = new QDoubleSpinBox;
    exponentSpin_->setRange(1.2, 6.0);
    exponentSpin_->setSingleStep(0.1);
    exponentSpin_->setValue(2.8);
    connect(exponentSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &MainWindow::onTuneChanged);
    form->addRow("PL exponent", exponentSpin_);

    shadowSpin_ = new QDoubleSpinBox;
    shadowSpin_->setRange(0.1, 20.0);
    shadowSpin_->setSingleStep(0.1);
    shadowSpin_->setValue(3.0);
    connect(shadowSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &MainWindow::onTuneChanged);
    form->addRow("Shadow σ (dB)", shadowSpin_);

    particlesSpin_ = new QSpinBox;
    particlesSpin_->setRange(128, 20000);
    particlesSpin_->setSingleStep(256);
    particlesSpin_->setValue(2000);
    connect(particlesSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
            &MainWindow::onTuneChanged);
    form->addRow("Particles", particlesSpin_);

    v->addLayout(form);

    auto* checks = new QHBoxLayout;
    gridChk_ = new QCheckBox("Grid");
    gridChk_->setChecked(true);
    trailsChk_ = new QCheckBox("Trails");
    trailsChk_->setChecked(true);
    entropyChk_ = new QCheckBox("Entropy");
    entropyChk_->setChecked(true);
    chiChk_ = new QCheckBox("χ²");
    chiChk_->setChecked(true);
    checks->addWidget(gridChk_);
    checks->addWidget(trailsChk_);
    checks->addWidget(entropyChk_);
    checks->addWidget(chiChk_);
    v->addLayout(checks);

    auto* adv = new NeonButton("Start LE reference advertisement");
    adv->setAccent(theme::amber());
    connect(adv, &QPushButton::clicked, this, &MainWindow::onReferenceAdvert);
    v->addWidget(adv);
    return box;
}

QWidget* MainWindow::buildAnchorPanel() {
    auto* box = new QGroupBox("ANCHORS (for trilateration)");
    auto* v = new QVBoxLayout(box);
    anchorBox_ = new QComboBox;
    v->addWidget(anchorBox_);
    auto* row = new QHBoxLayout;
    auto* addBtn = new NeonButton("Add");
    addBtn->setAccent(theme::green());
    connect(addBtn, &QPushButton::clicked, this, &MainWindow::onAddAnchor);
    row->addWidget(addBtn);
    v->addLayout(row);
    auto* hint = new QLabel(
        "Without anchors the filter still tracks range and rate of change along the "
        "line of sight. Add 3+ known transmitter positions to get a full 2D fix.");
    hint->setWordWrap(true);
    hint->setMinimumWidth(340);
    hint->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    hint->setStyleSheet("color:#688C98; font-size:10px;");
    v->addWidget(hint);

    const Config c = engine_.config();
    for (size_t i = 0; i < c.anchors.size(); ++i)
        anchorBox_->addItem(QString("%1  (%2, %3) m")
                                .arg(QString::fromStdString(c.anchors[i].id))
                                .arg(c.anchors[i].x, 0, 'f', 1)
                                .arg(c.anchors[i].y, 0, 'f', 1));
    return box;
}

void MainWindow::refreshSensorPanel() {
    const Snapshot s = engine_.snapshot();
    sensorTable_->setRowCount(static_cast<int>(s.sensors.size()));
    for (int i = 0; i < static_cast<int>(s.sensors.size()); ++i) {
        const auto& st = s.sensors[static_cast<size_t>(i)];
        auto set = [&](int col, const QString& text, const QString& colour = QString()) {
            auto* item = new QTableWidgetItem(text);
            if (!colour.isEmpty()) {
                item->setForeground(QColor(colour));
            }
            sensorTable_->setItem(i, col, item);
        };
        set(0, QString::fromStdString(st.name), "#C4E0E8");
        if (st.active) set(1, "ACTIVE", "#60FFA8");
        else if (st.available) set(1, "IDLE", "#FFBA42");
        else set(1, "OFF", "#FF5656");
        set(2, st.rateHz > 0 ? QString::number(st.rateHz, 'f', 0) + " Hz" : "—");
        // Wrap rather than truncate: the detail string is where the honest
        // failure reasons live, so cutting it off hides the diagnosis.
        set(3, QString::fromStdString(st.detail));
    }
}

void MainWindow::refreshTelemetry(const Snapshot& s) {
    mRange_->setText(s.fusion.ok || s.primaryRangeM > 0
                          ? QString::number(s.primaryRangeM, 'f', 2)
                          : QString("--"));
    mBearing_->setText(s.fusion.ok ? QString::number(s.fusion.bearingDeg, 'f', 0) + "°"
                                   : QString("--"));
    mSpeed_->setText(s.fusion.ok ? QString::number(s.fusion.speedMps, 'f', 3) : QString("--"));
    mRssi_->setText(s.primaryMeanDbm < -99.0 ? QString("--")
                                              : QString::number(s.primaryMeanDbm, 'f', 1));
    mJitter_->setText(s.primaryStdDbm < 1e-9 ? QString("--")
                                             : QString::number(s.primaryStdDbm, 'f', 2));
    mConfidence_->setText(QString::number(s.detection.confidence * 100.0, 'f', 0) + "%");
    mRate_->setText(QString::number(s.effectiveSampleRateHz, 'f', 0));
    mVelocity_->setText(s.velocityValid ? QString::number(s.radialVelocityMps, 'f', 3)
                                         : QString("--"));
    mStat_->setText(s.detection.statistic > 0 ? QString::number(s.detection.statistic, 'f', 3)
                                              : QString("--"));
    mThreshold_->setText(s.detection.threshold > 0
                             ? QString::number(s.detection.threshold, 'f', 3)
                             : QString("--"));

    mVelocity_->setStyleSheet(
        s.velocityValid ? "color:#FF5CAA;" : "");
    mConfidence_->setStyleSheet(
        s.detection.state == DetectionState::Present ? "color:#FF5656;" : "");

    if (s.starved) {
        verdict_->setText("⚠ " + QString::fromStdString(s.starveReason));
        verdict_->setStyleSheet("color:#FF5656; padding:4px 6px;");
    } else {
        verdict_->clear();
    }
}

void MainWindow::onTick() {
    engine_.tick();
    const Snapshot s = engine_.snapshot();

    scope_->setSnapshot(s);
    timeseries_->setSnapshot(s);
    waterfall_->setSnapshot(s);
    velocity_->setSnapshot(s);
    tracks_->setSnapshot(s);
    if (timeline_) timeline_->setSnapshot(s);
    if (tracking_) tracking_->setSnapshot(s);
    if (signature_) signature_->setSnapshot(s);

    scope_->setShowGrid(gridChk_->isChecked());
    scope_->setShowTrails(trailsChk_->isChecked());

    refreshTelemetry(s);
    refreshSensorPanel();

    const bool present = s.detection.state == DetectionState::Present;
    statusDot_->setStyleSheet(present ? "color:#FF5656; font-size:17px;"
                                      : "color:#60FFA8; font-size:17px;");
    statusText_->setText(present ? "MOTION DETECTED"
                                 : (engine_.running() ? "MONITORING" : "STOPPED"));
    statusText_->setStyleSheet(present ? "color:#FF5656;" : "color:#60FFA8;");

    // Push new events into the log.
    //
    // Tracked by index into the engine's ring, not by block count: the log caps
    // at 600 blocks while the engine caps at 400 events, so comparing the two
    // meant the comparison silently stopped matching once enough events had
    // accumulated and lines stopped appearing.
    if (eventsShown_ > s.events.size()) eventsShown_ = 0;  // engine reset
    for (size_t i = eventsShown_; i < s.events.size(); ++i) {
        const auto& e = s.events[i];
        const auto secs = static_cast<qint64>(std::floor(e.wallTime));
        const auto ms = static_cast<qint64>((e.wallTime - std::floor(e.wallTime)) * 1000.0);
        const QDateTime dt = QDateTime::fromSecsSinceEpoch(secs);

        QString line = dt.toString(QStringLiteral("HH:mm:ss")) +
                       QStringLiteral(".%1").arg(ms, 3, 10, QChar('0')) +
                       QStringLiteral("  ") + QString::fromStdString(e.text);

        // Range on the same line as the timestamp, with the interval that the
        // shadowing uncertainty actually implies rather than a bare number.
        if (e.rangeValid && e.rangeM > 0.0)
            line += QStringLiteral("  ·  %1 m  [%2–%3 m]")
                        .arg(e.rangeM, 0, 'f', 1)
                        .arg(e.rangeLoM, 0, 'f', 1)
                        .arg(e.rangeHiM, 0, 'f', 1);
        if (e.rssiDbm > -100.0)
            line += QStringLiteral("  ·  %1 dBm").arg(e.rssiDbm, 0, 'f', 1);
        if (e.velocityValid)
            line += QStringLiteral("  ·  %1 m/s").arg(e.velocityMps, 0, 'f', 3);
        if (e.contactId != 0)
            line += QStringLiteral("  ·  #%1").arg(e.contactId);

        eventLog_->appendPlainText(line);
    }
    eventsShown_ = s.events.size();
    if (eventCount_) eventCount_->setText(QString::number(eventsShown_));
}

void MainWindow::onClearEvents() {
    // Clears the visible history and re-syncs the index so the next engine event
    // is appended rather than the whole ring being replayed.
    if (eventLog_) eventLog_->clear();
    if (eventCount_) eventCount_->setText(QStringLiteral("0"));
    eventsShown_ = engine_.snapshot().events.size();
    if (timeline_) timeline_->clearMarks();
    if (notes_) notes_->appendPlainText(
        QStringLiteral("[%1]  history cleared by operator")
            .arg(QTime::currentTime().toString(QStringLiteral("HH:mm:ss"))));
}

void MainWindow::onStart() {
    if (engine_.running()) return;

    // Raw capture needs CAP_NET_RAW. Try, and say precisely why if it fails.
    std::string err;
    if (!engine_.start(&err)) {
        QMessageBox::warning(
            this, "Could not start",
            QString("No sensor started.\n\n%1\n\n"
                    "Radiometric capture needs root or CAP_NET_RAW:\n"
                    "    sudo setcap cap_net_raw,cap_net_admin+eip /usr/bin/rssiradar")
                .arg(QString::fromStdString(err.empty() ? "unknown error" : err)));
        pushEvent("START FAILED: " + QString::fromStdString(err), "#FF5656");
        return;
    }
    engine_.resetTracking();
    startBtn_->setEnabled(false);
    stopBtn_->setEnabled(true);
    const double hz = std::clamp(engine_.config().ui.refreshHz, 1.0, 60.0);
    pipelineTimer_.start(static_cast<int>(1000.0 / hz));
    pushEvent("SENSING STARTED", "#60FFA8");
}

void MainWindow::onStop() {
    if (!engine_.running()) return;
    engine_.stop();
    pipelineTimer_.stop();
    startBtn_->setEnabled(true);
    stopBtn_->setEnabled(false);
    statusText_->setText("STOPPED");
    statusDot_->setStyleSheet("color:#688C98; font-size:17px;");
    verdict_->clear();
    pushEvent("SENSING STOPPED", "#FFBA42");
}

void MainWindow::onPresetChanged(int index) {
    const auto presets = engine_.presets();
    if (index < 0 || index >= static_cast<int>(presets.size())) return;
    const Preset& p = presets[static_cast<size_t>(index)];
    tuning_ = true;
    engine_.applyPreset(p.name);
    const Config c = engine_.config();
    rangeSlider_->blockSignals(true);
    rangeSlider_->setValue(static_cast<int>(c.ui.rangeRingMetres));
    rangeSlider_->blockSignals(false);
    scope_->setRangeMetres(c.ui.rangeRingMetres);
    windowSlider_->setValue(static_cast<int>(c.window.windowSeconds));
    thresholdSlider_->setValue(static_cast<int>(c.detector.baseThresholdSigma * 10.0));
    decaySlider_->setValue(static_cast<int>(c.pathLoss.pathLossExponent * 50.0));
    exponentSpin_->setValue(c.pathLoss.pathLossExponent);
    shadowSpin_->setValue(c.pathLoss.shadowSigmaDb);
    particlesSpin_->setValue(c.estimator.particleCount);
    entropyChk_->setChecked(c.detector.useEntropy);
    chiChk_->setChecked(c.detector.useChiSquare);
    tuning_ = false;
    pushEvent("PRESET: " + QString::fromStdString(p.name), "#40E0FF");
}

void MainWindow::onRangeChanged(int value) {
    scope_->setRangeMetres(value);
    Config c = engine_.config();
    c.ui.rangeRingMetres = value;
    engine_.setConfig(c);
}

void MainWindow::onTuneChanged() {
    if (tuning_) return;
    Config c = engine_.config();
    c.window.windowSeconds = windowSlider_->value();
    c.detector.baseThresholdSigma = thresholdSlider_->value() / 10.0;
    c.pathLoss.pathLossExponent = decaySlider_->value() / 50.0;
    c.pathLoss.pathLossExponent = exponentSpin_->value();
    c.pathLoss.shadowSigmaDb = shadowSpin_->value();
    c.estimator.particleCount = particlesSpin_->value();
    c.detector.useEntropy = entropyChk_->isChecked();
    c.detector.useChiSquare = chiChk_->isChecked();
    c.presetName = "Custom";
    engine_.setConfig(c);
}

void MainWindow::onSaveConfig() {
    const QString path = QFileDialog::getSaveFileName(this, "Save configuration",
                                                      "radar-config.json", "JSON (*.json)");
    if (path.isEmpty()) return;
    const Config c = engine_.config();
    std::string err;
    if (c.save(path.toStdString(), &err))
        pushEvent("config saved: " + path, "#60FFA8");
    else
        pushEvent("save failed: " + QString::fromStdString(err), "#FF5656");
}

void MainWindow::onLoadConfig() {
    const QString path = QFileDialog::getOpenFileName(this, "Load configuration", "",
                                                      "JSON (*.json)");
    if (path.isEmpty()) return;
    std::string err;
    const Config c = Config::load(path.toStdString(), &err);
    if (!err.empty()) {
        pushEvent("load failed: " + QString::fromStdString(err), "#FF5656");
        return;
    }
    engine_.setConfig(c);
    pushEvent("config loaded: " + path, "#60FFA8");
}

void MainWindow::onAddAnchor() {
    Anchor a;
    a.id = "anchor-" + std::to_string(engine_.config().anchors.size() + 1);
    a.x = 4.0;
    a.y = 4.0;
    a.radio = RadioKind::Wifi;
    engine_.addAnchor(a);
    anchorBox_->addItem(QString::fromStdString(a.id));
    pushEvent("anchor added: " + QString::fromStdString(a.id), "#40E0FF");
}

void MainWindow::onReferenceAdvert() {
    const bool ok = engine_.startReferenceAdvert();
    pushEvent(ok ? "LE reference advertisement started (100 ms)"
                 : "cannot advertise: Bluetooth sensor not running",
              ok ? "#60FFA8" : "#FF5656");
}

void MainWindow::onSelftest() {
    const HardwareSurvey survey = engine_.hardware();
    QString text;
    text += "=== WIFI ===\n";
    text += QString("device      : %1\n").arg(qs(survey.wifiDevice));
    text += QString("driver      : %1\n").arg(qs(survey.wifiDriver));
    text += QString("firmware    : %1\n").arg(qs(survey.wifiFirmware));
    text += QString("reg domain  : %1%2\n")
                .arg(qs(survey.regDomainCountry),
                     survey.regDomainPassiveScan ? QString("  PASSIVE-SCAN") : QString());
    text += QString("monitor     : %1\n").arg(survey.monitorCapable ? "yes" : "no");
    text += QString("injection   : %1\n").arg(survey.injectionSupported ? "yes" : "no");
    text += QString("userspace CSI: %1\n").arg(survey.csiCapable ? "yes" : "NO");
    text += "\n=== BLUETOOTH ===\n";
    for (const auto& b : survey.bluetooth) {
        text += QString("device      : %1\n").arg(qs(b.device));
        text += QString("detail      : %1\n").arg(qs(b.detail));
        for (const auto& cap : b.capabilities) text += QString("  - %1\n").arg(qs(cap));
    }
    if (!survey.notes.empty()) {
        text += "\n=== NOTES ===\n";
        for (const auto& n : survey.notes) text += QString("\u2022 ") + qs(n) + QString("\n");
    }
    QMessageBox box(this);
    box.setWindowTitle("Hardware probe");
    box.setText("Raw hardware report (read from sysfs/netlink, not cached)");
    box.setDetailedText(text);
    box.exec();
}

void MainWindow::startSensingForTest() { onStart(); }

void MainWindow::selectTab(int index) {
    if (tabs_) tabs_->setCurrentIndex(index);
}
void MainWindow::setStatusForTest(const QString& text, const QString& colour) {
    statusText_->setText(text);
    statusDot_->setStyleSheet(QString("color:%1; font-size:17px;").arg(colour));
}

void MainWindow::pushEvent(const QString& text, const QString& colour) {
    const QString stamp = QTime::currentTime().toString("HH:mm:ss.zzz");
    eventLog_->appendPlainText(QString("%1  %2").arg(stamp, text));
    if (!colour.isEmpty()) {
        // Colour is applied via rich text on the last block.
        QTextCursor c = eventLog_->textCursor();
        c.movePosition(QTextCursor::End);
        QTextCharFormat f;
        f.setForeground(QColor(colour));
        c.setCharFormat(f);
    }
}

}  // namespace radar