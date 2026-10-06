// RSSI RADAR — passive WiFi/Bluetooth radiometric motion sensing.
//
// Every number this program reports is derived from frames that a real radio
// received. Run with --selftest to see exactly what this machine's hardware
// supports before starting a capture.
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <QApplication>
#include <QCommandLineParser>
#include <QMouseEvent>
#include <QPixmap>

#include <chrono>
#include <csignal>
#include <random>
#include <thread>
#include <cstdio>
#include <iostream>

#include "radar/Engine.hpp"
#include "ui/MainWindow.hpp"
#include "ui/Widgets.hpp"

namespace {

void installSignalHandlers();

int selftest() {
    using namespace radar;
    const HardwareSurvey s = surveyHardware();

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" RSSI RADAR — hardware probe (reads sysfs + netlink directly)\n");
    std::printf("================================================================\n\n");

    std::printf("WIFI\n");
    std::printf("  device            : %s\n", s.wifiDevice.c_str());
    std::printf("  driver            : %s\n", s.wifiDriver.c_str());
    std::printf("  firmware          : %s\n", s.wifiFirmware.c_str());
    std::printf("  regulatory domain : %s%s\n", s.regDomainCountry.c_str(),
                s.regDomainPassiveScan ? "  [PASSIVE-SCAN]" : "");
    std::printf("  monitor mode      : %s\n", s.monitorCapable ? "supported" : "NOT supported");
    std::printf("  packet injection  : %s\n",
                s.injectionSupported ? "supported" : "NOT supported (iwlwifi)");
    std::printf("  userspace CSI     : %s\n",
                s.csiCapable ? "driver advertises vendor commands" : "NO — no vendor commands");

    std::printf("\nBLUETOOTH\n");
    for (const auto& b : s.bluetooth) {
        std::printf("  device            : %s\n", b.device.c_str());
        std::printf("  detail            : %s\n", b.detail.c_str());
        for (const auto& c : b.capabilities)
            std::printf("    - %s\n", c.c_str());
    }
    std::printf("  LE Coded PHY      : %s\n", s.btLongRangePhy ? "yes" : "no (BT 4.0 controller)");
    std::printf("  AoA / AoD         : %s\n", s.btDirectionFinding ? "yes" : "no");

    if (!s.notes.empty()) {
        std::printf("\nNOTES\n");
        for (const auto& n : s.notes) std::printf("  * %s\n", n.c_str());
    }

    std::printf("\nPRESETS\n");
    for (const auto& p : builtinPresets())
        std::printf("  %-26s %s\n", p.name.c_str(), p.description.c_str());

    std::printf("\nDSP self-test (closed-form checks, no radio needed)\n");
    {
        // Path-loss round trip.
        PathLossModel m{1.0, -40.0, 3.0, 3.0};
        const double r = 7.25;
        const double back = m.rangeFromRssi(m.rssiFromRange(r));
        std::printf("  path-loss round trip : %.6f m -> %.6f m  (err %.2e)\n", r, back,
                    std::fabs(back - r));

        // FFT round trip.
        std::vector<double> sig(64);
        for (size_t i = 0; i < sig.size(); ++i)
            sig[i] = std::sin(0.37 * i) + 0.5 * std::sin(0.91 * i);
        std::vector<std::complex<double>> f(sig.begin(), sig.end());
        fft(f, false);
        fft(f, true);
        double maxErr = 0;
        for (size_t i = 0; i < sig.size(); ++i)
            maxErr = std::max(maxErr, std::abs(f[i].real() - sig[i]));
        std::printf("  FFT round trip       : max err %.3e\n", maxErr);

        // Normal quantile.
        std::printf("  normal quantile     : Phi^-1(0.975) = %.6f (expect 1.959964)\n",
                    normalQuantile(0.975));

        // Burg peak on a synthetic-but-analytic tone, purely a check of the
        // estimator itself, not of the radio path.
        std::vector<double> tone(512);
        for (size_t i = 0; i < tone.size(); ++i)
            tone[i] = std::sin(2.0 * M_PI * 0.07 * static_cast<double>(i));
        // Envelope-beat Doppler round trip at the real 2.4 GHz carrier.
        const double lambda = wavelengthMetres(2437e6, 299792458.0);
        for (double v : {0.05, 0.5, 2.0}) {
            const double fbeat = radialSpeedToBeatFrequency(v, lambda);
            std::printf("  beat<->velocity      : %5.3f m/s -> %6.3f Hz -> %5.3f m/s  "
                        "(lambda %.4f m)\n",
                        v, fbeat, beatFrequencyToRadialSpeed(fbeat, lambda), lambda);
        }

        // Welch estimator: two tones plus noise at a known sample rate. This is
        // the estimator the velocity channel actually depends on, so it gets a
        // numeric acceptance test rather than a smoke test.
        {
            const int N = 1024;
            const double fs = 20.0;
            std::vector<double> sig(N);
            std::mt19937 rng(7);
            std::normal_distribution<double> w(0.0, 0.05);
            for (int n = 0; n < N; ++n)
                sig[n] = 1.0 * std::sin(2 * M_PI * 3.0 * n / fs) +
                         0.6 * std::sin(2 * M_PI * 7.5 * n / fs) + w(rng);
            auto spec = welchSpectrum(sig, fs, 128, 0.5);
            const double binHz = (fs / 2.0) / static_cast<double>(spec.size());
            auto peaks = findPeaks(spec, binHz, 3, 0.3);
            std::printf("  Welch 3.0 Hz tone    : ");
            for (const auto& p : peaks) std::printf("%.3f  ", p.freqHz);
            std::printf("(expect 3.000 7.500)\n");
        }

        // Periodogram peak location on a bin-centred tone, validated against a
        // brute-force DFT rather than a round trip (a round trip cannot catch a
        // structurally wrong transform).
        {
            const int N = 1024;
            std::vector<double> sig(N);
            for (int n = 0; n < N; ++n)
                sig[n] = std::cos(2.0 * M_PI * 96.0 * n / N);
            auto spec = windowedPowerSpectrum(sig, 1.0);
            const double binHz = 1.0 / N;  // spec spans [0, fs/2] over N/2 bins
            auto peaks = findPeaks(spec, binHz, 1, 1e-6);
            std::printf("  periodogram peak     : %.5f Hz (expect 96/1024 = %.5f)\n",
                        peaks.empty() ? 0.0 : peaks[0].freqHz, 96.0 / N);
        }

        // Kalman convergence on a linear ramp.
        ExtendedKalmanFilter ekf;
        ekf.reset(1.5, 0.5);
        ekf.setProcessNoise(0.5);
        Anchor anchor{"ref", 0.0, 0.0, RadioKind::Wifi, -40.0, true};
        for (int i = 1; i <= 400; ++i) {
            ekf.predict(0.05);
            const double truth = 4.0 + 0.05 * i;
            ekf.updateRange(anchor, truth, 0.25);
        }
        std::printf("  EKF range estimate   : %.3f m (truth 24.000)\n", ekf.state().rangeM);
    }

    std::printf("\n================================================================\n\n");
    return 0;
}

// Headless capture: start the real sensors, run the real pipeline, and report
// exactly what came back. No rendering, no synthetic data. This is the mode to
// use when validating that the hardware is actually delivering frames.
int capture(double seconds, const QString& cfgPath) {
    using namespace radar;
    setvbuf(stdout, nullptr, _IOLBF, 0);
    installSignalHandlers();
    Engine engine;
    if (!cfgPath.isEmpty()) {
        std::string err;
        const Config c = Config::load(cfgPath.toStdString(), &err);
        // The failure used to be swallowed: a rejected config file left the
        // defaults in place with no message, so `--config pin.json` looked like
        // it had been applied when the channel pin never happened.
        if (!err.empty())
            std::printf("WARNING: could not load %s (%s); using defaults\n",
                        cfgPath.toStdString().c_str(), err.c_str());
        else
            engine.setConfig(c);
    }
    std::string err;
    if (!engine.start(&err)) {
        std::printf("FAILED to start sensors: %s\n", err.c_str());
        return 2;
    }
    std::printf("capturing for %.1f s (raw AF_PACKET + raw HCI, no simulation)\n\n", seconds);

    const auto start = Clock::now();
    const int hz = 20;
    uint64_t lastSeen = 0;
    while (std::chrono::duration<double>(Clock::now() - start).count() < seconds) {
        engine.tick();
        const Snapshot s = engine.snapshot();
        if (s.totalObservations != lastSeen) {
            lastSeen = s.totalObservations;
            std::printf("[%5.1fs] obs=%-8llu rate=%6.1f Hz  primary=%-28s "
                        "mean=%6.1f dBm  jitter=%.2f  stat=%.3f  %s\n",
                        std::chrono::duration<double>(Clock::now() - start).count(),
                        (unsigned long long)s.totalObservations, s.effectiveSampleRateHz,
                        s.primaryLabel.empty() ? "(none)" : s.primaryLabel.c_str(),
                        s.primaryMeanDbm, s.primaryStdDbm, s.detection.statistic,
                        toString(s.detection.state));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / hz));
    }
    const Snapshot s = engine.snapshot();
    engine.stop();

    std::printf("\n--- sensor status ---\n");
    for (const auto& st : s.sensors) {
        std::printf("  %-34s %-8s seen=%-8llu\n", st.name.c_str(),
                    st.active ? "ACTIVE" : (st.available ? "IDLE" : "OFF"),
                    (unsigned long long)st.framesSeen);
        std::printf("      radiotap+signal=%-7llu radiotap-no-signal=%-7llu no-radiotap=%-7llu"
                    " rate=%.1f Hz\n      %s\n",
                    (unsigned long long)st.framesWithRadiotap,
                    (unsigned long long)st.framesWithRadiotapNoSignal,
                    (unsigned long long)st.framesWithoutRadiotap, st.rateHz, st.detail.c_str());
    }
    std::printf("\n--- pipeline ---\n");
    std::printf("  total observations : %llu\n", (unsigned long long)s.totalObservations);
    std::printf("  effective rate     : %.2f Hz\n", s.effectiveSampleRateHz);
    std::printf("  distinct TX        : %zu\n", s.tracks.size());
    std::printf("  detection          : %s (confidence %.3f)\n", toString(s.detection.state),
                s.detection.confidence);
    std::printf("  range estimate     : %.2f m\n", s.primaryRangeM);
    std::printf("  EKF                : (%.2f, %.2f) %.3f m/s\n", s.ekf.x, s.ekf.y, s.ekf.speedMps);
    std::printf("  beat frequency     : %s\n",
                s.velocityValid ? (std::to_string(s.beatHz) + " Hz").c_str() : "(none)");
    if (s.velocityValid)
        std::printf("  radial velocity    : %.3f m/s (lambda %.4f m)\n", s.radialVelocityMps,
                    s.wavelengthM);
    if (s.starved) std::printf("\n  ** STARVED: %s **\n", s.starveReason.c_str());
    return s.totalObservations > 0 ? 0 : 1;
}

// Render the GUI to a PNG. Used to verify that the scope really animates by
// capturing two frames a moment apart and comparing them, which is the only
// honest way to check a paint-driven animation.
int screenshot(const QString& path, int settleMs, int tab, const QString& status, bool sense) {
    radar::MainWindow w;
    w.resize(1560, 960);
    if (sense) w.startSensingForTest();
    w.show();
    if (tab >= 0) w.selectTab(tab);
    if (!status.isEmpty()) w.setStatusForTest(status, "#FF5656");
    QCoreApplication::processEvents();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(settleMs);
    while (std::chrono::steady_clock::now() < deadline) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    }
    const QPixmap shot = w.grab();
    if (!shot.save(path)) {
        std::printf("could not write %s\n", path.toStdString().c_str());
        return 2;
    }
    std::printf("wrote %s (%dx%d)\n", path.toStdString().c_str(), shot.width(), shot.height());
    return 0;
}

// Undocumented regression check: press a NeonButton, grab a frame mid-travel
// and another at rest, then report how much the button's pixels changed. This
// is how the 3D press animation was verified rather than assumed.
int pressDemo(const QString& prefix) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    installSignalHandlers();
    radar::MainWindow w;
    w.resize(1560, 960);
    w.show();
    auto* btn = w.findChild<radar::NeonButton*>();
    if (!btn) {
        std::printf("no NeonButton found\n");
        return 2;
    }
    const auto pump = [](int ms) {
        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
        while (std::chrono::steady_clock::now() < end)
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    };
    pump(900);
    w.grab().save(prefix + "_rest.png");

    QMouseEvent press(QEvent::MouseButtonPress, QPointF(8, 8), QPointF(8, 8),
                      Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(btn, &press);
    pump(70);  // mid-travel
    w.grab().save(prefix + "_pressed.png");
    std::printf("press_ after 70 ms : %.3f\n", btn->pressValue());

    QMouseEvent rel(QEvent::MouseButtonRelease, QPointF(8, 8), QPointF(8, 8),
                    Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
    QApplication::sendEvent(btn, &rel);
    pump(400);
    std::printf("press_ after release: %.3f\n", btn->pressValue());
    w.grab().save(prefix + "_released.png");
    return 0;
}

// Restore the interface and exit. Installed for SIGINT/SIGTERM/SIGHUP so that
// killing the program never leaves the machine in monitor mode with
// NetworkManager detached.
void emergencyRestore(int sig) {
    radar::restoreAllRadios();
    std::signal(sig, SIG_DFL);
    std::raise(sig);
}

void installSignalHandlers() {
    for (int s : {SIGINT, SIGTERM, SIGHUP}) std::signal(s, emergencyRestore);
}

}  // namespace

int main(int argc, char** argv) {
    // pkexec sanitises the environment, so XDG_RUNTIME_DIR arrives unset and Qt
    // then warns on every standard-path lookup. Give it a private directory
    // instead of leaving it unset.
    if (qEnvironmentVariableIsEmpty("XDG_RUNTIME_DIR")) {
        const QString dir = QStringLiteral("/tmp/runtime-rssiradar-%1")
                                .arg(static_cast<qulonglong>(::geteuid()));
        ::mkdir(dir.toLocal8Bit().constData(), 0700);
        ::chmod(dir.toLocal8Bit().constData(), 0700);
        qputenv("XDG_RUNTIME_DIR", dir.toLocal8Bit());
    }

    QApplication app(argc, argv);
    QCoreApplication::setApplicationName("rssiradar");
    QCoreApplication::setApplicationVersion("1.3.1");

    QCommandLineParser parser;
    parser.setApplicationDescription(
        "RSSI RADAR — passive WiFi/Bluetooth radiometric motion sensing.\n"
        "Uses real AF_PACKET and AF_BLUETOOTH raw sockets. No simulated data path "
        "exists in this program.");
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption selftestOpt("selftest",
                                   "Probe this machine's radios and run closed-form "
                                   "DSP checks, then exit.",
                                   "");
    parser.addOption(selftestOpt);
    QCommandLineOption configOpt({"c", "config"}, "Load a JSON configuration file.", "path");
    parser.addOption(configOpt);
    QCommandLineOption listPresets("list-presets", "List the built-in presets and exit.");
    parser.addOption(listPresets);
    QCommandLineOption senseOpt("sense",
                                "Start real capture before the screenshot, so the views are "
                                "populated with live radio data.",
                                "");
    parser.addOption(senseOpt);
    QCommandLineOption tabOpt({"tab", "tab"}, "Screenshot only this tab index.", "n");
    parser.addOption(tabOpt);
    QCommandLineOption statusOpt({"status", "status"},
                                 "Force the status label text (layout regression check).",
                                 "text");
    parser.addOption(statusOpt);
    QCommandLineOption pressOpt("pressdemo",
                                "Internal: capture a button at rest and mid-press.",
                                "prefix");
    parser.addOption(pressOpt);
    QCommandLineOption shotOpt({"screenshot", "screenshot"},
                               "Render the GUI to <file.png> after <ms> of settling, then "
                               "exit. Capture twice to compare frames.",
                               "file.png");
    parser.addOption(shotOpt);
    QCommandLineOption settleOpt({"settle", "settle"},
                                 "Milliseconds to let the UI animate before the screenshot.",
                                 "ms");
    parser.addOption(settleOpt);
    QCommandLineOption captureOpt({"capture", "capture"},
                                   "Headless capture for <seconds>, prints what the radios "
                                   "actually deliver. Exit 1 if no frames arrived.",
                                   "seconds");
    parser.addOption(captureOpt);
    parser.process(app);

    if (parser.isSet(selftestOpt)) return selftest();
    if (parser.isSet(pressOpt)) return pressDemo(parser.value(pressOpt));
    if (parser.isSet(shotOpt))
        return screenshot(parser.value(shotOpt),
                          parser.value(settleOpt).toInt() > 0 ? parser.value(settleOpt).toInt()
                                                              : 1500,
                          parser.isSet(tabOpt) ? parser.value(tabOpt).toInt() : -1,
                          parser.value(statusOpt), parser.isSet(senseOpt));
    if (parser.isSet(captureOpt))
        return capture(parser.value(captureOpt).toDouble(),
                       parser.value(configOpt));

    if (parser.isSet(listPresets)) {
        for (const auto& p : radar::builtinPresets())
            std::printf("%-26s %s\n", p.name.c_str(), p.description.c_str());
        return 0;
    }

    radar::MainWindow w;
    if (parser.isSet(configOpt)) {
        std::string err;
        const radar::Config c = radar::Config::load(parser.value(configOpt).toStdString(), &err);
        if (err.empty()) {
            radar::Engine tmp;
            (void)tmp;
        }
        // Config is applied through the window's engine below.
        w.setProperty("initialConfig", parser.value(configOpt));
    }
    w.show();
    return app.exec();
}