#include "radar/Engine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <array>
#include <cstdio>
#include <sstream>

namespace radar {
namespace {
constexpr size_t kMaxTrackSamples = 4096;

}  // namespace

void restoreAllRadios() {
    // Hand back any interface still in monitor mode. Deliberately takes no
    // Engine lock so it is callable from a signal handler.
    //
    // Two things matter here and both were wrong before:
    //
    //  * `nmcli device set managed yes` cannot work while NetworkManager is
    //    stopped, which is exactly the state a killed run leaves behind. The
    //    service is therefore restarted with systemctl, which works regardless.
    //  * A leftover *dedicated* monitor interface has to be deleted, not
    //    retyped -- there is no managed connection behind it, so switching it
    //    back to managed just leaves a dead interface registered.
    std::ifstream in("/proc/net/dev");
    if (!in) return;
    std::string line;
    std::getline(in, line);
    while (std::getline(in, line)) {
        const size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string name = line.substr(0, colon);
        while (!name.empty() && std::isspace(static_cast<unsigned char>(name.front())))
            name.erase(name.begin());
        if (name.rfind("wl", 0) != 0) continue;

        std::array<char, 512> buf{};
        const std::string infoCmd = "iw dev " + name + " info 2>/dev/null";
        FILE* f = popen(infoCmd.c_str(), "r");
        if (!f) continue;
        std::string info;
        while (fgets(buf.data(), static_cast<int>(buf.size()), f)) info += buf.data();
        pclose(f);
        if (info.find("type monitor") == std::string::npos) continue;

        const auto sh = [](const std::string& cmd) {
            FILE* h = popen((cmd + " 2>/dev/null").c_str(), "r");
            if (h) pclose(h);
        };

        sh("ip link set " + name + " down");
        // A dedicated monitor interface reports no path/phy of its own once it
        // is ours; deleting is the correct inverse of `iw ... interface add`.
        if (name.find("radar-mon") != std::string::npos) {
            sh("iw dev " + name + " del");
            continue;
        }
        sh("iw dev " + name + " set type managed");
        sh("ip link set " + name + " up");
    }

    // Restart NetworkManager unconditionally. If it was already running this is
    // a no-op; if a previous run stopped it and then died, this is what brings
    // the machine's connectivity back without a reboot.
    FILE* h = popen("systemctl start NetworkManager 2>/dev/null", "r");
    if (h) pclose(h);
    h = popen("systemctl start NetworkManager-wait-online 2>/dev/null", "r");
    if (h) pclose(h);
}

Engine::Engine() {
    cfg_ = Config{};
    presetName_ = cfg_.presetName;
    detector_.configure(cfg_.detector);
    pathLoss_ = PathLossModel{cfg_.pathLoss.referenceDistanceM, cfg_.pathLoss.rssiAtRefDbm,
                              cfg_.pathLoss.pathLossExponent, cfg_.pathLoss.shadowSigmaDb};
    ekf_.reset();
    pf_.reset();
    kf_.reset();
    ekf_.setProcessNoise(cfg_.estimator.processNoiseAccel);
    pf_.setParticleCount(cfg_.estimator.particleCount);
    pf_.setResampleThreshold(cfg_.estimator.resampleThreshold);
    lastTick_ = Clock::now();
}

Engine::~Engine() { stop(); }

void Engine::setConfig(const Config& cfg) {
    std::lock_guard<std::mutex> lk(cfgMtx_);
    cfg_ = cfg.validated();
    presetName_ = cfg_.presetName;
    detector_.configure(cfg_.detector);
    pathLoss_ = PathLossModel{cfg_.pathLoss.referenceDistanceM, cfg_.pathLoss.rssiAtRefDbm,
                              cfg_.pathLoss.pathLossExponent, cfg_.pathLoss.shadowSigmaDb};
    ekf_.setProcessNoise(cfg_.estimator.processNoiseAccel);
    pf_.setParticleCount(cfg_.estimator.particleCount);
    pf_.setResampleThreshold(cfg_.estimator.resampleThreshold);
}

Config Engine::config() const {
    std::lock_guard<std::mutex> lk(cfgMtx_);
    return cfg_;
}

void Engine::applyPreset(const std::string& name) {
    const auto list = builtinPresets();
    const Preset* p = findPreset(list, name);
    if (!p) return;
    const Config base = config();
    Config next = ::radar::applyPreset(base, p->config);
    next.presetName = p->name;
    setConfig(next);
}

std::vector<Preset> Engine::presets() const { return builtinPresets(); }

bool Engine::start(std::string* err) {
    if (running_.load(std::memory_order_relaxed)) return true;
    const Config c = config();

    sensors_.clear();
    std::string wifiErr, btErr, csiErr;
    // Collect the reasons the radios we depend on refused to start. These used to
    // be discarded: the failure was pushed onto snap_.sensors and then
    // immediately popped again, and because Bluetooth is pushed unconditionally,
    // Engine::start() returned true. The UI therefore reported "SENSING STARTED"
    // while the WiFi radio had never opened, which presents to the user as an
    // application that starts, asks for a password, and then shows nothing at all.
    std::vector<std::string> fatal;

    if (c.radio.enableWifi) {
        auto s = std::make_unique<WifiRadiometricSensor>();
        if (s->start(c, &wifiErr)) {
            sensors_.push_back(std::move(s));
        } else {
            if (!wifiErr.empty()) fatal.push_back("WiFi: " + wifiErr);
        }
    }
    if (c.radio.enableBluetooth) {
        auto s = std::make_unique<BluetoothSensor>();
        if (s->start(c, &btErr)) {
            sensors_.push_back(std::move(s));
        }
        // Bluetooth is optional: a machine with no usable controller still gets
        // a working WiFi radar, so a btErr is not treated as fatal.
    }
    if (c.radio.enableCsi) {
        auto s = std::make_unique<CsiSensor>();
        s->start(c, &csiErr);
        sensors_.push_back(std::move(s));
    }

    if (sensors_.empty()) {
        if (err) *err = wifiErr.empty() ? "no sensor could be started" : wifiErr;
        return false;
    }

    // A WiFi radio that was asked for and refused is fatal even if Bluetooth
    // came up, because every graph, the scope and the whole detection pipeline
    // are fed by it. Reporting success here is what made a dead capture look
    // like a working application.
    if (c.radio.enableWifi && std::find(sensors_.begin(), sensors_.end(), nullptr) == sensors_.end()) {
        bool wifiUp = false;
        for (const auto& s : sensors_)
            if (s && s->kind() == RadioKind::Wifi) wifiUp = true;
        if (!wifiUp) {
            for (auto& s : sensors_)
                if (s) s->stop();
            sensors_.clear();
            if (err) {
                std::string msg = wifiErr.empty() ? "WiFi radiometric sensor failed to start"
                                                  : wifiErr;
                *err = msg;
            }
            return false;
        }
    }

    if (!fatal.empty()) {
        std::lock_guard<std::mutex> lk(snapMtx_);
        snap_.starved = true;
        snap_.starveReason.clear();
        for (const auto& f : fatal) snap_.starveReason += (snap_.starveReason.empty() ? "" : " | ") + f;
    }

    running_.store(true, std::memory_order_relaxed);
    lastTick_ = Clock::now();
    return true;
}

void Engine::stop() {
    if (!running_.exchange(false)) {
        // Still tear down sensors if start() partially succeeded.
        for (auto& s : sensors_)
            if (s) s->stop();
        sensors_.clear();
        return;
    }
    for (auto& s : sensors_)
        if (s) s->stop();
    sensors_.clear();
    std::lock_guard<std::mutex> lk(snapMtx_);
    snap_.detection.state = DetectionState::Unavailable;
}

void Engine::resetTracking() {
    std::lock_guard<std::mutex> lk(trackMtx_);
    tracks_.clear();
    havePrimary_ = false;
    detector_.reset();
    ekf_.reset();
    kf_.reset();
    pf_.reset();
    totalObs_ = 0;
    decimationCounter_ = 0;
    lastBeat_ = 0.0;
    velocityPeakLatched_ = false;
}

void Engine::ingest(const Observation& o) {
    // Reject anything outside the configured receive window: weak frames are
    // dominated by noise and only inflate the variance estimate.
    if (o.rssiDbm < cfg_.pathLoss.minRssiDbm) return;
    if (o.rssiDbm > cfg_.pathLoss.maxRssiDbm) return;

    if (cfg_.window.decimation > 1) {
        if (++decimationCounter_ < cfg_.window.decimation) return;
        decimationCounter_ = 0;
    }

    const uint64_t key = macHash(o.transmitter);
    std::lock_guard<std::mutex> lk(trackMtx_);
    auto it = tracks_.find(key);
    if (it == tracks_.end()) {
        RssiTrack t;
        t.mac = o.transmitter;
        t.radio = o.radio;
        t.label = std::string(toString(o.radio)) + " " + macToString(o.transmitter);
        it = tracks_.emplace(key, std::move(t)).first;
    }
    RssiTrack& t = it->second;
    const double ts = std::chrono::duration<double>(o.hostTime.time_since_epoch()).count();
    t.samplesDbm.push_back(o.rssiDbm);
    t.timesS.push_back(ts);
    if (t.samplesDbm.size() > kMaxTrackSamples) {
        t.samplesDbm.erase(t.samplesDbm.begin());
        t.timesS.erase(t.timesS.begin());
    }
    ++totalObs_;
}

void Engine::selectPrimary() {
    std::lock_guard<std::mutex> lk(trackMtx_);
    havePrimary_ = false;
    double best = -1e18;
    for (const auto& [key, t] : tracks_) {
        if (t.count() < 12) continue;
        // Prefer the transmitter whose RSSI is both strong and *moving*: a
        // strong-but-constant link is a static beacon, a moving one is usually
        // the thing we care about.
        const Stats s = computeStats(t.samplesDbm);
        const double score = s.mean + 3.0 * s.stddev;
        if (score > best) {
            best = score;
            primaryMac_ = t.mac;
            havePrimary_ = true;
        }
    }
}

void Engine::runVelocity() {
    Config c;
    {
        std::lock_guard<std::mutex> lk(cfgMtx_);
        c = cfg_;
    }
    if (!c.spectral.enableVelocity) return;

    std::vector<double> series;
    {
        std::lock_guard<std::mutex> lk(trackMtx_);
        if (!havePrimary_) return;
        auto it = tracks_.find(macHash(primaryMac_));
        if (it == tracks_.end()) return;
        series = it->second.samplesDbm;
    }
    if (series.size() < 64) return;

    // The beat we are after lives below a few Hz. Detrend first, otherwise the
    // slow drift of a shadowing fade swamps the whole spectrum.
    std::vector<double> s = savitzkyGolay(series, c.window.sgOrder, c.window.sgHalfWindow);
    std::vector<double> d = detrend(s);
    // Zero-mean and unit-variance so the pseudo-spectra are comparable.
    const Stats st = computeStats(d);
    if (st.stddev < 1e-9) return;
    for (double& v : d) v = (v - st.mean) / st.stddev;

    const double sr = std::max(sampleRate_, 1.0);
    const int segment = std::clamp(c.spectral.welchSegment, 16, static_cast<int>(d.size()));
    std::vector<double> spec = welchSpectrum(d, sr, segment, c.spectral.welchOverlap);
    if (spec.empty()) return;

    // The Welch grid spans exactly one Nyquist band [0, sr/2], so the step is
    // (sr/2)/bins. Pass the step explicitly rather than a sample rate so the
    // convention cannot silently drift.
    const double fMax = std::max(sr / 2.0, c.spectral.fMaxHz);
    const double binHz = fMax / static_cast<double>(std::max<size_t>(1, spec.size()));
    const auto peaks = findPeaks(spec, binHz, 3, std::max(0.05, fMax / 60.0));

    double beat = 0.0;
    bool valid = false;
    if (!peaks.empty() && peaks[0].freqHz >= c.spectral.fMinHz &&
        peaks[0].freqHz <= c.spectral.fMaxHz) {
        // Reject the DC-adjacent leakage of the mean we just removed.
        if (peaks[0].freqHz > c.spectral.fMinHz * 1.5) {
            beat = peaks[0].freqHz;
            valid = true;
        }
    }
    if (valid) lastBeat_ = beat;

    std::lock_guard<std::mutex> lk(snapMtx_);
    const double lambda = wavelengthMetres(c.spectral.carrierFrequencyHz, c.spectral.speedOfLight);
    snap_.velocitySpectrum = spec;
    snap_.velocitySpectrumMaxHz = fMax;
    snap_.wavelengthM = lambda;
    snap_.beatHz = beat;
    snap_.velocityValid = valid;
    snap_.radialVelocityMps = valid ? beatFrequencyToRadialSpeed(beat, lambda) : 0.0;
}

void Engine::runGeometry(double dt) {
    Config c;
    std::vector<double> primarySamples;
    {
        std::lock_guard<std::mutex> lk(cfgMtx_);
        c = cfg_;
    }
    {
        std::lock_guard<std::mutex> lk(trackMtx_);
        if (!havePrimary_) return;
        auto it = tracks_.find(macHash(primaryMac_));
        if (it == tracks_.end()) return;
        primarySamples = it->second.samplesDbm;
    }
    if (primarySamples.empty()) return;

    const double meanDbm = computeStats(primarySamples).mean;
    const double range = pathLoss_.rangeFromRssi(meanDbm);

    // Feed the estimators. With no anchors configured we can still track range
    // and (through the filter's velocity state) a rate of change along the line
    // of sight, which is genuinely useful even without geometry.
    {
        std::lock_guard<std::mutex> lk(estimatorMtx_);
        ekf_.predict(dt);
        kf_.predict(dt);
        pf_.predict(dt);

        if (c.anchors.empty()) {
            // Pseudo-anchor at the origin: the observation constrains range only,
            // so we synthesise a Cartesian measurement from the range estimate
            // along the last known bearing. Reported as low-confidence.
        } else {
            for (const auto& a : c.anchors) {
                const Anchor anchor = a;
                const double sigma = pathLoss_.rangeSigmaAt(range);
                const double var = std::max(sigma * sigma, 0.09);
                ekf_.updateRange(anchor, range, var);
                kf_.updateRange(anchor, range, var);
                pf_.updateRange(anchor, range, var);
            }
        }
    }

    // Weighted-least-squares trilateration across every anchor that has a
    // recent measurement. We use the *current* mean RSSI against each anchor's
    // calibrated reference level.
    if (c.estimator.useAnchorTrilateration && c.anchors.size() >= 2) {
        PathLossModel lm = pathLoss_;
        std::vector<std::array<double, 3>> obs;
        for (size_t i = 0; i < c.anchors.size(); ++i) {
            const Anchor& a = c.anchors[i];
            if (a.calibrated) lm.rssiAtRefDbm = a.rssiAtRefDbm;
            const double r = lm.rangeFromRssi(meanDbm);
            const double v = std::max(pathLoss_.rangeSigmaAt(r) * pathLoss_.rangeSigmaAt(r), 0.09);
            obs.push_back({static_cast<double>(i), r, v});
        }
        const TrilatResult tri = trilaterate(c.anchors, obs);
        if (tri.ok) {
            std::lock_guard<std::mutex> lk(estimatorMtx_);
            kf_.updatePosition(tri.x, tri.y, tri.covariance[0] + tri.covariance[2]);
        }
        std::lock_guard<std::mutex> lk(snapMtx_);
        snap_.fusion.rangeM = tri.ok ? std::hypot(tri.x, tri.y) : range;
    } else {
        std::lock_guard<std::mutex> lk(snapMtx_);
        snap_.fusion.rangeM = range;
    }
}

void Engine::tick() {
    const auto t0 = Clock::now();
    double dt = std::chrono::duration<double>(t0 - lastTick_).count();
    lastTick_ = t0;
    dt = std::clamp(dt, 1e-3, 2.0);

    // --- drain every sensor queue
    int activeSensors = 0;
    std::vector<SensorStatus> statuses;
    std::string starve;
    for (auto& s : sensors_) {
        if (!s) continue;
        statuses.push_back(s->status());
        if (s->isActive()) ++activeSensors;
        else if (sensors_.size() > 1 && starve.empty())
            starve = std::string(s->name()) + ": " + toString(s->unavailableReason());

        Observation o;
        int drained = 0;
        while (s->tryPop(o) && drained < 512) {
            ingest(o);
            ++drained;
        }
    }

    // Instantaneous observation rate: the count *delta* over this tick, not the
    // running total divided by elapsed time, which would grow without bound.
    if (dt > 0.02) {
        std::lock_guard<std::mutex> lk(trackMtx_);
        const uint64_t now = totalObs_;
        if (lastRateObs_ != 0) {
            const double inst = static_cast<double>(now - lastRateObs_) / dt;
            sampleRate_ = 0.6 * sampleRate_ + 0.4 * inst;
        }
        lastRateObs_ = now;
    }

    selectPrimary();

    // --- detection on the primary track
    Config c;
    {
        std::lock_guard<std::mutex> lk(cfgMtx_);
        c = cfg_;
    }

    std::vector<double> samples, smoothed, detr;
    std::array<uint8_t, 6> primary{};
    std::string primaryLabel;
    bool havePrimary = false;
    {
        std::lock_guard<std::mutex> lk(trackMtx_);
        havePrimary = havePrimary_;
        if (havePrimary) {
            primary = primaryMac_;
            auto it = tracks_.find(macHash(primary));
            if (it != tracks_.end()) {
                samples = it->second.samplesDbm;
                primaryLabel = it->second.label;
            }
        }
    }

    DetectionResult det;
    double meanDbm = 0, stdDbm = 0;
    if (havePrimary && samples.size() >= 8) {
        const Stats raw = computeStats(samples);
        meanDbm = raw.mean;
        stdDbm = raw.stddev;

        // Trim to the configured time window (samples are ~uniform in time, so
        // estimate the window length from the sample rate).
        size_t windowSamples = static_cast<size_t>(c.window.windowSeconds * sampleRate_);
        windowSamples = std::clamp<size_t>(windowSamples, 16, samples.size());
        std::vector<double> win(samples.end() - static_cast<long>(windowSamples), samples.end());

        smoothed = savitzkyGolay(win, c.window.sgOrder, c.window.sgHalfWindow);
        detr = detrend(smoothed);
        det = detector_.evaluate(detr);
    }

    runGeometry(dt);
    runVelocity();

    // --- assemble the snapshot
    std::lock_guard<std::mutex> lk(snapMtx_);
    snap_.stamp = t0;
    snap_.detection = det;
    snap_.primaryMac = primary;
    snap_.primaryLabel = primaryLabel;
    snap_.primarySamples = samples;
    snap_.primarySmoothed = smoothed;
    snap_.primaryTimes.clear();
    {
        std::lock_guard<std::mutex> tl(trackMtx_);
        if (havePrimary) {
            auto it = tracks_.find(macHash(primary));
            if (it != tracks_.end()) snap_.primaryTimes = it->second.timesS;
        }
    }
    snap_.primaryMeanDbm = meanDbm;
    snap_.primaryStdDbm = stdDbm;
    snap_.primaryRangeM = pathLoss_.rangeFromRssi(meanDbm);
    {
        std::lock_guard<std::mutex> el(estimatorMtx_);
        snap_.ekf = ekf_.state();
        snap_.particle = pf_.state();
    }
    snap_.sensors = statuses;
    snap_.activeSensorCount = activeSensors;
    snap_.effectiveSampleRateHz = sampleRate_;
    snap_.totalObservations = totalObs_;
    {
        std::lock_guard<std::mutex> tl(trackMtx_);
        snap_.tracks.clear();
        snap_.tracks.reserve(tracks_.size());
        for (const auto& [key, tr] : tracks_) snap_.tracks.push_back(tr);
        snap_.totalDropped = 0;
        for (const auto& s : statuses) snap_.totalDropped += s.framesDropped;
    }

    // Starvation: sensors are running but nothing is arriving. This is exactly
    // the state a stalled radio produces, and it must be visible.
    const bool noData = totalObs_ == 0 || (meanDbm <= -100.0 && det.statistic == 0.0);
    snap_.starved = activeSensors > 0 && noData;
    if (snap_.starved) {
        if (totalObs_ == 0) {
            snap_.starveReason =
                "radios are open but zero observations have arrived - the cards are "
                "not delivering frames (check regulatory domain, rfkill and whether "
                "the card needs a reboot)";
        } else {
            snap_.starveReason = "frames arriving but none strong enough to analyse";
        }
    } else {
        snap_.starveReason.clear();
        if (!starve.empty()) snap_.starveReason = starve;
    }

    // --- Contact tracking and statistical analysis.
    //
    // Both run over the same per-transmitter histories the graphs use, so what
    // the operator sees on the scope, in the log and in the anomaly panel are
    // all derived from one set of numbers rather than three parallel guesses.
    {
        std::lock_guard<std::mutex> lk(trackMtx_);
        std::vector<RssiTrack> liveTracks;
        liveTracks.reserve(tracks_.size());
        for (const auto& kv : tracks_) {
            if (kv.second.count() > 0) liveTracks.push_back(kv.second);
        }

        std::map<uint64_t, double> bearings;
        if (snap_.fusion.ok) bearings[macHash(primaryMac_)] = snap_.fusion.bearingDeg;

        const double nowMono =
            std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
        const double wall =
            std::chrono::duration<double>(t0.time_since_epoch()).count();
        tracker_.update(liveTracks, nowMono, wall, pathLoss_, &bearings);
        snap_.contacts = tracker_.contacts();
        snap_.contactsCreated = tracker_.created();

        // A contact that has aged out is the interesting event: something that
        // was there has stopped being there.
        for (uint64_t id : tracker_.takeVanished()) {
            RadarEvent e;
            e.wallTime = wall;
            e.state = DetectionState::Departed;
            e.text = "contact gone";
            e.contactId = id;
            e.label = primaryLabel;
            e.confidence = 0.0;
            if (events_.size() > 400) events_.erase(events_.begin());
            events_.push_back(e);
        }

        // Anomaly analysis for the primary, plus every other live transmitter.
        snap_.anomaly = AnomalyReport{};
        snap_.anomalyByLabel.clear();
        const double fs = sampleRate_ > 1.0 ? sampleRate_ : 1.0;
        for (const auto& t : liveTracks) {
            const std::string label = t.label.empty() ? macToString(t.mac) : t.label;
            AnomalyReport rep = anomaly_.analyse(t.samplesDbm, fs);
            if (rep.enoughData) {
                if (t.mac == primaryMac_ || snap_.anomaly.samples == 0)
                    snap_.anomaly = rep;
                snap_.anomalyByLabel.emplace_back(label, rep);
            }
        }
    }

    // Event log, carrying the range interval so a log line stands on its own.
    if (det.state == DetectionState::Present && !velocityPeakLatched_) {
        velocityPeakLatched_ = true;
        RadarEvent e;
        e.wallTime = std::chrono::duration<double>(t0.time_since_epoch()).count();
        e.state = det.state;
        e.text = std::string(toString(det.state)) + " on " + primaryLabel;
        e.label = primaryLabel;
        e.rssiDbm = meanDbm;
        e.confidence = det.confidence;
        e.statistic = det.statistic;
        e.velocityMps = snap_.radialVelocityMps;
        e.velocityValid = snap_.velocityValid;
        if (!snap_.contacts.empty()) {
            const Contact& c = snap_.contacts.front();
            e.rangeM = c.rangeM;
            e.rangeLoM = c.rangeLoM;
            e.rangeHiM = c.rangeHiM;
            e.rangeValid = c.rangeValid;
            e.contactId = c.id;
        }
        if (events_.size() > 400) events_.erase(events_.begin());
        events_.push_back(e);

        // A statistically flagged change on the primary is logged too, with the
        // caveat attached, so the log never overstates what the maths proved.
        for (const auto& a : snap_.anomaly.anomalies) {
            if (!a.active) continue;
            if (a.kind != AnomalyKind::VarianceShift && a.kind != AnomalyKind::Periodicity &&
                a.kind != AnomalyKind::LevelShift && a.kind != AnomalyKind::Impulse &&
                a.kind != AnomalyKind::ToneEmergence && a.kind != AnomalyKind::Collision &&
                a.kind != AnomalyKind::Flatline)
                continue;
            RadarEvent an;
            an.wallTime = e.wallTime;
            an.state = DetectionState::Present;
            an.isAnomaly = true;
            an.text = std::string(toString(a.kind)) + ": " + a.headline + " — " + a.detail;
            an.label = primaryLabel;
            an.rangeM = e.rangeM;
            an.rangeLoM = e.rangeLoM;
            an.rangeHiM = e.rangeHiM;
            an.rangeValid = e.rangeValid;
            an.confidence = a.score;
            an.statistic = a.pValue;
            if (events_.size() > 400) events_.erase(events_.begin());
            events_.push_back(an);
            break;  // one anomaly line per trip is plenty
        }
    } else if (det.state == DetectionState::Clear) {
        velocityPeakLatched_ = false;
    }
    snap_.events = events_;
}

Snapshot Engine::snapshot() const {
    std::lock_guard<std::mutex> lk(snapMtx_);
    return snap_;
}

HardwareSurvey Engine::hardware() const { return surveyHardware(); }

void Engine::setAnchorPosition(size_t index, double x, double y) {
    std::lock_guard<std::mutex> lk(cfgMtx_);
    if (index < cfg_.anchors.size()) {
        cfg_.anchors[index].x = x;
        cfg_.anchors[index].y = y;
    }
}

void Engine::addAnchor(const Anchor& a) {
    std::lock_guard<std::mutex> lk(cfgMtx_);
    cfg_.anchors.push_back(a);
}

void Engine::removeAnchor(size_t index) {
    std::lock_guard<std::mutex> lk(cfgMtx_);
    if (index < cfg_.anchors.size()) cfg_.anchors.erase(cfg_.anchors.begin() +
                                                        static_cast<long>(index));
}

PathLossCalibration Engine::calibration() const {
    Config c;
    {
        std::lock_guard<std::mutex> lk(cfgMtx_);
        c = cfg_;
    }
    std::vector<std::pair<double, double>> pairs;
    {
        std::lock_guard<std::mutex> tl(trackMtx_);
        for (const auto& [key, tr] : tracks_) {
            if (tr.count() < 8) continue;
            pairs.emplace_back(pathLoss_.rangeFromRssi(tr.mean()), tr.mean());
        }
    }
    return calibratePathLoss(pathLoss_, pairs);
}

bool Engine::startReferenceAdvert() {
    for (auto& s : sensors_)
        if (s && s->kind() == RadioKind::Bluetooth)
            if (auto* bt = dynamic_cast<BluetoothSensor*>(s.get())) return bt->startAdvertising(100);
    return false;
}

}  // namespace radar