#include "radar/Config.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace radar {
namespace {

using nlohmann::json;

json toJ(const PathLossConfig& c) {
    return json{{"referenceDistanceM", c.referenceDistanceM},
                {"rssiAtRefDbm", c.rssiAtRefDbm},
                {"pathLossExponent", c.pathLossExponent},
                {"shadowSigmaDb", c.shadowSigmaDb},
                {"minRssiDbm", c.minRssiDbm},
                {"maxRssiDbm", c.maxRssiDbm}};
}
PathLossConfig fromJ(const json& j, const PathLossConfig& d) {
    PathLossConfig c = d;
    if (!j.is_null()) {
        c.referenceDistanceM = j.value("referenceDistanceM", d.referenceDistanceM);
        c.rssiAtRefDbm = j.value("rssiAtRefDbm", d.rssiAtRefDbm);
        c.pathLossExponent = j.value("pathLossExponent", d.pathLossExponent);
        c.shadowSigmaDb = j.value("shadowSigmaDb", d.shadowSigmaDb);
        c.minRssiDbm = j.value("minRssiDbm", d.minRssiDbm);
        c.maxRssiDbm = j.value("maxRssiDbm", d.maxRssiDbm);
    }
    return c;
}

json toJ(const WindowConfig& c) {
    return json{{"windowSeconds", c.windowSeconds},
                {"decimation", c.decimation},
                {"emaAlpha", c.emaAlpha},
                {"sgOrder", c.sgOrder},
                {"sgHalfWindow", c.sgHalfWindow}};
}
WindowConfig fromJ(const json& j, const WindowConfig& d) {
    WindowConfig c = d;
    if (!j.is_null()) {
        c.windowSeconds = j.value("windowSeconds", d.windowSeconds);
        c.decimation = j.value("decimation", d.decimation);
        c.emaAlpha = j.value("emaAlpha", d.emaAlpha);
        c.sgOrder = j.value("sgOrder", d.sgOrder);
        c.sgHalfWindow = j.value("sgHalfWindow", d.sgHalfWindow);
    }
    return c;
}

json toJ(const EstimatorConfig& c) {
    return json{{"processNoiseAccel", c.processNoiseAccel},
                {"rangeNoiseM", c.rangeNoiseM},
                {"initialPositionSigmaM", c.initialPositionSigmaM},
                {"initialVelocitySigmaMps", c.initialVelocitySigmaMps},
                {"particleCount", c.particleCount},
                {"resampleThreshold", c.resampleThreshold},
                {"resampleEvery", c.resampleEvery},
                {"useAnchorTrilateration", c.useAnchorTrilateration},
                {"minAnchorsForFix", c.minAnchorsForFix},
                {"anchorWeightFloor", c.anchorWeightFloor}};
}
EstimatorConfig fromJ(const json& j, const EstimatorConfig& d) {
    EstimatorConfig c = d;
    if (!j.is_null()) {
        c.processNoiseAccel = j.value("processNoiseAccel", d.processNoiseAccel);
        c.rangeNoiseM = j.value("rangeNoiseM", d.rangeNoiseM);
        c.initialPositionSigmaM = j.value("initialPositionSigmaM", d.initialPositionSigmaM);
        c.initialVelocitySigmaMps = j.value("initialVelocitySigmaMps", d.initialVelocitySigmaMps);
        c.particleCount = j.value("particleCount", d.particleCount);
        c.resampleThreshold = j.value("resampleThreshold", d.resampleThreshold);
        c.resampleEvery = j.value("resampleEvery", d.resampleEvery);
        c.useAnchorTrilateration = j.value("useAnchorTrilateration", d.useAnchorTrilateration);
        c.minAnchorsForFix = j.value("minAnchorsForFix", d.minAnchorsForFix);
        c.anchorWeightFloor = j.value("anchorWeightFloor", d.anchorWeightFloor);
    }
    return c;
}

json toJ(const SpectralConfig& c) {
    return json{{"welchSegment", c.welchSegment},
                {"welchOverlap", c.welchOverlap},
                {"fMinHz", c.fMinHz},
                {"fMaxHz", c.fMaxHz},
                {"carrierFrequencyHz", c.carrierFrequencyHz},
                {"speedOfLight", c.speedOfLight},
                {"enableVelocity", c.enableVelocity}};
}
SpectralConfig fromJ(const json& j, const SpectralConfig& d) {
    SpectralConfig c = d;
    if (!j.is_null()) {
        c.welchSegment = j.value("welchSegment", d.welchSegment);
        c.welchOverlap = j.value("welchOverlap", d.welchOverlap);
        c.fMinHz = j.value("fMinHz", d.fMinHz);
        c.fMaxHz = j.value("fMaxHz", d.fMaxHz);
        c.carrierFrequencyHz = j.value("carrierFrequencyHz", d.carrierFrequencyHz);
        c.speedOfLight = j.value("speedOfLight", d.speedOfLight);
        c.enableVelocity = j.value("enableVelocity", d.enableVelocity);
    }
    return c;
}

json toJ(const DetectorConfig& c) {
    return json{{"baseThresholdSigma", c.baseThresholdSigma},
                {"cfarGuardCells", c.cfarGuardCells},
                {"cfarTrainCells", c.cfarTrainCells},
                {"falseAlarmRate", c.falseAlarmRate},
                {"useEntropy", c.useEntropy},
                {"entropyThreshold", c.entropyThreshold},
                {"histogramBins", c.histogramBins},
                {"useChiSquare", c.useChiSquare},
                {"chiSquareThreshold", c.chiSquareThreshold},
                {"holdSeconds", c.holdSeconds},
                {"departSeconds", c.departSeconds}};
}
DetectorConfig fromJ(const json& j, const DetectorConfig& d) {
    DetectorConfig c = d;
    if (!j.is_null()) {
        c.baseThresholdSigma = j.value("baseThresholdSigma", d.baseThresholdSigma);
        c.cfarGuardCells = j.value("cfarGuardCells", d.cfarGuardCells);
        c.cfarTrainCells = j.value("cfarTrainCells", d.cfarTrainCells);
        c.falseAlarmRate = j.value("falseAlarmRate", d.falseAlarmRate);
        c.useEntropy = j.value("useEntropy", d.useEntropy);
        c.entropyThreshold = j.value("entropyThreshold", d.entropyThreshold);
        c.histogramBins = j.value("histogramBins", d.histogramBins);
        c.useChiSquare = j.value("useChiSquare", d.useChiSquare);
        c.chiSquareThreshold = j.value("chiSquareThreshold", d.chiSquareThreshold);
        c.holdSeconds = j.value("holdSeconds", d.holdSeconds);
        c.departSeconds = j.value("departSeconds", d.departSeconds);
    }
    return c;
}

json toJ(const FusionConfig& c) {
    return json{{"inverseVarianceWeighting", c.inverseVarianceWeighting},
                {"wifiWeight", c.wifiWeight},
                {"bluetoothWeight", c.bluetoothWeight},
                {"radioGateConfidence", c.radioGateConfidence},
                {"motionLogOdds", c.motionLogOdds},
                {"stillLogOdds", c.stillLogOdds},
                {"priorLogOddsMotion", c.priorLogOddsMotion},
                {"confidenceEmaAlpha", c.confidenceEmaAlpha}};
}
FusionConfig fromJ(const json& j, const FusionConfig& d) {
    FusionConfig c = d;
    if (!j.is_null()) {
        c.inverseVarianceWeighting = j.value("inverseVarianceWeighting", d.inverseVarianceWeighting);
        c.wifiWeight = j.value("wifiWeight", d.wifiWeight);
        c.bluetoothWeight = j.value("bluetoothWeight", d.bluetoothWeight);
        c.radioGateConfidence = j.value("radioGateConfidence", d.radioGateConfidence);
        c.motionLogOdds = j.value("motionLogOdds", d.motionLogOdds);
        c.stillLogOdds = j.value("stillLogOdds", d.stillLogOdds);
        c.priorLogOddsMotion = j.value("priorLogOddsMotion", d.priorLogOddsMotion);
        c.confidenceEmaAlpha = j.value("confidenceEmaAlpha", d.confidenceEmaAlpha);
    }
    return c;
}

// Fetch an optional sub-object.
//
// `j.value("radio", nullptr)` does not work here: nlohmann deduces ValueType as
// std::nullptr_t and then tries to extract it, which throws
// "type must be null, but is object" whenever the key is present. The effect was
// that *any* partial configuration file was rejected outright -- so
// `--config` with just the one setting you wanted to change silently fell back
// to the defaults. Since the error was swallowed by the caller, it looked like
// the file had been applied.
json sectionOf(const json& j, const char* key) {
    if (!j.is_object()) return json();
    const auto it = j.find(key);
    if (it == j.end() || it->is_null()) return json();
    return *it;
}

json toJ(const RadioConfig& c) {
    return json{{"enableWifi", c.enableWifi},
                {"enableBluetooth", c.enableBluetooth},
                {"enableCsi", c.enableCsi},
                {"autoMonitorMode", c.autoMonitorMode},
                {"stopNetworkManager", c.stopNetworkManager},
                {"wifiInterface", c.wifiInterface},
                {"bluetoothDevice", c.bluetoothDevice},
                {"wifiChannel", c.wifiChannel},
                {"requestRootForMonitor", c.requestRootForMonitor}};
}
RadioConfig fromJ(const json& j, const RadioConfig& d) {
    RadioConfig c = d;
    if (!j.is_null()) {
        c.enableWifi = j.value("enableWifi", d.enableWifi);
        c.enableBluetooth = j.value("enableBluetooth", d.enableBluetooth);
        c.enableCsi = j.value("enableCsi", d.enableCsi);
        c.autoMonitorMode = j.value("autoMonitorMode", d.autoMonitorMode);
        c.stopNetworkManager = j.value("stopNetworkManager", d.stopNetworkManager);
        c.wifiInterface = j.value("wifiInterface", d.wifiInterface);
        c.bluetoothDevice = j.value("bluetoothDevice", d.bluetoothDevice);
        c.wifiChannel = j.value("wifiChannel", d.wifiChannel);
        c.requestRootForMonitor = j.value("requestRootForMonitor", d.requestRootForMonitor);
    }
    return c;
}

json toJ(const UiConfig& c) {
    return json{{"refreshHz", c.refreshHz},
                {"historySeconds", c.historySeconds},
                {"waterfallRows", c.waterfallRows},
                {"showGrid", c.showGrid},
                {"showTrails", c.showTrails},
                {"showVelocitySpectrum", c.showVelocitySpectrum},
                {"rangeRingMetres", c.rangeRingMetres},
                {"theme", c.theme}};
}
UiConfig fromJ(const json& j, const UiConfig& d) {
    UiConfig c = d;
    if (!j.is_null()) {
        c.refreshHz = j.value("refreshHz", d.refreshHz);
        c.historySeconds = j.value("historySeconds", d.historySeconds);
        c.waterfallRows = j.value("waterfallRows", d.waterfallRows);
        c.showGrid = j.value("showGrid", d.showGrid);
        c.showTrails = j.value("showTrails", d.showTrails);
        c.showVelocitySpectrum = j.value("showVelocitySpectrum", d.showVelocitySpectrum);
        c.rangeRingMetres = j.value("rangeRingMetres", d.rangeRingMetres);
        c.theme = j.value("theme", d.theme);
    }
    return c;
}

}  // namespace

json Config::toJson() const {
    json j;
    j["presetName"] = presetName;
    j["pathLoss"] = toJ(pathLoss);
    j["window"] = toJ(window);
    j["estimator"] = toJ(estimator);
    j["spectral"] = toJ(spectral);
    j["detector"] = toJ(detector);
    j["fusion"] = toJ(fusion);
    j["radio"] = toJ(radio);
    j["ui"] = toJ(ui);
    json a = json::array();
    for (const auto& an : anchors) {
        a.push_back(json{{"id", an.id},
                         {"x", an.x},
                         {"y", an.y},
                         {"radio", toString(an.radio)},
                         {"rssiAtRefDbm", an.rssiAtRefDbm},
                         {"calibrated", an.calibrated}});
    }
    j["anchors"] = a;
    return j;
}

Config Config::fromJson(const json& j) {
    Config c;
    if (!j.is_null() && !j.is_object()) return c;
    c.presetName = j.value("presetName", c.presetName);
    c.pathLoss = fromJ(sectionOf(j, "pathLoss"), c.pathLoss);
    c.window = fromJ(sectionOf(j, "window"), c.window);
    c.estimator = fromJ(sectionOf(j, "estimator"), c.estimator);
    c.spectral = fromJ(sectionOf(j, "spectral"), c.spectral);
    c.detector = fromJ(sectionOf(j, "detector"), c.detector);
    c.fusion = fromJ(sectionOf(j, "fusion"), c.fusion);
    c.radio = fromJ(sectionOf(j, "radio"), c.radio);
    c.ui = fromJ(sectionOf(j, "ui"), c.ui);
    c.anchors.clear();
    if (j.contains("anchors") && j["anchors"].is_array()) {
        for (const auto& a : j["anchors"]) {
            Anchor an;
            an.id = a.value("id", std::string{});
            an.x = a.value("x", 0.0);
            an.y = a.value("y", 0.0);
            const std::string r = a.value("radio", std::string{"Wifi"});
            an.radio = r == "Bluetooth" ? RadioKind::Bluetooth
                       : r == "WifiCsi" ? RadioKind::WifiCsi
                                        : RadioKind::Wifi;
            an.rssiAtRefDbm = a.value("rssiAtRefDbm", -40.0);
            an.calibrated = a.value("calibrated", false);
            c.anchors.push_back(an);
        }
    }
    return c;
}

bool Config::save(const std::string& path, std::string* err) const {
    std::ofstream f(path);
    if (!f) {
        if (err) *err = "cannot open " + path + " for writing";
        return false;
    }
    f << toJson().dump(2) << "\n";
    return f.good();
}

Config Config::load(const std::string& path, std::string* err) {
    std::ifstream f(path);
    if (!f) {
        if (err) *err = "cannot open " + path;
        return Config{};
    }
    std::stringstream ss;
    ss << f.rdbuf();
    try {
        return fromJson(json::parse(ss.str()));
    } catch (const std::exception& e) {
        if (err) *err = std::string("JSON parse error: ") + e.what();
        return Config{};
    }
}

Config Config::validated() const {
    Config c = *this;
    c.pathLoss.referenceDistanceM = std::clamp(c.pathLoss.referenceDistanceM, 0.05, 10.0);
    c.pathLoss.pathLossExponent = std::clamp(c.pathLoss.pathLossExponent, 1.2, 6.0);
    c.pathLoss.shadowSigmaDb = std::clamp(c.pathLoss.shadowSigmaDb, 0.1, 30.0);
    c.pathLoss.rssiAtRefDbm = std::clamp(c.pathLoss.rssiAtRefDbm, -100.0, 0.0);
    c.pathLoss.minRssiDbm = std::clamp(c.pathLoss.minRssiDbm, -110.0, -40.0);
    c.pathLoss.maxRssiDbm = std::clamp(c.pathLoss.maxRssiDbm, -80.0, 10.0);
    if (c.pathLoss.maxRssiDbm <= c.pathLoss.minRssiDbm) c.pathLoss.maxRssiDbm = c.pathLoss.minRssiDbm + 5.0;

    c.window.windowSeconds = std::clamp(c.window.windowSeconds, 0.25, 120.0);
    c.window.decimation = std::clamp(c.window.decimation, 1, 4096);
    c.window.emaAlpha = std::clamp(c.window.emaAlpha, 0.01, 1.0);
    c.window.sgOrder = std::clamp(c.window.sgOrder, 1, 8);
    c.window.sgHalfWindow = std::clamp(c.window.sgHalfWindow, 1, 256);

    c.estimator.processNoiseAccel = std::clamp(c.estimator.processNoiseAccel, 0.01, 20.0);
    c.estimator.rangeNoiseM = std::clamp(c.estimator.rangeNoiseM, 0.05, 50.0);
    c.estimator.particleCount = std::clamp(c.estimator.particleCount, 128, 40000);
    c.estimator.resampleThreshold = std::clamp(c.estimator.resampleThreshold, 0.1, 1.0);
    c.estimator.minAnchorsForFix = std::clamp(c.estimator.minAnchorsForFix, 2, 12);

    c.spectral.welchSegment = std::clamp(c.spectral.welchSegment, 16, 65536);
    c.spectral.welchOverlap = std::clamp(c.spectral.welchOverlap, 0.0, 0.95);
    c.spectral.fMinHz = std::clamp(c.spectral.fMinHz, 1e-3, 1e4);
    c.spectral.fMaxHz = std::clamp(c.spectral.fMaxHz, c.spectral.fMinHz * 2.0, 1e5);
    c.spectral.carrierFrequencyHz = std::clamp(c.spectral.carrierFrequencyHz, 2.4e9, 6.0e9);
    c.spectral.speedOfLight = std::clamp(c.spectral.speedOfLight, 2.5e8, 3.1e8);

    c.detector.baseThresholdSigma = std::clamp(c.detector.baseThresholdSigma, 0.1, 12.0);
    c.detector.falseAlarmRate = std::clamp(c.detector.falseAlarmRate, 1e-9, 0.5);
    c.detector.cfarGuardCells = std::clamp(c.detector.cfarGuardCells, 0, 64);
    c.detector.cfarTrainCells = std::clamp(c.detector.cfarTrainCells, 2, 256);
    c.detector.entropyThreshold = std::clamp(c.detector.entropyThreshold, 1e-3, 1.0);
    c.detector.histogramBins = std::clamp(c.detector.histogramBins, 4, 128);
    c.detector.chiSquareThreshold = std::clamp(c.detector.chiSquareThreshold, 0.1, 1e5);
    c.detector.holdSeconds = std::clamp(c.detector.holdSeconds, 0.05, 60.0);
    c.detector.departSeconds = std::clamp(c.detector.departSeconds, 0.05, 120.0);

    c.fusion.wifiWeight = std::clamp(c.fusion.wifiWeight, 0.0, 10.0);
    c.fusion.bluetoothWeight = std::clamp(c.fusion.bluetoothWeight, 0.0, 10.0);
    c.fusion.radioGateConfidence = std::clamp(c.fusion.radioGateConfidence, 0.0, 1.0);
    c.fusion.priorLogOddsMotion = std::clamp(c.fusion.priorLogOddsMotion, -15.0, 15.0);

    c.ui.refreshHz = std::clamp(c.ui.refreshHz, 1.0, 120.0);
    c.ui.historySeconds = std::clamp(c.ui.historySeconds, 5.0, 600.0);
    c.ui.waterfallRows = std::clamp(c.ui.waterfallRows, 32, 2000);
    c.ui.rangeRingMetres = std::clamp(c.ui.rangeRingMetres, 0.5, 500.0);

    // Keep RSSI-derived range noise consistent with the shadowing model.
    c.estimator.rangeNoiseM =
        std::clamp(c.estimator.rangeNoiseM, c.pathLoss.shadowSigmaDb * 0.2,
                   c.pathLoss.shadowSigmaDb * 4.0 + 0.5);
    return c;
}

// =============================================================== presets

std::vector<Preset> builtinPresets() {
    std::vector<Preset> out;
    Config base;

    {
        Preset p;
        p.name = "Balanced Indoor";
        p.description =
            "Default operating point for a cluttered indoor room. 2.8 path-loss "
            "exponent, 4 s window, all three detectors must agree.";
        p.config = base;
        out.push_back(p);
    }
    {
        Preset p;
        p.name = "Breath Detection";
        p.description =
            "Maximises sensitivity to millimetre-scale chest displacement. Long "
            "8 s window, heavy Savitzky-Golay smoothing, low threshold, path-loss "
            "exponent pushed to 3.4 for heavy clutter. Expect false alarms in a "
            "room with fans or blinds.";
        p.config = base;
        p.config.window.windowSeconds = 8.0;
        p.config.window.sgHalfWindow = 12;
        p.config.window.sgOrder = 4;
        p.config.window.emaAlpha = 0.18;
        p.config.detector.baseThresholdSigma = 1.6;
        p.config.detector.falseAlarmRate = 0.08;
        p.config.detector.holdSeconds = 3.0;
        p.config.detector.entropyThreshold = 0.10;
        p.config.pathLoss.pathLossExponent = 3.4;
        p.config.pathLoss.shadowSigmaDb = 2.2;
        out.push_back(p);
    }
    {
        Preset p;
        p.name = "Fast Intrusion";
        p.description =
            "Minimum latency for door and hallway events. 0.75 s window, no "
            "smoothing, aggressive CFAR. Trades false alarms for a sub-second "
            "time-to-detect.";
        p.config = base;
        p.config.window.windowSeconds = 0.75;
        p.config.window.sgHalfWindow = 2;
        p.config.window.emaAlpha = 0.7;
        p.config.detector.baseThresholdSigma = 2.6;
        p.config.detector.falseAlarmRate = 0.01;
        p.config.detector.cfarTrainCells = 16;
        p.config.detector.holdSeconds = 0.5;
        p.config.detector.departSeconds = 0.8;
        p.config.estimator.processNoiseAccel = 2.5;
        out.push_back(p);
    }
    {
        Preset p;
        p.name = "Doppler Velocity Focus";
        p.description =
            "Optimised for the Burg/MUSIC beat-frequency estimator: 16 s record so "
            "the 0.02-4 Hz band is resolvable, high AR order, window trimmed to the "
            "radial-velocity band. Reports m/s directly from the two-path model.";
        p.config = base;
        p.config.window.windowSeconds = 16.0;
        p.config.window.sgHalfWindow = 3;
        p.config.window.decimation = 1;
        p.config.spectral.welchSegment = 256;
        p.config.spectral.fMinHz = 0.2;
        p.config.spectral.fMaxHz = 18.0;
        p.config.spectral.enableVelocity = true;
        p.config.detector.baseThresholdSigma = 2.2;
        p.config.detector.holdSeconds = 1.0;
        out.push_back(p);
    }
    {
        Preset p;
        p.name = "Precision Triangulation";
        p.description =
            "Position-first mode. Requires four anchors before it will produce a "
            "fix, heavy process damping, 12000 particles. Slower, but the "
            "trajectory is far steadier.";
        p.config = base;
        p.config.estimator.useAnchorTrilateration = true;
        p.config.estimator.minAnchorsForFix = 4;
        p.config.estimator.processNoiseAccel = 0.35;
        p.config.estimator.particleCount = 12000;
        p.config.estimator.rangeNoiseM = 0.9;
        p.config.pathLoss.shadowSigmaDb = 2.4;
        p.config.window.windowSeconds = 3.0;
        p.config.detector.baseThresholdSigma = 2.8;
        out.push_back(p);
    }
    {
        Preset p;
        p.name = "Far-Field Presence";
        p.description =
            "Long-range occupancy for a room or hallway. Stiff path-loss exponent "
            "4.0, wide 6 s window, and the BT radio weighted up because it is the "
            "only one still hearing at distance on this hardware.";
        p.config = base;
        p.config.pathLoss.pathLossExponent = 4.0;
        p.config.pathLoss.shadowSigmaDb = 4.5;
        p.config.pathLoss.maxRssiDbm = -30.0;
        p.config.window.windowSeconds = 6.0;
        p.config.window.sgHalfWindow = 8;
        p.config.fusion.bluetoothWeight = 1.1;
        p.config.fusion.wifiWeight = 0.8;
        p.config.detector.baseThresholdSigma = 2.0;
        p.config.detector.holdSeconds = 2.5;
        out.push_back(p);
    }
    {
        Preset p;
        p.name = "Dense Urban Clutter";
        p.description =
            "Multipath hell: hallways, metal, mirrors. High shadow sigma so the "
            "CFAR floor rises with the clutter, strict entropy gate, and a 3.2 "
            "exponent to stop furniture from reading as a person.";
        p.config = base;
        p.config.pathLoss.pathLossExponent = 3.2;
        p.config.pathLoss.shadowSigmaDb = 6.0;
        p.config.window.windowSeconds = 5.0;
        p.config.detector.baseThresholdSigma = 3.6;
        p.config.detector.falseAlarmRate = 0.004;
        p.config.detector.entropyThreshold = 0.24;
        p.config.detector.chiSquareThreshold = 26.0;
        p.config.detector.useChiSquare = true;
        out.push_back(p);
    }
    {
        Preset p;
        p.name = "Lab Baseline (Strict)";
        p.description =
            "For validating a measurement rig with no people in the room. Both the "
            "entropy and chi-square gates must trip and the threshold is set from a "
            "6-sigma baseline. This is the preset you use to measure your own false "
            "alarm rate.";
        p.config = base;
        p.config.detector.baseThresholdSigma = 6.0;
        p.config.detector.falseAlarmRate = 1e-5;
        p.config.detector.entropyThreshold = 0.30;
        p.config.detector.chiSquareThreshold = 40.0;
        p.config.detector.useEntropy = true;
        p.config.detector.useChiSquare = true;
        p.config.detector.cfarTrainCells = 32;
        p.config.detector.cfarGuardCells = 6;
        p.config.window.windowSeconds = 10.0;
        out.push_back(p);
    }
    return out;
}

const Preset* findPreset(const std::vector<Preset>& list, const std::string& name) {
    for (const auto& p : list)
        if (p.name == name) return &p;
    return nullptr;
}

Config applyPreset(const Config& base, const Config& presetCfg) {
    Config out = presetCfg;
    // A preset tunes the algorithm; it must not silently relocate the operator's
    // access points.
    out.anchors = base.anchors;
    out.ui = base.ui;
    out.radio = base.radio;
    out.presetName = presetCfg.presetName;
    return out.validated();
}

}  // namespace radar