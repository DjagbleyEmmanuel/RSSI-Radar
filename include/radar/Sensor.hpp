// Pluggable sensor layer.
//
// Every radio implements this interface. The engine never knows or cares which
// one produced an Observation, so a CSI-capable dongle dropped in later appears
// as RadioKind::WifiCsi and takes the same downstream path as the RSSI radios,
// just with the csi field actually populated.
#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "radar/Config.hpp"
#include "radar/Types.hpp"

namespace radar {

// Privilege tests.
//
// These were previously `geteuid() == 0`, which is wrong for the installed
// build: the binary carries cap_net_raw,cap_net_admin as *file* capabilities,
// so an ordinary launch from the desktop runs with euid 1000 while holding
// CAP_NET_ADMIN in its effective set. Testing the uid therefore reported "no
// privilege" for a process that could in fact perform every ioctl the sensor
// needs, so monitor mode was silently skipped and the raw socket was opened on a
// managed interface -- where there is no radiotap, hence no readings and no
// visible reason why.
bool hasNetAdminPrivilege();

// Driving NetworkManager is different: `systemctl`/`nmcli` talk to D-Bus and
// polkit checks the real uid, which file capabilities do not satisfy. Without
// this the channel cannot be pinned and the capture listens to channel 1.
bool hasNetworkManagerPrivilege();

struct Capabilities {
    bool rssi = false;
    bool csi = false;
    bool transmitterIdentity = false;
    bool kernelTimestamps = false;
    bool perAntennaRssi = false;
    bool injection = false;
    bool passiveOnly = true;
    bool directionFinding = false;
    bool longRangePhy = false;
    std::string notes;
};

// Why a sensor is unavailable. Shown verbatim in the UI so the operator is
// never misled into thinking a working pipeline is producing real data.
enum class UnavailableReason {
    None,
    NotPresent,      // no such device on this machine
    Permissions,     // needs root / CAP_NET_RAW
    DriverMismatch,  // present, but the driver cannot do what we need
    Busy,            // device held by another program
    RadioDisabled,   // rfkill
    Stalled          // present but producing no data
};

const char* toString(UnavailableReason r);

class ISensor {
  public:
    virtual ~ISensor() = default;

    virtual std::string name() const = 0;
    virtual RadioKind kind() const = 0;
    virtual Capabilities capabilities() const = 0;

    virtual bool start(const Config& cfg, std::string* err) = 0;
    virtual void stop() = 0;
    virtual bool isActive() const = 0;

    virtual SensorStatus status() const = 0;
    // Reason this sensor cannot currently contribute.
    virtual UnavailableReason unavailableReason() const = 0;

    // Non-blocking. Returns false when the queue is empty. Implemented once in
    // the base against the shared queue; subclasses only fill it.
    virtual bool tryPop(Observation& out);
    // Blocking up to `timeoutMs`. Returns false on timeout or after stop().
    virtual bool pop(Observation& out, int timeoutMs);

    // Human-readable probe of what the hardware actually reports, used by
    // --selftest. Never fabricates: it reads the real device state.
    virtual std::vector<std::string> probe(std::string* err) = 0;

  protected:
    void push(const Observation& o);
    void setActive(bool a) { active_.store(a, std::memory_order_relaxed); }
    void requestStop() { stopping_.store(true); }
    bool stopRequested() const { return stopping_.load(); }
    void wakeConsumers() { qCv_.notify_all(); }
    void bumpSeen() { seen_.fetch_add(1, std::memory_order_relaxed); }
    void bumpDropped() { dropped_.fetch_add(1, std::memory_order_relaxed); }
    std::mutex& detailMtx() const { return detailMtx_; }
    std::string detail() const {
        std::lock_guard<std::mutex> lk(detailMtx_);
        return detail_;
    }
    // Stable configuration description, e.g. "monitor mode on channel 6".
    //
    // Appends rather than overwrites. It used to assign, so a sensor kept only
    // the single most recent line and every earlier diagnostic -- which channel
    // was chosen, whether NetworkManager was released, what the monitor-mode
    // attempt returned -- was silently discarded. Diagnosing a silent capture
    // then meant guessing, because the status line showed the last thing that
    // happened rather than the sequence that led there.
    void note(const std::string& s) {
        std::lock_guard<std::mutex> lk(detailMtx());
        if (!detail_.empty()) detail_ += "  |  ";
        detail_ += s;
        while (detail_.size() > 900) {
            const size_t cut = detail_.find("  |  ");
            if (cut == std::string::npos) {
                detail_.erase(0, detail_.size() - 900);
                break;
            }
            detail_.erase(0, cut + 4);
        }
    }
    // Transient runtime problem, reported separately so it never overwrites the
    // configuration description.
    void setLastError(const std::string& s) {
        std::lock_guard<std::mutex> lk(detailMtx());
        lastError_ = s;
    }
    // True when the error string predates the most recent successful read.
    bool lastErrorIsStale() const { return !lastError().empty(); }
    std::string lastError() const {
        std::lock_guard<std::mutex> lk(detailMtx_);
        return lastError_;
    }

    std::atomic<bool> active_{false};
    std::atomic<bool> stopping_{false};
    std::condition_variable qCv_;
    uint64_t seen() const { return seen_.load(std::memory_order_relaxed); }
    uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

  private:
    mutable std::mutex detailMtx_;
    std::string detail_;
    std::string lastError_;
    mutable std::mutex qMtx_;
    std::queue<Observation> queue_;
    std::atomic<uint64_t> seen_{0};
    std::atomic<uint64_t> dropped_{0};
    static constexpr size_t kMaxQueue = 65536;
};

// ---------------------------------------------------------------- backends

// Passive WiFi radiometry over AF_PACKET with radiotap header parsing and
// kernel timestamping. This is the workhorse: it works on the Intel 7265,
// which cannot provide CSI but does provide a clean per-frame RSSI.
class WifiRadiometricSensor : public ISensor {
  public:
    WifiRadiometricSensor();
    ~WifiRadiometricSensor() override;

    std::string name() const override { return "WiFi Radiometric (AF_PACKET)"; }
    RadioKind kind() const override { return RadioKind::Wifi; }
    Capabilities capabilities() const override;

    bool start(const Config& cfg, std::string* err) override;
    void stop() override;
    bool isActive() const override { return active_.load(std::memory_order_relaxed); }
    SensorStatus status() const override;
    UnavailableReason unavailableReason() const override { return reason_; }
    std::vector<std::string> probe(std::string* err) override;

    // Number of frames whose 802.11 header we could parse as a usable
    // transmitter observation.
    uint64_t usableFrames() const { return usable_.load(std::memory_order_relaxed); }
    // Frames the kernel delivered that carried no radiotap header, i.e. the
    // interface is not in monitor mode and no RSSI exists.
    uint64_t framesWithoutRadiotap() const { return noRadiotap_.load(std::memory_order_relaxed); }

  private:
    void captureLoop();
    bool openSocket(std::string* err);
    // A short-lived datagram socket used purely for SIOCGIFFLAGS/SIOCSIFFLAGS.
    int sockForIoctl();
    // Returns empty on success, otherwise why monitor mode could not be entered.
    // pinChannel=false lets the firmware tune itself, which is what actually
    // works on iwlwifi; pinning is only honoured when explicitly configured.
    std::string enterMonitorMode(int chan, bool pinChannel);
    // True once at least one frame has actually arrived from the kernel.
    // True once a frame carrying a real radiotap header has arrived.
    bool waitForRadiometricFrame(int timeoutMs);
    // Channel of the strongest visible network, 0 if unknown.
    int findBestChannel();
    // Create a dedicated monitor interface alongside the managed one. This is
    // the reliable path on iwlwifi: converting the managed interface itself
    // repeatedly left the driver delivering bare 802.11 with no radiotap, and
    // it disturbs the machine's connectivity. Returns the capture interface.
    std::string ensureMonitorInterface();
    void removeMonitorInterface();
    void closeSocket();
    bool parseRadiotap(const uint8_t* buf, size_t len, Observation& out, bool& isData);
    static bool parseDot11(const uint8_t* buf, size_t len, Observation& out);

    // Hand every wireless interface still in monitor mode back to
    // NetworkManager. Called on start, so a previous crash or `kill -9` can
    // never leave the machine without a working connection.
    void reclaimStaleInterfaces();
    // Bring the saved wireless profile back up and wait for a real associated
    // channel. Returns the channel, or 0 if the band could not be resolved.
    int forceReassociate();
    // Record the network this interface is on now, so exactly that one can be
    // rejoined later. Never activates a profile on its own.
    void rememberCurrentNetwork();
    // Run a command needing real root, escalating through pkexec when the
    // process is not root.
    bool runPrivileged(const std::string& cmd, std::string* out);
    // Hand the interface and NetworkManager back, exactly once, in that order.
    void restoreNetworkManager();
    // Switch the interface back to managed via nl80211. 0 on success.
    int nl80211SetInterfaceManaged();
    // Switch the interface type via nl80211 (monitor / managed). 0 on success.
    int nl80211SetInterfaceType(uint8_t iftype);
    // Pin the synthesiser to a channel. 0 on success, -EBUSY when already tuned.
    int nl80211SetChannel(int phy, int channel);
    uid_t getpwnam_real_uid() const;
    // One recovery attempt when capture has gone quiet. Returns true if the
    // interface was reconfigured and the caller should keep going.
    bool attemptRecovery();
    // Re-pin the synthesiser to the target channel, which is what silently
    // breaks when the access point roams to another channel.
    bool repinChannel();

    int fd_ = -1;
    std::thread thread_;
    std::string iface_;      // the physical device, used for iw/nmcli commands
    std::string capIface_;   // the interface we actually capture on
    std::string monIface_;   // dedicated monitor interface, when we created one
    uint32_t ifIndex_ = 0;
    bool promisc_ = false;
    UnavailableReason reason_ = UnavailableReason::None;
    std::atomic<uint64_t> usable_{0};
    std::atomic<uint64_t> radiotapFails_{0};
    std::atomic<uint64_t> noRadiotap_{0};
    std::atomic<uint64_t> noSignal_{0};
    bool haveSignalField_ = false;
    bool monitorMode_ = false;
    bool nmStopped_ = false;
    bool nmReleased_ = false;   // device handed back via `nmcli ... managed no`
    // The network this interface was associated with at start-up. Only this one
    // is ever reconnected.
    std::string savedProfile_;
    std::string savedSsid_;
    // The channel is pinned by configuration, so no association is needed.
    bool channelAlreadyKnown_ = false;
    int autoChannel_ = 0;
    double sampleRate_ = 100.0;

    // Stall detection. lastGoodFrameNs_ is stamped whenever a usable
    // radiometric frame is parsed; if it goes stale the capture thread tries to
    // recover instead of leaving the UI showing a frozen trace forever.
    std::atomic<uint64_t> lastGoodFrameNs_{0};
    std::atomic<uint64_t> recoveryCount_{0};
    std::atomic<int> stalled_{0};
    std::string stallReason_;
    mutable std::mutex stallMtx_;
};

// CSI provider. Auto-detects a Qualcomm Atheros interface that actually exposes
// CSI (ath10k/ath9k), and reports honestly when only iwlwifi is present.
class CsiSensor : public ISensor {
  public:
    CsiSensor();
    ~CsiSensor() override;

    std::string name() const override { return "WiFi CSI (ath10k/ath9k)"; }
    RadioKind kind() const override { return RadioKind::WifiCsi; }
    Capabilities capabilities() const override;

    bool start(const Config& cfg, std::string* err) override;
    void stop() override;
    bool isActive() const override { return active_.load(std::memory_order_relaxed); }
    SensorStatus status() const override;
    UnavailableReason unavailableReason() const override { return reason_; }
    std::vector<std::string> probe(std::string* err) override;

  private:
    void captureLoop();
    UnavailableReason reason_ = UnavailableReason::None;
    std::string driver_;
    std::string iface_;
    std::string csiIface_;
    int fd_ = -1;
    int csiFd_ = -1;
    std::thread thread_;
    std::atomic<uint64_t> usable_{0};
};

// Bluetooth LE / BR-EDR sensor over a raw AF_BLUETOOTH HCI socket. Harvests
// real RSSI out of LE advertising reports and inquiry responses.
class BluetoothSensor : public ISensor {
  public:
    BluetoothSensor();
    ~BluetoothSensor() override;

    std::string name() const override { return "Bluetooth LE (raw HCI)"; }
    RadioKind kind() const override { return RadioKind::Bluetooth; }
    Capabilities capabilities() const override;

    bool start(const Config& cfg, std::string* err) override;
    void stop() override;
    bool isActive() const override { return active_.load(std::memory_order_relaxed); }
    SensorStatus status() const override;
    UnavailableReason unavailableReason() const override { return reason_; }
    std::vector<std::string> probe(std::string* err) override;

    // Locally advertised beacon so the pipeline has a known, controllable
    // transmitter to observe. Real over-the-air frames, not synthetic.
    bool startAdvertising(uint16_t advIntervalMs = 100);

  private:
    void hciLoop();
    bool sendCommand(uint16_t ogf, uint16_t ocf, const std::vector<uint8_t>& params);
    void handleEvent(const uint8_t* p, size_t len);

    int fd_ = -1;
    int devId_ = 0;
    std::string devName_;
    std::thread thread_;
    UnavailableReason reason_ = UnavailableReason::None;
    std::atomic<uint64_t> advReports_{0};
    std::atomic<uint64_t> inquiryResults_{0};
    std::atomic<bool> scanning_{false};
};

// --------------------------------------------------------------- registry

// Hardware survey. Populates every backend that this machine can actually run,
// so the UI can show present-but-unavailable alongside live sensors.
struct HardwareSurvey {
    std::vector<SensorStatus> wifi;
    std::vector<SensorStatus> bluetooth;
    std::vector<SensorStatus> csi;
    std::string wifiDriver;
    std::string wifiDevice;
    std::string wifiFirmware;
    std::string regDomainCountry;
    bool regDomainPassiveScan = false;
    bool monitorCapable = false;
    bool injectionSupported = false;
    bool csiCapable = false;
    bool btLongRangePhy = false;
    bool btDirectionFinding = false;
    std::vector<std::string> notes;
};

HardwareSurvey surveyHardware();
std::string helperRun(const std::string& exe, const std::string& args);

}  // namespace radar