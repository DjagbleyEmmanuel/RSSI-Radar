// WiFi radiometric sensor.
//
// Real AF_PACKET capture. Every RSSI this file produces is read out of a
// radiotap header that the hardware/driver actually emitted -- there is no
// model, no simulator and no fallback that invents a value.
#include "radar/Sensor.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <linux/if_packet.h>
#include <linux/netlink.h>

#include <linux/genetlink.h>
#include <linux/nl80211.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>

namespace radar {

namespace {

constexpr int kSolSocket = 1;

// How long the capture may produce nothing usable before the watchdog decides
// something has gone wrong and tries to fix it. Deliberately generous: a quiet
// band is not a fault, and recovering needlessly would itself cause churn.
constexpr std::chrono::seconds kStallTimeout{8};
constexpr uint64_t kStallTimeoutNs = 8000000000ULL;

uint64_t nowNs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
            .count());
}

// Single-quote a string for /bin/sh. Interface and connection names routinely
// contain spaces, and every one of these commands is assembled as a shell
// string, so this is not optional.
std::string shellQuote(const std::string& s) {
    std::string out = "'";
    for (char c : s) {
        if (c == '\'')
            out += "'\\''";
        else
            out += c;
    }
    out += "'";
    return out;
}

constexpr int kSoTimestampNs = 35;   // SO_TIMESTAMPNS
constexpr int kScmTimestampNs = 35;  // SCM_TIMESTAMPNS
constexpr int kPacketMrPromisc = 1;
constexpr int kPacketAddMembership = 1;
constexpr uint16_t kEthPAll = 0x0003;
constexpr size_t kFrameCap = 4096;

#pragma pack(push, 1)
struct RadiotapHeader {
    uint8_t version;
    uint8_t pad;
    uint16_t len;
    uint32_t present;
};
#pragma pack(pop)

// Private-ioctl payload. struct iwreq from <linux/wireless.h> is
// iwl-specific enough to vary, so mirror only the fields we touch.
struct RadarIwReq {
    // NOT named ifr_name: <net/if.h> #defines that token.
    char dev[16];
    uint16_t ifr_magic;   // 0x8B1C
    uint16_t ifr_mode;
    uint16_t ifr_bitrate;
    void* ifr_unused;
};

constexpr uint16_t kIwMagic = 0x8B1C;
constexpr uint16_t kIwModeMonitor = 1;
constexpr int kSiocDevPrivate = 0x89F0;
constexpr int kSiocSIWMode = 0x8B1C;

// Radiotap present-bit indices we care about.
enum RadiotapIndex {
    kTsft = 0,
    kFlags = 1,
    kRate = 2,
    kChannel = 3,
    kFhss = 4,
    kAntsignal = 5,
    kAntnoise = 6,
    kLockQuality = 7,
    kTxAttenuation = 8,
    kDbmTxPower = 9,
    kAntenna = 11,
    kAntsignalMax = 12,
    kXchannel = 18,
    kMcs = 19,
    kAmpdu = 20,
    kVht = 21,
    kTimestamp = 22,
    kHe = 23,
};

// Defined below; runs a helper and returns its exit status.
int runCapture(const std::string& cmd, std::string* out);

}  // namespace

const char* toString(RadioKind k) {
    switch (k) {
        case RadioKind::Wifi: return "WiFi";
        case RadioKind::WifiCsi: return "WiFi-CSI";
        case RadioKind::Bluetooth: return "Bluetooth";
        default: return "Unknown";
    }
}

const char* toString(FrameClass c) {
    switch (c) {
        case FrameClass::Management: return "Management";
        case FrameClass::Control: return "Control";
        case FrameClass::Data: return "Data";
        case FrameClass::Extension: return "Extension";
        default: return "Unknown";
    }
}

const char* toString(DetectionState s) {
    switch (s) {
        case DetectionState::Clear: return "CLEAR";
        case DetectionState::Present: return "PRESENT";
        case DetectionState::Departed: return "DEPARTED";
        case DetectionState::Unavailable: return "UNAVAILABLE";
    }
    return "?";
}

const char* toString(UnavailableReason r) {
    switch (r) {
        case UnavailableReason::None: return "ready";
        case UnavailableReason::NotPresent: return "device not present";
        case UnavailableReason::Permissions: return "insufficient privileges (needs CAP_NET_RAW)";
        case UnavailableReason::DriverMismatch: return "driver cannot provide required data";
        case UnavailableReason::Busy: return "device held by another program";
        case UnavailableReason::RadioDisabled: return "blocked by rfkill";
        case UnavailableReason::Stalled: return "device present but producing no data";
    }
    return "?";
}

std::string macToString(const std::array<uint8_t, 6>& mac) {
    char buf[18];
    std::snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2],
                  mac[3], mac[4], mac[5]);
    return std::string(buf);
}

uint64_t macHash(const std::array<uint8_t, 6>& mac) {
    uint64_t h = 1469598103934665603ull;
    for (uint8_t b : mac) {
        h ^= b;
        h *= 1099511628211ull;
    }
    return h;
}

double RssiTrack::mean() const {
    if (samplesDbm.empty()) return 0.0;
    double s = 0;
    for (double v : samplesDbm) s += v;
    return s / static_cast<double>(samplesDbm.size());
}

double RssiTrack::variance() const {
    if (samplesDbm.size() < 2) return 0.0;
    const double m = mean();
    double s = 0;
    for (double v : samplesDbm) s += (v - m) * (v - m);
    return s / static_cast<double>(samplesDbm.size());
}

double RssiTrack::stddev() const { return std::sqrt(variance()); }

double RssiTrack::min() const {
    if (samplesDbm.empty()) return 0.0;
    return *std::min_element(samplesDbm.begin(), samplesDbm.end());
}

double RssiTrack::max() const {
    if (samplesDbm.empty()) return 0.0;
    return *std::max_element(samplesDbm.begin(), samplesDbm.end());
}

double RssiTrack::meanAbsDiff() const {
    if (samplesDbm.size() < 2) return 0.0;
    double s = 0;
    for (size_t i = 1; i < samplesDbm.size(); ++i)
        s += std::fabs(samplesDbm[i] - samplesDbm[i - 1]);
    return s / static_cast<double>(samplesDbm.size() - 1);
}

void ISensor::push(const Observation& o) {
    {
        std::lock_guard<std::mutex> lk(qMtx_);
        if (queue_.size() >= kMaxQueue) {
            queue_.pop();
            dropped_.fetch_add(1, std::memory_order_relaxed);
        }
        queue_.push(o);
    }
    qCv_.notify_one();
}

bool ISensor::tryPop(Observation& out) {
    std::lock_guard<std::mutex> lk(qMtx_);
    if (queue_.empty()) return false;
    out = queue_.front();
    queue_.pop();
    return true;
}

bool ISensor::pop(Observation& out, int timeoutMs) {
    std::unique_lock<std::mutex> lk(qMtx_);
    if (queue_.empty()) {
        qCv_.wait_for(lk, std::chrono::milliseconds(timeoutMs),
                      [this] { return !queue_.empty() || stopping_.load(); });
    }
    if (queue_.empty()) return false;
    out = queue_.front();
    queue_.pop();
    return true;
}

// ==================================================== WifiRadiometricSensor

WifiRadiometricSensor::WifiRadiometricSensor() = default;

WifiRadiometricSensor::~WifiRadiometricSensor() { stop(); }

Capabilities WifiRadiometricSensor::capabilities() const {
    Capabilities c;
    c.rssi = true;
    // Deliberately false. Intel iwlwifi has the internal mCSI/TOF code compiled
    // in, but mainline exports no nl80211 vendor command for it, so userspace
    // cannot read phase/amplitude. Verified on this machine: `iw phy` lists no
    // supported vendor commands at all.
    c.csi = false;
    c.transmitterIdentity = true;
    c.kernelTimestamps = true;  // SO_TIMESTAMPNS on the packet socket
    c.perAntennaRssi = false;    // needs XCHANNEL antenna mask + multi-chain
    c.injection = false;         // iwlwifi/8265 injection is not supported
    c.passiveOnly = true;
    c.directionFinding = false;
    c.longRangePhy = false;
    c.notes =
        "RSSI only. Per-frame received power with kernel-nanosecond timestamps. "
        "No CSI: iwlwifi does not expose phase/amplitude to userspace.";
    return c;
}

bool WifiRadiometricSensor::start(const Config& cfg, std::string* err) {
    stop();
    requestStop();
    stopping_.store(false);

    // Resolve the interface: honour an explicit choice, otherwise pick the
    // first wireless device that actually exists.
    iface_ = cfg.radio.wifiInterface;
    if (iface_.empty() || iface_ == "auto") {
        std::ifstream in("/proc/net/dev");
        std::string line;
        std::getline(in, line);
        while (std::getline(in, line)) {
            const size_t colon = line.find(':');
            if (colon == std::string::npos) continue;
            std::string name = line.substr(0, colon);
            while (!name.empty() && std::isspace(static_cast<unsigned char>(name.front())))
                name.erase(name.begin());
            if (name.rfind("wl", 0) == 0) {
                iface_ = name;
                break;
            }
        }
        if (iface_.empty()) {
            reason_ = UnavailableReason::NotPresent;
            if (err) *err = "no wireless interface found in /proc/net/dev";
            return false;
        }
    }

    ifIndex_ = ::if_nametoindex(iface_.c_str());
    if (ifIndex_ == 0) {
        reason_ = UnavailableReason::NotPresent;
        if (err) *err = "interface " + iface_ + " does not exist";
        return false;
    }

    // Optionally flip the interface to monitor mode. Skipped when we lack
    // privileges, because a half-configured card is worse than a managed one.
    // Prefer a dedicated monitor interface. It leaves the managed connection
    // completely alone, so NetworkManager never fights us and the user keeps
    // their internet while the radar runs.
    capIface_ = iface_;

    // NetworkManager has to be stopped *before* the channel can be pinned.
    //
    // While NM holds the device, iwlwifi rejects SIOCSIWCHAN with EBUSY, so the
    // channel can never be set and a monitor interface stays on the synthesiser
    // default of channel 1. If the access point is on any other channel the card
    // then hears literally nothing -- which is exactly what was observed here:
    //
    //   NM running            -> set channel refused, 0 frames
    //   NM stopped, ch pinned -> 40/40 frames carried radiotap and a signal
    //
    // This is the only configuration that produced radiometric frames on this
    // card, so it is the default.
    //
    // Do this *after* reclaiming, so a stale monitor interface from a crashed
    // run is cleaned up before anything else touches the device.
    // Recorded first: after this point the interface is taken out of managed
    // mode and the association is gone, so there is no second chance to learn
    // which network we were on.
    rememberCurrentNetwork();
    reclaimStaleInterfaces();

    // Resolve the channel to listen on *while NetworkManager is still running*.
    // Once it is stopped the only channel information available is whatever the
    // synthesiser happens to be tuned to, and if the interface is not associated
    // that is the power-on default of channel 1 -- which is why an unresolved
    // channel has to be treated as a hard error rather than a fallback.
    autoChannel_ = cfg.radio.wifiChannel;
    if (autoChannel_ <= 0) {
        std::string info;
        runCapture("iw dev " + iface_ + " info", &info);
        const size_t k = info.find("channel ");
        if (k != std::string::npos) autoChannel_ = std::atoi(info.c_str() + k + 8);
    }
    if (autoChannel_ <= 0) autoChannel_ = forceReassociate();

    // Still nothing: the device is most likely sitting `unmanaged` -- left that
    // way by an earlier run that died between releasing it and handing it back.
    // That is exactly the kind of state this program is supposed to repair by
    // itself, so repair it rather than only reporting it.
    if (autoChannel_ <= 0) {
        std::string info;
        runCapture("iw dev " + iface_ + " info", &info);
        // `iw dev info` reports "type managed" even for a device NetworkManager
        // has released, so the authoritative test is NetworkManager's own view.
        // The field is GENERAL.NM-MANAGED; GENERAL.MANAGED is rejected by nmcli
        // as an invalid field, which made this whole recovery unreachable.
        if (info.find("unmanaged") != std::string::npos ||
            runCapture("nmcli -t -f GENERAL.NM-MANAGED device show " + iface_, &info) == 0) {
            std::string out;
            runCapture("nmcli device set " + iface_ + " managed yes", &out);
            note("reclaimed " + iface_ + " from a previous run (was unmanaged)");
            autoChannel_ = forceReassociate();
        }
    }

    if (autoChannel_ <= 0) {
        // Distinguish "not joined to anything" from "NetworkManager has lost
        // track of the adapter". The second needs root to fix, so saying so is
        // the difference between a user who can fix it in one command and one
        // who goes hunting through the network stack.
        std::string known;
        runCapture("nmcli -t -f GENERAL.DEVICE device show", &known);
        const bool nmSeesDevice = known.find(iface_) != std::string::npos;
        if (err) {
            if (!nmSeesDevice)
                *err = "NetworkManager no longer knows about " + iface_ +
                       ". This happens when the service was stopped outside this "
                       "program; it cannot be repaired without root. Run:\n"
                       "    sudo systemctl restart NetworkManager\n"
                       "then reconnect to your network and start again.";
            else
                *err = "could not determine a WiFi channel to listen on; " + iface_ +
                       " is not associated with any network. Connect to a network "
                       "and start again.";
        }
        reason_ = UnavailableReason::NotPresent;
        restoreNetworkManager();   // never leave the device released on failure
        return false;
    }
    note(std::string("listening on channel ") + std::to_string(autoChannel_) +
         (cfg.radio.wifiChannel > 0 ? " (configured)" : " (from the associated network)"));

    // Free the synthesiser before pinning the channel.
    //
    // Measured on this card: with NetworkManager still associated the interface
    // accepts the type switch but delivers nothing at all, because the firmware
    // stays in its association follow-set. Releasing the device is what makes the
    // pin take effect. As root the service can simply be stopped; unprivileged,
    // `nmcli device set managed no` is permitted for the active local user
    // without a password and is enough.
    nmReleased_ = false;
    if (cfg.radio.stopNetworkManager) {
        std::string out;
        const bool nmActive = runCapture("systemctl is-active NetworkManager", &out) == 0 &&
                              out.find("active") != std::string::npos;
        if (nmActive) {
            if (hasNetworkManagerPrivilege()) {
                std::string err;
                if (runPrivileged("systemctl stop NetworkManager", &err)) {
                    nmStopped_ = true;
                    nmReleased_ = true;
                    note("NetworkManager stopped so the channel can be pinned");
                } else {
                    note("could not stop NetworkManager (" + err + ")");
                }
            } else if (runCapture("nmcli device set " + iface_ + " managed no", &out) == 0) {
                nmReleased_ = true;
                note("released " + iface_ + " from NetworkManager so the channel can be pinned");
            } else if (!nmActive) {
                nmReleased_ = true;
            } else {
                note("could not release " + iface_ + " from NetworkManager (" + out + ")");
            }
        }
    }

    if (cfg.radio.autoMonitorMode && hasNetAdminPrivilege()) {
        std::string report = "no attempt made";
        for (int attempt = 1; attempt <= 3; ++attempt) {
            // 0 means "read the channel from the associated access point", which
            // enterMonitorMode does while the link is still up.
            report = enterMonitorMode(cfg.radio.wifiChannel, true);
            if (report.empty()) {
                if (!openSocket(err)) return false;
                const bool got = waitForRadiometricFrame(3000);
                closeSocket();
                if (got) {
                    monitorMode_ = true;
                    note(detail());
                    break;
                }
                report = "converted to monitor mode but no radiotap frame arrived";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(700));
        }
        monitorMode_ = report.empty();
        if (!monitorMode_) note("monitor mode unusable: " + report);
    }

    // Open the packet socket only after the mode switch and the final link-up:
    // an AF_PACKET binding does not survive an interface type change or a down
    // transition, so a socket opened first is left attached to nothing.
    if (fd_ < 0 && !openSocket(err)) return false;

    reason_ = UnavailableReason::None;
    setActive(true);
    thread_ = std::thread(&WifiRadiometricSensor::captureLoop, this);
    return true;
}

bool WifiRadiometricSensor::openSocket(std::string* err) {
    fd_ = ::socket(AF_PACKET, SOCK_RAW, ::htons(kEthPAll));
    if (fd_ < 0) {
        reason_ = (errno == EPERM || errno == EACCES) ? UnavailableReason::Permissions
                                                       : UnavailableReason::NotPresent;
        if (err)
            *err = std::string("AF_PACKET socket: ") + std::strerror(errno) +
                   " (raw capture needs root or CAP_NET_RAW)";
        return false;
    }
    // Kernel nanosecond receive timestamps: this is what gives the pipeline
    // real inter-packet timing rather than userspace scheduling jitter.
    int one = 1;
    ::setsockopt(fd_, kSolSocket, kSoTimestampNs, &one, sizeof(one));

    sockaddr_ll sll{};
    sll.sll_family = AF_PACKET;
    sll.sll_protocol = ::htons(kEthPAll);
    sll.sll_ifindex = ::if_nametoindex(capIface_.empty() ? iface_.c_str() : capIface_.c_str());
    if (sll.sll_ifindex == 0) {
        if (err) *err = "capture interface does not exist";
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    ifIndex_ = static_cast<uint32_t>(sll.sll_ifindex);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&sll), sizeof(sll)) < 0) {
        if (err)
            *err = std::string("bind to ") + capIface_ + ": " + std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        reason_ = UnavailableReason::Busy;
        return false;
    }
    // Promiscuous membership is required to see stations that never address
    // us -- but ONLY when capturing in managed mode.
    //
    // On a monitor interface it is actively harmful: adding PACKET_MR_PROMISC
    // there makes the kernel deliver frames through the promiscuous path, which
    // for this driver means bare 802.11 with the radiotap header stripped. Every
    // frame then fails the radiotap check and the capture yields no received
    // power at all, even though a plain socket on the same interface sees them
    // perfectly. A monitor interface already receives everything on its own.
    promisc_ = false;
    if (!capIface_.empty() && capIface_ != iface_) {
        note("capture interface is a monitor interface; promiscuous membership "
             "deliberately not requested");
    } else {
        packet_mreq mr{};
        mr.mr_ifindex = static_cast<int>(ifIndex_);
        mr.mr_type = kPacketMrPromisc;
        mr.mr_alen = 6;
        if (::setsockopt(fd_, SOL_PACKET, kPacketAddMembership, &mr, sizeof(mr)) == 0) {
            promisc_ = true;
        } else {
            note("promiscuous mode refused (err " + std::to_string(errno) + ")");
        }
    }

    // Large receive buffer: we would much rather drop at the application layer
    // than in the kernel ring.
    int rcvbuf = 4 * 1024 * 1024;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    return true;
}

namespace {

// Run a helper and capture its output plus exit status. Used for the monitor
// mode switch and channel pin, because the legacy SIOCSIWMODE private ioctl is
// deprecated (iwlwifi answers EOPNOTSUPP) and `iw` drives nl80211 correctly.
int runCapture(const std::string& cmd, std::string* out) {
    std::array<char, 4096> buf{};
    FILE* f = popen((cmd + " 2>&1").c_str(), "r");
    if (!f) return -1;
    std::string text;
    while (fgets(buf.data(), static_cast<int>(buf.size()), f)) text += buf.data();
    const int rc = pclose(f);
    if (out) *out = text;
    return WIFEXITED(rc) ? WEXITSTATUS(rc) : -1;
}

std::string trimStr(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return {};
    const size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}
std::vector<std::string> splitLines(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == '\n') {
            out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    out.push_back(cur);
    return out;
}

}  // namespace

std::string WifiRadiometricSensor::ensureMonitorInterface() {
    // Reuse ours if it is already there.
    std::string list;
    if (runCapture("iw dev", &list) == 0) {
        for (const auto& raw : splitLines(list)) {
            const std::string l = trimStr(raw);
            if (l == "Interface " + monIface_) return monIface_;
        }
    }
    for (int suffix = 0; suffix < 8; ++suffix) {
        const std::string name = monIface_.empty()
                                     ? (suffix == 0 ? std::string("radar-mon")
                                                    : "radar-mon" + std::to_string(suffix))
                                     : monIface_;
        std::string out;
        if (runCapture("iw dev " + iface_ + " interface add " + name + " type monitor",
                       &out) == 0 &&
            ::if_nametoindex(name.c_str()) != 0) {
            monIface_ = name;
            return name;
        }
    }
    monIface_.clear();
    return {};
}

void WifiRadiometricSensor::removeMonitorInterface() {
    if (monIface_.empty()) return;
    std::string out;
    if (::if_nametoindex(monIface_.c_str()) != 0) {
        int ctl = sockForIoctl();
        if (ctl >= 0) {
            ifreq r{};
            std::strncpy(r.ifr_name, monIface_.c_str(), IFNAMSIZ - 1);
            ::ioctl(ctl, SIOCGIFFLAGS, &r);
            r.ifr_flags &= static_cast<short>(~IFF_UP);
            ::ioctl(ctl, SIOCSIFFLAGS, &r);
        }
        runCapture("iw dev " + monIface_ + " del", &out);
    }
    monIface_.clear();
}

// Find the channel of the strongest visible network. Robust to the interface
// not being associated, which is exactly when a hardcoded channel fails.
int WifiRadiometricSensor::findBestChannel() {
    std::string out;
    // -t is dwell time in ms. (-T is not a flag; iw prints usage and we would
    // silently parse nothing.)
    if (runCapture("iw dev " + iface_ + " scan -t 60", &out) != 0) return 0;
    int bestCh = 0;
    double bestSig = -1e9;
    int curFreq = 0;
    for (const auto& raw : splitLines(out)) {
        const std::string l = trimStr(raw);
        if (l.rfind("BSS ", 0) == 0) {
            curFreq = 0;
            continue;
        }
        if (l.rfind("freq:", 0) == 0) {
            curFreq = std::atoi(l.c_str() + 5);
        } else if (l.rfind("signal:", 0) == 0 && curFreq > 0) {
            const double sig = std::atof(l.c_str() + 7);
            if (sig > bestSig) {
                bestSig = sig;
                const int ch = (curFreq == 2484) ? 14 : (curFreq - 2407) / 5;
                if (ch >= 1 && ch <= 14) bestCh = ch;
            }
        }
    }
    if (bestCh > 0)
        note("strongest network " + std::to_string(static_cast<int>(bestSig)) + " dBm on channel " +
             std::to_string(bestCh));
    return bestCh;
}

// ---------------------------------------------------------------- in-process nl80211
//
// Interface type and channel are set in-process rather than by shelling out to
// `iw` or `ip`, and the reason is privilege rather than elegance.
//
// The installed binary carries cap_net_raw and cap_net_admin as *file*
// capabilities. Those belong to this executable and are not conferred on
// subprocesses, so every privileged step delegated to `iw` or `ip` failed with
// EPERM even though the parent held CAP_NET_ADMIN. The symptom was an
// application that started, released the device from NetworkManager, and then
// reported "monitor mode unusable: Operation not permitted".
//
// SIOCSIWMODE and SIOCSIWCHAN are not usable substitutes: iwlwifi answers
// EOPNOTSUPP for both, so nl80211 over a netlink socket is the only route.
//
// Every constant below comes from <linux/nl80211.h>. Hand-copied values were
// wrong three times over -- NL80211_CMD_SET_INTERFACE is 6 (5 is
// NL80211_CMD_GET_INTERFACE, so the request was a *get*), NL80211_ATTR_IFTYPE is
// 5 (not 6), and NL80211_CMD_SET_WIPHY is 2 (not 35) -- and the kernel's reply
// to the malformed message was EOPNOTSUPP, which points at the driver and sends
// you hunting in exactly the wrong place.

namespace {

// libnl is not a build dependency, and the kernel uapi headers do not define
// these two.
#ifndef NLA_HDRLEN
#define NLA_HDRLEN ((int)NLA_ALIGN(sizeof(struct nlattr)))
#endif
#ifndef NLA_DATA
#define NLA_DATA(pa) ((void*)((char*)(pa) + NLA_HDRLEN))
#endif

constexpr unsigned kNl80211Family = 0x10;  // GENL_ID_CTRL, resolved as "nl80211"

uint32_t seqCounter() {
    static uint32_t s = 0;
    return ++s;
}

struct NlAttr {
    uint16_t type;
    const void* data;
    uint16_t len;  // payload length, excluding the header
};

// Resolve the nl80211 generic-netlink family id at runtime.
//
// This is not optional. The id is NOT reliably 0x10: on this kernel strace
// resolves our 0x10 messages as "nlctrl" and rejects them with EOPNOTSUPP,
// while iw's identical-looking message comes back as "nl80211" and succeeds.
// The difference is that iw first asks the generic-netlink controller which id
// the nl80211 family was registered under, and iw's own request for that
// information is visible in a trace as a CTRL_CMD_GETFAMILY carrying the family
// name "nl80211".
//
// Hardcoding 0x10 produced a message that looked correct in every respect,
// carried the right command and the right attributes, and was still refused.
// Netlink sockets get a receive timeout. Without one a malformed or unanswered
// request blocks in recv() forever: the process stops responding, the operator
// sees a frozen application, and any buffered output is lost when it is killed.
int netlinkSocketBounded() {
    const int fd = ::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_GENERIC);
    if (fd < 0) return -1;
    timeval tv{};
    tv.tv_sec = 3;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    return fd;
}

uint16_t nl80211FamilyId() {
    static uint16_t cached = 0;
    if (cached != 0) return cached;

    constexpr uint16_t kNlctrlFamily = 0x10;  // GENL_ID_CTRL
    constexpr uint8_t kCtrlCmdGetFamily = 3;
    const uint16_t kAttrFamilyId = CTRL_ATTR_FAMILY_ID;
    const uint16_t kAttrFamilyName = CTRL_ATTR_FAMILY_NAME;
    static const char kName[] = "nl80211";

    char buf[256];
    std::memset(buf, 0, sizeof(buf));
    auto* n = reinterpret_cast<nlmsghdr*>(buf);
    n->nlmsg_len = NLMSG_LENGTH(sizeof(genlmsghdr));
    n->nlmsg_type = kNlctrlFamily;
    n->nlmsg_flags = NLM_F_REQUEST;
    n->nlmsg_seq = seqCounter();
    auto* g = reinterpret_cast<genlmsghdr*>(NLMSG_DATA(n));
    g->cmd = kCtrlCmdGetFamily;
    g->version = 1;

    const size_t off = NLMSG_ALIGN(n->nlmsg_len);
    const uint16_t alen = static_cast<uint16_t>(sizeof(kName));  // includes NUL
    auto* a = reinterpret_cast<nlattr*>(buf + off);
    a->nla_type = kAttrFamilyName;
    a->nla_len = static_cast<uint16_t>(NLA_HDRLEN + alen);
    std::memcpy(NLA_DATA(a), kName, alen);
    n->nlmsg_len = static_cast<__u32>(off + NLA_HDRLEN + NLA_ALIGN(alen));

    const int fd = netlinkSocketBounded();
    if (fd < 0) return 0;
    sockaddr_nl sa{};
    sa.nl_family = AF_NETLINK;
    if (::bind(fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) != 0) {
        ::close(fd);
        return 0;
    }
    if (::send(fd, buf, n->nlmsg_len, 0) < 0) {
        ::close(fd);
        return 0;
    }

    // Large enough for the whole GETFAMILY reply. The response for "nl80211"
    // declares nlmsg_len = 2516 and arrives as a multipart message; with a
    // 1024-byte buffer recv returned a truncated 1024 bytes, NLMSG_OK then
    // compared 1024 against the declared 2516, judged the message malformed and
    // skipped it -- so the lookup silently reported "not found" and every
    // subsequent nl80211 call returned EOPNOTSUPP.
    char rbuf[8192];
    uint16_t found = 0;
    for (int guard = 0; guard < 8 && found == 0; ++guard) {
        const ssize_t got = ::recv(fd, rbuf, sizeof(rbuf), 0);
        if (got <= 0) break;
        ssize_t left = got;
        for (auto* h = reinterpret_cast<nlmsghdr*>(rbuf);
             NLMSG_OK(h, static_cast<unsigned>(left)); h = NLMSG_NEXT(h, left)) {
            if (h->nlmsg_type == NLMSG_DONE || h->nlmsg_type == NLMSG_ERROR) break;

            // Walk the attributes from *after* the generic-netlink header. The
            // previous version pointed at NLMSG_DATA(h) and treated the 4-byte
            // genlmsghdr as if it were an attribute, which both misaligned the
            // walk and, because those bytes read as a zero nla_len, terminated
            // it immediately. It returned 31 instead of the real id, so every
            // subsequent command was addressed to the wrong family and came back
            // EOPNOTSUPP.
            const char* p = reinterpret_cast<const char*>(NLMSG_DATA(h)) + GENL_HDRLEN;
            const char* end = reinterpret_cast<const char*>(h) + NLMSG_ALIGN(h->nlmsg_len);
            while (p + NLA_HDRLEN <= end) {
                const auto* a = reinterpret_cast<const nlattr*>(p);
                const uint16_t alen = a->nla_len;
                if (alen < NLA_HDRLEN || p + alen > end) break;
                // CTRL_ATTR_FAMILY_ID is a u16.
                if ((a->nla_type & NLA_TYPE_MASK) == kAttrFamilyId && alen >= NLA_HDRLEN + 2) {
                    std::memcpy(&found, p + NLA_HDRLEN, sizeof(found));
                    break;
                }
                p += NLA_ALIGN(alen);
            }
            if (found != 0) break;
        }
    }
    ::close(fd);
    cached = found;
    return cached;
}

// One generic-netlink request carrying up to four scalar attributes.
int nl80211Request(int cmd, int ifindex, const NlAttr* attrs, size_t count) {
    char buf[512] = {};

    const uint16_t family = nl80211FamilyId();
    if (family == 0) return -EOPNOTSUPP;  // nl80211 is not present in this kernel

    auto* n = reinterpret_cast<nlmsghdr*>(buf);
    n->nlmsg_len = NLMSG_LENGTH(sizeof(genlmsghdr));
    n->nlmsg_type = family;
    n->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
    n->nlmsg_seq = seqCounter();

    auto* g = reinterpret_cast<genlmsghdr*>(NLMSG_DATA(n));
    g->cmd = static_cast<__u8>(cmd);
    // Matches what iw sends: version 0, not 1.
    g->version = 0;

    // Bounds-checked append. An earlier version appended past the end of a fixed
    // struct, which compiled cleanly and then aborted with "buffer overflow
    // detected" as soon as a real request was sent.
    const auto add = [&](uint16_t type, const void* src, uint16_t alen) {
        const size_t off = NLMSG_ALIGN(n->nlmsg_len);
        const size_t need = off + NLA_HDRLEN + NLA_ALIGN(alen);
        if (need > sizeof(buf)) return;
        auto* a = reinterpret_cast<nlattr*>(buf + off);
        a->nla_type = type;
        a->nla_len = static_cast<__u16>(NLA_HDRLEN + alen);
        if (alen && src) std::memcpy(NLA_DATA(a), src, alen);
        n->nlmsg_len = static_cast<__u32>(need);
    };

    // Deliberately no CTRL_ATTR_FAMILY_ID here. Traced from iw:
    //   sendmsg(... nlmsg_len=36 ... "\x06\x00\x00\x00"   genl {cmd=6,ver=0}
    //                     "\x08\x00\x03\x00\x03\x00\x00\x00"  IFINDEX=3
    //                     "\x08\x00\x05\x00\x06\x00\x00\x00") IFTYPE=6 as u32
    // iw resolves the family id in a preceding nlctrl GETFAMILY message and then
    // sends the nl80211 command with no family attribute at all. Including one
    // put an attribute the command's policy does not declare into the request.
    if (ifindex > 0) {
        const int idx = ifindex;
        add(NL80211_ATTR_IFINDEX, &idx, sizeof(idx));
    }
    for (size_t i = 0; i < count && i < 4; ++i)
        add(attrs[i].type, attrs[i].data, attrs[i].len);

    const int fd = netlinkSocketBounded();
    if (fd < 0) return errno ? errno : -ENOSYS;

    sockaddr_nl sa{};
    sa.nl_family = AF_NETLINK;

    int rc = 0;
    if (::bind(fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) != 0) {
        rc = errno;
    } else if (::send(fd, buf, n->nlmsg_len, 0) < 0) {
        rc = errno;
    } else {
        // Wait for the ACK, so failure is reported rather than assumed.
        char rbuf[2048];
        for (;;) {
            const ssize_t got = ::recv(fd, rbuf, sizeof(rbuf), 0);
            if (got < 0) {
                if (errno == EINTR) continue;
                rc = errno;
                break;
            }
            if (got == 0) {
                rc = -EIO;
                break;
            }
            ssize_t left = got;
            auto* h = reinterpret_cast<nlmsghdr*>(rbuf);
            for (; NLMSG_OK(h, static_cast<unsigned>(left)); h = NLMSG_NEXT(h, left)) {
                if (h->nlmsg_type == NLMSG_ERROR) {
                    const auto* e = static_cast<const nlmsgerr*>(NLMSG_DATA(h));
                    rc = e->error;  // 0 == acknowledged
                    break;
                }
                if (h->nlmsg_type == NLMSG_DONE) {
                    rc = 0;
                    break;
                }
            }
            if (rc != -EINPROGRESS) break;
        }
    }
    ::close(fd);
    return rc;
}

// phy80211 index for an interface, from the /sys/class/net/<if>/phy80211 link.
int phyIndexOf(const std::string& iface) {
    char buf[256];
    const ssize_t n = ::readlink(("/sys/class/net/" + iface + "/phy80211").c_str(), buf,
                                 sizeof(buf) - 1);
    if (n <= 0) return -1;
    buf[n] = '\0';
    std::string name(buf);
    // The link target is "../../ieee80211/phy0", not "phy0", so match the last
    // component rather than requiring the string to start with "phy".
    const size_t slash = name.rfind('/');
    if (slash != std::string::npos) name = name.substr(slash + 1);
    if (name.rfind("phy", 0) != 0) return -1;
    const int v = std::atoi(name.c_str() + 3);
    return v >= 0 ? v : -1;
}

// Channel number to centre frequency in MHz.
uint32_t channelToFreqMhz(int ch) {
    if (ch == 14) return 2484;
    if (ch >= 1 && ch <= 13) return static_cast<uint32_t>(2407 + 5 * ch);
    if (ch >= 32 && ch <= 177) return static_cast<uint32_t>(5000 + 5 * ch);  // 5 GHz
    return 0;
}

}  // namespace

// Pin the synthesiser. This uapi header has no NL80211_ATTR_WIPHY_CHANNEL, so
// the centre frequency goes out as NL80211_ATTR_WIPHY_FREQ alongside the channel
// width. SIOCSIWCHAN is not an option: iwlwifi answers EOPNOTSUPP. The unit is
// mHz, not the kHz older kernels used -- sending kHz was rejected with EINVAL.
int WifiRadiometricSensor::nl80211SetChannel(int phy, int channel) {
    const uint32_t mhz = channelToFreqMhz(channel);
    if (mhz == 0) return -EINVAL;
    const uint32_t freqMhz = mhz;  // modern nl80211 takes WIPHY_FREQ in mHz, not kHz
    const uint32_t width = NL80211_CHAN_WIDTH_20;
    const NlAttr attrs[3] = {
        {NL80211_ATTR_WIPHY, &phy, sizeof(phy)},
        {NL80211_ATTR_WIPHY_FREQ, &freqMhz, sizeof(freqMhz)},
        {NL80211_ATTR_CHANNEL_WIDTH, &width, sizeof(width)},
    };
    return nl80211Request(NL80211_CMD_SET_WIPHY, 0, attrs, 3);
}

std::string WifiRadiometricSensor::enterMonitorMode(int chan, bool pinChannel) {
    int ctl = sockForIoctl();
    if (ctl < 0) return "no control socket";

    // Read the channel while the link is still UP: once it is down, iw reports
    // no channel at all and there is nothing to follow.
    int follow = autoChannel_;
    if (follow <= 0) {
        std::string pre;
        runCapture("iw dev " + iface_ + " info", &pre);
        const size_t k = pre.find("channel ");
        if (k != std::string::npos) follow = std::atoi(pre.c_str() + k + 8);
    }

    ifreq r{};
    std::strncpy(r.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(ctl, SIOCGIFFLAGS, &r) != 0) return "SIOCGIFFLAGS failed";
    const short savedFlags = r.ifr_flags;
    r.ifr_flags &= static_cast<short>(~IFF_UP);
    ::ioctl(ctl, SIOCSIFFLAGS, &r);

    const int target = follow > 0 ? follow : chan;

    // Interface type, in-process so this binary's file capabilities apply.
    const int rcType = nl80211SetInterfaceType(NL80211_IFTYPE_MONITOR);
    if (rcType != 0)
        return "nl80211 set interface type monitor: " + std::string(std::strerror(-rcType));
    note("monitor mode via nl80211 (family " + std::to_string(nl80211FamilyId()) + ")");

    // Link up BEFORE pinning the channel: iwlwifi rejects SIOCSIWCHAN with EBUSY
    // while the interface is down, and an unpinned monitor interface never locks
    // a synthesiser and delivers nothing.
    ifreq u{};
    std::strncpy(u.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    u.ifr_flags = static_cast<short>(savedFlags | IFF_UP | IFF_BROADCAST);
    if (::ioctl(ctl, SIOCSIFFLAGS, &u) != 0) return "could not bring the link up";

    // Give the firmware a moment to pick a channel of its own, then pin ours.
    for (int i = 0; i < 10; ++i) {
        std::string info;
        runCapture("iw dev " + iface_ + " info", &info);
        if (info.find("channel") != std::string::npos) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    // Only pin a channel when the operator asked for a specific one. This
    // driver answers EBUSY to SIOCSIWCHAN whenever the card is already tuned,
    // which after a normal association it always is -- and when it does accept
    // the change it has already picked the right channel from its own
    // association history. Pinning by default therefore turned a working
    // capture into silence.
    bool pinned = false;
    if (pinChannel) {
        const int phy = phyIndexOf(iface_);
        if (phy < 0) {
            note("cannot pin channel " + std::to_string(target) +
                 ": no phy index for " + iface_ + "; using the card's choice");
        } else {
            const int rc = nl80211SetChannel(phy, target);
            if (rc == 0) {
                pinned = true;
                note("pinned to channel " + std::to_string(target));
            } else if (rc == -EBUSY) {
                // Expected and harmless: the synthesiser is already there.
                note("channel already tuned; keeping channel " + std::to_string(target));
            } else {
                note("channel pin failed: " + std::string(std::strerror(-rc)) +
                     "; using the channel the card selected");
            }
        }
    }

    // NB: do NOT reject on "iw dev info reports no channel". This driver
    // routinely omits the channel line for a monitor interface while delivering
    // frames perfectly well; treating its absence as failure abandoned working
    // captures. The only honest test is whether a frame arrives, which the
    // caller does with waitForFirstFrame().
    {
        std::string now;
        runCapture("iw dev " + iface_ + " info", &now);
        const size_t k = now.find("channel ");
        const int on = k != std::string::npos ? std::atoi(now.c_str() + k + 8) : 0;
        // This driver routinely omits the channel line for a monitor interface
        // while delivering frames perfectly well, so "channel 0" said nothing
        // useful. Report the target we asked for and, separately, whether the
        // card has reported a channel of its own yet.
        std::string where = "target channel " + std::to_string(target);
        where += on > 0 ? ", card reports channel " + std::to_string(on) : ", card reports none yet";
        if (pinned) where += " (pinned)";
        note("monitor mode, " + where);
    }
    return {};
}

// Block until the kernel hands us at least one frame, or the timeout expires.
// This is the only honest test that the radio is actually delivering.
// Wait for a frame that actually carries a radiotap header.
//
// Merely receiving a frame is not sufficient: when the driver is left in a half
// state the kernel still advertises link-type IEEE802_11_RADIO but delivers bare
// 802.11 with no radiotap, so there is no received-power field anywhere and the
// capture is useless for radiometry. Only a frame that parses as radiotap counts.
bool WifiRadiometricSensor::waitForRadiometricFrame(int timeoutMs) {
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeoutMs);
    std::array<uint8_t, 2048> buf{};
    while (Clock::now() < deadline) {
        const ssize_t n = ::recv(fd_, buf.data(), buf.size(), MSG_DONTWAIT);
        if (n > 0) {
            const size_t len = static_cast<size_t>(n);
            if (len >= 8) {
                const uint16_t rlen = static_cast<uint16_t>(buf[2] | (buf[3] << 8));
                if (buf[0] == 0 && rlen >= 8 && rlen <= len) return true;
            }
            continue;
        }
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

int WifiRadiometricSensor::sockForIoctl() {
    static thread_local int s = -1;
    if (s < 0) s = ::socket(AF_INET, SOCK_DGRAM, 0);
    return s;
}

void WifiRadiometricSensor::closeSocket() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    promisc_ = false;
}

void WifiRadiometricSensor::stop() {
    // Stop reading first, then hand everything back.
    setActive(false);
    requestStop();
    wakeConsumers();
    if (thread_.joinable()) thread_.join();
    closeSocket();

    restoreNetworkManager();
}

void WifiRadiometricSensor::captureLoop() {
    std::vector<uint8_t> buf(kFrameCap);
    std::vector<uint8_t> cbuf(256);
    auto wallStart = Clock::now();
    uint64_t counter = 0;

    // Stall watchdog. A capture can go quiet for reasons that are entirely
    // outside this process -- most often the access point roams to another
    // channel and the synthesiser is left listening to an empty one. The UI
    // then shows a frozen trace while the application itself is perfectly
    // responsive, which is exactly the confusing failure being fixed here.
    // A quiet period alone is not proof of a fault (a genuinely empty band is
    // quiet too), so recovery is attempted on a timer and is idempotent.
    auto lastRecovery = Clock::now();
    lastGoodFrameNs_.store(nowNs(), std::memory_order_relaxed);

    while (active_.load(std::memory_order_relaxed)) {
        // Checked on the EAGAIN path below as well, so it fires even when the
        // socket is returning megabytes of unusable frames.
        if (Clock::now() - lastRecovery > kStallTimeout) {
            lastRecovery = Clock::now();
            const uint64_t quietNs = nowNs() - lastGoodFrameNs_.load(std::memory_order_relaxed);
            if (quietNs > kStallTimeoutNs) {
                attemptRecovery();
                continue;
            }
        }
        iovec iov{};
        iov.iov_base = buf.data();
        iov.iov_len = buf.size();

        msghdr mh{};
        mh.msg_name = nullptr;
        mh.msg_namelen = 0;
        mh.msg_iov = &iov;
        mh.msg_iovlen = 1;
        mh.msg_control = cbuf.data();
        mh.msg_controllen = cbuf.size();

        const ssize_t n = ::recvmsg(fd_, &mh, MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::microseconds(200));
                continue;
            }
            if (errno == EINTR) continue;
            // ENETDOWN means the interface was deleted or retyped underneath
            // us. Reopen rather than spin on a dead descriptor.
            if (errno == ENETDOWN || errno == EBADF) {
                closeSocket();
                std::this_thread::sleep_for(std::chrono::milliseconds(400));
                std::string ignored;
                if (active_.load(std::memory_order_relaxed) && !openSocket(&ignored))
                    std::this_thread::sleep_for(std::chrono::milliseconds(600));
                continue;
            }
            setLastError(std::string("recvmsg: ") + std::strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        if (n == 0) continue;

        if (lastErrorIsStale()) setLastError(std::string());
        bumpSeen();

        Observation obs;
        obs.radio = RadioKind::Wifi;
        obs.hostTime = Clock::now();

        // Kernel receive timestamp, if the driver gave us one.
        for (cmsghdr* c = CMSG_FIRSTHDR(&mh); c; c = CMSG_NXTHDR(&mh, c)) {
            if (c->cmsg_level == kSolSocket && c->cmsg_type == kScmTimestampNs) {
                timespec ts{};
                std::memcpy(&ts, CMSG_DATA(c), sizeof(ts));
                obs.ktimeNs = static_cast<double>(ts.tv_sec) * 1e9 +
                              static_cast<double>(ts.tv_nsec);
            }
        }

        bool isData = false;
        size_t dot11 = 0;
        haveSignalField_ = false;
        const bool haveRadiotap = parseRadiotap(buf.data(), static_cast<size_t>(n), obs, isData);
        if (haveRadiotap && n >= 4) {
            dot11 = static_cast<uint16_t>(buf[2] | (buf[3] << 8));
            if (!haveSignalField_) {
                // A radiotap header that carries no DBM_ANTSIGNAL tells us the
                // driver is not reporting received power. Worth counting on its
                // own, because it is a different fault from "no radiotap at all".
                noSignal_.fetch_add(1, std::memory_order_relaxed);
            }
        } else {
            // No radiotap header: the kernel is not in monitor mode, so there is
            // no received-power field anywhere in the frame and this observation
            // cannot be used radiometrically. Count it so the UI can explain
            // itself rather than just showing nothing.
            noRadiotap_.fetch_add(1, std::memory_order_relaxed);
            dot11 = 0;
        }
        bool usable = false;
        if (dot11 > 0 && static_cast<size_t>(dot11) + 26 <= static_cast<size_t>(n)) {
            usable = parseDot11(buf.data() + dot11, static_cast<size_t>(n) - dot11, obs);
        }

        if (usable) {
            // Estimate the effective sample rate from the elapsed wall time.
            const auto el = std::chrono::duration<double>(obs.hostTime - wallStart).count();
            if (el > 1.0) {
                sampleRate_ = static_cast<double>(counter) / el;
                counter = 0;
                wallStart = obs.hostTime;
            }
            ++counter;
            usable_.fetch_add(1, std::memory_order_relaxed);
            // Any usable frame proves the capture is alive, so this is what
            // clears the stall flag and resets the watchdog.
            lastGoodFrameNs_.store(nowNs(), std::memory_order_relaxed);
            stalled_.store(0, std::memory_order_relaxed);
            push(obs);
        }
    }
}

// ============================================================ self-healing
//
// Everything in this section exists because a crashed or killed run used to be
// able to leave the wireless interface in monitor mode with NetworkManager
// stopped. The machine would then have no network at all and the only way out
// was a reboot. Two rules remove that possibility:
//
//   1. On start, anything still in monitor mode is handed back *before* we
//      begin. A stale interface from a previous run can never be inherited.
//   2. On exit, the interface is restored before NetworkManager is restarted,
//      and the signal handler path restores NetworkManager itself rather than
//      trusting nmcli (which cannot reach D-Bus while NM is stopped).

// Run a command that needs real root. `systemctl` and `nmcli` authorise against
// the uid through D-Bus/polkit, which file capabilities do not satisfy, so a
// capability-only launch still cannot stop NetworkManager -- and without that
// the channel cannot be pinned and the capture receives nothing. Escalating just
// this one step keeps the rest of the application unprivileged.
bool WifiRadiometricSensor::runPrivileged(const std::string& cmd, std::string* out) {
    if (::geteuid() == 0) return runCapture(cmd, out) == 0;
    if (::geteuid() != getpwnam_real_uid())
        return false;  // setuid escalation is not available; do not pretend
    if (::access("/usr/bin/pkexec", X_OK) != 0) {
        if (out) *out = "pkexec is not installed, so NetworkManager cannot be stopped";
        return false;
    }
    return runCapture("pkexec " + cmd, out) == 0;
}

uid_t WifiRadiometricSensor::getpwnam_real_uid() const { return ::getuid(); }

int WifiRadiometricSensor::nl80211SetInterfaceType(uint8_t iftype) {
    const int ifIdx = static_cast<int>(::if_nametoindex(iface_.c_str()));
    if (ifIdx == 0) return -ENODEV;
    // Sent as a 32-bit value, not a byte: iw uses nla_put_u32 for this
    // attribute and the kernel reads it with nla_get_u8, so both lengths are
    // accepted but matching the reference implementation removes the doubt.
    const uint32_t v = iftype;
    const NlAttr a{NL80211_ATTR_IFTYPE, &v, sizeof(v)};
    return nl80211Request(NL80211_CMD_SET_INTERFACE, ifIdx, &a, 1);
}

int WifiRadiometricSensor::nl80211SetInterfaceManaged() {
    const int ifIdx = static_cast<int>(::if_nametoindex(iface_.c_str()));
    if (ifIdx == 0) return -ENODEV;
    const uint32_t station = NL80211_IFTYPE_STATION;
    const NlAttr a{NL80211_ATTR_IFTYPE, &station, sizeof(station)};
    return nl80211Request(NL80211_CMD_SET_INTERFACE, ifIdx, &a, 1);
}

// Give the interface and NetworkManager back, in the right order and exactly
// once. Every failure path after the device has been released must go through
// this: an early `return false` that skipped it left the interface `unmanaged`
// with no network, which is the very state this code exists to prevent.
void WifiRadiometricSensor::restoreNetworkManager() {
    // Restore whenever *anything* needs undoing. The previous guard required
    // nmReleased_ or nmStopped_, both of which are false when the interface was
    // already in monitor mode on arrival -- for instance after the service was
    // stopped outside this program. The result was that the application exited
    // leaving the wireless device in monitor mode and NetworkManager with no
    // connection, which is the exact failure this whole path was written to
    // eliminate.
    if (!monitorMode_ && !nmReleased_ && !nmStopped_) return;

    if (!iface_.empty() && monitorMode_ && hasNetAdminPrivilege()) {
        int ctl = sockForIoctl();
        if (ctl >= 0) {
            ifreq r{};
            std::strncpy(r.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
            if (::ioctl(ctl, SIOCGIFFLAGS, &r) == 0) {
                r.ifr_flags &= static_cast<short>(~IFF_UP);
                ::ioctl(ctl, SIOCSIFFLAGS, &r);
            }
        }
        std::string out;
        if (nl80211SetInterfaceManaged() != 0)
            note("could not restore " + iface_ + " to managed mode (" + out + ")");
        monitorMode_ = false;
    }

    if (nmStopped_) {
        std::string out;
        runPrivileged("systemctl start NetworkManager", &out);
        nmStopped_ = false;
    } else if (nmReleased_) {
        std::string out;
        runCapture("nmcli device set " + iface_ + " managed yes", &out);
    }
    nmReleased_ = false;

    // Re-associate so the user gets their connection back without touching
    // anything. Failing here is not fatal, only worth saying.
    const int ch = forceReassociate();
    note(ch > 0 ? iface_ + " restored to NetworkManager on channel " + std::to_string(ch)
                : iface_ + " restored to NetworkManager");
}

void WifiRadiometricSensor::reclaimStaleInterfaces() {
    // Enumerate wireless interfaces from /proc/net/dev so nothing is hardcoded.
    std::ifstream in("/proc/net/dev");
    if (!in) return;
    std::string line;
    std::getline(in, line);  // header
    std::vector<std::string> names;
    while (std::getline(in, line)) {
        const size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string name = line.substr(0, colon);
        while (!name.empty() && std::isspace(static_cast<unsigned char>(name.front())))
            name.erase(name.begin());
        if (name.rfind("wl", 0) == 0) names.push_back(name);
    }

    for (const auto& name : names) {
        std::string info;
        runCapture("iw dev " + name + " info", &info);
        if (info.find("type monitor") == std::string::npos) continue;

        // A leftover dedicated monitor interface from a previous run is simply
        // removed; we create our own when we need one.
        if (name != iface_ && monIface_.empty()) {
            runCapture("ip link set " + name + " down", nullptr);
            runCapture("iw dev " + name + " del", nullptr);
            note("removed a stale monitor interface left by a previous run: " + name);
            continue;
        }

        // The managed device itself is stuck in monitor mode: this is the state
        // that used to require a reboot. Restore it properly.
        note("found " + name + " still in monitor mode from a previous session; restoring");
        runCapture("ip link set " + name + " down", nullptr);
        runCapture("iw dev " + name + " set type managed", nullptr);
        runCapture("ip link set " + name + " up", nullptr);
    }

    // NetworkManager may itself have been left stopped, which is why systemctl
    // is used rather than nmcli.
    runCapture("systemctl start NetworkManager", nullptr);
    nmStopped_ = false;
    monitorMode_ = false;

    // Restoring the type is not enough: NetworkManager does not automatically
    // re-associate a device it finds already "connected" when it starts, and
    // until it does, `iw dev info` reports no channel at all. Without a channel
    // the synthesiser cannot be pinned and the capture silently listens to
    // channel 1, which is the exact failure this whole path exists to prevent.
    // Waiting here is only for the type change to land, not for a channel: the
    // association and the channel are forceReassociate()'s job, and waiting for
    // a channel that will never appear on an empty band just burned six seconds
    // on every start.
    for (int i = 0; i < 10; ++i) {
        std::string state;
        runCapture("iw dev " + iface_ + " info", &state);
        if (state.find("type managed") != std::string::npos) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // So the network this interface was on is brought back explicitly -- and
    // only that one. See forceReassociate().
    if (savedProfile_.empty()) rememberCurrentNetwork();
    forceReassociate();
}

// Bring the saved wireless connection back up and wait until the interface
// genuinely reports an associated channel. Returns the channel, or 0.
// Record which network this interface is on *before* anything is changed, so the
// connection can be handed back to exactly that one on exit.
void WifiRadiometricSensor::rememberCurrentNetwork() {
    savedProfile_.clear();
    savedSsid_.clear();

    // The SSID the radio is actually associated with.
    std::string info;
    runCapture("iw dev " + iface_ + " info", &info);
    const size_t s = info.find("ssid ");
    if (s != std::string::npos) {
        size_t e = info.find('\n', s);
        if (e == std::string::npos) e = info.size();
        savedSsid_ = info.substr(s + 5, e - s - 5);
        while (!savedSsid_.empty() && std::isspace(static_cast<unsigned char>(savedSsid_.back())))
            savedSsid_.pop_back();
    }

    // The profile NetworkManager currently has active on this device.
    std::string active;
    runCapture("nmcli -t -f NAME,DEVICE connection show --active", &active);
    std::istringstream as(active);
    std::string row;
    while (std::getline(as, row)) {
        const size_t c1 = row.find(':');
        if (c1 == std::string::npos) continue;
        // The device column is last, so a row can legitimately contain only one
        // separator: "My Home Network:wlp3s0". Treating a missing second colon
        // as "malformed" skipped every single-column row, which is most of them,
        // and left the app with no preferred network to rejoin.
        const size_t c2 = row.find(':', c1 + 1);
        const std::string dev =
            c2 == std::string::npos ? row.substr(c1 + 1) : row.substr(c1 + 1, c2 - c1 - 1);
        if (dev != iface_) continue;
        savedProfile_ = row.substr(0, c1);
        break;
    }

    if (!savedProfile_.empty()) note("connected network: " + savedProfile_);
    else if (!savedSsid_.empty()) note("associated with \"" + savedSsid_ + "\" (no active profile)");
    else note("no associated network recorded");
}

// Re-establish the association, and return the channel.
//
// This deliberately never activates an arbitrary saved profile. It used to take
// whichever 802.11 profile happened to be first in the nmcli list and run
// `nmcli con up` on it, which meant that starting the radar could spend twenty
// seconds trying to authenticate to a network the user had not touched in
// years, and quitting could leave the machine switched onto it. Only the network
// this interface was already on is ever reconnected.
int WifiRadiometricSensor::forceReassociate() {
    if (iface_.empty()) return 0;

    runCapture("nmcli radio wifi on", nullptr);

    const bool haveProfile = !savedProfile_.empty();
    if (haveProfile) {
        std::string ignored;
        // The profile name very often contains spaces ("My Home Network"), and
        // an unquoted name makes nmcli fail with "unknown connection", which
        // then looks exactly like an interface that will not reassociate.
        runCapture("nmcli con up " + shellQuote(savedProfile_), &ignored);
    } else {
        // Nothing to reconnect to. Do not go fishing through the saved list --
        // just let NetworkManager autoconnect to whatever it would have chosen,
        // and scan so the channel can be learned.
        runCapture("nmcli device wifi rescan", nullptr);
        note("no previously active network to rejoin; scanning instead");
    }

    // A profile that exists will associate in a second or two. With nothing
    // targeted there is no reason to wait the full twenty seconds.
    const int attempts = haveProfile ? 40 : 12;
    for (int i = 0; i < attempts; ++i) {
        std::string info;
        runCapture("iw dev " + iface_ + " info", &info);
        const size_t k = info.find("channel ");
        if (info.find("type managed") != std::string::npos && k != std::string::npos) {
            const int ch = std::atoi(info.c_str() + k + 8);
            if (ch > 0) {
                note("reassociated on channel " + std::to_string(ch) +
                     (haveProfile ? " on " + savedProfile_ : std::string()));
                return ch;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (haveProfile) note("could not rejoin " + savedProfile_ + "; scanning");

    // Still nothing. Scan explicitly, which works in managed mode and is the
    // last chance to learn the channel before NetworkManager is released.
    const int scanned = findBestChannel();
    if (scanned > 0)
        note("channel " + std::to_string(scanned) + " found by scan");
    return scanned;
}


bool WifiRadiometricSensor::repinChannel() {
    if (autoChannel_ <= 0) return false;
    const int phy = phyIndexOf(iface_);
    if (phy < 0) return false;
    // EBUSY means the synthesiser is already where we want it, which is success
    // for the purpose of recovering a stalled capture.
    for (int attempt = 0; attempt < 3; ++attempt) {
        const int rc = nl80211SetChannel(phy, autoChannel_);
        if (rc == 0 || rc == -EBUSY) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }
    return false;
}

bool WifiRadiometricSensor::attemptRecovery() {
    recoveryCount_.fetch_add(1, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(stallMtx_);
        stallReason_ = "no usable frame for " +
                       std::to_string(static_cast<int>(kStallTimeoutNs / 1000000000)) + "s";
    }
    stalled_.store(1, std::memory_order_relaxed);
    note("capture went quiet; attempting recovery");

    // Step 1, the common case: the access point roamed and the synthesiser is
    // still on the old channel.
    if (monitorMode_ && repinChannel()) {
        std::string out;
        if (runCapture("iw dev " + (capIface_.empty() ? iface_ : capIface_) + " info", &out) != 0) {
            note("re-pinned to channel " + std::to_string(autoChannel_));
            lastGoodFrameNs_.store(nowNs(), std::memory_order_relaxed);
            return true;
        }
    }

    // Step 2: the interface may have been retyped or deleted behind us.
    if (::if_nametoindex((capIface_.empty() ? iface_ : capIface_).c_str()) == 0) {
        note("capture interface vanished; rebuilding it");
        monitorMode_ = false;
        const std::string mon = ensureMonitorInterface();
        if (!mon.empty()) capIface_ = mon;
    }

    // Step 3: reopen the socket and, if we manage it, re-enter monitor mode.
    closeSocket();
    std::string err;
    if (!openSocket(&err)) {
        note("recovery could not reopen the socket: " + err);
        return false;
    }
    if (!monitorMode_ && hasNetAdminPrivilege()) {
        monitorMode_ = enterMonitorMode(autoChannel_, true).empty();
        closeSocket();
        if (!openSocket(&err)) return false;
    }
    lastGoodFrameNs_.store(nowNs(), std::memory_order_relaxed);
    note("capture recovered");
    return true;
}

// Walk the radiotap present-bitmap and pull out the fields we can use. This is
// the only place RSSI enters the program.
// Bit sizes of the radiotap fields we understand. Anything else terminates the
// walk, because an unknown present bit gives no way to know how far to advance.
namespace {
struct FieldDef {
    int bit;
    size_t size;
};
constexpr FieldDef kFieldTable[] = {
    {0, 8},    // TSFT
    {1, 1},    // FLAGS
    {2, 1},    // RATE
    {3, 4},    // CHANNEL
    {4, 3},    // FHSS
    {5, 1},    // DBM_ANTSIGNAL
    {6, 1},    // DBM_ANTNOISE
    {7, 2},    // LOCK_QUALITY
    {8, 2},    // TX_ATTENUATION
    {9, 2},    // DBM_TX_POWER
    {10, 2},   // DBM_ANT_TX_POWER
    {11, 1},   // ANTENNA
    {12, 1},   // DBM_ANTSIGNAL_MAX
    {13, 1},   // DBM_ANTNOISE_MAX
    {14, 1},   // TX_FLAGS
    {18, 8},   // XCHANNEL
    {19, 3},   // MCS
    {21, 12},  // VHT
    {22, 12},  // TIMESTAMP
};
constexpr int kExtBit = 31;

size_t fieldSize(int bit) {
    for (const auto& f : kFieldTable)
        if (f.bit == bit) return f.size;
    return 0;  // unknown
}

// Walk the present-word chain. iwlwifi emits three words here, and stopping at
// the first one shifts every field and silently yields the wrong RSSI.
bool collectPresentWords(const uint8_t* buf, size_t len, size_t headerLen,
                         std::vector<uint64_t>& words) {
    words.clear();
    size_t off = headerLen;
    for (int guard = 0; guard < 8; ++guard) {
        if (off + 4 > len) return false;
        uint32_t w;
        std::memcpy(&w, buf + off, 4);
        off += 4;
        words.push_back(w);
        if (!(w & (1u << kExtBit))) return true;  // chain ends here
    }
    return false;  // absurd chain length
}

// Locate the offset of a field, given a padding policy. Returns SIZE_MAX when
// the bitmap does not contain it.
size_t findFieldOffset(const std::vector<uint64_t>& words, size_t start, int targetBit,
                       bool align) {
    size_t off = start;
    for (size_t w = 0; w < words.size(); ++w) {
        uint64_t bits = words[w];
        for (int b = 0; b < 32; ++b) {
            const int bit = static_cast<int>(w) * 32 + b;
            if (bit > targetBit) break;
            if (!(bits & (1ull << b))) continue;
            const size_t sz = fieldSize(bit);
            if (sz == 0) return SIZE_MAX;
            if (bit == targetBit) return off;
            off += sz;
            if (align) off = (off + 3) & ~static_cast<size_t>(3);
        }
    }
    return SIZE_MAX;
}
}  // namespace

bool WifiRadiometricSensor::parseRadiotap(const uint8_t* buf, size_t len, Observation& out,
                                          bool& isData) {
    isData = false;
    if (len < 8) return false;
    RadiotapHeader h{};
    std::memcpy(&h, buf, sizeof(h));
    if (h.version != 0) return false;
    if (h.len < 8 || h.len > len) return false;

    std::vector<uint64_t> words;
    if (!collectPresentWords(buf, len, 4, words)) return false;
    const size_t fieldsStart = 4 + words.size() * 4;

    // Two candidate layouts: strictly aligned (what the radiotap specification
    // mandates) and tightly packed (what iwlwifi actually emits). Choose by
    // which one puts ANTSIGNAL on a physically possible RSSI value, rather than
    // hard-coding one driver's habit.
    size_t sigOff = SIZE_MAX, noiseOff = SIZE_MAX, chanOff = SIZE_MAX;
    for (int pass = 0; pass < 2; ++pass) {
        const bool align = (pass == 0);
        sigOff = findFieldOffset(words, fieldsStart, kAntsignal, align);
        if (sigOff != SIZE_MAX && sigOff < h.len) {
            const int8_t v = static_cast<int8_t>(buf[sigOff]);
            // A real received-power reading sits well below 0 dBm; accepting 0
            // would let a mis-aligned layout masquerade as a valid sample.
            if (v >= -110 && v <= -20) {
                noiseOff = findFieldOffset(words, fieldsStart, kAntnoise, align);
                chanOff = findFieldOffset(words, fieldsStart, kChannel, align);
                break;
            }
            sigOff = SIZE_MAX;
        }
    }
    if (sigOff == SIZE_MAX || sigOff >= h.len) return false;

    const int8_t sig = static_cast<int8_t>(buf[sigOff]);
    out.rssiDbm = static_cast<double>(sig);
    haveSignalField_ = true;
    if (noiseOff != SIZE_MAX && noiseOff < h.len)
        out.snrDb = sig - static_cast<double>(static_cast<int8_t>(buf[noiseOff]));

    if (chanOff != SIZE_MAX && chanOff + 4 <= h.len) {
        uint16_t freq = 0, flags = 0;
        // This driver emits {frequency, flags}; the specification says the
        // reverse. Detect which by checking that the frequency is plausible.
        std::memcpy(&freq, buf + chanOff, 2);
        std::memcpy(&flags, buf + chanOff + 2, 2);
        uint16_t freqLE = flags;
        std::memcpy(&freqLE, buf + chanOff, 2);
        uint16_t freqBE;
        std::memcpy(&freqBE, buf + chanOff + 2, 2);
        const uint16_t chosen = (freqBE >= 2400 && freqBE <= 5900) ? freqBE : freqLE;
        out.frequencyHz = static_cast<double>(chosen) * 1e6;
        out.channel = out.frequencyHz > 0 ? (out.frequencyHz - 2407e6) / 5e6 : 0.0;
        const uint16_t maskWord = (freqBE == chosen) ? freqLE : freqBE;
        out.antennaCount = static_cast<int>(maskWord & 0x0F);
        if (out.antennaCount > 0 && out.antennaCount <= 4)
            out.antennaRssi[0] = static_cast<int8_t>(sig);
    }

    // Hardware TSFT: the card's own time stamp, far better than userspace
    // scheduling jitter for any timing work.
    const size_t tsftOff = findFieldOffset(words, fieldsStart, kTsft, true);
    const size_t tsftOffPacked = findFieldOffset(words, fieldsStart, kTsft, false);
    for (size_t cand : {tsftOffPacked, tsftOff}) {
        if (cand == SIZE_MAX || cand + 8 > h.len) continue;
        uint64_t tsft = 0;
        std::memcpy(&tsft, buf + cand, 8);
        if (tsft > 1000) {
            out.ktimeNs = static_cast<double>(tsft);
            break;
        }
    }
    return true;
}

// addr2 is the transmitter address in every 802.11 frame type; addr1 is the
// address the frame was addressed to. That is all we need for a radiometric
// per-transmitter measurement.
bool WifiRadiometricSensor::parseDot11(const uint8_t* buf, size_t len, Observation& out) {
    if (len < 10) return false;
    const uint16_t fc = static_cast<uint16_t>(buf[0] | (buf[1] << 8));
    const int type = static_cast<int>((fc >> 2) & 0x3);
    const int subtype = static_cast<int>((fc >> 4) & 0xF);
    switch (type) {
        case 0: out.frameClass = FrameClass::Management; break;
        case 1: out.frameClass = FrameClass::Control; break;
        case 2: out.frameClass = FrameClass::Data; break;
        case 3: out.frameClass = FrameClass::Extension; break;
        default: return false;
    }
    // Duration + addr1 + addr2 is the minimum we read.
    if (len < 24) return false;
    std::memcpy(out.receiver.data(), buf + 4, 6);
    std::memcpy(out.transmitter.data(), buf + 10, 6);
    // A zero transmitter means this is not a usable radiometric reference.
    bool allZero = true;
    for (uint8_t b : out.transmitter)
        if (b) allZero = false;
    if (allZero) return false;
    (void)subtype;
    return true;
}

SensorStatus WifiRadiometricSensor::status() const {
    SensorStatus s;
    s.name = name();
    s.kind = kind();
    s.device = iface_;
    s.active = isActive();
    s.available = fd_ >= 0 || isActive();
    s.requiresRoot = true;
    s.framesSeen = seen();
    s.framesDropped = dropped();
    s.framesWithRadiotap = usable_.load(std::memory_order_relaxed);
    s.framesWithRadiotapNoSignal = noSignal_.load(std::memory_order_relaxed);
    s.framesWithoutRadiotap = noRadiotap_.load(std::memory_order_relaxed);
    s.rateHz = usable_.load(std::memory_order_relaxed) > 0 ? sampleRate_ : 0.0;
    Capabilities c = capabilities();
    s.capabilities.push_back(c.rssi ? "RSSI" : "no-RSSI");
    s.capabilities.push_back(c.csi ? "CSI" : "no-CSI");
    s.capabilities.push_back(c.kernelTimestamps ? "kernel-ts" : "no-kernel-ts");
    s.capabilities.push_back(c.perAntennaRssi ? "per-antenna" : "no-per-antenna");
    s.capabilities.push_back(c.injection ? "injection" : "no-injection");
    s.capabilities.push_back("promiscuous=" + std::string(promisc_ ? "yes" : "no"));
    s.detail = detail();
    if (s.detail.empty() && s.active) s.detail = "capturing on " + iface_;
    const std::string err = lastError();
    if (!err.empty()) s.detail += "  [" + err + "]";
    // Surfaced rather than hidden: a stalled capture is otherwise
    // indistinguishable from a genuinely empty band.
    if (stalled_.load(std::memory_order_relaxed) && s.active) {
        std::string why;
        {
            std::lock_guard<std::mutex> lk(stallMtx_);
            why = stallReason_;
        }
        s.detail += "  [STALLED: " + why + "; recoveries=" +
                    std::to_string(recoveryCount_.load(std::memory_order_relaxed)) + "]";
    }
    return s;
}

std::vector<std::string> WifiRadiometricSensor::probe(std::string* err) {
    std::vector<std::string> out;
    if (iface_.empty() || ifIndex_ == 0) {
        if (err) *err = "sensor not started";
        return out;
    }
    out.push_back("interface: " + iface_ + " (ifindex " + std::to_string(ifIndex_) + ")");
    out.push_back("AF_PACKET fd: " + std::to_string(fd_));
    out.push_back("promiscuous: " + std::string(promisc_ ? "yes" : "no"));
    {
        out.push_back("state: " + detail());
    }
    out.push_back("frames seen: " + std::to_string(seen()));
    out.push_back("usable transmitter observations: " +
                  std::to_string(usable_.load(std::memory_order_relaxed)));
    out.push_back("frames without radiotap (not in monitor mode): " +
                  std::to_string(noRadiotap_.load(std::memory_order_relaxed)));
    out.push_back("dropped: " + std::to_string(dropped()));
    const Capabilities c = capabilities();
    out.push_back("capabilities: " + c.notes);
    return out;
}

}  // namespace radar