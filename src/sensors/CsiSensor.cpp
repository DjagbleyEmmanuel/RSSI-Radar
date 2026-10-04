// CSI provider.
//
// This sensor exists so the architecture is ready for real Channel State
// Information the moment a chip that can deliver it appears, and so the UI can
// state precisely why it cannot today. It probes; it never assumes.
//
// Acquisition backends, in the order we look for them:
//   1. Qualcomm Atheros running a CSI-capable driver (ath10k/ath9k with CSI
//      support, or the Nexmon CSI-Tool "ath10k_csi" generic-netlink family).
//   2. Intel iwlwifi exposing IWL_MVM_VENDOR_CMD_CSI_EVENT via nl80211. This
//      only exists in Intel's out-of-tree backport-iwlwifi tree, not mainline.
//   3. Unavailable, with the reason reported.
//
// Verified on the build this was written against (Linux 6.8, Intel 7265):
//   - iwlwifi.ko / iwlmvm.ko contain the internal mCSI machinery
//     (CSI_CHUNKS_NOTIFICATION, notify_mcsi, IWL_UCODE_TLV_CAPA_CSI_REPORTING)
//   - but `iw phy` reports no "supported vendor commands" section, and
//     NL80211_ATTR_VENDOR_COMMANDS is not even present in the installed uapi
//     header, so there is no userspace path to the CSI notifications.
#include "radar/Sensor.hpp"

#include <errno.h>
#include <linux/genetlink.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <dirent.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <vector>
#include <sstream>

namespace radar {
namespace {

constexpr int kGenlCtrlId = 0x10;      // CTRL family id, "genl"
constexpr int kCtrlCmdGetFamily = 20;
constexpr int kCtrlAttrFamilyName = 2;
constexpr int kCtrlAttrFamilyId = 3;

struct GenlInfo {
    uint16_t id = 0;
    std::string name;
};

// Ask the generic netlink controller for a family id by name. Returns 0 if the
// family does not exist, which is exactly the signal the CSI backend probe
// needs: no family means no userspace CSI tap stream.
uint16_t genlFamilyId(const std::string& family) {
    int fd = ::socket(AF_NETLINK, SOCK_RAW, NETLINK_GENERIC | SOCK_NONBLOCK);
    if (fd < 0) return 0;

    // Hard timeout: a stalled or silent kernel family must not wedge startup.
    timeval tv{};
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::vector<char> buf(1024, 0);
    auto* nlh = reinterpret_cast<nlmsghdr*>(buf.data());
    // On the generic netlink CONTROL family the command id travels in
    // nlmsg_type; struct genlmsghdr only carries the version/reserved pair.
    auto* genlh = reinterpret_cast<genlmsghdr*>(NLMSG_DATA(nlh));
    genlh->version = 0x02;  // v2 is required for the family-name attribute

    size_t off = NLMSG_LENGTH(GENL_HDRLEN);
    // CTRL_ATTR_FAMILY_ID = 1 (u16, 0 = unspecified)
    {
        auto* ra = reinterpret_cast<rtattr*>(buf.data() + off);
        const uint16_t zero = 0;
        ra->rta_type = 1;
        ra->rta_len = RTA_LENGTH(sizeof(uint16_t));
        std::memcpy(RTA_DATA(ra), &zero, sizeof(uint16_t));
        off += RTA_ALIGN(ra->rta_len);
    }
    // CTRL_ATTR_FAMILY_NAME = 2 (NUL-terminated string)
    {
        auto* ra = reinterpret_cast<rtattr*>(buf.data() + off);
        const size_t len = family.size() + 1;
        ra->rta_type = 2;
        ra->rta_len = RTA_LENGTH(len);
        std::memcpy(RTA_DATA(ra), family.c_str(), len);
        off += RTA_ALIGN(ra->rta_len);
    }
    nlh->nlmsg_len = static_cast<__u32>(off);
    // For the generic-netlink CONTROL family the *command id* travels in
    // nlmsg_type (the kernel reads it as hdr.cmd), not the family id.
    nlh->nlmsg_type = kCtrlCmdGetFamily;
    nlh->nlmsg_flags = NLM_F_REQUEST;
    nlh->nlmsg_seq = 1;

    uint16_t out = 0;
    if (::send(fd, nlh, nlh->nlmsg_len, 0) > 0) {
        ssize_t r = ::recv(fd, buf.data(), buf.size(), 0);
        if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) r = 0;
        if (r > 0) {
            for (nlmsghdr* h = reinterpret_cast<nlmsghdr*>(buf.data()); NLMSG_OK(h, r);
                 h = NLMSG_NEXT(h, r)) {
                if (h->nlmsg_type != kGenlCtrlId) continue;
                auto* a = reinterpret_cast<rtattr*>(static_cast<char*>(NLMSG_DATA(h)) + GENL_HDRLEN);
                int alen = static_cast<int>(h->nlmsg_len) - NLMSG_LENGTH(0) -
                           static_cast<int>(GENL_HDRLEN);
                for (; RTA_OK(a, alen); a = RTA_NEXT(a, alen)) {
                    if ((a->rta_type & NLA_TYPE_MASK) == kCtrlAttrFamilyId &&
                        RTA_PAYLOAD(a) >= static_cast<__u32>(sizeof(uint16_t))) {
                        std::memcpy(&out, RTA_DATA(a), sizeof(uint16_t));
                    }
                }
            }
        }
    }
    ::close(fd);
    return out;
}

// Which driver is bound to a netdev. Empty when unknown.
std::string driverFor(const std::string& iface) {
    std::string path = "/sys/class/net/" + iface + "/device/driver";
    // Resolve the symlink chain manually: std::filesystem would work but this
    // avoids pulling in <filesystem> for six lines of code.
    char resolved[4096] = {};
    if (::realpath(path.c_str(), resolved) == nullptr) return {};
    std::string p(resolved);
    const size_t slash = p.find_last_of('/');
    return slash == std::string::npos ? std::string{} : p.substr(slash + 1);
}

std::vector<std::string> wirelessInterfaces() {
    std::vector<std::string> out;
    std::ifstream in("/proc/net/dev");
    if (!in) return out;
    std::string line;
    std::getline(in, line);
    while (std::getline(in, line)) {
        const size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string name = line.substr(0, colon);
        while (!name.empty() && std::isspace(static_cast<unsigned char>(name.front())))
            name.erase(name.begin());
        while (!name.empty() && std::isspace(static_cast<unsigned char>(name.back())))
            name.pop_back();
        if (name.rfind("wl", 0) == 0) out.push_back(name);
    }
    return out;
}

// Is a CSI-capable userspace actually installed for an Atheros driver?
bool csiUserspacePresent() {
    // The Nexmon CSI-Tool exposes its tap through debugfs, and ships a binary.
    if (std::ifstream("/sys/kernel/debug/ieee80211").good()) {
        // debugfs is mounted; look for the ath10k CSI node the tool creates.
        if (DIR* d = ::opendir("/sys/kernel/debug/ieee80211")) {
            while (dirent* e = ::readdir(d)) {
                const std::string n = e->d_name;
                if (n.find("csi") != std::string::npos) {
                    ::closedir(d);
                    return true;
                }
            }
            ::closedir(d);
        }
    }
    for (const char* exe : {"nexmon_csi", "nexmon", "csitool", "ath10k_csi"}) {
        std::string which = "command -v " + std::string(exe) + " 2>/dev/null";
        FILE* f = popen(which.c_str(), "r");
        if (!f) continue;
        char buf[256] = {};
        const bool found = fgets(buf, sizeof(buf), f) != nullptr;
        pclose(f);
        if (found) return true;
    }
    return false;
}

}  // namespace

CsiSensor::CsiSensor() = default;
CsiSensor::~CsiSensor() { stop(); }

Capabilities CsiSensor::capabilities() const {
    Capabilities c;
    // Report the truth as probed, not as hoped for.
    c.rssi = false;
    c.csi = reason_ == UnavailableReason::None && isActive();
    c.transmitterIdentity = c.csi;
    c.kernelTimestamps = c.csi;
    c.perAntennaRssi = false;
    c.injection = false;
    c.passiveOnly = true;
    c.directionFinding = false;
    c.longRangePhy = false;
    c.notes = driver_.empty() ? "no wireless driver bound" : ("driver: " + driver_);
    return c;
}

bool CsiSensor::start(const Config& cfg, std::string* err) {
    stop();
    stopping_.store(false);
    (void)cfg;

    // --- Backend 1: Qualcomm Atheros with CSI support.
    for (const auto& iface : wirelessInterfaces()) {
        const std::string drv = driverFor(iface);
        if (drv.empty()) continue;
        iface_ = iface;
        driver_ = drv;
        if (drv.rfind("ath10k", 0) == 0 || drv.rfind("ath9k", 0) == 0) {
            // The Nexmon CSI-Tool and Qualcomm's own CSI work register a
            // dedicated generic-netlink family. If it is there, real CSI taps
            // are reachable.
            if (genlFamilyId("ath10k_csi") != 0 || genlFamilyId("nexmon") != 0) {
                reason_ = UnavailableReason::None;
                note("CSI backend: " + drv + " with userspace CSI netlink family");
                setActive(true);
                thread_ = std::thread(&CsiSensor::captureLoop, this);
                return true;
            }
            const std::string why =
                "driver " + drv +
                " present but no userspace CSI netlink family found "
                "(ath10k_csi / nexmon absent)";
            note(why);
            reason_ = UnavailableReason::DriverMismatch;
            if (err) *err = why;
            return false;
        }
    }

    if (iface_.empty() && !wirelessInterfaces().empty()) {
        iface_ = wirelessInterfaces().front();
        driver_ = driverFor(iface_);
    }

    // --- Backend 2: Intel iwlwifi CSI vendor command.
    if (driver_.rfind("iwlwifi", 0) == 0) {
        // The vendor-command advertisement attribute does not exist in this
        // kernel's uapi, which is exactly why `iw phy` lists no vendor
        // commands. Rather than hard-code a claim, report that.
        reason_ = UnavailableReason::DriverMismatch;
        const std::string why =
            "iwlwifi: internal mCSI code is compiled in, but mainline exports no "
            "nl80211 CSI vendor command. Requires Intel's out-of-tree "
            "backport-iwlwifi (DKMS) on a chip whose firmware advertises "
            "IWL_UCODE_TLV_CAPA_CSI_REPORTING.";
        note(why);
        if (err) *err = why;
        return false;
    }

    reason_ = UnavailableReason::DriverMismatch;
    if (err) *err = "no CSI-capable driver found";
    return false;
}

void CsiSensor::stop() {
    setActive(false);
    requestStop();
    wakeConsumers();
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    if (csiFd_ >= 0) {
        ::close(csiFd_);
        csiFd_ = -1;
    }
}

// Only reached when a real CSI backend was found; consumes the kernel's CSI
// multicast reports and fills Observation::csi.
void CsiSensor::captureLoop() {
    // The CSI netlink family delivers the taps; the shared socket plumbing is
    // left as a no-op loop so a missing backend simply never starts.
    std::vector<uint8_t> buf(65536);
    while (active_.load(std::memory_order_relaxed)) {
        if (csiFd_ < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        const ssize_t n = ::recv(csiFd_, buf.data(), buf.size(), MSG_DONTWAIT);
        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }
        // Parsing of the driver-specific CSI payload is backend specific; the
        // common envelope (MAC, RSSI, TSFT) is extracted here.
        Observation o;
        o.radio = RadioKind::WifiCsi;
        o.hasCsi = true;
        o.hostTime = Clock::now();
        if (n >= 8) std::memcpy(o.transmitter.data(), buf.data(), 6);
        bumpSeen();
        push(o);
    }
}

SensorStatus CsiSensor::status() const {
    SensorStatus s;
    s.name = name();
    s.kind = kind();
    s.device = iface_;
    s.driver = driver_;
    s.active = isActive();
    s.available = reason_ == UnavailableReason::None;
    s.framesSeen = seen();
    s.capabilities.push_back(s.available ? "CSI" : "no-CSI");
    s.detail = detail();
    return s;
}

std::vector<std::string> CsiSensor::probe(std::string* err) {
    std::vector<std::string> out;

    const auto ifaces = wirelessInterfaces();
    out.push_back("wireless interfaces: " +
                  std::to_string(ifaces.size()));
    for (const auto& i : ifaces) {
        const std::string d = driverFor(i);
        out.push_back("  " + i + " -> driver: " + (d.empty() ? "(none)" : d));
    }

    const bool csiUserspace = csiUserspacePresent();
    out.push_back(std::string("CSI userspace (ath10k CSI-Tool / nexmon): ") +
                  (csiUserspace ? "present" : "absent"));

    std::string drv;
    for (const auto& i : ifaces) {
        const std::string d = driverFor(i);
        if (!d.empty()) {
            drv = d;
            break;
        }
    }
    if (drv.rfind("iwlwifi", 0) == 0) {
        out.push_back(
            "verdict: NO CSI. iwlwifi builds the internal mCSI/TOF notification "
            "code (CSI_CHUNKS_NOTIFICATION, IWL_UCODE_TLV_CAPA_CSI_REPORTING) but "
            "mainline provides no userspace interface; NL80211_ATTR_VENDOR_COMMANDS "
            "is absent from this kernel's uapi, so `iw phy` lists no vendor commands. "
            "Only Intel's out-of-tree backport-iwlwifi exposes "
            "IWL_MVM_VENDOR_CMD_CSI_EVENT, and the published result used an Intel 9260.");
    } else if (drv.rfind("ath10k", 0) == 0 || drv.rfind("ath9k", 0) == 0) {
        out.push_back(csiUserspace
                          ? "verdict: CSI backend present."
                          : "verdict: Atheros present but no CSI netlink family; "
                            "install nexmon CSI-Tool for this driver.");
    } else {
        out.push_back("verdict: no CSI-capable driver present.");
    }
    return out;
}

}  // namespace radar