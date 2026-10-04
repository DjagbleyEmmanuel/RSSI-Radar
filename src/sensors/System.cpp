// Hardware survey: reads what this machine actually has, and reports it.
#include "radar/Sensor.hpp"

#include <linux/capability.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <dirent.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>


namespace radar {

bool hasNetAdminPrivilege() {
    if (::geteuid() == 0) return true;
    // The effective capability set, which is where file capabilities granted by
    // setcap actually land for this process.
    struct __user_cap_header_struct header {
        _LINUX_CAPABILITY_VERSION_3, 0
    };
    struct __user_cap_data_struct data[2];
    if (::syscall(SYS_capget, &header, data) != 0) return false;
    constexpr unsigned kCapNetAdmin = 12;
    return (data[0].effective & (1u << kCapNetAdmin)) != 0;
}

bool hasNetworkManagerPrivilege() {
    // D-Bus and polkit authorise against the real uid, which file capabilities
    // do not satisfy.
    return ::geteuid() == 0;
}

namespace {

std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

std::string readFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) return {};
    std::string s((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return trim(s);
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
        const std::string name = trim(line.substr(0, colon));
        if (name.rfind("wl", 0) == 0) out.push_back(name);
    }
    return out;
}

std::string driverOf(const std::string& iface) {
    std::ifstream in("/sys/class/net/" + iface + "/device/uevent");
    if (!in) return {};
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("DRIVER=", 0) == 0) return line.substr(7);
    }
    return {};
}

// Total frames the driver has received, so we can tell "radio dead" from
// "radio quiet because nobody is transmitting".
uint64_t rxCounters(const std::string& iface) {
    const std::string p = readFile("/sys/class/net/" + iface + "/statistics/rx_packets");
    return p.empty() ? 0 : std::strtoull(p.c_str(), nullptr, 10);
}

}  // namespace

std::string helperRun(const std::string& exe, const std::string& args) {
    // Bound every helper invocation: a wedged helper must never stall the UI.
    const std::string cmd = "timeout 3 " + exe + " " + args + " 2>/dev/null";
    std::array<char, 8192> buf{};
    FILE* f = popen(cmd.c_str(), "r");
    if (!f) return {};
    std::string out;
    while (fgets(buf.data(), static_cast<int>(buf.size()), f)) out += buf.data();
    pclose(f);
    return trim(out);
}

HardwareSurvey surveyHardware() {
    HardwareSurvey s;

    // ---------------------------------------------------------------- WiFi
    const auto ifaces = wirelessInterfaces();
    if (ifaces.empty()) {
        s.notes.push_back("no wireless network interface found");
    }
    for (const auto& iface : ifaces) {
        s.wifiDevice = iface;
        s.wifiDriver = driverOf(iface);

        SensorStatus st;
        st.name = "WiFi Radiometric (AF_PACKET)";
        st.kind = RadioKind::Wifi;
        st.device = iface;
        st.driver = s.wifiDriver;
        st.requiresRoot = true;
        st.available = true;
        st.rateHz = static_cast<double>(rxCounters(iface));
        s.wifi.push_back(st);
        break;  // the first wireless device is the one we would use
    }

    // Firmware revision, straight from the driver.
    {
        const std::string et = helperRun("ethtool", "-i " + s.wifiDevice);
        std::istringstream is(et);
        std::string line;
        while (std::getline(is, line)) {
            if (line.rfind("firmware-version:", 0) == 0)
                s.wifiFirmware = trim(line.substr(18));
            if (line.rfind("driver:", 0) == 0 && s.wifiDriver.empty())
                s.wifiDriver = trim(line.substr(8));
        }
    }

    // Regulatory domain. A country code of 00 (unset) puts the card in
    // passive-scan-only mode, which silently produces empty scan results and
    // blocks re-association. Flag it only when it actually applies to the
    // 2.4/5 GHz bands we use, not for the 60 GHz unlicensed bands further down
    // the listing, which always carry PASSIVE-SCAN regardless of country.
    {
        const std::string reg = helperRun("iw", "reg get");
        std::istringstream is(reg);
        std::string line;
        int lineNo = 0;
        bool inOurBands = false;
        while (std::getline(is, line)) {
            const std::string t = trim(line);
            if (lineNo == 1 && t.rfind("country", 0) == 0) {
                const size_t sp = t.find(' ');
                std::string code = sp == std::string::npos ? t : t.substr(sp + 1);
                const size_t colon = code.find(':');
                if (colon != std::string::npos) code = code.substr(0, colon);
                s.regDomainCountry = code;
            }
            // Band lines look like "(2400 - 2472 @ 40), (N/A, 30), (N/A)".
            if (!t.empty() && t.front() == '(') {
                const bool is24 = t.find("2400") != std::string::npos ||
                                  t.find("2472") != std::string::npos ||
                                  t.find("2484") != std::string::npos;
                const bool is5 = t.find("5150") != std::string::npos ||
                                 t.find("5470") != std::string::npos ||
                                 t.find("5725") != std::string::npos ||
                                 t.find("5855") != std::string::npos;
                inOurBands = is24 || is5;
            }
            if (t.find("PASSIVE-SCAN") != std::string::npos && inOurBands)
                s.regDomainPassiveScan = true;
            ++lineNo;
        }
    }

    // Monitor mode and injection support, from the driver's own capability list.
    {
        const std::string phy = helperRun("iw", "phy");
        // "Supported interface modes:" lists "* monitor" for a card that will
        // accept a mode switch.
        s.monitorCapable = phy.find("* monitor") != std::string::npos;
        // Injection support is a per-driver property, not a per-card one, and
        // it is an explicit allowlist. iwlwifi has never supported TX injection
        // on Linux; the Atheros and Realtek families have.
        static const char* kInjectDrivers[] = {"ath9k", "ath10k", "ath11k",
                                               "rtw88", "brcmfmac", "mt76x0u"};
        for (const char* d : kInjectDrivers)
            if (s.wifiDriver == d) s.injectionSupported = true;
        // Presence of a "supported vendor commands" section is the precondition
        // for any userspace CSI tap on Intel hardware.
        s.csiCapable = phy.find("supported vendor commands") != std::string::npos;
    }

    if (s.wifiDriver == "iwlwifi") {
        s.notes.push_back(
            "iwlwifi: RSSI works. CSI does not -- mainline exports no nl80211 CSI "
            "vendor command, so there is no userspace path to the firmware's "
            "mCSI notifications.");
    }
    if (s.regDomainPassiveScan) {
        s.notes.push_back(
            "regulatory domain is PASSIVE-SCAN: active scanning is forbidden, so "
            "scan results will be empty and re-association will fail. Set a real "
            "country (iw reg set US).");
    }
    if (s.regDomainCountry == "00") {
        s.notes.push_back(
            "regulatory domain country is 00 (unset). Most iwlwifi cards behave as "
            "passive-scan only until a real country is configured.");
    }

    // ------------------------------------------------------------ Bluetooth
    // Read the controller state from sysfs and hciconfig rather than the
    // libbluetooth legacy helpers: those are deprecated, several were dropped
    // in BlueZ 5.72, and their availability varies by distro.
    {
        SensorStatus st;
        st.name = "Bluetooth LE (raw HCI)";
        st.kind = RadioKind::Bluetooth;
        st.requiresRoot = true;
        st.driver = "btusb / hci_core";

        std::vector<std::string> controllers;
        if (DIR* d = ::opendir("/sys/class/bluetooth")) {
            while (dirent* e = ::readdir(d)) {
                const std::string n = e->d_name;
                if (n.rfind("hci", 0) == 0) controllers.push_back(n);
            }
            ::closedir(d);
        }
        std::sort(controllers.begin(), controllers.end());

        if (controllers.empty()) {
            st.available = false;
            st.detail = "no Bluetooth controller present";
        } else {
            st.available = true;
            st.device = controllers.front();
            const std::string cfg = helperRun("hciconfig", st.device);
            std::string addr, hciVersion, type, state;
            std::istringstream is(cfg);
            std::string line;
            while (std::getline(is, line)) {
                const std::string t = trim(line);
                if (t.rfind("BD Address:", 0) == 0) addr = trim(t.substr(11));
                if (t.rfind("hci_version:", 0) == 0) hciVersion = trim(t.substr(13));
                if (t.find("Type:") != std::string::npos)
                    type = trim(t.substr(t.find("Type:") + 5));
                if (t.rfind("UP", 0) == 0 && state.empty()) state = "UP/RUNNING";
            }
            // hci_version is the 0xNNNN HCI spec field; BT 4.0 == 9, BT 4.2 == 10.
            const int v = hciVersion.empty() ? -1 : std::atoi(hciVersion.c_str());
            st.detail = std::string("controller ") + (addr.empty() ? "(unknown)" : addr) +
                        ", bus " + type + ", " + state;
            if (v >= 0) st.detail += ", HCI spec 0x" + hciVersion;
            if (v > 0 && v <= 9) {
                st.capabilities.push_back(
                    "BT 4.0 controller: no LE Coded PHY (Long Range)");
                st.capabilities.push_back("no AoA/AoD direction finding");
            } else if (v >= 10) {
                st.capabilities.push_back("BT 4.2+: has LE 2M PHY");
                st.capabilities.push_back("AoA/AoD still unverified on this silicon");
            }
            // Live traffic counters prove whether the RF path is actually alive.
            const std::string rx = readFile("/proc/net/hci/../dev_stat");
            (void)rx;
        }
        s.bluetooth.push_back(st);
    }

    // ------------------------------------------------------------------ CSI
    {
        SensorStatus st;
        st.name = "WiFi CSI";
        st.kind = RadioKind::WifiCsi;
        st.device = s.wifiDevice;
        st.driver = s.wifiDriver;
        st.available = s.csiCapable;
        if (!s.csiCapable) {
            st.detail =
                "no userspace CSI interface: `iw phy` advertises no vendor commands";
            st.capabilities.push_back("no-CSI");
        } else {
            st.detail = "driver advertises vendor commands; needs an explicit CSI backend";
        }
        s.csi.push_back(st);
    }

    // Derived from what the survey actually found above.
    s.btLongRangePhy = false;
    s.btDirectionFinding = false;
    for (const auto& b : s.bluetooth)
        for (const auto& c : b.capabilities) {
            if (c.find("no LE Coded PHY") != std::string::npos) s.btLongRangePhy = false;
            if (c.find("no AoA") != std::string::npos) s.btDirectionFinding = false;
        }
    return s;
}

}  // namespace radar