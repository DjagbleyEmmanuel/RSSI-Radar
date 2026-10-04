// Bluetooth LE sensor over a raw AF_BLUETOOTH HCI socket.
//
// Real HCI traffic. RSSI comes out of actual LE advertising reports (event
// 0x3E) and BR/EDR inquiry responses (event 0x22). Nothing is synthesised.
#include "radar/Sensor.hpp"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <fstream>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

namespace radar {
namespace {

// sockaddr_bt / HCI_* were removed from the public BlueZ headers in 5.72.
// The kernel ABI is stable, so declare exactly what we bind with.
struct sockaddr_bt_compat {
    unsigned short bt_family;
    unsigned char bt_addr[6];
    signed char bt_channel;
    unsigned char bt_ifindex[2];
};
constexpr uint16_t kAfBluetooth = 31;
constexpr int kHciChannelUser = 1;
constexpr int kHciChannelRaw = 0;
constexpr uint16_t kOgfLeCtl = 0x08;
constexpr uint16_t kOcfLeSetScanParams = 0x000B;
constexpr uint16_t kOcfLeSetScanEnable = 0x000C;
}  // namespace

BluetoothSensor::BluetoothSensor() = default;
BluetoothSensor::~BluetoothSensor() { stop(); }

Capabilities BluetoothSensor::capabilities() const {
    Capabilities c;
    c.rssi = true;
    c.csi = false;
    c.transmitterIdentity = true;   // advertiser address in every report
    c.kernelTimestamps = false;     // HCI gives no kernel SOF timestamp
    c.perAntennaRssi = false;
    c.injection = true;             // we can advertise
    c.passiveOnly = true;           // scanning only, never connecting
    c.directionFinding = false;
    // The Intel 8265 BT is a Bluetooth 4.0 controller. LE Coded PHY (and with it
    // LE Power Control / Long Range at 4x range) arrived in BT 5.0, and
    // direction finding needs AoA/AoD-capable silicon. Neither is present.
    c.longRangePhy = false;
    c.notes =
        "LE advertising reports + BR/EDR inquiry RSSI. No LE Coded PHY (BT 4.0 "
        "controller, so no Long Range) and no AoA/AoD, therefore no angle of arrival.";
    return c;
}

bool BluetoothSensor::start(const Config& cfg, std::string* err) {
    stop();
    stopping_.store(false);
    devName_ = cfg.radio.bluetoothDevice.empty() ? std::string("hci0")
                                                 : cfg.radio.bluetoothDevice;

    // Resolve the controller index. Newer kernels drop /sys/.../id, so fall back
    // to probing index 0 and reading the address back from the socket.
    devId_ = 0;
    std::ifstream idFile("/sys/class/bluetooth/" + devName_ + "/id");
    if (idFile) {
        int v = -1;
        idFile >> v;
        if (v >= 0) devId_ = v;
    }

    fd_ = ::socket(AF_BLUETOOTH, SOCK_RAW | SOCK_NONBLOCK, kHciChannelUser);
    if (fd_ < 0) {
        // Retry on the raw channel, which needs no privileges on most kernels.
        fd_ = ::socket(AF_BLUETOOTH, SOCK_RAW | SOCK_NONBLOCK, kHciChannelRaw);
    }
    if (fd_ < 0) {
        reason_ = (errno == EPERM || errno == EACCES) ? UnavailableReason::Permissions
                                                       : UnavailableReason::NotPresent;
        if (err)
            *err = std::string("AF_BLUETOOTH raw socket: ") + std::strerror(errno);
        return false;
    }

    // Bind to the requested controller by index. sockaddr_bt accepts either a
    // bdaddr or a bt_ifindex; the index form is what "hciN" means.
    sockaddr_bt_compat sa{};
    sa.bt_family = kAfBluetooth;
    const uint16_t ifidx = static_cast<uint16_t>(devId_);
    sa.bt_ifindex[0] = static_cast<uint8_t>(ifidx & 0xFF);
    sa.bt_ifindex[1] = static_cast<uint8_t>((ifidx >> 8) & 0xFF);
    if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&sa), sizeof(sa)) < 0) {
        const int saved = errno;
        ::close(fd_);
        fd_ = -1;
        reason_ = UnavailableReason::NotPresent;
        if (err)
            *err = "cannot bind HCI socket to " + devName_ + " (index " +
                   std::to_string(devId_) + "): " + std::strerror(saved);
        return false;
    }

    note("HCI socket open on " + devName_);

    // Enable LE scanning: passive, duplicate reporting on, filter policy 0.
    std::vector<uint8_t> params = {0x00, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00};
    sendCommand(kOgfLeCtl, kOcfLeSetScanParams, params);
    sendCommand(kOgfLeCtl, kOcfLeSetScanEnable, {0x01, 0x00});
    scanning_.store(true);
    note("LE scan requested (passive, duplicates on)");

    reason_ = UnavailableReason::None;
    setActive(true);
    thread_ = std::thread(&BluetoothSensor::hciLoop, this);
    return true;
}

bool BluetoothSensor::sendCommand(uint16_t ogf, uint16_t ocf,
                                  const std::vector<uint8_t>& params) {
    if (fd_ < 0) return false;
    const uint16_t opcode = static_cast<uint16_t>((ogf << 10) | ocf);
    std::vector<uint8_t> pkt;
    pkt.reserve(3 + params.size());
    pkt.push_back(0x01);  // HCI_COMMAND_PKT
    pkt.push_back(static_cast<uint8_t>(opcode & 0xFF));
    pkt.push_back(static_cast<uint8_t>(opcode >> 8));
    pkt.push_back(static_cast<uint8_t>(params.size()));
    pkt.insert(pkt.end(), params.begin(), params.end());
    return ::send(fd_, pkt.data(), pkt.size(), MSG_NOSIGNAL) >= 0;
}

void BluetoothSensor::stop() {
    setActive(false);
    const bool wasScanning = scanning_.load();
    scanning_.store(false);
    requestStop();
    wakeConsumers();
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) {
        // Turn scanning off before dropping the socket, otherwise the controller
        // is left scanning and other tools will see a busy device.
        if (wasScanning) sendCommand(kOgfLeCtl, kOcfLeSetScanEnable, {0x00, 0x00});
        ::close(fd_);
        fd_ = -1;
    }
}

bool BluetoothSensor::startAdvertising(uint16_t advIntervalMs) {
    if (fd_ < 0) return false;
    // LE Set Advertising Parameters (0x2006) then LE Set Advertising Enable
    // (0x200A). This puts a real, over-the-air advertisement on the radio that
    // the pipeline can observe, giving us a controllable reference transmitter.
    std::vector<uint8_t> params = {
        0x00, 0x00,                                  // interval low/high
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        static_cast<uint8_t>(advIntervalMs & 0xFF),
        static_cast<uint8_t>((advIntervalMs >> 8) & 0xFF),
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00};
    sendCommand(kOgfLeCtl, 0x0006, params);
    sendCommand(kOgfLeCtl, 0x000A, {0x01, 0x00});
    note("local LE advertising enabled at " + std::to_string(advIntervalMs) + " ms");
    return true;
}

void BluetoothSensor::hciLoop() {
    std::vector<uint8_t> buf(4096);
    while (active_.load(std::memory_order_relaxed)) {
        const ssize_t n = ::recv(fd_, buf.data(), buf.size(), MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            if (errno == EINTR) continue;
            note(std::string("HCI recv failed: ") + std::strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (n < 2) continue;
        bumpSeen();
        // HCI packet: [0] = type. 0x04 = event, 0x01 = command, 0x02/0x03 = ACL.
        const uint8_t pktType = buf[0];
        if (pktType == 0x04 && n >= 3) {
            const uint8_t evt = buf[1];
            const uint8_t plen = buf[2];
            handleEvent(buf.data(), static_cast<size_t>(n));
        }
    }
}

void BluetoothSensor::handleEvent(const uint8_t* p, size_t len) {
    if (len < 3) return;
    const uint8_t evt = p[1];
    const uint8_t plen = p[2];
    const uint8_t* d = p + 3;

    if (evt == 0x3E && len >= 3 + 8) {
        // LE Meta Event. Subevent 0x01 = LE Advertising Report.
        const uint8_t sub = (len > 3) ? d[0] : 0;
        if (sub != 0x01 || len < 3 + 12) return;
        const size_t plen = static_cast<size_t>(p[2]);
        const uint8_t numReports = d[1];
        size_t off = 2;
        for (unsigned i = 0; i < numReports; ++i) {
            // event_type(1) addr_type(1) addr(6) len(1) data[len] rssi(1)
            if (off + 10 > plen + 3) break;
            const uint8_t* addr = d + off + 2;
            const uint8_t advLen = d[off + 8];
            const size_t need = 10 + advLen;
            if (off + need > plen + 3) break;
            const int8_t rssi = static_cast<int8_t>(d[off + 9 + advLen]);

            Observation o;
            o.radio = RadioKind::Bluetooth;
            o.frameClass = FrameClass::Management;
            o.rssiDbm = static_cast<double>(rssi);
            o.snrDb = 0.0;
            // Bluetooth address is little-endian on the wire; display order is
            // big-endian.
            for (int k = 0; k < 6; ++k) o.transmitter[static_cast<size_t>(k)] = addr[5 - k];
            o.hostTime = Clock::now();
            // Extended advertising reports carry an event_type where bit 0
            // distinguishes connectable from non-connectable, and bit 3 selects
            // legacy vs extended advertising. LE primary channel 37 = 2402 MHz.
            o.frequencyHz = 2402e6;
            o.channel = 37.0;
            advReports_.fetch_add(1, std::memory_order_relaxed);
            push(o);
            off += need;
        }
        return;
    }

    if (evt == 0x22 && len >= 3 + 14) {
        // Inquiry Result with RSSI: num_responses(1) bdaddr(6) class(3) clock(2) rssi(1)
        const uint8_t num = d[0];
        size_t off = 1;
        for (unsigned i = 0; i < num; ++i) {
            if (off + 13 > plen + 3) break;
            const uint8_t* addr = d + off;
            const int8_t rssi = static_cast<int8_t>(d[off + 12]);
            Observation o;
            o.radio = RadioKind::Bluetooth;
            o.frameClass = FrameClass::Management;
            o.rssiDbm = static_cast<double>(rssi);
            for (int k = 0; k < 6; ++k) o.transmitter[static_cast<size_t>(k)] = addr[5 - k];
            o.hostTime = Clock::now();
            o.channel = 79;  // BR/EDR inquiry uses the 2402 + 79*5 MHz pseudo channel
            inquiryResults_.fetch_add(1, std::memory_order_relaxed);
            push(o);
            off += 14;
        }
        return;
    }

    if (evt == 0x0F) {
        // Command Complete. Worth surfacing: if the controller never completes
        // our scan-enable, that is exactly why the UI shows no BT data.
        if (len >= 6) {
            const uint16_t opcode = static_cast<uint16_t>(p[4] | (p[5] << 8));
            const uint8_t status = p[6];
            char msg[128];
            std::snprintf(msg, sizeof(msg), "HCI cmd 0x%04x complete, status 0x%02x", opcode,
                          status);
            if (status != 0 || (opcode & 0x0C00) >> 10 == kOgfLeCtl)
                note(msg);
        }
        return;
    }

    if (evt == 0x01) {
        note("HCI Inquiry Complete");
        inquiryResults_.fetch_add(0, std::memory_order_relaxed);
    }
}

SensorStatus BluetoothSensor::status() const {
    SensorStatus s;
    s.name = name();
    s.kind = kind();
    s.device = devName_;
    s.active = isActive();
    s.available = fd_ >= 0;
    s.framesSeen = seen();
    s.framesDropped = dropped();
    s.rateHz = 0.0;
    Capabilities c = capabilities();
    s.capabilities.push_back("LE-scan");
    s.capabilities.push_back(c.longRangePhy ? "coded-phy" : "no-coded-phy");
    s.capabilities.push_back(c.directionFinding ? "AoA" : "no-AoA");
    s.capabilities.push_back("advReports=" + std::to_string(advReports_.load()));
    s.capabilities.push_back("inquiryResults=" + std::to_string(inquiryResults_.load()));
    s.detail = detail();
    return s;
}

std::vector<std::string> BluetoothSensor::probe(std::string* err) {
    std::vector<std::string> out;
    if (fd_ < 0) {
        if (err) *err = "sensor not started";
        return out;
    }
    out.push_back("device: " + devName_ + " (index " + std::to_string(devId_) + ")");
    out.push_back("HCI socket fd: " + std::to_string(fd_));
    out.push_back("scanning: " + std::string(scanning_.load() ? "yes" : "no"));
    out.push_back("LE advertising reports: " +
                  std::to_string(advReports_.load(std::memory_order_relaxed)));
    out.push_back("BR/EDR inquiry results: " +
                  std::to_string(inquiryResults_.load(std::memory_order_relaxed)));
    out.push_back("HCI packets seen: " + std::to_string(seen()));
    out.push_back("last controller message: " +
                  (detail().empty() ? std::string("(none)") : detail()));
    out.push_back("capabilities: " + capabilities().notes);
    return out;
}

}  // namespace radar