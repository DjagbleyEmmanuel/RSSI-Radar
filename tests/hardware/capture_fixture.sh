#!/bin/bash
# Capture a real 802.11 frame for use as a parser regression fixture, and
# report the live parse rate.
#
# This is what caught the parser rejecting 100% of frames on this driver
# (radiotap present-bit 29 NAMESPACE). Re-run it if you change _RT_FIELDS.
source "$(dirname "$0")/common.sh"
require_root
trap restore_iface EXIT INT TERM

sudo -n nmcli connection down "$CON" >/dev/null 2>&1
sleep 2
sudo -n nmcli device set "$IFACE" managed no
sudo -n ip link set "$IFACE" down
sudo -n iw  dev "$IFACE" set type monitor
sudo -n ip link set "$IFACE" up

sudo -n env RSSI_RADAR_ROOT="$ROOT" RSSI_RADAR_IFACE="$IFACE" \
  python3 - <<'PY'
import os, sys, socket, struct
sys.path.insert(0, os.environ["RSSI_RADAR_ROOT"])
import rssi_radar_no_hotspot_v3 as R

iface = os.environ["RSSI_RADAR_IFACE"]
s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003))
s.bind((iface, 0)); s.settimeout(1.0)

raw = ok = 0
sample = None
while raw < 400:
    try:
        f = s.recv(4096)
    except socket.timeout:
        break
    raw += 1
    if R.RadiotapBeaconReader._parse(f) is not None:
        ok += 1
        if sample is None:
            sample = f

print(f"RAW frames: {raw}   PARSED OK: {ok}")
if sample is None:
    raise SystemExit("parser rejected every frame — check _RT_FIELDS / NAMESPACE handling")

rssi, bssid, freq = R.RadiotapBeaconReader._parse(sample)
print(f"first accepted frame: rssi={rssi:.0f} dBm  bssid={bssid}  freq={freq:.0f} MHz")
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                   "fixtures", "real_frame.hex")
with open(out, "w") as fh:
    fh.write(sample.hex())
print(f"saved {len(sample)}-byte fixture -> {os.path.normpath(out)}")
PY