#!/bin/bash
# Monitor-mode verification as ROOT.
#
# Monitor mode needs CAP_NET_RAW, which `sudo -n iw` cannot supply to an
# unprivileged process, so the whole radar has to run under sudo here.
# Expect: monitor engages, packets arrive continuously, RSSI is live.
source "$(dirname "$0")/common.sh"
require_root
trap restore_iface EXIT INT TERM

sudo -n nmcli connection down "$CON" >/dev/null 2>&1
sleep 3
echo "=== disconnected; running radar AS ROOT ==="

sudo -n env RSSI_RADAR_ROOT="$ROOT" RSSI_RADAR_IFACE="$IFACE" \
  timeout 45 python3 - <<'PY'
import os, sys, time
sys.path.insert(0, os.environ["RSSI_RADAR_ROOT"])
import rssi_radar_no_hotspot_v3 as R

print("euid =", os.geteuid())
radar = R.RSSIRadar(R.Config(primary=os.environ["RSSI_RADAR_IFACE"]))
for i in range(38):
    time.sleep(1.0)
    d = radar.snapshot()
    r = d["rssi"]
    print(f"t={i+1:2d}s mon={str(d['monitor_active']):5} aps={d['monitor_beacons']} "
          f"pkts={d['monitor_packets']:5d} age={d['monitor_packet_age']:5.2f}s "
          f"rf={d['monitor_rf_status']:12} "
          f"rssi={'None' if r is None else f'{r:+6.1f}'} "
          f"motion={str(d['motion']):5} mode={[s.mode for s in d['ifaces']]}")
print("\npackets captured:", d["monitor_packets"])
radar.stop()
print("monitor active after stop():", radar._monitor.active)
PY

echo
echo "=== CARD STATE AFTER EXIT ==="
echo "    type      : $(iw dev "$IFACE" info 2>/dev/null | awk '/type/{print $2}')"
echo "    NM-MANAGED: $(nmcli -g GENERAL.NM-MANAGED dev show "$IFACE" 2>/dev/null)"