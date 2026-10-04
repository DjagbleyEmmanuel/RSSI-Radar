#!/bin/bash
# Hotspot-off verification (runs UNPRIVILEGED on purpose).
#
# Reproduces the original bug: with the shared hotspot switched off, the radar
# must still use the built-in NIC. Expects it to fall back to beacon-table scan
# mode and produce live data, and must leave the card usable afterwards.
#
#   CON="My Network" ./hotspot_off_test.sh
source "$(dirname "$0")/common.sh"
trap restore_iface EXIT INT TERM

echo "=== DISCONNECTING '$CON' (simulating hotspot off) ==="
sudo -n nmcli connection down "$CON" >/dev/null 2>&1
sleep 3
echo "    link: $(iw dev "$IFACE" link 2>&1 | head -1)"

python3 - <<'PY'
import os, sys, time
sys.path.insert(0, os.environ["RSSI_RADAR_ROOT"])
import rssi_radar_no_hotspot_v3 as R

radar = R.RSSIRadar(R.Config())
print("=== HOTSPOT-OFF: expect scan fallback with live data ===")
prev = None
for i in range(30):
    time.sleep(1.0)
    d = radar.snapshot()
    state = (d["monitor_active"], tuple(s.mode for s in d["ifaces"]))
    r = d["rssi"]
    mark = "  <-- STATE CHANGE" if state != prev else ""
    prev = state
    print(f"t={i+1:2d}s mon={str(d['monitor_active']):5} "
          f"mode={[s.mode for s in d['ifaces']]!s:20} "
          f"rssi={'None' if r is None else f'{r:+6.1f}'} "
          f"motion={str(d['motion']):5} age={d['sample_age']:5.2f}s "
          f"lost={d['signal_lost']}{mark}")
radar.stop()
print("\nmonitor active after stop():", radar._monitor.active)
PY

echo
echo "=== CARD STATE AFTER RADAR EXITED ==="
echo "    type      : $(iw dev "$IFACE" info 2>/dev/null | awk '/type/{print $2}')"
echo "    NM-MANAGED: $(nmcli -g GENERAL.NM-MANAGED dev show "$IFACE" 2>/dev/null)"