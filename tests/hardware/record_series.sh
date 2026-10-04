#!/bin/bash
# Record a stationary RSSI series for detector tuning / validation.
#
# Records BOTH operating modes so the threshold choice stays auditable:
#   connected_series_long.json  - the normal case (hotspot on)
#   monitor_series.json         - raw 802.11 capture (hotspot off, needs root)
#
# The machine must be genuinely still while recording: these recordings are
# the "no motion" ground truth.
source "$(dirname "$0")/common.sh"
require_root
trap restore_iface EXIT INT TERM

DUR="${DUR:-45}"
sudo -n nmcli connection up "$CON" >/dev/null 2>&1
sleep 4
echo "=== connected: $(iw dev "$IFACE" link | sed -n 2p) ==="

sudo -n env RSSI_RADAR_ROOT="$ROOT" RSSI_RADAR_IFACE="$IFACE" \
     RSSI_RADAR_OUT="$OUT" RSSI_RADAR_DUR="$DUR" python3 - <<'PY'
import json, os, statistics, sys, time
sys.path.insert(0, os.environ["RSSI_RADAR_ROOT"])
import rssi_radar_no_hotspot_v3 as R

iface = os.environ["RSSI_RADAR_IFACE"]
out   = os.environ["RSSI_RADAR_OUT"]
dur   = float(os.environ["RSSI_RADAR_DUR"])

def report(tag, series):
    p = os.path.join(out, tag + ".json")
    json.dump(series, open(p, "w"))
    print(f"{tag}: n={len(series)} mean={statistics.fmean(series):.2f} "
          f"sd={statistics.pstdev(series):.2f} "
          f"range=[{min(series):.1f},{max(series):.1f}] -> {p}")

# ---- connected mode: distinct readings only ----
out_c, last, t0 = [], None, time.time()
while time.time() - t0 < dur:
    r, _ = R._proc_net_rssi(iface)
    if r is not None and r != last:
        out_c.append(r); last = r
    time.sleep(0.05)
el = time.time() - t0
print(f"connected: {len(out_c)} distinct readings in {el:.0f}s "
      f"= {len(out_c)/el:.2f}/sec")
report("connected_series_long", out_c)
PY

# ---- monitor mode ----
sudo -n nmcli connection down "$CON" >/dev/null 2>&1
sleep 2
sudo -n nmcli device set "$IFACE" managed no
sudo -n ip link set "$IFACE" down
sudo -n iw  dev "$IFACE" set type monitor
sudo -n ip link set "$IFACE" up

sudo -n env RSSI_RADAR_ROOT="$ROOT" RSSI_RADAR_IFACE="$IFACE" \
     RSSI_RADAR_OUT="$OUT" RSSI_RADAR_DUR="$DUR" python3 - <<'PY'
import json, os, statistics, sys, threading, time
sys.path.insert(0, os.environ["RSSI_RADAR_ROOT"])
import rssi_radar_no_hotspot_v3 as R

iface = os.environ["RSSI_RADAR_IFACE"]
out   = os.environ["RSSI_RADAR_OUT"]
dur   = float(os.environ["RSSI_RADAR_DUR"])

rd = R.RadiotapBeaconReader(iface, 5.0)
ok, err = rd.open()
if not ok:
    raise SystemExit("raw capture failed: " + err)
rd._running = True
threading.Thread(target=rd._capture, daemon=True).start()

series, last, t0 = [], None, time.time()
while time.time() - t0 < dur:
    r = rd.best_rssi()
    if r is not None and r != last:
        series.append(round(r, 2)); last = r
    time.sleep(0.05)
rd.stop()
p = os.path.join(out, "monitor_series.json")
json.dump(series, open(p, "w"))
print(f"monitor: n={len(series)} mean={statistics.fmean(series):.2f} "
      f"sd={statistics.pstdev(series):.2f} "
      f"range=[{min(series):.1f},{max(series):.1f}] -> {p}")
PY