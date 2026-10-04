"""
Detection accuracy harness: does the radar stay quiet on a still link and
still fire on real motion, across a range of RSSI noise levels?
"""
import sys, os, random, statistics

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import rssi_radar_no_hotspot_v3 as R


def run(rssi_series, cfg):
    d = R.InterfaceDetector(cfg)
    fired = []
    for rssi in rssi_series:
        d.update(rssi, -95.0, 0.0, 0.0, -999.0)
        fired.append(d.detected)
    return fired


def calm_series(sigma, n=400, seed=1):
    random.seed(seed)
    return [-60.0 + random.gauss(0, sigma) for _ in range(n)]


def motion_series(sigma, n=400, seed=2, amp=10.0, at=200, ramp=1.0):
    random.seed(seed)
    out = []
    for i in range(n):
        if i < at:
            out.append(-60.0 + random.gauss(0, sigma))
        else:
            f = min(1.0, (i - at) / (ramp / 0.05))
            out.append(-60.0 + amp * f + random.gauss(0, sigma))
    return out


# exercise the SHIPPED defaults, not a stale hardcoded copy
CFG = R.Config()
print("defaults: window=%d threshold=%.1f debounce=%d ratio=%.2f\n"
      % (CFG.window_size, CFG.motion_threshold, CFG.debounce_count, CFG.motion_ratio))

print(f"{'sigma':>6} | {'CALM false-fire rate':>22} | {'MOTION detected':>15} | {'latency':>9}")
print("-" * 62)
rows = []
for sigma in (0.15, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0):
    calm = run(calm_series(sigma), CFG)
    # ignore the warm-up window before the detector has a full baseline
    warm = CFG.window_size
    calm_fire = sum(calm[warm:]) / len(calm[warm:])

    mot = run(motion_series(sigma), CFG)
    at = 200
    detected = any(mot[at:])
    latency = ((mot.index(True, at) - at) * 0.05) if True in mot[at:] else float("nan")

    flag = ""
    if calm_fire > 0.02 or not detected:
        flag = "  <-- BAD"
    print(f"{sigma:6.2f} | {calm_fire:21.1%} | {str(detected):>15} | {latency:7.2f}s{flag}")
    rows.append((sigma, calm_fire, detected))

bad = [r for r in rows if r[1] > 0.02 or not r[2]]
print()
if bad:
    print(f"FAIL: {len(bad)} of {len(rows)} noise levels misbehave")
    sys.exit(1)
print("PASS: quiet when still, fires on motion at every noise level")