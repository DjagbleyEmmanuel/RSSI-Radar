# Tests

## Quick check (safe, no root, no wifi disruption)

```bash
cd tests
python3 test_radar.py     # 33 regression tests
python3 accuracy.py       # detection accuracy across RSSI noise levels
```

`test_radar.py` covers the bugs that were fixed:

| Area | What it locks down |
|---|---|
| `TestMonitorEnableRollback` | a failed monitor entry must restore link + NetworkManager (the reported bug) |
| | an unprivileged run must refuse *before* touching the interface |
| | `cleanup()` restores even when monitor never became active |
| | `_enable()` must not hold its lock across privileged subprocesses |
| `TestRadiotapParse` / `TestRealRadiotapFrame` | the parser accepts a genuine on-air frame from this machine |
| | CHANNEL width probing, NAMESPACE bit 29, truncated/malformed input |
| `TestStickySource` | the tracked transmitter must not flap between APs |
| `TestScanCacheStaleness` | a dead scan thread must not serve one frozen RSSI forever |
| `TestCalmness` | a motionless link reads quiet at every noise level |
| `TestProcParsing` | `wlp3s0` must not match `wlp3s01` |
| `TestWelfordWindow` | degenerate `--window` values do not crash; math is exact |
| `TestDetectorRobustness` | a real excursion is detected; no divide-by-zero |

## Hardware tests (root, and they disconnect your Wi-Fi)

> These take the interface down and into monitor mode. They restore it via a
> shell `trap`, but **do not run them over a remote session** — if the machine
> loses power mid-run you will need to reconnect by hand.

```bash
cd tests/hardware
CON="My Network" ./hotspot_off_test.sh    # unprivileged; expects scan fallback
CON="My Network" ./monitor_root_test.sh   # as root; expects monitor mode
CON="My Network" ./record_series.sh       # record fresh tuning data
CON="My Network" ./capture_fixture.sh     # re-capture real_frame.hex
```

`CON` is auto-detected from the active NetworkManager profile if omitted.
`IFACE` defaults to `wlp3s0`. Output lands in `tests/hardware/out/`.

## Fixtures

`fixtures/` holds evidence rather than mocks, so the tuned defaults stay
auditable instead of being magic numbers:

- `real_frame.hex` — a real 802.11 frame captured from this machine's Intel
  AX211. Its radiotap header sets present bits 29 (`NAMESPACE`) and 31, which
  the old parser treated as fatal, so it discarded **100%** of real frames.
- `monitor_series.json`, `monitor_series2.json` — stationary monitor-mode RSSI
- `connected_series_long.json` — stationary connected-mode RSSI

Replaying these through the detector is how `--window 120 --threshold 2.5` were
chosen. At the old `--window 60 --threshold 1.2` they produced **27–40% false
"motion"** on a motionless link.

Measured characteristics worth remembering:

- RSSI noise is **correlated**, not white (lag-1 autocorrelation ≈ 0.85), which
  is why a short statistics window produces unstable z-scores.
- Connected mode yields only **~0.58 distinct readings/sec** (integer dBm,
  refreshed ~1 Hz) while monitor mode yields ~6/sec. `--window` therefore
  counts *distinct* readings, and the activation bar is deliberately lower
  than the window length so connected mode can start at all.
- Single-AP RSSI motion detection is genuinely marginal (~17–33% response).
  The old code's apparent perfect sensitivity was entirely false positives.