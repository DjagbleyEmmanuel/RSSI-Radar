"""Regression tests for rssi_radar_no_hotspot_v3.py"""
import sys, os, io, time, threading, struct, unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                      # the radar lives one level up
sys.path.insert(0, ROOT)
import rssi_radar_no_hotspot_v3 as R

FIXTURES = os.path.join(HERE, "fixtures")


class TestWelfordWindow(unittest.TestCase):
    def test_degenerate_maxlen_does_not_crash(self):
        """--window 0/negative used to IndexError on the first push."""
        for ml in (0, -5, 1):
            w = R.WelfordWindow(ml)
            for i in range(50):
                w.push(-50.0 + i % 5)
            self.assertGreater(w._count, 0)

    def test_numerically_exact(self):
        import random, statistics
        random.seed(3)
        w = R.WelfordWindow(10); ref = []
        for _ in range(300):
            v = random.gauss(-60, 4); w.push(v); ref.append(v)
            if len(ref) > 10: ref.pop(0)
            if len(ref) > 1:
                self.assertAlmostEqual(w.mean, statistics.fmean(ref), places=6)
                self.assertAlmostEqual(w.std, statistics.pstdev(ref), places=6)


class TestRadiotapParse(unittest.TestCase):
    @staticmethod
    def _frame(fields):
        """fields: list of (bit, packed_bytes, align). Build a radiotap frame."""
        present = 0
        for bit, _, _ in fields:
            present |= (1 << bit)
        body = b""
        for bit, data, align in fields:
            if align > 1:
                pad = (-len(body)) % align
                body += b"\x00" * pad
            body += data
            del bit
        header = struct.pack("<BBHI", 0, 0, 8 + len(body), present)
        # fc + dur + Addr1 + Addr2
        fc = struct.pack("<H", (2 << 2) | (3 << 8))     # type=data
        mac = bytes.fromhex("aabbccddeeff")
        return header + body + fc + b"\x00\x00" + mac + mac + b"\x00" * 20

    def test_channel_field_size_no_longer_corrupts_rssi(self):
        """
        Bit 3 (CHANNEL) is 6 bytes (freq, flags, freq_offset), not 4. With the
        wrong size, DBM_ANTSIGNAL was read 2 bytes early and produced garbage.
        """
        body = struct.pack("<H", 2412) + struct.pack("<H", 0x0000) + struct.pack("<H", 0)
        frame = self._frame([
            (0, struct.pack("<Q", 123456789), 8),   # TSFT
            (2, struct.pack("<B", 0x0c), 1),       # RATE
            (3, body, 2),                          # CHANNEL (6 bytes)
            (5, struct.pack("<b", -42), 1),        # DBM_ANTSIGNAL
        ])
        got = R.RadiotapBeaconReader._parse(frame)
        self.assertIsNotNone(got, "channel-present radiotap failed to parse")
        self.assertEqual(got[0], -42.0)
        self.assertEqual(got[1], "aa:bb:cc:dd:ee:ff")
        self.assertEqual(got[2], 2412.0)

    def test_tsft_only_rejected_for_rssi(self):
        frame = self._frame([(0, struct.pack("<Q", 1), 8)])
        self.assertIsNone(R.RadiotapBeaconReader._parse(frame))

    def test_implausible_rssi_rejected(self):
        frame = self._frame([(5, struct.pack("<b", 100), 1)])
        self.assertIsNone(R.RadiotapBeaconReader._parse(frame))

    def test_control_frames_rejected(self):
        fc = struct.pack("<H", (1 << 2) | (3 << 8))
        mac = bytes.fromhex("aabbccddeeff")
        body = struct.pack("<b", -50)
        frame = struct.pack("<BBHI", 0, 0, 8 + len(body), 1 << 5) + body + fc + b"\x00\x00" + mac + mac
        self.assertIsNone(R.RadiotapBeaconReader._parse(frame))

    def test_short_and_malformed_frames_do_not_raise(self):
        for junk in (b"", b"\x00", b"\x00" * 7, b"\x00" * 8, bytes(64), b"\x01\x00" * 40):
            R.RadiotapBeaconReader._parse(junk)


class TestStickySource(unittest.TestCase):
    def test_pinned_source_survives_hopping(self):
        """
        Free-running 'strongest transmitter' selection jumped APs every hop,
        producing fake motion. The reader must stay locked on one BSSID.
        """
        rd = R.RadiotapBeaconReader("test0")
        now = time.monotonic()
        with rd._lock:
            rd._sources["aa:aa:aa:aa:aa:aa"] = {"ema": -60.0, "last": now, "count": 50.0,
                                               "channel": 1.0, "peak": -55.0}
            rd._sources["bb:bb:bb:bb:bb:bb"] = {"ema": -40.0, "last": now, "count": 50.0,
                                               "channel": 6.0, "peak": -35.0}
        first = rd.best_rssi()
        self.assertEqual(first, -40.0)             # picks strongest on 1st frame
        for _ in range(50):
            self.assertEqual(rd.best_rssi(), first)   # then must NOT flap
        self.assertEqual(rd._pinned, "bb:bb:bb:bb:bb:bb")

    def test_repins_only_when_pinned_source_goes_stale(self):
        rd = R.RadiotapBeaconReader("test0")
        now = time.monotonic()
        with rd._lock:
            rd._sources["aa:aa:aa:aa:aa:aa"] = {"ema": -80.0, "last": now - 99, "count": 50.0,
                                               "channel": 1.0, "peak": -70.0}
            rd._sources["bb:bb:bb:bb:bb:bb"] = {"ema": -45.0, "last": now, "count": 50.0,
                                               "channel": 6.0, "peak": -40.0}
        rd._pinned = "aa:aa:aa:aa:aa:aa"
        self.assertEqual(rd.best_rssi(), -45.0)   # hands over to the live AP

    def test_no_sources_returns_none(self):
        self.assertIsNone(R.RadiotapBeaconReader("t").best_rssi())


class TestMonitorEnableRollback(unittest.TestCase):
    """The root cause of 'built-in wifi refuses to work after hotspot off'."""

    def setUp(self):
        # tests run unprivileged; stub the CAP_NET_RAW pre-flight so the
        # monitor path can be exercised at all
        self._orig_open = R.RadiotapBeaconReader.open
        R.RadiotapBeaconReader.open = lambda self: (True, "")
        self.addCleanup(lambda: setattr(R.RadiotapBeaconReader, "open", self._orig_open))
        self._orig_shutil_which = R.shutil.which

    def _make(self, fail_monitor=False, fail_reader=False):
        m = R.MonitorModeManager("wlanX")
        calls = []
        def priv(*cmd):
            calls.append(list(cmd))
            if "monitor" in cmd and "type" in cmd and fail_monitor:
                return False, "Device or resource busy (-16)"
            return True, ""
        def run(*cmd):
            calls.append(list(cmd)); return True, ""
        m._privileged, m._run = priv, run
        R.shutil.which = lambda c: f"/usr/bin/{c}" if c == "nmcli" else None
        return m, calls

    def test_failure_restores_link_and_nm(self):
        m, calls = self._make(fail_monitor=True)
        st = m._enable()
        self.assertTrue(st.startswith("error"))
        self.assertFalse(m.active)
        self.assertFalse(m._nm_suspended, "NetworkManager must not stay disabled")
        flat = [" ".join(c) for c in calls]
        self.assertIn("nmcli device set wlanX managed yes", flat)
        self.assertIn("iw dev wlanX set type managed", flat)
        self.assertIn("ip link set wlanX up", flat)
        # link must come back UP
        self.assertGreater(flat.index("ip link set wlanX up"),
                           flat.index("nmcli device set wlanX managed no"))

    def test_cleanup_restores_even_if_never_active(self):
        """cleanup() used to be gated on self.active, so a failed enable
        left the card down forever."""
        m, calls = self._make()
        m._nm_suspended = True
        m.cleanup()
        self.assertIn("ip link set wlanX up", [" ".join(c) for c in calls])

    def test_raw_capture_denied_never_touches_interface(self):
        """
        Without CAP_NET_RAW the run must abort before disabling NetworkManager
        or cycling the link, not after. Verified on real hardware: an
        unprivileged run used to un-manage NM, bounce the card, then hit EPERM.
        """
        R.RadiotapBeaconReader.open = lambda self: (
            False, "raw capture needs root/CAP_NET_RAW")
        m, calls = self._make()
        st = m._enable()
        self.assertFalse(m.active)
        self.assertIn("needs root", st)
        self.assertEqual(calls, [], "interface was touched despite no raw access")
        self.assertFalse(m._nm_suspended)

    def test_success_path_marks_active(self):
        m, _ = self._make()
        R.RadiotapBeaconReader.start = lambda self: (True, "")
        R.RadiotapBeaconReader.stop = lambda self: None
        m._discover_channels = lambda: [1, 6, 11]
        m._hop_thread = None
        try:
            st = m._enable()
            self.assertTrue(m.active)
            self.assertIn("sweep", st)
        finally:
            m._hop_running = False
            R.RadiotapBeaconReader.stop = lambda self: None


class TestMonitorNoLockHeldDuringWork(unittest.TestCase):
    def test_enable_does_not_hold_lock_during_privileged_work(self):
        """
        _enable() runs several privileged subprocesses. If it holds self._lock
        across them, the UI thread's snapshot() blocks and the curses display
        freezes for seconds. best_rssi() must stay responsive throughout.
        """
        _orig = R.RadiotapBeaconReader.open
        R.RadiotapBeaconReader.open = lambda self: (True, "")
        self.addCleanup(lambda: setattr(R.RadiotapBeaconReader, "open", _orig))
        m = R.MonitorModeManager("wlanX")
        inside = threading.Event(); release = threading.Event()
        def slow_priv(*cmd):
            inside.set(); release.wait(3); return True, ""
        m._privileged = slow_priv
        R.shutil.which = lambda c: None
        try:
            t = threading.Thread(target=lambda: m._enable(), daemon=True)
            t.start()
            self.assertTrue(inside.wait(2), "_enable never reached the subprocess")
            done = threading.Event()
            threading.Thread(target=lambda: (m.best_rssi(), m.beacon_count(),
                                             done.set()), daemon=True).start()
            responsive = done.wait(0.5)
            release.set(); t.join(3)
            self.assertTrue(responsive, "UI accessors blocked behind _enable()")
        finally:
            import shutil as sh
            R.shutil.which = sh.which
            m._hop_running = False
            R.RadiotapBeaconReader.stop = lambda self: None


class TestScanCacheStaleness(unittest.TestCase):
    def test_dead_refresh_thread_does_not_serve_frozen_rssi(self):
        """A failed refresh used to leave one immortal RSSI value, so the
        display looked live but the detector could never see change."""
        r = R.InterfaceReader("wlanX")
        r._has_proc = False
        with r._scan_lock:
            r._scan_rssi = -55.0
            r._scan_age = 0.0
            r._scan_updated_at = time.monotonic() - 10_000.0
        self.assertEqual(r._scan_dump_rssi(), (None, -999.0))

    def test_fresh_cache_is_returned(self):
        r = R.InterfaceReader("wlanX")
        r._has_proc = False
        with r._scan_lock:
            r._scan_rssi = -55.0
            r._scan_age = 0.4
            r._scan_updated_at = time.monotonic()
            r._last_scan_t = time.monotonic()
        self.assertEqual(r._scan_dump_rssi(), (-55.0, -999.0))

    def test_monitor_suspended_short_circuits(self):
        r = R.InterfaceReader("wlanX")
        r.set_monitor_suspended(True)
        self.assertEqual(r._scan_dump_rssi(), (None, -999.0))
        self.assertEqual(r.mode, "monitor")


class TestProcParsing(unittest.TestCase):
    FAKE = (
        "Inter-| sta-|   Quality        |   Discarded packets\n"
        " face | tus | link level noise |  nwid  crypt   frag  retry   misc\n"
        "wlp3s0: 0000   70.  -45.  -256        0      0      0      0   1091\n"
        "wlp3s01: 0000   30.  -80.  -256        0      0      0      0      0\n"
    )
    def _patch(self, text):
        # these readers open /proc/net/wireless in binary mode
        from unittest import mock
        p = mock.patch("builtins.open", lambda *a, **k: io.BytesIO(text.encode()))
        p.start()
        self.addCleanup(p.stop)

    def test_prefix_names_do_not_cross_match(self):
        self._patch(self.FAKE)
        rssi, noise = R._proc_net_rssi("wlp3s0")
        self.assertEqual(rssi, -45.0)
        self.assertEqual(R._proc_net_link_quality("wlp3s0"), 70.0)
        rssi2, _ = R._proc_net_rssi("wlp3s01")
        self.assertEqual(rssi2, -80.0)
        self.assertEqual(R._proc_net_link_quality("wlp3s01"), 30.0)

    def test_unknown_iface_is_none(self):
        self._patch(self.FAKE)
        self.assertEqual(R._proc_net_rssi("wlp3s0x"), (None, -999.0))
        self.assertEqual(R._proc_net_link_quality("nope0"), 0.0)

    def test_never_associated_reports_none(self):
        self._patch(" face | tus | link level noise\n"
                    "wlp3s0: 0000    0.    0.    0\n")
        self.assertEqual(R._proc_net_rssi("wlp3s0"), (None, -999.0))
        self.assertEqual(R._proc_net_link_quality("wlp3s0"), 0.0)


class TestFusion(unittest.TestCase):
    def _s(self, iface, rssi, primary, **kw):
        return R.InterfaceSample(iface=iface, rssi=rssi, smoothed=rssi, velocity=0.0,
                                 noise=-999.0, snr=-999.0, retry_rate=0.0, drop_rate=0.0,
                                 beacon_rssi=-999.0, confidence=kw.get("conf", 0.0),
                                 timestamp=time.time(), is_primary=primary, **{
                                     k: v for k, v in kw.items() if k.startswith("base")})

    def test_carries_real_baseline_not_proxies(self):
        cfg = R.Config()
        s = R.InterfaceSample(iface="w", rssi=-60, smoothed=-58, velocity=3.0, noise=-999.0,
                              snr=-999.0, retry_rate=0, drop_rate=0, beacon_rssi=-999.0,
                              confidence=0.1, timestamp=time.time(), is_primary=True,
                              baseline_mean=-61.5, baseline_std=2.25)
        f = R._fuse([s], cfg)
        self.assertAlmostEqual(f.baseline_mean, -61.5)
        self.assertAlmostEqual(f.baseline_std, 2.25)

    def test_degenerate_weights_do_not_divide_by_zero(self):
        cfg = R.Config(primary_weight=1.0)
        s = self._s("w", -60, primary=False, conf=0.3)
        f = R._fuse([s], cfg)           # only secondaries exist
        self.assertIsNotNone(f)
        self.assertAlmostEqual(f.rssi, -60.0)

    def test_empty_returns_none(self):
        self.assertIsNone(R._fuse([], R.Config()))


class TestDetectorRobustness(unittest.TestCase):
    def test_no_div_by_zero_on_tiny_debounce(self):
        cfg = R.Config(window_size=4, debounce_count=1)
        d = R.InterfaceDetector(cfg)
        for _ in range(50):
            d.update(-60.0, -95.0, 0.0, 0.0, -999.0)

    def test_detects_large_step(self):
        """
        Uses a controllable clock: the RSSI window only accepts DISTINCT
        readings plus a 1 s heartbeat, so a tight real-time loop would never
        advance it and this test would measure the harness, not the detector.
        """
        clock = {"t": 1000.0}
        real_time = R.time.time
        R.time.time = lambda: clock["t"]
        self.addCleanup(lambda: setattr(R.time, "time", real_time))

        cfg = R.Config(window_size=60, debounce_count=5, motion_threshold=2.0)
        d = R.InterfaceDetector(cfg)
        # steady link: every 50 ms, always the same value
        for _ in range(400):
            clock["t"] += 0.05
            d.update(-60.0, -95.0, 0.0, 0.0, -999.0)
        self.assertFalse(d.detected, "steady link must be quiet")
        # person walks in: RSSI steps up and stays there
        for _ in range(80):
            clock["t"] += 0.05
            d.update(-48.0, -95.0, 0.0, 0.0, -999.0)
        self.assertTrue(d.detected, "12 dB sustained step should trigger motion")

    def test_heartbeat_fills_window_on_dead_link(self):
        """A link whose RSSI never changes must still reach activation, via
        the 1 s heartbeat - otherwise detection stays off forever."""
        clock = {"t": 1000.0}
        real_time = R.time.time
        R.time.time = lambda: clock["t"]
        self.addCleanup(lambda: setattr(R.time, "time", real_time))
        cfg = R.Config(window_size=120, debounce_count=20)
        d = R.InterfaceDetector(cfg)
        for _ in range(4000):          # 200 s at 50 ms
            clock["t"] += 0.05
            d.update(-60.0, -95.0, 0.0, 0.0, -999.0)
        self.assertGreaterEqual(d._rssi_win.n, cfg._min_samples if hasattr(cfg, "_min_samples") else R.MIN_ACTIVATION_SAMPLES)
        self.assertFalse(d.detected, "still must be quiet")

    def test_adaptive_threshold_strong_signal(self):
        self.assertLess(R._adaptive_threshold(-40, 1.2), R._adaptive_threshold(-80, 1.2))


class TestDisplayHelpers(unittest.TestCase):
    def test_sparkline_widths(self):
        self.assertEqual(len(R._sparkline([], 10)), 10)
        self.assertEqual(len(R._sparkline([-50.0, -51, -52], 10)), 10)
        self.assertEqual(len(R._sparkline(list(range(100)), 20)), 20)

    def test_bar_clamps(self):
        # defaults are lo=0.0 hi=1.0
        self.assertTrue(R._bar(0.5, 5).startswith("█"))
        self.assertTrue(R._bar(999.0, 5).startswith("█"))
        self.assertTrue(R._bar(-999.0, 5).startswith("░"))
        self.assertEqual(len(R._bar(0.5, 7)), 7)
        # explicit dBm window
        self.assertTrue(R._bar(-40.0, 5, lo=-105.0, hi=-30.0).startswith("█"))
        self.assertTrue(R._bar(-100.0, 5, lo=-105.0, hi=-30.0).startswith("░"))


class TestCalmness(unittest.TestCase):
    """Sanity: a perfectly stable signal must not produce motion."""

    def test_steady_signal_no_motion(self):
        import random
        cfg = R.Config(window_size=60, debounce_count=20, motion_threshold=1.2)
        for sigma in (0.15, 0.25, 0.5, 1.0, 1.5, 2.0):
            random.seed(11)
            d = R.InterfaceDetector(cfg)
            flags = []
            for _ in range(500):
                d.update(-60.0 + random.gauss(0, sigma), -95.0, 0.0, 0.0, -999.0)
                flags.append(d.detected)
            warm = cfg.window_size
            rate = sum(flags[warm:]) / len(flags[warm:])
            self.assertLess(rate, 0.02,
                            f"still link (sigma={sigma}) reported motion {rate:.0%} of the time")


class TestRealRadiotapFrame(unittest.TestCase):
    """Fixture captured live from wlp3s0 (Intel AX211) in monitor mode.

    This driver's frames set present bits 29 (NAMESPACE) and 31 (extended).
    The parser used to reject 100% of them ("unknown bit -> discard"), which is
    why monitor mode reported 0 packets. It also emits a 4-byte CHANNEL field.
    """

    @classmethod
    def setUpClass(cls):
        path = os.path.join(FIXTURES, "real_frame.hex")
        if not os.path.exists(path):
            raise unittest.SkipTest("fixture real_frame.hex missing")
        with open(path) as fh:
            cls.HEX = fh.read().strip()

    def test_real_frame_parses(self):
        f = bytes.fromhex(self.HEX)
        got = R.RadiotapBeaconReader._parse(f)
        self.assertIsNotNone(got, "parser rejected a genuine on-air frame")
        rssi, bssid, freq = got
        self.assertTrue(-95 <= rssi <= -25, "implausible RSSI " + str(rssi))
        self.assertRegex(bssid, r"^([0-9a-f]{2}:){5}[0-9a-f]{2}$")
        self.assertTrue(2400 <= freq <= 2500 or 4900 <= freq <= 5900, "bad freq")

    def test_truncated_copies_do_not_raise(self):
        f = bytes.fromhex(self.HEX)
        for cut in (0, 4, 8, 12, 20, 40, 56, len(f) - 1):
            R.RadiotapBeaconReader._parse(f[:cut])


if __name__ == "__main__":
    unittest.main(verbosity=2)