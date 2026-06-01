# ============================================================
#  lidar.py
#  YDLIDAR X4 reader using PyLidar3
#  pip install PyLidar3
#
#  Usage:
#    from lidar import LidarX4
#    lidar = LidarX4()
#    lidar.start()
#    scan = lidar.get_scan()   # list [(angle_deg, dist_m)]
#    lidar.stop()
#
#  Angle convention (after CW→CCW flip):
#    0° = robot forward, 90° = left, 180° = rear, 270° = right
# ============================================================

import threading
import time
import logging
from PyLidar3 import YdLidarX4

logger = logging.getLogger(__name__)

LIDAR_PORT   = "/dev/ttyUSB0"
MAX_DIST_M   = 10.0
MIN_DIST_M   = 0.12

# Noise filter — an angle must appear in at least this many scans within the
# buffer window before it is published. With AVERAGE_WINDOW=0.5s at ~7Hz,
# the buffer holds ~3-4 scans; MIN_SCAN_HITS=2 rejects single-scan speckle
# while still responding quickly to real obstacles.
MIN_SCAN_HITS = 2

# Mounting calibration — degrees added after the CW→CCW flip.
# The YDLIDAR X4 angles increase clockwise (viewed from above).
# We flip to CCW so 90° = left / 270° = right in the navigator.
# Adjust ANGLE_OFFSET until 0° points straight forward on the robot;
# increase to rotate the scan clockwise, decrease to rotate CCW.
ANGLE_OFFSET = 0   # degrees — tune for your mounting orientation


class LidarX4:
    """
    Continuous background reader for the YDLIDAR X4.
    Thread safe — call get_scan() from any thread.

    Per-angle minimum is kept over a short buffer window (not mean) so
    that an approaching obstacle is never diluted by older, farther readings.
    Sensor dropout is detected via is_connected(); get_sector_min() returns
    0.0 (below STOP_DIST) when no data is available so callers fail safe.
    """

    def __init__(self, port=LIDAR_PORT):
        self.port        = port
        self._lidar      = YdLidarX4(port)
        self._lock       = threading.Lock()
        self._stop_event = threading.Event()

        self._scan       = {}
        self._scan_count = 0
        self._last_scan  = 0.0
        self._thread     = None

        # Keep per-angle minimum over this window (shorter = more responsive)
        self.AVERAGE_WINDOW = 0.5   # seconds (~3-4 scans at 7 Hz)
        self._scan_buffer   = []    # list of (timestamp, scan_dict)
        self._min_scan      = {}    # per-angle minimum over the buffer

    # ── Public API ───────────────────────────────────────────

    def start(self):
        """Connect to LIDAR and start background reader thread."""
        ok = self._lidar.Connect()
        if not ok:
            raise RuntimeError(f"[LIDAR] Failed to connect on {self.port}")
        logger.info(f"[LIDAR] Connected on {self.port}")
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        logger.info("[LIDAR] Reader thread started")

    def stop(self):
        """Stop scanning and disconnect."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3)
        try:
            self._lidar.StopScanning()
            self._lidar.Disconnect()
            logger.info("[LIDAR] Disconnected")
        except Exception as e:
            logger.error(f"[LIDAR] Disconnect error: {e}")

    def get_scan(self) -> list:
        """
        Return scan as list of (angle_deg, dist_m).
        Angle 0 = forward, 90 = left, 270 = right (CCW convention).
        """
        with self._lock:
            return [(a, d) for a, d in self._min_scan.items()
                    if MIN_DIST_M <= d <= MAX_DIST_M]

    def get_scan_raw(self) -> dict:
        """Return raw scan dict {angle: dist_m} including zeros."""
        with self._lock:
            return dict(self._scan)

    def get_closest(self) -> tuple:
        """
        Return (angle_deg, dist_m) of closest valid obstacle.
        Returns (None, None) when no valid scan data is available.
        """
        scan = self.get_scan()
        if not scan:
            return (None, None)
        return min(scan, key=lambda x: x[1])

    def get_sector_min(self, angle_center: float, half_width: float) -> float:
        """
        Minimum distance in a sector (metres).
        Returns 0.0 when no valid readings exist — callers must treat this
        as 'blocked / sensor unavailable', not 'all clear'.
        """
        scan = self.get_scan()
        dists = []
        for a, d in scan:
            diff = abs(a - angle_center)
            if diff > 180:
                diff = 360 - diff
            if diff <= half_width:
                dists.append(d)
        return min(dists) if dists else 0.0

    def is_connected(self) -> bool:
        """True if a scan arrived within the last 2 seconds."""
        with self._lock:
            last = self._last_scan
        return (time.time() - last) < 2.0

    def get_stats(self) -> dict:
        with self._lock:
            count = self._scan_count
            last  = self._last_scan
        return {
            "scan_count": count,
            "last_scan":  last,
            "connected":  (time.time() - last) < 2.0,
        }

    # ── Private ──────────────────────────────────────────────

    def _reader_loop(self):
        logger.info("[LIDAR] Reader loop running")
        try:
            gen = self._lidar.StartScanning()
            while not self._stop_event.is_set():
                try:
                    raw = next(gen)  # dict {angle(int 0-359): distance(float, mm)}

                    # Flip CW→CCW and apply mounting offset so that
                    # 0° = forward, 90° = left, 270° = right.
                    # PyLidar3 confirmed to yield distances in millimetres;
                    # divide by 1000 to get metres.
                    scan_m = {
                        (360 - a + ANGLE_OFFSET) % 360: d / 1000.0
                        for a, d in raw.items()
                    }

                    now = time.time()
                    with self._lock:
                        self._scan       = scan_m
                        self._scan_count += 1
                        self._last_scan  = now

                        # Maintain rolling buffer
                        self._scan_buffer.append((now, scan_m))
                        self._scan_buffer = [
                            (t, s) for t, s in self._scan_buffer
                            if now - t <= self.AVERAGE_WINDOW
                        ]

                        # Per-angle minimum over the buffer window, with a
                        # consistency gate: an angle must appear in at least
                        # MIN_SCAN_HITS scans before it is published. This
                        # keeps sensitivity to approaching obstacles while
                        # rejecting single-scan noise spikes.
                        angle_mins = {}
                        angle_hits = {}
                        for _, s in self._scan_buffer:
                            for a, d in s.items():
                                if MIN_DIST_M <= d <= MAX_DIST_M:
                                    angle_hits[a] = angle_hits.get(a, 0) + 1
                                    if a not in angle_mins or d < angle_mins[a]:
                                        angle_mins[a] = d
                        self._min_scan = {
                            a: d for a, d in angle_mins.items()
                            if angle_hits.get(a, 0) >= MIN_SCAN_HITS
                        }

                    valid = sum(1 for v in scan_m.values() if MIN_DIST_M <= v <= MAX_DIST_M)
                    logger.debug(f"[LIDAR] Scan {self._scan_count}: {valid} valid pts "
                                 f"buf={len(self._scan_buffer)}")

                except StopIteration:
                    logger.warning("[LIDAR] Scan generator stopped")
                    break
                except Exception as e:
                    logger.error(f"[LIDAR] Scan error: {e}")
                    time.sleep(0.1)
        except Exception as e:
            logger.error(f"[LIDAR] Reader loop error: {e}")


# ── Quick test ───────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    print("=" * 50)
    print("  YDLIDAR X4 — PyLidar3 Test")
    print("=" * 50)

    lidar = LidarX4()
    lidar.start()

    print("Waiting for first scan...")
    for _ in range(20):
        time.sleep(0.5)
        if lidar.is_connected():
            scan    = lidar.get_scan()
            closest = lidar.get_closest()
            stats   = lidar.get_stats()
            print(f"\nScan {stats['scan_count']}: {len(scan)} valid points")
            if closest[0] is not None:
                print(f"Closest: {closest[1]:.3f}m at {closest[0]}deg")
            print(f"Front (0+-20deg): {lidar.get_sector_min(0, 20):.3f}m")
            print("\nAngle distribution (45deg sectors):")
            sectors = [0] * 8
            for a, d in scan:
                sectors[int(a / 45)] += 1
            for i, c in enumerate(sectors):
                bar = "#" * min(c // 2, 40)
                print(f"  {i*45:3d}-{i*45+45:3d}deg: {c:4d}  {bar}")
            break
    else:
        print("No scan received")

    lidar.stop()
    print("\nDone")
