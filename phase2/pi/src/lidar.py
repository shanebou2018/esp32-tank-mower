# ============================================================
#  lidar.py
#  YDLIDAR X4 reader using PyLidar3
#  pip install PyLidar3
#
#  Usage:
#    from lidar import LidarX4
#    lidar = LidarX4()
#    lidar.start()
#    scan = lidar.get_scan()   # dict {angle_int: dist_m} or list [(angle, dist)]
#    lidar.stop()
# ============================================================

import threading
import time
import logging
from PyLidar3 import YdLidarX4

logger = logging.getLogger(__name__)

LIDAR_PORT   = "/dev/ttyUSB0"
MAX_DIST_M   = 10.0
MIN_DIST_M   = 0.12


class LidarX4:
    """
    Continuous background reader for the YDLIDAR X4.
    Uses PyLidar3 which handles the protocol correctly.
    Thread safe — call get_scan() from any thread.
    """

    def __init__(self, port=LIDAR_PORT):
        self.port        = port
        self._lidar      = YdLidarX4(port)
        self._lock       = threading.Lock()
        self._stop_event = threading.Event()

        # Latest scan — dict {angle_deg(int 0-359): dist_m}
        self._scan       = {}
        self._scan_count = 0
        self._last_scan  = 0.0
        self._thread     = None

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
        Return latest scan as list of (angle_deg, dist_m).
        Filters out zero/invalid distances.
        """
        with self._lock:
            return [(a, d) for a, d in self._scan.items()
                    if MIN_DIST_M <= d <= MAX_DIST_M]

    def get_scan_raw(self) -> dict:
        """Return raw scan dict {angle: dist_m} including zeros."""
        with self._lock:
            return dict(self._scan)

    def get_closest(self) -> tuple:
        """Return (angle_deg, dist_m) of closest valid obstacle."""
        scan = self.get_scan()
        if not scan:
            return (0, 999)
        return min(scan, key=lambda x: x[1])

    def get_sector_min(self, angle_center: float, half_width: float) -> float:
        """Minimum distance in a sector. Returns 999 if no valid readings."""
        scan = self.get_scan()
        dists = []
        for a, d in scan:
            diff = abs(a - angle_center)
            if diff > 180:
                diff = 360 - diff
            if diff <= half_width:
                dists.append(d)
        return min(dists) if dists else 999

    def is_connected(self) -> bool:
        """True if scan data arrived within last 2 seconds."""
        return (time.time() - self._last_scan) < 2.0

    def get_stats(self) -> dict:
        return {
            "scan_count": self._scan_count,
            "last_scan":  self._last_scan,
            "connected":  self.is_connected()
        }

    # ── Private ──────────────────────────────────────────────

    def _reader_loop(self):
        logger.info("[LIDAR] Reader loop running")
        try:
            gen = self._lidar.StartScanning()
            while not self._stop_event.is_set():
                try:
                    raw = next(gen)  # dict {angle(int): distance(float, mm)}
                    # Convert mm to m
                    scan_m = {a: d / 1000.0 for a, d in raw.items()}
                    with self._lock:
                        self._scan       = scan_m
                        self._scan_count += 1
                        self._last_scan  = time.time()
                    logger.debug(f"[LIDAR] Scan {self._scan_count}: "
                                 f"{sum(1 for v in scan_m.values() if v > 0)} valid pts")
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
