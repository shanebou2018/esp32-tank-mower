# ============================================================
#  lidar.py
#  Direct serial parser for YDLIDAR X2 / X2 Pro
#  No SDK needed — reads raw packets from /dev/ttyUSB0
#
#  Usage:
#    from lidar import LidarX2
#    lidar = LidarX2()
#    lidar.start()
#    scan = lidar.get_scan()   # list of (angle_deg, dist_m)
#    lidar.stop()
# ============================================================

import serial
import threading
import time
import math
import logging

logger = logging.getLogger(__name__)

LIDAR_PORT     = "/dev/ttyUSB0"
LIDAR_BAUD     = 115200
HEADER_A       = 0xD6
HEADER_B       = 0x2D
MAX_DIST_M     = 8.0    # X2 max range
MIN_DIST_M     = 0.1    # X2 min range
INVALID_DIST   = 8.224  # X2 sends this for no-return points


class LidarX2:
    """
    Continuous background reader for the YDLIDAR X2/X2 Pro.
    Parses raw serial packets and assembles full 360 degree scans.
    Thread safe — call get_scan() from any thread.
    """

    def __init__(self, port=LIDAR_PORT, baud=LIDAR_BAUD):
        self.port  = port
        self.baud  = baud
        self._ser  = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # Latest complete 360 scan — list of (angle_deg, dist_m)
        self._scan       = []
        self._scan_count = 0
        self._last_scan  = 0.0

        # Stats
        self._packets_parsed = 0
        self._packets_error  = 0

        self._thread = None

    # ── Public API ───────────────────────────────────────────

    def start(self):
        """Open serial port and start background reader thread."""
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=1.0)
            logger.info(f"[LIDAR] Serial open: {self.port} @ {self.baud}")
        except serial.SerialException as e:
            logger.error(f"[LIDAR] Failed to open {self.port}: {e}")
            raise

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        logger.info("[LIDAR] Reader thread started")

    def stop(self):
        """Stop the reader thread and close serial port."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3)
        if self._ser and self._ser.is_open:
            self._ser.close()
            logger.info("[LIDAR] Serial closed")

    def get_scan(self) -> list:
        """
        Return a copy of the latest complete 360 scan.
        Each entry is (angle_deg, dist_m).
        Returns empty list if no scan yet.
        """
        with self._lock:
            return list(self._scan)

    def get_scan_polar(self) -> list:
        """
        Return scan as list of (angle_rad, dist_m) for plotting.
        Filters out invalid and out-of-range points.
        """
        with self._lock:
            return [
                (math.radians(a), d)
                for a, d in self._scan
                if MIN_DIST_M <= d <= MAX_DIST_M and abs(d - INVALID_DIST) > 0.01
            ]

    def get_closest(self) -> tuple:
        """
        Return (angle_deg, dist_m) of the closest valid obstacle.
        Returns (0, 999) if no scan available.
        """
        scan = self.get_scan()
        valid = [(a, d) for a, d in scan
                 if MIN_DIST_M <= d <= MAX_DIST_M and abs(d - INVALID_DIST) > 0.01]
        if not valid:
            return (0, 999)
        return min(valid, key=lambda x: x[1])

    def get_sector_min(self, angle_center: float, half_width: float) -> float:
        """
        Return minimum distance in a sector centered on angle_center
        with +/- half_width degrees. Useful for obstacle avoidance.
        Returns 999 if no valid readings in sector.
        """
        scan = self.get_scan()
        dists = []
        for a, d in scan:
            diff = abs(a - angle_center)
            if diff > 180:
                diff = 360 - diff
            if diff <= half_width and MIN_DIST_M <= d <= MAX_DIST_M and abs(d - INVALID_DIST) > 0.01:
                dists.append(d)
        return min(dists) if dists else 999

    def is_connected(self) -> bool:
        """True if scan data arrived within the last 2 seconds."""
        return (time.time() - self._last_scan) < 2.0

    def get_stats(self) -> dict:
        return {
            "scan_count":     self._scan_count,
            "packets_parsed": self._packets_parsed,
            "packets_error":  self._packets_error,
            "last_scan":      self._last_scan,
            "connected":      self.is_connected()
        }

    # ── Private ──────────────────────────────────────────────

    def _reader_loop(self):
        """Background thread — reads serial and parses packets into scans."""
        logger.info("[LIDAR] Reader loop running")
        buf = bytearray()
        partial_scan = []

        # Scan emit strategy: use zero-packet flag (bit0 of type byte)
        # Zero packet = start of new revolution on X2
        # Fallback: emit every 150ms if we have enough points
        last_emit_time = time.time()
        EMIT_INTERVAL  = 0.15   # seconds — one revolution at ~7Hz

        while not self._stop_event.is_set():
            try:
                data = self._ser.read(128)
                if not data:
                    continue
                buf.extend(data)

                # Parse all complete packets from buffer
                while len(buf) >= 10:
                    # Find header 0xD6 0x2D
                    idx = -1
                    for i in range(len(buf) - 1):
                        if buf[i] == HEADER_A and buf[i+1] == HEADER_B:
                            idx = i
                            break

                    if idx == -1:
                        buf = buf[-1:]
                        break
                    if idx > 0:
                        buf = buf[idx:]
                    if len(buf) < 10:
                        break

                    num_points = buf[3]
                    if num_points == 0:
                        buf = buf[2:]
                        continue

                    pkt_len = 10 + num_points * 2
                    if len(buf) < pkt_len:
                        break

                    # Type byte bit 0 = zero/start packet (new revolution)
                    pkt_type   = buf[2]
                    is_zero    = bool(pkt_type & 0x01)

                    # Decode angles
                    start_ang = (((buf[5] << 8) | buf[4]) >> 1) / 64.0 % 360.0
                    end_ang   = (((buf[7] << 8) | buf[6]) >> 1) / 64.0 % 360.0

                    # Decode points with official angle correction formula
                    # First level: linear interpolation between FSA and LSA
                    # Second level: AngCorrect = atan(21.8*(155.3-D)/(155.3*D))
                    if num_points > 1:
                        if end_ang >= start_ang:
                            diff_ang = end_ang - start_ang
                        else:
                            diff_ang = end_ang + 360.0 - start_ang
                        step = diff_ang / (num_points - 1)
                    else:
                        step = 0

                    for i in range(num_points):
                        offset   = 10 + i * 2
                        dist_raw = ((buf[offset+1] << 8) | buf[offset]) >> 2
                        dist_m   = dist_raw / 1000.0

                        # First level angle
                        angle = (start_ang + step * i) % 360.0

                        # Second level angle correction (official formula)
                        if dist_m > 0.0:
                            ang_correct = math.degrees(
                                math.atan(21.8 * (155.3 - dist_m * 1000.0) /
                                         (155.3 * dist_m * 1000.0))
                            )
                            angle = (angle + ang_correct) % 360.0

                        partial_scan.append((angle, dist_m))

                    self._packets_parsed += 1
                    buf = buf[pkt_len:]

                    # Emit scan on zero packet OR time window
                    now = time.time()
                    elapsed = now - last_emit_time
                    if (is_zero or elapsed >= EMIT_INTERVAL) and len(partial_scan) >= 100:
                        with self._lock:
                            self._scan = sorted(partial_scan, key=lambda x: x[0])
                            self._scan_count += 1
                            self._last_scan = now
                        logger.debug(f"[LIDAR] Scan {self._scan_count}: {len(partial_scan)} pts zero={is_zero}")
                        partial_scan = []
                        last_emit_time = now

            except serial.SerialException as e:
                logger.error(f"[LIDAR] Serial error: {e}")
                time.sleep(0.5)
            except Exception as e:
                logger.error(f"[LIDAR] Parser error: {e}")
                self._packets_error += 1
                time.sleep(0.1)


# ── Quick test ───────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    print("=" * 50)
    print("  YDLIDAR X2 Direct Parser — Test")
    print("=" * 50)

    lidar = LidarX2()
    lidar.start()

    print("Waiting for first scan...")
    for _ in range(20):
        time.sleep(0.5)
        if lidar.is_connected():
            scan = lidar.get_scan()
            if scan:
                print(f"\nGot scan with {len(scan)} points")
                closest = lidar.get_closest()
                print(f"Closest obstacle: {closest[1]:.3f}m at {closest[0]:.1f} degrees")
                print(f"Front sector (0 +/-20deg): {lidar.get_sector_min(0, 20):.3f}m")
                print(f"Stats: {lidar.get_stats()}")

                print("\nSample points:")
                for i in range(0, min(len(scan), 360), 36):
                    a, d = scan[i]
                    print(f"  {a:6.1f} deg -> {d:.3f}m")
                break
    else:
        print("No scan received — check wiring")

    lidar.stop()
    print("\nDone")
