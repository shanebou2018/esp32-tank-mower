# ============================================================
#  lidar.py
#  Direct serial parser for YDLIDAR X2 / X2 Pro
#  Based on official EAI X2 Development Manual V1.1
#
#  Packet structure (little-endian):
#    PH(2B)  = 0xAA 0x55  packet header
#    CT(1B)  = packet type (bit0=1 means start/zero packet)
#    LSN(1B) = number of sample points
#    FSA(2B) = start angle raw
#    LSA(2B) = end angle raw
#    CS(2B)  = checksum (XOR of all 16-bit words except CS)
#    Si(2B)  = sample data x LSN (distance = Si >> 2, mm)
#
#  Angle decoding:
#    AngleFSA = (FSA >> 1) / 64.0
#    AngleLSA = (LSA >> 1) / 64.0
#    Anglei   = diff(Angle)/(LSN-1) * (i-1) + AngleFSA
#    AngCorrect = atan(21.8 * (155.3 - Di) / (155.3 * Di))  [Di in mm]
# ============================================================

import serial
import threading
import time
import math
import logging

logger = logging.getLogger(__name__)

LIDAR_PORT    = "/dev/ttyUSB0"
LIDAR_BAUD    = 115200
HEADER_A      = 0xAA   # PH low byte
HEADER_B      = 0x55   # PH high byte
MAX_DIST_M    = 8.0
MIN_DIST_M    = 0.1


class LidarX2:
    """
    Continuous background reader for the YDLIDAR X2/X2 Pro.
    Uses official packet format with checksum validation.
    Thread safe — call get_scan() from any thread.
    """

    def __init__(self, port=LIDAR_PORT, baud=LIDAR_BAUD):
        self.port  = port
        self.baud  = baud
        self._ser  = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        self._scan       = []
        self._scan_count = 0
        self._last_scan  = 0.0

        self._packets_ok    = 0
        self._packets_bad   = 0
        self._packets_error = 0

        self._thread = None

    # ── Public API ───────────────────────────────────────────

    def start(self):
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
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3)
        if self._ser and self._ser.is_open:
            self._ser.close()
            logger.info("[LIDAR] Serial closed")

    def get_scan(self) -> list:
        with self._lock:
            return list(self._scan)

    def get_closest(self) -> tuple:
        scan = self.get_scan()
        valid = [(a, d) for a, d in scan
                 if MIN_DIST_M <= d <= MAX_DIST_M]
        if not valid:
            return (0, 999)
        return min(valid, key=lambda x: x[1])

    def get_sector_min(self, angle_center: float, half_width: float) -> float:
        scan = self.get_scan()
        dists = []
        for a, d in scan:
            diff = abs(a - angle_center)
            if diff > 180:
                diff = 360 - diff
            if diff <= half_width and MIN_DIST_M <= d <= MAX_DIST_M:
                dists.append(d)
        return min(dists) if dists else 999

    def is_connected(self) -> bool:
        return (time.time() - self._last_scan) < 2.0

    def get_stats(self) -> dict:
        return {
            "scan_count":    self._scan_count,
            "packets_ok":    self._packets_ok,
            "packets_bad":   self._packets_bad,
            "packets_error": self._packets_error,
            "last_scan":     self._last_scan,
            "connected":     self.is_connected()
        }

    # ── Private ──────────────────────────────────────────────

    @staticmethod
    def _checksum(buf, num_points):
        """XOR all 16-bit words: PH, CT|LSN, FSA, LSA, all Si."""
        ph      = (buf[1] << 8) | buf[0]
        ct_lsn  = (buf[2]     ) | (buf[3] << 8)
        fsa     = (buf[5] << 8) | buf[4]
        lsa     = (buf[7] << 8) | buf[6]
        cs = ph ^ ct_lsn ^ fsa ^ lsa
        for i in range(num_points):
            o = 10 + i * 2
            cs ^= (buf[o+1] << 8) | buf[o]
        return cs

    @staticmethod
    def _decode_angle(raw):
        """First-level angle decode: (raw >> 1) / 64.0"""
        return (raw >> 1) / 64.0

    @staticmethod
    def _angle_correct(dist_mm):
        """Second-level angle correction per official formula."""
        if dist_mm <= 0:
            return 0.0
        return math.degrees(math.atan(21.8 * (155.3 - dist_mm) / (155.3 * dist_mm)))

    def _parse_packet(self, buf, num_points):
        """Parse a validated packet. Returns list of (angle_deg, dist_m)."""
        fsa = (buf[5] << 8) | buf[4]
        lsa = (buf[7] << 8) | buf[6]

        ang_fsa = self._decode_angle(fsa)
        ang_lsa = self._decode_angle(lsa)

        # Clockwise diff
        if ang_lsa >= ang_fsa:
            diff = ang_lsa - ang_fsa
        else:
            diff = ang_lsa + 360.0 - ang_fsa

        points = []
        for i in range(num_points):
            o        = 10 + i * 2
            si       = (buf[o+1] << 8) | buf[o]
            dist_mm  = si >> 2
            dist_m   = dist_mm / 1000.0

            # First level angle
            if num_points > 1:
                angle = (diff / (num_points - 1)) * i + ang_fsa
            else:
                angle = ang_fsa

            angle = angle % 360.0

            # Second level correction
            angle = (angle + self._angle_correct(dist_mm)) % 360.0

            points.append((angle, dist_m))

        return points

    def _reader_loop(self):
        logger.info("[LIDAR] Reader loop running")
        buf = bytearray()
        partial_scan = []
        last_emit    = time.time()
        EMIT_INTERVAL = 0.15   # ~7Hz scan rate

        while not self._stop_event.is_set():
            try:
                data = self._ser.read(128)
                if not data:
                    continue
                buf.extend(data)

                while len(buf) >= 10:
                    # Find header 0xAA 0x55
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

                    # Validate checksum
                    cs_stored = (buf[9] << 8) | buf[8]
                    cs_calc   = self._checksum(buf, num_points)

                    if cs_calc != cs_stored:
                        self._packets_bad += 1
                        buf = buf[2:]   # skip past this header and try again
                        continue

                    # Good packet
                    self._packets_ok += 1
                    ct       = buf[2]
                    is_zero  = bool(ct & 0x01)

                    points = self._parse_packet(buf, num_points)
                    partial_scan.extend(points)
                    buf = buf[pkt_len:]

                    # Emit scan on zero packet or time window
                    now = time.time()
                    if (is_zero or (now - last_emit) >= EMIT_INTERVAL) and len(partial_scan) >= 100:
                        with self._lock:
                            self._scan = sorted(partial_scan, key=lambda x: x[0])
                            self._scan_count += 1
                            self._last_scan  = now
                        logger.debug(f"[LIDAR] Scan {self._scan_count}: {len(partial_scan)} pts zero={is_zero}")
                        partial_scan = []
                        last_emit    = now

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
    print("  YDLIDAR X2 Parser — Official Protocol Test")
    print("=" * 50)

    lidar = LidarX2()
    lidar.start()

    print("Waiting for first scan...")
    for _ in range(30):
        time.sleep(0.5)
        stats = lidar.get_stats()
        if stats["scan_count"] > 0:
            scan    = lidar.get_scan()
            closest = lidar.get_closest()
            print(f"\nScan {stats['scan_count']}: {len(scan)} points")
            print(f"Packets OK={stats['packets_ok']}  Bad={stats['packets_bad']}")
            print(f"Closest: {closest[1]:.3f}m at {closest[0]:.1f}deg")
            print(f"Front (0+-20deg): {lidar.get_sector_min(0, 20):.3f}m")
            print("\nAngle distribution (45deg sectors):")
            sectors = [0] * 8
            valid = [(a,d) for a,d in scan if MIN_DIST_M <= d <= MAX_DIST_M]
            for a, d in valid:
                sectors[int(a / 45)] += 1
            for i, c in enumerate(sectors):
                bar = "#" * min(c, 40)
                print(f"  {i*45:3d}-{i*45+45:3d}deg: {c:4d}  {bar}")
            break
        elif stats["packets_ok"] > 0 or stats["packets_bad"] > 0:
            print(f"  Packets OK={stats['packets_ok']} Bad={stats['packets_bad']}...")
    else:
        print("No scan received")

    lidar.stop()
    print("\nDone")
