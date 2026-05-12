# ============================================================
#  compass.py
#  BNO055 9-DOF IMU reader
#  Address 0x29 (ADR pin high)
#
#  Usage:
#    from compass import Compass
#    compass = Compass()
#    compass.start()
#    heading = compass.get_heading()
#    data    = compass.get_all()
#    compass.stop()
# ============================================================

import threading
import time
import logging
import board
import busio
import adafruit_bno055

logger = logging.getLogger(__name__)

BNO055_ADDRESS  = 0x29   # ADR pin high
POLL_INTERVAL   = 0.05   # 20Hz polling


class Compass:
    """
    Background reader for BNO055 IMU.
    Polls at 20Hz and caches latest values.
    Thread safe.
    """

    def __init__(self, address=BNO055_ADDRESS):
        self.address     = address
        self._lock       = threading.Lock()
        self._stop_event = threading.Event()
        self._thread     = None
        self._sensor     = None

        # Cached sensor values
        self._heading    = 0.0    # degrees 0-360
        self._roll       = 0.0
        self._pitch      = 0.0
        self._temp       = 0.0
        self._cal        = (0, 0, 0, 0)   # sys, gyro, accel, mag
        self._last_read  = 0.0
        self._read_count = 0
        self._errors     = 0

    # ── Public API ───────────────────────────────────────────

    def start(self):
        """Initialise I2C and start background polling thread."""
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self._sensor = adafruit_bno055.BNO055_I2C(i2c, address=self.address)
            logger.info(f"[COMPASS] BNO055 connected at 0x{self.address:02X}")
            # Quick check
            temp = self._sensor.temperature
            logger.info(f"[COMPASS] Temperature: {temp}C")
        except Exception as e:
            logger.error(f"[COMPASS] Failed to connect: {e}")
            raise

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        logger.info("[COMPASS] Polling thread started")

    def stop(self):
        """Stop polling thread."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2)
        logger.info("[COMPASS] Stopped")

    def get_heading(self) -> float:
        """Return current heading in degrees (0-360). 0=North."""
        with self._lock:
            return self._heading

    def get_all(self) -> dict:
        """Return all sensor data."""
        with self._lock:
            return {
                "heading":    self._heading,
                "roll":       self._roll,
                "pitch":      self._pitch,
                "temp":       self._temp,
                "cal_sys":    self._cal[0],
                "cal_gyro":   self._cal[1],
                "cal_accel":  self._cal[2],
                "cal_mag":    self._cal[3],
                "calibrated": self._cal[0] >= 2 and self._cal[3] >= 2,
                "last_read":  self._last_read,
                "read_count": self._read_count,
                "errors":     self._errors,
                "connected":  self.is_connected()
            }

    def is_connected(self) -> bool:
        """True if a reading arrived within the last 2 seconds."""
        return (time.time() - self._last_read) < 2.0

    def is_calibrated(self) -> bool:
        """True if sys and mag calibration are both >= 2."""
        with self._lock:
            return self._cal[0] >= 2 and self._cal[3] >= 2

    # ── Private ──────────────────────────────────────────────

    def _poll_loop(self):
        logger.info("[COMPASS] Poll loop running")
        while not self._stop_event.is_set():
            try:
                euler = self._sensor.euler
                cal   = self._sensor.calibration_status
                temp  = self._sensor.temperature

                if euler and euler[0] is not None:
                    with self._lock:
                        self._heading    = round(euler[0], 2)
                        self._roll       = round(euler[1], 2) if euler[1] else 0.0
                        self._pitch      = round(euler[2], 2) if euler[2] else 0.0
                        self._temp       = temp if temp else 0.0
                        self._cal        = cal if cal else (0, 0, 0, 0)
                        self._last_read  = time.time()
                        self._read_count += 1
                    logger.debug(f"[COMPASS] H={self._heading:.1f} "
                                 f"R={self._roll:.1f} P={self._pitch:.1f} "
                                 f"cal={cal}")

            except Exception as e:
                self._errors += 1
                logger.error(f"[COMPASS] Read error: {e}")
                time.sleep(0.5)
                continue

            time.sleep(POLL_INTERVAL)


# ── Quick test ───────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    print("=" * 50)
    print("  BNO055 Compass — Test")
    print("=" * 50)

    compass = Compass()
    compass.start()

    print("Reading for 5 seconds — rotate the sensor to test...\n")
    for i in range(10):
        time.sleep(0.5)
        d = compass.get_all()
        cal_str = f"sys={d['cal_sys']} gyro={d['cal_gyro']} accel={d['cal_accel']} mag={d['cal_mag']}"
        print(f"Heading={d['heading']:6.1f}  Roll={d['roll']:6.1f}  Pitch={d['pitch']:6.1f}  "
              f"Cal: {cal_str}  {'CALIBRATED' if d['calibrated'] else 'calibrating...'}")

    compass.stop()
    print("\nDone")
