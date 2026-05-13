# ============================================================
# odometry.py
# Dead-reckoning pose estimator for the tank mower.
#
# Reads enc_l / enc_r tick counts from the ESP32 serial bridge
# and optionally corrects heading using the BNO055 compass.
#
# Pose is (x, y, theta) in metres and radians:
#   x, y   — position in the local frame (origin = where start() was called)
#   theta  — heading in radians, 0 = forward, CCW positive
#
# Usage:
#   from odometry import Odometry
#   from serial_bridge import ESP32Bridge
#   from compass import Compass
#
#   bridge  = ESP32Bridge(); bridge.start()
#   compass = Compass();     compass.start()
#
#   odom = Odometry(bridge, compass)
#   odom.start()
#
#   pose = odom.get_pose()          # {"x": 0.0, "y": 0.0, "theta": 0.0, ...}
#   odom.reset()                    # zero the pose
#   odom.stop()
#
# Coordinate frame:
#   Robot starts at (0, 0) facing theta=0 (north / forward).
#   Moving forward increases x (East).  Turn left → theta increases.
#   This matches standard robotics convention (REP-103 ENU).
#
# Heading fusion:
#   When compass is available and calibrated (cal_sys >= 2, cal_mag >= 2),
#   the BNO055 heading is blended in with a complementary filter:
#       theta = ALPHA * imu_rad  +  (1 - ALPHA) * encoder_theta
#   ALPHA = 0.02 (2% IMU per update at 20Hz → slow drift correction,
#   won't fight the encoders on tight turns).
#   Set COMPASS_ALPHA = 0.0 to use pure encoder dead-reckoning.
#
# Wheel geometry (must match firmware constants):
#   WHEEL_DIAMETER_M = 0.254  (10 inches)
#   GEAR_RATIO       = 16.0   (motor revs per wheel rev)
#   TRACK_WIDTH_M    = 0.457  (18 inches between track centres — MEASURE YOURS)
#
# The AS5600 reports cumulative degree ticks (one sign change per degree of
# motor shaft rotation). The firmware accumulates these as enc_l / enc_r.
# We only ever look at the DELTA between consecutive readings, so absolute
# tick values and wrap-around don't matter.
# ============================================================

import threading
import time
import math
import logging

logger = logging.getLogger(__name__)

# ── Wheel geometry ───────────────────────────────────────────
# !! MEASURE TRACK_WIDTH_M on your physical mower !!
# It is the distance between the centrelines of the two tracks.
# Getting this right is the single biggest factor in turn accuracy.
WHEEL_DIAMETER_M = 0.254        # 10 inches — matches firmware
GEAR_RATIO       = 16.0         # motor revs per wheel rev — matches firmware
TRACK_WIDTH_M    = 0.457        # 18 inches — ADJUST TO YOUR BUILD

# Metres of travel per encoder degree tick
# dist = (pi * D) / (gear_ratio * 360)
DIST_PER_TICK_M = (math.pi * WHEEL_DIAMETER_M) / (GEAR_RATIO * 360.0)

# ── Complementary filter weight for IMU heading ──────────────
# 0.0 = pure encoder dead-reckoning (no compass)
# 0.02 = gentle drift correction (~2% IMU per 50ms update)
# 0.1 = stronger correction (use if track slippage is bad)
COMPASS_ALPHA = 0.02

# ── Update rate ──────────────────────────────────────────────
POLL_INTERVAL = 0.05   # 20 Hz — matches ESP32 telemetry rate

# ── Tick sanity limit ────────────────────────────────────────
# Maximum plausible tick delta per 50ms update.
# 127 PWM / 255 max * full speed ≈ ~50% speed.
# At 10 inch wheels, 16:1 gear, ~1 m/s max → 1/0.05 * 50ms = 0.05m max/update
# 0.05 / DIST_PER_TICK_M ≈ ~720 ticks/update at full speed.
# We use 2000 as a generous sanity cap.
MAX_DELTA_TICKS = 2000


class Odometry:
    """
    Continuous dead-reckoning pose estimator.

    Integrates encoder tick deltas at 20Hz.
    Optionally fuses BNO055 heading via complementary filter.
    Thread-safe — call get_pose() from any thread.
    """

    def __init__(self, bridge, compass=None):
        """
        Args:
            bridge:  ESP32Bridge instance (must be started before odom.start())
            compass: Compass instance or None (heading fusion disabled if None)
        """
        self._bridge  = bridge
        self._compass = compass

        self._lock        = threading.Lock()
        self._stop_event  = threading.Event()
        self._thread      = None

        # Pose — updated atomically under _lock
        self._x     = 0.0   # metres
        self._y     = 0.0   # metres
        self._theta = 0.0   # radians, CCW positive

        # Previous tick counts (to compute deltas)
        self._prev_enc_l = None
        self._prev_enc_r = None

        # Stats
        self._update_count = 0
        self._error_count  = 0
        self._last_update  = 0.0

        # Compass heading at start (offset so initial heading = 0)
        self._heading_offset = 0.0
        self._compass_active = False

    # ── Public API ───────────────────────────────────────────

    def start(self):
        """Start background pose integration thread."""
        # Grab initial tick counts so first delta is 0
        state = self._bridge.get_state()
        self._prev_enc_l = state.get("enc_l", 0)
        self._prev_enc_r = state.get("enc_r", 0)

        # Capture compass heading offset so pose starts at theta=0
        if self._compass and self._compass.is_calibrated():
            raw_deg = self._compass.get_heading()
            self._heading_offset = math.radians(raw_deg)
            self._compass_active = True
            logger.info(f"[ODOM] Compass active, offset={math.degrees(self._heading_offset):.1f}deg")
        else:
            self._compass_active = False
            logger.info("[ODOM] No compass or not calibrated — pure encoder dead-reckoning")

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()
        logger.info("[ODOM] Pose integration started")
        logger.info(f"[ODOM] DIST_PER_TICK_M={DIST_PER_TICK_M*1000:.4f}mm  "
                    f"TRACK_WIDTH={TRACK_WIDTH_M*100:.1f}cm")

    def stop(self):
        """Stop background thread."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2)
        logger.info("[ODOM] Stopped")

    def reset(self, x=0.0, y=0.0, theta=0.0):
        """
        Reset pose to (x, y, theta). Also resets prev tick reference
        so the next update computes a fresh delta from the current ticks.
        """
        state = self._bridge.get_state()
        with self._lock:
            self._x     = x
            self._y     = y
            self._theta = theta
            self._prev_enc_l = state.get("enc_l", 0)
            self._prev_enc_r = state.get("enc_r", 0)
        logger.info(f"[ODOM] Pose reset to x={x:.3f} y={y:.3f} theta={math.degrees(theta):.1f}deg")

    def get_pose(self) -> dict:
        """
        Return current pose estimate.

        Returns:
            dict with keys:
                x, y        — position in metres
                theta       — heading in radians
                heading_deg — heading in degrees (0=forward, CCW positive)
                update_count — total integration steps
                compass_active — True if IMU fusion is running
                last_update — timestamp of last update
        """
        with self._lock:
            return {
                "x":              round(self._x, 4),
                "y":              round(self._y, 4),
                "theta":          round(self._theta, 4),
                "heading_deg":    round(math.degrees(self._theta) % 360, 1),
                "update_count":   self._update_count,
                "error_count":    self._error_count,
                "compass_active": self._compass_active,
                "last_update":    self._last_update,
            }

    def get_position(self) -> tuple:
        """Return (x, y, theta) as a simple tuple."""
        with self._lock:
            return (self._x, self._y, self._theta)

    def is_running(self) -> bool:
        """True if update loop is alive and receiving fresh data."""
        return (time.time() - self._last_update) < 0.5

    # ── Private ──────────────────────────────────────────────

    def _update_loop(self):
        """Background thread — integrates encoder deltas at POLL_INTERVAL."""
        logger.info("[ODOM] Update loop running")

        while not self._stop_event.is_set():
            try:
                self._integrate()
            except Exception as e:
                self._error_count += 1
                logger.error(f"[ODOM] Integration error: {e}")

            time.sleep(POLL_INTERVAL)

    def _integrate(self):
        """
        Core dead-reckoning step.

        1. Read current enc_l / enc_r from bridge
        2. Compute tick deltas since last call
        3. Convert to wheel distances
        4. Compute arc segment (differential drive kinematics)
        5. Update (x, y, theta)
        6. Optionally blend BNO055 heading
        """
        state = self._bridge.get_state()

        # Guard: no telemetry yet
        if state.get("last_rx", 0) == 0:
            return

        enc_l = state.get("enc_l", 0)
        enc_r = state.get("enc_r", 0)

        # First call — initialise reference, no integration yet
        if self._prev_enc_l is None:
            self._prev_enc_l = enc_l
            self._prev_enc_r = enc_r
            return

        # ── Tick deltas ────────────────────────────────────
        delta_l = enc_l - self._prev_enc_l
        delta_r = enc_r - self._prev_enc_r

        self._prev_enc_l = enc_l
        self._prev_enc_r = enc_r

        # Sanity check — reject implausibly large jumps
        # (serial glitch, encoder reset, or first packet after reconnect)
        if abs(delta_l) > MAX_DELTA_TICKS or abs(delta_r) > MAX_DELTA_TICKS:
            logger.warning(f"[ODOM] Tick jump rejected: dL={delta_l} dR={delta_r}")
            self._error_count += 1
            return

        # ── Wheel distances ────────────────────────────────
        dist_l = delta_l * DIST_PER_TICK_M
        dist_r = delta_r * DIST_PER_TICK_M

        # ── Differential drive kinematics ─────────────────
        # Centre-of-robot travel distance and turn angle
        dist_centre = (dist_r + dist_l) / 2.0
        delta_theta  = (dist_r - dist_l) / TRACK_WIDTH_M

        with self._lock:
            theta = self._theta

        # Heading at midpoint of arc (more accurate than using start heading)
        theta_mid = theta + delta_theta / 2.0

        dx = dist_centre * math.cos(theta_mid)
        dy = dist_centre * math.sin(theta_mid)

        new_theta = theta + delta_theta

        # ── Compass heading fusion (complementary filter) ─
        if self._compass_active and self._compass and self._compass.is_calibrated():
            raw_deg = self._compass.get_heading()
            # BNO055 reports 0–360 CW from North.
            # Convert to our CCW frame relative to startup heading.
            imu_abs = math.radians(raw_deg)
            imu_rel = imu_abs - self._heading_offset
            # Normalise to -pi..+pi
            imu_rel = math.atan2(math.sin(imu_rel), math.cos(imu_rel))
            # Complementary filter: lean heavily on encoders, nudge toward IMU
            new_theta = (COMPASS_ALPHA * imu_rel) + ((1.0 - COMPASS_ALPHA) * new_theta)

        # Normalise theta to -pi..+pi
        new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))

        # ── Commit ────────────────────────────────────────
        with self._lock:
            self._x     += dx
            self._y     += dy
            self._theta  = new_theta
            self._update_count += 1
            self._last_update   = time.time()

        logger.debug(
            f"[ODOM] dL={delta_l:+d} dR={delta_r:+d}  "
            f"dist={dist_centre*100:.1f}cm  dTheta={math.degrees(delta_theta):.2f}deg  "
            f"x={self._x:.3f} y={self._y:.3f} hdg={math.degrees(new_theta):.1f}deg"
        )


# ── Quick standalone test ────────────────────────────────────
if __name__ == "__main__":
    import sys
    import os

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    print("=" * 60)
    print("  Odometry — Standalone Test")
    print("  Reads live ticks from ESP32 via serial bridge")
    print("=" * 60)

    sys.path.insert(0, os.path.dirname(__file__))
    from serial_bridge import ESP32Bridge

    # Try to import compass — optional
    compass = None
    try:
        from compass import Compass
        compass = Compass()
        compass.start()
        print("[TEST] Compass started")
    except Exception as e:
        print(f"[TEST] No compass ({e}) — pure dead-reckoning")

    bridge = ESP32Bridge()
    try:
        bridge.start()
    except Exception as e:
        print(f"[FATAL] Could not open serial bridge: {e}")
        sys.exit(1)

    odom = Odometry(bridge, compass)
    odom.start()

    print("\nDriving will be reflected in pose. Press Ctrl+C to stop.\n")
    print(f"{'Updates':>8}  {'x (m)':>8}  {'y (m)':>8}  {'hdg (deg)':>10}  {'compass':>8}")
    print("-" * 55)

    try:
        while True:
            time.sleep(0.5)
            p = odom.get_pose()
            compass_str = "active" if p["compass_active"] else "off"
            print(
                f"{p['update_count']:>8d}  "
                f"{p['x']:>8.3f}  "
                f"{p['y']:>8.3f}  "
                f"{p['heading_deg']:>10.1f}  "
                f"{compass_str:>8}"
            )
    except KeyboardInterrupt:
        print("\n[TEST] Stopped by user")
    finally:
        odom.stop()
        bridge.close()
        if compass:
            compass.stop()
        print("[TEST] Done")
