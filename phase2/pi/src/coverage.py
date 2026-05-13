# ============================================================
# coverage.py
# Autonomous coverage patterns for the tank mower.
#
# Two modes:
#   BOUSTROPHEDON — back-and-forth strips (default, recommended first)
#   SPIRAL        — inside-out expanding spiral
#
# Both modes:
#   - Use compass to hold straight headings
#   - Use LiDAR (via Navigator) for obstacle avoidance
#   - Use odometry for strip width / distance tracking
#   - Respect the state machine — can be paused/stopped cleanly
#
# Usage:
#   from coverage import CoveragePlanner, CoverageMode
#
#   planner = CoveragePlanner(navigator, odometry, compass, lidar)
#   planner.start(mode=CoverageMode.BOUSTROPHEDON)
#   # ... mower runs autonomously ...
#   planner.stop()
#   status = planner.get_status()
#
# Boustrophedon overview:
#   1. Drive forward on current compass heading (strip heading)
#   2. If fence detected (LiDAR front < FENCE_DIST) OR strip length reached:
#        a. Stop
#        b. Turn 90° (compass-guided)
#        c. Drive one strip width
#        d. Turn 90° again (now facing opposite direction)
#        e. Repeat
#   3. If mid-strip obstacle: navigator handles avoidance, planner resumes
#
# Spiral overview:
#   1. Drive forward on current tangent heading
#   2. Continuously turn at a rate that increases the radius by
#      STRIP_WIDTH per full revolution
#   3. When fence detected on any forward quarter: transition to
#      fence-skirting (drive parallel to fence) then stop
#
# Tuning constants are at the top — adjust to your mower's deck width
# and field size before first run.
# ============================================================

import threading
import time
import math
import logging

logger = logging.getLogger(__name__)

# ── Coverage geometry ────────────────────────────────────────
STRIP_WIDTH_M     = 0.40   # metres between strip centres (< blade width for overlap)
MAX_STRIP_LEN_M   = 50.0   # give up on a strip after this distance (safety)
FENCE_DIST_M      = 0.70   # LiDAR front distance that means "fence/boundary"
OBSTACLE_DIST_M   = 0.55   # same as navigator STOP_DIST — let navigator handle it

# ── Speed settings ───────────────────────────────────────────
CRUISE_SPEED      = 80     # forward speed during mowing
TURN_SPEED        = 55     # speed during 90° turns
CREEP_SPEED       = 35     # speed during strip-width offset move

# ── Timing and tolerances ────────────────────────────────────
HEADING_TOLERANCE = 6.0    # degrees — close enough to target heading
TURN_POLL         = 0.05   # seconds between heading checks during turns
TURN_TIMEOUT      = 8.0    # max seconds for a single compass-guided turn
SETTLE_TIME       = 0.3    # pause after each turn to let IMU settle
STRIP_POLL        = 0.05   # main loop interval (20 Hz)

# ── Spiral parameters ────────────────────────────────────────
# At each step the robot drives SPIRAL_STEP_M then turns SPIRAL_TURN_DEG.
# Smaller step + smaller turn = smoother spiral but more commands.
SPIRAL_STEP_M     = 0.20   # metres per spiral segment
SPIRAL_TURN_DEG   = 15.0   # degrees to turn after each segment
# Radius increases by STRIP_WIDTH per full 360° revolution
# turn_radius_increase = STRIP_WIDTH * SPIRAL_TURN_DEG / 360


class CoverageMode:
    BOUSTROPHEDON = "BOUSTROPHEDON"
    SPIRAL        = "SPIRAL"


class CoverageState:
    IDLE      = "IDLE"
    MOWING    = "MOWING"
    TURNING   = "TURNING"
    OFFSETTING = "OFFSETTING"
    AVOIDING  = "AVOIDING"
    DONE      = "DONE"
    STOPPED   = "STOPPED"


class CoveragePlanner:
    """
    High-level autonomous coverage planner.

    Drives the mower in a systematic pattern, delegating obstacle
    avoidance to the Navigator layer and using the compass to hold
    straight headings and execute precise turns.
    """

    def __init__(self, navigator, odometry, compass, lidar):
        self._nav     = navigator
        self._odom    = odometry
        self._compass = compass
        self._lidar   = lidar

        self._lock       = threading.Lock()
        self._stop_event = threading.Event()
        self._thread     = None

        self._mode         = CoverageMode.BOUSTROPHEDON
        self._state        = CoverageState.IDLE
        self._strip_count  = 0
        self._total_dist_m = 0.0
        self._start_time   = 0.0

        # Boustrophedon state
        self._strip_heading  = 0.0   # compass degrees — direction of current strip
        self._strip_start_x  = 0.0
        self._strip_start_y  = 0.0
        self._strip_dir      = 1     # +1 or -1 (alternates each strip)

        # Spiral state
        self._spiral_heading    = 0.0
        self._spiral_radius_m   = 0.0
        self._spiral_step_count = 0

    # ── Public API ───────────────────────────────────────────

    def start(self, mode: str = CoverageMode.BOUSTROPHEDON, initial_heading: float = None):
        """
        Start autonomous coverage.

        Args:
            mode:            CoverageMode.BOUSTROPHEDON or CoverageMode.SPIRAL
            initial_heading: Starting compass heading in degrees.
                             None = use current compass reading.
        """
        if self._thread and self._thread.is_alive():
            logger.warning("[COV] Already running — stop first")
            return

        self._mode  = mode
        self._odom.reset()

        if initial_heading is not None:
            hdg = initial_heading
        elif self._compass and self._compass.is_calibrated():
            hdg = self._compass.get_heading()
        else:
            hdg = 0.0
            logger.warning("[COV] No compass — using 0° as strip heading")

        with self._lock:
            self._strip_heading  = hdg
            self._spiral_heading = hdg
            self._strip_count    = 0
            self._total_dist_m   = 0.0
            self._start_time     = time.time()
            self._state          = CoverageState.MOWING
            self._strip_dir      = 1

        self._stop_event.clear()
        target = (self._run_boustrophedon if mode == CoverageMode.BOUSTROPHEDON
                  else self._run_spiral)
        self._thread = threading.Thread(target=target, daemon=True, name="coverage")
        self._thread.start()

        logger.info(f"[COV] Starting {mode} coverage, initial heading={hdg:.1f}°")

    def stop(self):
        """Stop coverage immediately."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3)
        self._nav.request_drive(0, 0)
        with self._lock:
            self._state = CoverageState.STOPPED
        logger.info("[COV] Coverage stopped")

    def get_status(self) -> dict:
        """Return current coverage status for web UI / logging."""
        with self._lock:
            elapsed = time.time() - self._start_time if self._start_time else 0
            return {
                "mode":          self._mode,
                "state":         self._state,
                "strip_count":   self._strip_count,
                "total_dist_m":  round(self._total_dist_m, 2),
                "strip_heading": round(self._strip_heading, 1),
                "nav_state":     self._nav.get_nav_state(),
                "elapsed_s":     round(elapsed, 1),
            }

    # ── Boustrophedon pattern ────────────────────────────────

    def _run_boustrophedon(self):
        """Main boustrophedon loop — runs in its own thread."""
        logger.info("[COV] Boustrophedon loop started")

        while not self._stop_event.is_set():
            # ── Drive one strip ───────────────────────────
            fence_hit = self._drive_strip()

            if self._stop_event.is_set():
                break

            if not fence_hit:
                # Navigator stuck or max distance — stop coverage
                logger.warning("[COV] Strip ended without fence — stopping")
                break

            # ── End of strip — do 180° turn manoeuvre ────
            logger.info(f"[COV] End of strip {self._strip_count} — turning")
            with self._lock:
                self._state = CoverageState.TURNING

            # Turn 90° toward offset side
            offset_heading = _add_heading(self._strip_heading, 90.0 * self._strip_dir)
            ok = self._compass_turn_to(offset_heading)
            if not ok or self._stop_event.is_set():
                break

            # Drive one strip width across
            with self._lock:
                self._state = CoverageState.OFFSETTING
            ok = self._drive_distance(STRIP_WIDTH_M)
            if not ok or self._stop_event.is_set():
                break

            # Turn 90° back to face the opposite strip direction
            new_strip_heading = _add_heading(self._strip_heading, 180.0)
            ok = self._compass_turn_to(new_strip_heading)
            if not ok or self._stop_event.is_set():
                break

            with self._lock:
                self._strip_heading  = new_strip_heading
                self._strip_dir     *= -1   # flip offset side for next turn
                self._strip_count   += 1
                self._state          = CoverageState.MOWING

            logger.info(f"[COV] Strip {self._strip_count} — heading={new_strip_heading:.1f}°")

        with self._lock:
            self._state = CoverageState.DONE
        self._nav.request_drive(0, 0)
        logger.info(f"[COV] Boustrophedon complete — {self._strip_count} strips")

    def _drive_strip(self) -> bool:
        """
        Drive forward on the current strip heading until fence or max distance.
        Returns True if stopped by fence, False if stopped by stuck/error.
        """
        pose_start = self._odom.get_pose()
        start_x, start_y = pose_start["x"], pose_start["y"]

        with self._lock:
            self._strip_start_x = start_x
            self._strip_start_y = start_y

        while not self._stop_event.is_set():
            # Check nav state
            nav_state = self._nav.get_nav_state()
            if nav_state == "STUCK":
                logger.error("[COV] Navigator stuck mid-strip")
                return False

            # Check fence
            front = self._lidar.get_sector_min(0, 30)
            if front <= FENCE_DIST_M:
                logger.info(f"[COV] Fence detected at {front:.2f}m")
                self._nav.request_drive(0, 0)
                time.sleep(SETTLE_TIME)
                return True

            # Check strip distance
            pose = self._odom.get_pose()
            dist = math.hypot(pose["x"] - start_x, pose["y"] - start_y)
            with self._lock:
                self._total_dist_m += dist - (self._total_dist_m % 0.001)

            if dist >= MAX_STRIP_LEN_M:
                logger.warning(f"[COV] Max strip length {MAX_STRIP_LEN_M}m reached")
                self._nav.request_drive(0, 0)
                return True

            # Apply compass heading correction while driving
            fwd_speed = self._heading_corrected_drive()
            self._nav.request_drive(fwd_speed[0], fwd_speed[1])

            time.sleep(STRIP_POLL)

        return False

    def _heading_corrected_drive(self) -> tuple:
        """
        Return (left, right) speeds that drive forward while correcting
        for heading drift using the compass.
        A proportional controller nudges the differential.
        """
        if not self._compass or not self._compass.is_calibrated():
            return (CRUISE_SPEED, CRUISE_SPEED)

        current = self._compass.get_heading()
        with self._lock:
            target = self._strip_heading

        error = _angle_diff(target, current)   # signed -180..+180
        # P-gain: 1° error → 2 PWM units of differential
        correction = int(error * 2.0)
        correction = max(-30, min(30, correction))

        left  = CRUISE_SPEED - correction
        right = CRUISE_SPEED + correction
        left  = max(-127, min(127, left))
        right = max(-127, min(127, right))
        return (left, right)

    def _drive_distance(self, target_m: float) -> bool:
        """
        Drive straight for target_m metres using odometry.
        Returns True on success, False if stopped.
        """
        pose_start = self._odom.get_pose()
        sx, sy = pose_start["x"], pose_start["y"]

        while not self._stop_event.is_set():
            pose = self._odom.get_pose()
            dist = math.hypot(pose["x"] - sx, pose["y"] - sy)
            if dist >= target_m:
                self._nav.request_drive(0, 0)
                time.sleep(SETTLE_TIME)
                return True
            self._nav.request_drive(CREEP_SPEED, CREEP_SPEED)
            time.sleep(STRIP_POLL)

        return False

    def _compass_turn_to(self, target_deg: float) -> bool:
        """
        Spin in place until compass heading matches target_deg ± HEADING_TOLERANCE.
        Returns True on success, False on timeout or stop.
        """
        if not self._compass:
            # No compass — use timed turn as fallback
            return self._timed_turn(target_deg)

        deadline = time.time() + TURN_TIMEOUT
        logger.info(f"[COV] Turning to {target_deg:.1f}°")

        while not self._stop_event.is_set() and time.time() < deadline:
            if not self._compass.is_calibrated():
                time.sleep(0.1)
                continue

            current = self._compass.get_heading()
            error   = _angle_diff(target_deg, current)

            if abs(error) <= HEADING_TOLERANCE:
                self._nav.request_drive(0, 0)
                time.sleep(SETTLE_TIME)
                logger.info(f"[COV] Turn complete at {current:.1f}°")
                return True

            # Proportional turn speed — slow down as we approach target
            spd = int(min(TURN_SPEED, max(25, abs(error) * 1.5)))
            if error > 0:   # need to turn right (CW = heading increases on BNO055)
                self._nav.request_drive(spd, -spd)
            else:
                self._nav.request_drive(-spd, spd)

            time.sleep(TURN_POLL)

        logger.warning(f"[COV] Turn timeout — target={target_deg:.1f}°")
        self._nav.request_drive(0, 0)
        return False

    def _timed_turn(self, target_deg: float) -> bool:
        """Fallback: timed 90° turn when compass unavailable."""
        # Rough estimate — tune TURN_SPEED / TURN_TIME to match your mower
        self._nav.request_drive(TURN_SPEED, -TURN_SPEED)
        time.sleep(TURN_TIMEOUT / 2)
        self._nav.request_drive(0, 0)
        time.sleep(SETTLE_TIME)
        return True

    # ── Spiral pattern ───────────────────────────────────────

    def _run_spiral(self):
        """
        Inside-out Archimedean spiral.

        Strategy:
          - Drive SPIRAL_STEP_M forward
          - Turn SPIRAL_TURN_DEG (compass-guided when available)
          - Repeat — radius grows automatically because strip offset
            accumulates over each full revolution
          - Fence detected → done
        """
        logger.info("[COV] Spiral loop started")

        with self._lock:
            self._spiral_heading    = self._strip_heading
            self._spiral_radius_m   = 0.0
            self._spiral_step_count = 0

        while not self._stop_event.is_set():

            # Check fence on all forward quarters
            front = self._lidar.get_sector_min(0, 45)
            if front <= FENCE_DIST_M:
                logger.info(f"[COV] Spiral fence hit at {front:.2f}m — done")
                break

            nav_state = self._nav.get_nav_state()
            if nav_state == "STUCK":
                logger.error("[COV] Navigator stuck in spiral")
                break

            # Drive one segment
            ok = self._drive_distance(SPIRAL_STEP_M)
            if not ok or self._stop_event.is_set():
                break

            # Turn SPIRAL_TURN_DEG in place
            with self._lock:
                self._spiral_step_count += 1
                new_heading = _add_heading(
                    self._spiral_heading, SPIRAL_TURN_DEG)
                self._spiral_heading = new_heading

            ok = self._compass_turn_to(new_heading)
            if not ok or self._stop_event.is_set():
                break

            # Track approximate radius
            steps_per_rev = 360.0 / SPIRAL_TURN_DEG
            with self._lock:
                self._spiral_radius_m = (
                    self._spiral_step_count / steps_per_rev) * STRIP_WIDTH_M

        with self._lock:
            self._state = CoverageState.DONE
        self._nav.request_drive(0, 0)
        logger.info("[COV] Spiral complete")


# ── Heading helpers ──────────────────────────────────────────

def _add_heading(base: float, delta: float) -> float:
    """Add delta degrees to base heading, keep in 0–360."""
    return (base + delta) % 360.0


def _angle_diff(target: float, current: float) -> float:
    """Signed difference target - current, normalised to -180..+180."""
    return (target - current + 180) % 360 - 180
