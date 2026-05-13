# ============================================================
# navigator.py
# Reactive LiDAR obstacle avoidance layer.
#
# Runs at 20Hz and continuously classifies the space around
# the mower into five states:
#
#   CLEAR    — nothing within WARN_DIST, full speed ahead
#   WARN     — something between WARN_DIST and STOP_DIST,
#              slow to SLOW_SPEED
#   BLOCKED  — obstacle within STOP_DIST in front, stop and
#              decide which way to go around
#   AVOIDING — actively steering around an obstacle
#   STUCK    — couldn't find a clear path after STUCK_TIMEOUT
#
# Usage:
#   from navigator import Navigator
#   nav = Navigator(lidar, bridge, compass)
#   nav.start()
#
#   # In your coverage loop — call this instead of bridge.drive():
#   nav.request_drive(left, right)
#
#   state = nav.get_state()   # "CLEAR" / "WARN" / "BLOCKED" / "AVOIDING" / "STUCK"
#   nav.stop()
#
# Sector layout (YDLIDAR X4 — 0deg = forward on robot):
#
#        FRONT (0 ± FRONT_HALF)
#          |
#   LEFT --+-- RIGHT
#          |
#        REAR  (180 ± REAR_HALF)
#
# Obstacle avoidance strategy:
#   1. Obstacle detected in front → stop
#   2. Check left sector vs right sector
#   3. Turn toward the clearer side
#   4. Drive forward until front is clear
#   5. Resume original heading using compass
#   6. Caller's coverage loop continues
#
# All compass angles are in degrees (0–360, BNO055 convention).
# ============================================================

import threading
import time
import math
import logging

logger = logging.getLogger(__name__)

# ── LiDAR sector definitions (degrees, 0 = forward) ─────────
FRONT_CENTER  =   0
FRONT_HALF    =  30    # ±30° cone ahead = 60° total front zone
FRONT_NEAR    =  15    # ±15° tight cone for very close detection

LEFT_CENTER   =  90    # left side
RIGHT_CENTER  = 270    # right side
SIDE_HALF     =  40    # ±40° each side

REAR_CENTER   = 180
REAR_HALF     =  30

# ── Distance thresholds (metres) ────────────────────────────
WARN_DIST     = 1.2    # slow down beyond here
STOP_DIST     = 0.55   # full stop + avoid beyond here
CLEAR_DIST    = 0.80   # obstacle must be this far before resuming

# ── Speed settings ───────────────────────────────────────────
CRUISE_SPEED  = 80     # normal forward speed (−255..+255)
SLOW_SPEED    = 45     # warn-zone speed
TURN_SPEED    = 60     # speed used during avoidance turns
BACK_SPEED    = 50     # reverse speed during backup manoeuvre

# ── Timing ───────────────────────────────────────────────────
POLL_INTERVAL   = 0.05   # 20 Hz
BACKUP_TIME     = 0.6    # seconds to reverse before turning
TURN_TIME       = 1.2    # seconds to turn 90° (tune to your mower)
AVOID_FWD_TIME  = 1.5    # seconds to drive forward during avoid arc
STUCK_TIMEOUT   = 12.0   # give up and halt after this many seconds avoiding
RESUME_HOLD     = 1.0    # seconds to hold avoidance clear before resuming


class NavState:
    CLEAR    = "CLEAR"
    WARN     = "WARN"
    BLOCKED  = "BLOCKED"
    AVOIDING = "AVOIDING"
    STUCK    = "STUCK"


class Navigator:
    """
    Reactive obstacle avoidance wrapper around the serial bridge.

    The coverage planner calls request_drive(l, r) instead of
    bridge.drive(l, r) directly. The navigator either passes
    the command through (CLEAR/WARN) or overrides it with avoidance
    manoeuvres (BLOCKED/AVOIDING).
    """

    def __init__(self, lidar, bridge, compass=None):
        self._lidar   = lidar
        self._bridge  = bridge
        self._compass = compass

        self._lock       = threading.Lock()
        self._stop_event = threading.Event()
        self._thread     = None

        # Requested drive from coverage planner
        self._req_left  = 0
        self._req_right = 0

        # Current nav state
        self._state          = NavState.CLEAR
        self._avoid_start    = 0.0
        self._avoid_turn_dir = 1     # +1 = turn right, -1 = turn left
        self._resume_heading = None  # compass heading to resume after avoid
        self._clear_since    = None  # timestamp when front last became clear

        # Stats
        self._avoidance_count = 0
        self._stuck_count     = 0

        # Control gates — both off on boot until user enables from web UI
        self._movement_enabled  = False
        self._avoidance_enabled = True   # takes effect when movement is enabled

    # ── Public API ───────────────────────────────────────────

    def enable_movement(self):
        with self._lock:
            self._movement_enabled = True
            # Re-enter CLEAR so any stale STUCK state doesn't persist
            self._state = NavState.CLEAR
        logger.info("[NAV] Movement ENABLED")

    def disable_movement(self):
        with self._lock:
            self._movement_enabled = False
            self._state = NavState.CLEAR
        # Don't zero the bridge here — caller decides (coverage keeps state)
        logger.info("[NAV] Movement DISABLED — motor commands blocked")

    def enable_avoidance(self):
        with self._lock:
            self._avoidance_enabled = True
        logger.info("[NAV] Obstacle avoidance ENABLED")

    def disable_avoidance(self):
        with self._lock:
            self._avoidance_enabled = False
            self._state = NavState.CLEAR
        logger.info("[NAV] Obstacle avoidance DISABLED — drive commands pass through")

    def is_movement_enabled(self) -> bool:
        with self._lock:
            return self._movement_enabled

    def is_avoidance_enabled(self) -> bool:
        with self._lock:
            return self._avoidance_enabled

    def start(self):
        """Start the reactive control loop."""
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._control_loop, daemon=True, name="navigator")
        self._thread.start()
        logger.info("[NAV] Reactive obstacle avoidance started")
        logger.info(f"[NAV] STOP={STOP_DIST}m  WARN={WARN_DIST}m  "
                    f"CLEAR={CLEAR_DIST}m  CRUISE={CRUISE_SPEED}")

    def stop(self):
        """Stop the control loop and zero the bridge."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._bridge.stop()
        logger.info("[NAV] Stopped")

    def request_drive(self, left: int, right: int):
        """
        Coverage planner calls this instead of bridge.drive().
        Navigator may override with avoidance commands.
        """
        with self._lock:
            self._req_left  = int(left)
            self._req_right = int(right)

    def get_nav_state(self) -> str:
        """Return current nav state string."""
        with self._lock:
            return self._state

    def get_status(self) -> dict:
        """Return full status dict for web UI / logging."""
        front = self._lidar.get_sector_min(FRONT_CENTER, FRONT_HALF)
        left  = self._lidar.get_sector_min(LEFT_CENTER,  SIDE_HALF)
        right = self._lidar.get_sector_min(RIGHT_CENTER, SIDE_HALF)
        with self._lock:
            return {
                "state":              self._state,
                "movement_enabled":   self._movement_enabled,
                "avoidance_enabled":  self._avoidance_enabled,
                "front_m":            round(front, 2),
                "left_m":             round(left,  2),
                "right_m":            round(right, 2),
                "avoidance_count":    self._avoidance_count,
                "stuck_count":        self._stuck_count,
            }

    def is_clear(self) -> bool:
        with self._lock:
            return self._state == NavState.CLEAR

    # ── Private control loop ─────────────────────────────────

    def _control_loop(self):
        logger.info("[NAV] Control loop running")
        while not self._stop_event.is_set():
            try:
                self._tick()
            except Exception as e:
                logger.error(f"[NAV] Tick error: {e}")
            time.sleep(POLL_INTERVAL)

    def _tick(self):
        """Called every POLL_INTERVAL. State machine tick."""
        with self._lock:
            if not self._movement_enabled:
                return   # gates closed — no motor commands issued

        front = self._lidar.get_sector_min(FRONT_CENTER, FRONT_HALF)
        left  = self._lidar.get_sector_min(LEFT_CENTER,  SIDE_HALF)
        right = self._lidar.get_sector_min(RIGHT_CENTER, SIDE_HALF)

        with self._lock:
            state = self._state

        if state == NavState.CLEAR or state == NavState.WARN:
            self._tick_normal(front, left, right)

        elif state == NavState.BLOCKED:
            self._tick_blocked(front, left, right)

        elif state == NavState.AVOIDING:
            self._tick_avoiding(front, left, right)

        elif state == NavState.STUCK:
            # Hard stop — coverage planner must intervene
            self._bridge.stop()

    def _tick_normal(self, front, left, right):
        """CLEAR or WARN — pass through or scale coverage request."""
        with self._lock:
            req_l            = self._req_left
            req_r            = self._req_right
            avoidance_on     = self._avoidance_enabled

        if not avoidance_on:
            # Avoidance disabled — pass drive request straight through
            self._bridge.drive(req_l, req_r)
            with self._lock:
                self._state = NavState.CLEAR
            return

        if front <= STOP_DIST:
            # Transition to BLOCKED
            logger.warning(f"[NAV] BLOCKED — front={front:.2f}m")
            self._bridge.stop()
            with self._lock:
                self._state       = NavState.BLOCKED
                self._avoid_start = time.time()
                # Choose turn direction toward the clearer side
                self._avoid_turn_dir = 1 if right >= left else -1
                # Remember heading to restore after avoid
                if self._compass and self._compass.is_calibrated():
                    self._resume_heading = self._compass.get_heading()
                else:
                    self._resume_heading = None
            return

        if front <= WARN_DIST:
            # Scale speed proportionally between WARN and STOP
            ratio = (front - STOP_DIST) / (WARN_DIST - STOP_DIST)
            ratio = max(0.0, min(1.0, ratio))
            scale = ratio * (1.0 - SLOW_SPEED / CRUISE_SPEED) + SLOW_SPEED / CRUISE_SPEED
            scaled_l = int(req_l * scale)
            scaled_r = int(req_r * scale)
            self._bridge.drive(scaled_l, scaled_r)
            with self._lock:
                self._state = NavState.WARN
        else:
            # Full clear
            self._bridge.drive(req_l, req_r)
            with self._lock:
                self._state = NavState.CLEAR

    def _tick_blocked(self, front, left, right):
        """BLOCKED — back up briefly, then begin turn."""
        with self._lock:
            elapsed = time.time() - self._avoid_start
            turn_dir = self._avoid_turn_dir

        if elapsed < BACKUP_TIME:
            # Phase 1: reverse
            self._bridge.drive(-BACK_SPEED, -BACK_SPEED)
        else:
            # Phase 2: start turning — transition to AVOIDING
            logger.info(f"[NAV] AVOIDING — turning {'right' if turn_dir > 0 else 'left'}")
            with self._lock:
                self._state          = NavState.AVOIDING
                self._avoid_start    = time.time()
                self._avoidance_count += 1
                self._clear_since    = None

    def _tick_avoiding(self, front, left, right):
        """AVOIDING — turn then arc forward until front is clear."""
        with self._lock:
            elapsed  = time.time() - self._avoid_start
            turn_dir = self._avoid_turn_dir
            stuck_t  = STUCK_TIMEOUT

        # Check for stuck
        if elapsed > stuck_t:
            logger.error("[NAV] STUCK — could not clear obstacle")
            self._bridge.stop()
            with self._lock:
                self._state = NavState.STUCK
                self._stuck_count += 1
            return

        if elapsed < TURN_TIME:
            # Turn phase: spin in place toward clearer side
            if turn_dir > 0:   # turn right
                self._bridge.drive(TURN_SPEED, -TURN_SPEED)
            else:              # turn left
                self._bridge.drive(-TURN_SPEED, TURN_SPEED)
        else:
            # Forward arc phase: creep forward
            self._bridge.drive(SLOW_SPEED, SLOW_SPEED)

            if front >= CLEAR_DIST:
                # Front clearing — start hold timer
                with self._lock:
                    if self._clear_since is None:
                        self._clear_since = time.time()
                    clear_elapsed = time.time() - self._clear_since

                if clear_elapsed >= RESUME_HOLD:
                    # Confirmed clear — resume coverage
                    logger.info(f"[NAV] Clear — resuming after {elapsed:.1f}s avoid")
                    self._resume_coverage()
            else:
                # Obstacle still present — reset clear timer
                with self._lock:
                    self._clear_since = None

    def _resume_coverage(self):
        """Transition back to CLEAR and restore compass heading if available."""
        with self._lock:
            self._state          = NavState.CLEAR
            resume_hdg           = self._resume_heading
            self._resume_heading = None
            self._clear_since    = None

        if resume_hdg is not None and self._compass and self._compass.is_calibrated():
            # Spin back toward original heading
            self._correct_heading(resume_hdg)

        logger.info("[NAV] Avoidance complete — coverage resumed")

    def _correct_heading(self, target_deg: float, timeout: float = 3.0):
        """
        Spin in place until compass heading is within 10° of target.
        Blocks for up to `timeout` seconds.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self._compass.is_calibrated():
                break
            current = self._compass.get_heading()
            error   = _angle_diff(target_deg, current)  # signed -180..+180
            if abs(error) < 10.0:
                break
            # Turn toward target
            if error > 0:
                self._bridge.drive(TURN_SPEED, -TURN_SPEED)
            else:
                self._bridge.drive(-TURN_SPEED, TURN_SPEED)
            time.sleep(0.05)
        self._bridge.stop()
        time.sleep(0.1)


def _angle_diff(target: float, current: float) -> float:
    """Signed difference target - current, normalised to -180..+180."""
    d = (target - current + 180) % 360 - 180
    return d
