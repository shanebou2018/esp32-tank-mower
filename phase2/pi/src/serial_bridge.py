# ============================================================
# serial_bridge.py
# Handles all serial communication between Pi and ESP32
#
# ESP32 wiring:
#   ESP32 TX2 (GPIO17) -> Pi GPIO15 (RX)
#   ESP32 RX2 (GPIO16) -> Pi GPIO14 (TX)
#   Common GND
#
# Usage:
#   from serial_bridge import ESP32Bridge
#   bridge = ESP32Bridge()
#   bridge.start()
#   bridge.drive(100, 100)   # left, right  -255..+255
#   state = bridge.get_state()
#   bridge.stop()
#   bridge.close()
#
# Changes from original:
#   v1.1 — Fixed deadlock: _send() no longer acquires _lock.
#          Callers that need thread safety must hold _lock themselves,
#          or use the public API (drive/stop/set_relay) which snapshot
#          values under _lock then call _send() outside it.
#   v1.1 — Added enc_l, enc_r, dist_l, dist_r, spd_l, spd_r to
#          initial _state so odometry.py always finds these keys.
#   v1.1 — _drive_loop snapshots drive values under _lock then sends
#          outside the lock (fixes the original deadlock path).
# ============================================================

import serial
import threading
import json
import time
import logging

logger = logging.getLogger(__name__)

# Serial port for Pi 5 hardware UART
# If /dev/ttyAMA0 doesn't work try /dev/serial0
SERIAL_PORT    = "/dev/ttyAMA0"
BAUD_RATE      = 115200
READ_TIMEOUT   = 1.0   # seconds
DRIVE_INTERVAL = 0.05  # resend drive command every 50ms so ESP32 doesn't timeout


class ESP32Bridge:
    """
    Thread-safe serial bridge to the ESP32.

    A background reader thread continuously processes incoming telemetry.
    A background drive thread resends the active drive command at 20Hz.
    Public methods are safe to call from any thread.
    """

    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        self.port = port
        self.baud = baud
        self._ser  = None

        # _lock guards _state and _drive_* fields.
        # RULE: never call _send() while holding _lock.
        self._lock       = threading.Lock()
        self._stop_event = threading.Event()

        # Latest telemetry received from ESP32.
        # All fields the firmware sends are pre-populated so consumers
        # (odometry.py, web_ui.py) always find the keys they expect.
        self._state = {
            # Encoder / odometry (populated by Phase 2 firmware)
            "enc_l":       0,
            "enc_r":       0,
            "dist_l":      0.0,
            "dist_r":      0.0,
            "spd_l":       0.0,
            "spd_r":       0.0,
            # Relay states
            "relay_arm":   0,
            "relay_motor": 0,
            "relay_turbo": 0,
            # Controller / system
            "connected":   0,   # PS4 controller connected on ESP32
            "mode":        "DUAL",
            "batt":        0,
            # Internal bookkeeping
            "last_rx":     0.0, # timestamp of last good telemetry packet
        }

        # Drive command — resent at DRIVE_INTERVAL by _drive_loop
        self._drive_left     = 0
        self._drive_right    = 0
        self._drive_active   = False
        self._last_drive_sent = 0.0

        self._reader_thread = None
        self._drive_thread  = None

    # ── Public API ───────────────────────────────────────────

    def start(self):
        """Open serial port and start background threads."""
        try:
            self._ser = serial.Serial(
                self.port,
                self.baud,
                timeout=READ_TIMEOUT
            )
            time.sleep(0.5)   # let ESP32 settle after port open
            logger.info(f"[BRIDGE] Serial open: {self.port} @ {self.baud}")
        except serial.SerialException as e:
            logger.error(f"[BRIDGE] Failed to open {self.port}: {e}")
            raise

        self._stop_event.clear()

        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="bridge-reader")
        self._drive_thread = threading.Thread(
            target=self._drive_loop, daemon=True, name="bridge-drive")

        self._reader_thread.start()
        self._drive_thread.start()
        logger.info("[BRIDGE] Threads started")

        if self.ping():
            logger.info("[BRIDGE] ESP32 responded to ping — comms OK")
        else:
            logger.warning("[BRIDGE] No ping response from ESP32 — check wiring/baud")

    def close(self):
        """Stop background threads and close serial port."""
        self._stop_event.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=2)
        if self._drive_thread:
            self._drive_thread.join(timeout=2)
        if self._ser and self._ser.is_open:
            self.stop()
            self._ser.close()
        logger.info("[BRIDGE] Serial closed")

    def drive(self, left: int, right: int):
        """
        Set drive speeds. Values -255..+255.
        Command is resent every DRIVE_INTERVAL seconds automatically
        so the ESP32 watchdog never times out.
        """
        left  = max(-255, min(255, int(left)))
        right = max(-255, min(255, int(right)))
        with self._lock:
            self._drive_left   = left
            self._drive_right  = right
            self._drive_active = True
        logger.debug(f"[BRIDGE] Drive set L={left} R={right}")

    def stop(self):
        """Send stop command immediately and clear the drive keepalive."""
        with self._lock:
            self._drive_left   = 0
            self._drive_right  = 0
            self._drive_active = False
        # Send outside lock — _send() must never be called under _lock
        self._send({"cmd": "stop"})
        logger.info("[BRIDGE] Stop sent")

    def set_relay(self, relay_id: str, state: bool):
        """
        Control a relay by name.
          relay_id: "motor" or "turbo"
          state:    True = ON, False = OFF

        Note: "motor" triggers the ESP32 arm sequence for safety.
        """
        self._send({"cmd": "relay", "id": relay_id, "state": 1 if state else 0})
        logger.info(f"[BRIDGE] Relay {relay_id} -> {'ON' if state else 'OFF'}")

    def ping(self, timeout: float = 2.0) -> bool:
        """
        Send ping and wait for any telemetry response.
        Returns True if the ESP32 is sending data within `timeout` seconds.
        """
        self._send({"cmd": "ping"})
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                last_rx = self._state["last_rx"]
            if last_rx > time.time() - timeout:
                return True
            time.sleep(0.05)
        return False

    def get_state(self) -> dict:
        """Return a snapshot copy of the latest telemetry state."""
        with self._lock:
            return dict(self._state)

    def is_connected(self) -> bool:
        """True if ESP32 telemetry arrived within the last second."""
        with self._lock:
            return (time.time() - self._state["last_rx"]) < 1.0

    def motor_running(self) -> bool:
        with self._lock:
            return bool(self._state["relay_motor"])

    def turbo_running(self) -> bool:
        with self._lock:
            return bool(self._state["relay_turbo"])

    # ── Private helpers ──────────────────────────────────────

    def _send(self, obj: dict):
        """
        Serialise obj to JSON and write to serial.

        IMPORTANT: Do NOT call this method while holding self._lock.
        The method uses the serial port directly (no lock) because:
          - Serial writes are atomic at the OS level for short packets
          - Holding _lock during a blocking write would deadlock the
            reader thread which also needs _lock to update _state
        """
        if not self._ser or not self._ser.is_open:
            return
        line = json.dumps(obj, separators=(",", ":")) + "\n"
        try:
            self._ser.write(line.encode("utf-8"))
            logger.debug(f"[BRIDGE] Tx: {line.strip()}")
        except serial.SerialException as e:
            logger.error(f"[BRIDGE] Write error: {e}")

    def _reader_loop(self):
        """Background thread — reads lines from ESP32 and updates _state."""
        logger.info("[BRIDGE] Reader thread running")
        buf = ""
        while not self._stop_event.is_set():
            try:
                if self._ser.in_waiting:
                    char = self._ser.read(1).decode("utf-8", errors="ignore")
                    if char == "\n":
                        self._process_line(buf.strip())
                        buf = ""
                    elif len(buf) < 512:
                        buf += char
                else:
                    time.sleep(0.001)
            except serial.SerialException as e:
                logger.error(f"[BRIDGE] Read error: {e}")
                time.sleep(0.5)
            except Exception as e:
                logger.error(f"[BRIDGE] Unexpected reader error: {e}")
                time.sleep(0.1)

    def _process_line(self, line: str):
        """Parse a line received from the ESP32."""
        if not line:
            return
        if line.startswith("{"):
            try:
                data = json.loads(line)
                with self._lock:
                    self._state.update(data)
                    self._state["last_rx"] = time.time()
                logger.debug(f"[BRIDGE] Rx: {data}")
            except json.JSONDecodeError:
                logger.warning(f"[BRIDGE] Bad JSON: {line!r}")
        else:
            # Plain debug output from ESP32 Serial.println()
            logger.debug(f"[ESP32] {line}")

    def _drive_loop(self):
        """
        Background thread — resends the active drive command every DRIVE_INTERVAL.

        Snapshots drive values under _lock, then sends OUTSIDE the lock.
        This is the pattern that avoids the original deadlock where _send()
        was called inside a with self._lock block.
        """
        logger.info("[BRIDGE] Drive loop running")
        while not self._stop_event.is_set():
            now = time.time()

            # Snapshot under lock
            with self._lock:
                active = self._drive_active
                left   = self._drive_left
                right  = self._drive_right
                due    = (now - self._last_drive_sent) >= DRIVE_INTERVAL
                if active and due:
                    self._last_drive_sent = now

            # Send outside lock
            if active and due:
                self._send({"cmd": "drive", "l": left, "r": right})

            time.sleep(0.01)


# ── Quick test — run directly to verify comms ────────────────
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s"
    )

    print("=" * 55)
    print("  ESP32 Serial Bridge — Comms Test")
    print("=" * 55)

    bridge = ESP32Bridge()
    try:
        bridge.start()

        print("\n[TEST] Bridge started. Listening for 5 seconds...\n")
        print(f"{'#':>4}  {'enc_l':>8}  {'enc_r':>8}  "
              f"{'dist_l':>8}  {'dist_r':>8}  {'motor':>6}  {'batt':>5}")
        print("-" * 60)

        for i in range(10):
            time.sleep(0.5)
            s = bridge.get_state()
            print(
                f"{i+1:>4}  "
                f"{s['enc_l']:>8}  {s['enc_r']:>8}  "
                f"{s['dist_l']:>8.3f}  {s['dist_r']:>8.3f}  "
                f"{'ON' if s['relay_motor'] else 'off':>6}  "
                f"{s['batt']:>4}%"
            )

        print("\n[TEST] Driving forward (L=80 R=80) for 2 seconds...")
        bridge.drive(80, 80)
        time.sleep(2)

        print("[TEST] Stopping...")
        bridge.stop()
        time.sleep(0.5)

        print("\n[TEST] Done.")

    except KeyboardInterrupt:
        print("\n[TEST] Interrupted")
    finally:
        bridge.close()
        print("[TEST] Bridge closed")
