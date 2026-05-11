# ============================================================
#  serial_bridge.py
#  Handles all serial communication between Pi and ESP32
#
#  ESP32 wiring:
#    ESP32 TX2 (GPIO17) -> Pi GPIO15 (RX)
#    ESP32 RX2 (GPIO16) -> Pi GPIO14 (TX)
#    Common GND
#
#  Usage:
#    from serial_bridge import ESP32Bridge
#    bridge = ESP32Bridge()
#    bridge.start()
#    bridge.drive(100, 100)       # left, right  -255..+255
#    bridge.stop()
#    state = bridge.get_state()
#    bridge.close()
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
READ_TIMEOUT   = 1.0    # seconds
DRIVE_INTERVAL = 0.05   # resend drive command every 50ms so ESP32 doesn't timeout


class ESP32Bridge:
    """
    Thread-safe serial bridge to the ESP32.
    A background reader thread continuously processes incoming telemetry.
    Public methods are safe to call from any thread.
    """

    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        self.port  = port
        self.baud  = baud
        self._ser  = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # Latest telemetry received from ESP32
        self._state = {
            "enc_l":       0,
            "enc_r":       0,
            "relay_arm":   0,
            "relay_motor": 0,
            "relay_turbo": 0,
            "connected":   0,   # PS4 controller connected
            "mode":        "DUAL",
            "batt":        0,
            "last_rx":     0.0  # timestamp of last good telemetry packet
        }

        # Current drive command (resent on interval so ESP32 doesn't time out)
        self._drive_left  = 0
        self._drive_right = 0
        self._drive_active = False
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
            time.sleep(0.5)  # let ESP32 settle
            logger.info(f"[BRIDGE] Serial open: {self.port} @ {self.baud}")
        except serial.SerialException as e:
            logger.error(f"[BRIDGE] Failed to open {self.port}: {e}")
            raise

        self._stop_event.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._drive_thread  = threading.Thread(target=self._drive_loop,  daemon=True)
        self._reader_thread.start()
        self._drive_thread.start()
        logger.info("[BRIDGE] Threads started")

        # Ping to confirm comms
        if self.ping():
            logger.info("[BRIDGE] ESP32 responded to ping -- comms OK")
        else:
            logger.warning("[BRIDGE] No ping response from ESP32 -- check wiring")

    def close(self):
        """Stop threads and close serial port."""
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
        Command is resent every DRIVE_INTERVAL seconds automatically.
        """
        left  = max(-255, min(255, int(left)))
        right = max(-255, min(255, int(right)))
        with self._lock:
            self._drive_left   = left
            self._drive_right  = right
            self._drive_active = True
        logger.debug(f"[BRIDGE] Drive set L={left} R={right}")

    def stop(self):
        """Send stop command and clear drive state."""
        with self._lock:
            self._drive_left   = 0
            self._drive_right  = 0
            self._drive_active = False
        self._send({"cmd": "stop"})
        logger.info("[BRIDGE] Stop sent")

    def set_relay(self, relay_id: str, state: bool):
        """
        Control a relay by name.
        relay_id: "motor" or "turbo"
        state: True = ON, False = OFF
        Note: motor relay uses ESP32 arm sequence for safety.
        """
        self._send({"cmd": "relay", "id": relay_id, "state": 1 if state else 0})
        logger.info(f"[BRIDGE] Relay {relay_id} -> {'ON' if state else 'OFF'}")

    def ping(self, timeout=2.0) -> bool:
        """Send ping and wait for pong response. Returns True if ESP32 responds."""
        self._send({"cmd": "ping"})
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                # pong arrives as telemetry line -- just check we're getting data
                if self._state["last_rx"] > time.time() - timeout:
                    return True
            time.sleep(0.05)
        return False

    def get_state(self) -> dict:
        """Return a copy of the latest telemetry state."""
        with self._lock:
            return dict(self._state)

    def is_connected(self) -> bool:
        """True if ESP32 telemetry is arriving (last packet within 1 second)."""
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
        """Serialize obj to JSON and write to serial. Thread-safe."""
        if not self._ser or not self._ser.is_open:
            return
        line = json.dumps(obj) + "\n"
        try:
            with self._lock:
                self._ser.write(line.encode("utf-8"))
            logger.debug(f"[BRIDGE] Tx: {line.strip()}")
        except serial.SerialException as e:
            logger.error(f"[BRIDGE] Write error: {e}")

    def _reader_loop(self):
        """Background thread — reads lines from ESP32 and updates state."""
        logger.info("[BRIDGE] Reader thread running")
        buf = ""
        while not self._stop_event.is_set():
            try:
                if self._ser.in_waiting:
                    char = self._ser.read(1).decode("utf-8", errors="ignore")
                    if char == "\n":
                        self._process_line(buf.strip())
                        buf = ""
                    else:
                        if len(buf) < 512:
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
        """Parse a line from the ESP32."""
        if not line:
            return

        # JSON telemetry
        if line.startswith("{"):
            try:
                data = json.loads(line)
                with self._lock:
                    self._state.update(data)
                    self._state["last_rx"] = time.time()
                logger.debug(f"[BRIDGE] Rx telem: {data}")
            except json.JSONDecodeError:
                logger.warning(f"[BRIDGE] Bad JSON: {line}")
        else:
            # Plain debug line from ESP32 -- just log it
            logger.debug(f"[ESP32] {line}")

    def _drive_loop(self):
        """Background thread — resends active drive command at DRIVE_INTERVAL."""
        logger.info("[BRIDGE] Drive loop running")
        while not self._stop_event.is_set():
            now = time.time()
            with self._lock:
                active = self._drive_active
                left   = self._drive_left
                right  = self._drive_right
            if active and (now - self._last_drive_sent >= DRIVE_INTERVAL):
                self._send({"cmd": "drive", "l": left, "r": right})
                self._last_drive_sent = now
            time.sleep(0.01)


# ── Quick test — run directly to verify comms ────────────────
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s"
    )

    print("=" * 50)
    print("  ESP32 Serial Bridge — Comms Test")
    print("=" * 50)

    bridge = ESP32Bridge()

    try:
        bridge.start()
        print("\n[TEST] Bridge started. Listening for 5 seconds...\n")

        for i in range(10):
            time.sleep(0.5)
            state = bridge.get_state()
            print(f"[STATE] connected={state['connected']} "
                  f"motor={state['relay_motor']} "
                  f"turbo={state['relay_turbo']} "
                  f"mode={state['mode']} "
                  f"batt={state['batt']}% "
                  f"last_rx={state['last_rx']:.2f}")

        print("\n[TEST] Sending drive forward (L=80 R=80) for 2 seconds...")
        bridge.drive(80, 80)
        time.sleep(2)

        print("[TEST] Sending stop...")
        bridge.stop()
        time.sleep(1)

        print("\n[TEST] Done.")

    except KeyboardInterrupt:
        print("\n[TEST] Interrupted")
    finally:
        bridge.close()
        print("[TEST] Bridge closed")
