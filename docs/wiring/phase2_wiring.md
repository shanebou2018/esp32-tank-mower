# Phase 2 Wiring

## ESP32 → Raspberry Pi (Serial Bridge)

Serial2 on the ESP32 carries the JSON telemetry and command protocol at 115200 baud.

| ESP32 Pin | Direction | Pi Pin | Pi GPIO | Notes |
|-----------|-----------|--------|---------|-------|
| GPIO 17 (TX2) | → | GPIO 15 | RX | ESP32 sends telemetry to Pi |
| GPIO 16 (RX2) | ← | GPIO 14 | TX | Pi sends drive/relay commands |
| GND | — | GND | — | Common ground required |

> Pi GPIO is 3.3V — compatible with ESP32 logic. No level shifter needed.

---

## ESP32 → AS5600 Wheel Encoders (x2)

Both encoders are wired in PWM output mode. The EncAS5600 library reads pulse width via hardware timers.

| Signal | ESP32 Pin | Notes |
|--------|-----------|-------|
| Left encoder PWM | GPIO 34 | Input only pin — no pull-up needed |
| Right encoder PWM | GPIO 35 | Input only pin — no pull-up needed |
| VCC | 3.3V | AS5600 operates on 3.3V |
| GND | GND | Common ground |

> GPIO 34 and 35 are input-only on the ESP32 — do not drive them as outputs.
> The AS5600 OUT pin connects directly; no voltage divider needed at 3.3V.

**AS5600 I2C address config (for magnet placement):**
Place the diametrically magnetised disc magnet centered above the AS5600 IC.
The DIR pin on the AS5600 sets rotation direction — tie HIGH or LOW to match your motor orientation.

---

## ESP32 → Cytron MDDS30 (dual-channel, replaces 2× MD13S)

| ESP32 Pin | MDDS30 | Track |
|-----------|--------|-------|
| 25 | PWM1 | Left |
| 26 | DIR1 | Left |
| 27 | PWM2 | Right |
| 14 | DIR2 | Right |
| GND | GND | Both |

> Do NOT power the ESP32s from the MDDS30 5V pin — rail sags under BT TX peaks
> causing BT throttling/resets. Use a dedicated 5V supply for the ESP32s.

---

## ESP32 → Relays (unchanged from Phase 1)

| ESP32 Pin | Relay | Function |
|-----------|-------|----------|
| 32 | ARM | Safety interlock — pulses during start, stays HIGH with motor |
| 33 | MOTOR | Mower blade motor — latching |
| 15 | TURBO | Turbo — latching |
| GND | GND | All relays |

---

## Raspberry Pi → Sensors (Planned)

| Sensor | Interface | Pi Pins | Notes |
|--------|-----------|---------|-------|
| LiDAR (RPLiDAR A1/A2) | USB | Any USB port | USB-serial adapter included |
| GPS (u-blox NEO-M8N) | UART | GPIO 14 TX / 15 RX | Or use USB variant |
| Compass / IMU (BNO055) | I2C | GPIO 2 SDA / 3 SCL | 3.3V |
| Camera | CSI / USB | CSI ribbon or USB | Pi Camera or USB webcam |

---

## Power

- ESP32 (both): dedicated 5V supply (NOT from MDDS30) — MDDS30 rail sags under BT TX peaks
- MDDS30: main drive battery (7V–35V)
- Relay coils: 5V — use relay modules with onboard optocouplers
- AS5600 encoders: 3.3V from ESP32
- Raspberry Pi: 5V / 3A dedicated supply (USB-C on Pi 4/5)
- Common GND across all subsystems

---

## Notes

- Keep serial wires (GPIO 16/17) away from motor power cables to reduce noise
- Add a ferrite bead on the Pi power cable if encoder readings are noisy
- The ARM relay must be rated for the mower blade motor's startup current
