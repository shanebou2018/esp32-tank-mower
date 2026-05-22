# ESP32 Tank Mower — v3.2.0 Notes

Quick-reference for picking up this project mid-session.

---

## File map

| File | What lives here |
|---|---|
| `types.h` | Every `#define`, enum, struct, `extern` global, and function declaration. **Single include for all .cpp files.** |
| `32E_tank_controller_bp32_v3.2.0.ino` | Global variable *definitions*, `setup()`, `loop()` |
| `debug.cpp` | `debugSep / debugBanner / printBTAddr / debugRelays / debugFullStatus` |
| `drive.cpp` | LED, stick curve, motor bytes, arm state machine |
| `bt_callbacks.cpp` | `onConnectedGamepad / onDisconnectedGamepad` |
| `io_task.cpp` | `publishCtrl2IO`, `ioTask`, and all `ioXxx` helpers (static) |

**Why .cpp files instead of multiple .ino files?**  
The Arduino IDE auto-prototype scanner inserts generated function prototypes after the last `#include` line in the `.ino`. Multiple `.ino` files are concatenated before this happens — types defined in one `.ino` can still be invisible when prototypes are inserted. `.cpp` files bypass the scanner entirely and use normal C++ linkage, so there is no ordering constraint.

---

## Hardware

| Peripheral | Connection |
|---|---|
| ESP32 module | **WROOM-32UE** (DevKitC-32UE) — U.FL external antenna |
| Cytron MDDS30 dual motor driver | GPIO 25 → IN1 (Serial Simplified, TX-only) |
| Relay ARM | GPIO 32 (momentary pulse) |
| Relay MOTOR (blade) | GPIO 33 (latching) |
| Relay TURBO | GPIO 27 (latching) — moved from GPIO 15 (strapping pin violation) |
| AS5600 encoders ×2 | GPIO 34 (L), GPIO 35 (R) — PWM mode, input-only |
| PCF8575 I/O expander | I2C 0x20, SDA=21, SCL=22 (optional — guarded by `pcfPresent`) |
| Pi 5 bridge | UART1 TX=GPIO17, RX=GPIO13 @ 115200 |
| Pip-Boy ESP32 | GPIO4 RX — shared from UART1 TX via GPIO matrix fanout |
| PS4 DualShock4 | Bluepad32 Classic BT (BR/EDR) |

**Power:** ESP32 on its own 5V supply — NOT the MDDS30 5V pin. That rail sags under BT TX peaks and causes BT dropouts.

**Antenna:** External U.FL antenna (WROOM-32UE) — much better range than PCB-antenna WROOM-32E. Keep the antenna away from metal and route it as far from the MDDS30 / motor wiring as practical.

**MDDS30 DIP switches must be `11011111`:**
- SW1=ON SW2=ON → Serial mode
- SW3=OFF → Serial Simplified
- SW4=ON SW5=ON → Independent Both ← SW4 was OFF in earlier hardware — check this
- SW6=ON SW7=ON SW8=ON → 115200 baud

---

## RTOS architecture

| Core | What runs |
|---|---|
| Core 0 | `ioTask` — PCF8575 poll, Pi serial bridge, telemetry TX, Pip-Boy scene |
| Core 1 | Arduino `loop()` — BP32.update(), button handling, motor output |

Two mutexes:
- `g_mutex_c2i` — Core 1 writes, Core 0 reads (`Ctrl2IO` struct)
- `g_mutex_i2c` — Core 0 writes, Core 1 reads (`IO2Ctrl` struct)

`loop()` takes `g_mutex_i2c` with timeout=0 (non-blocking — never stalls BT).  
`ioTask` takes both mutexes with `portMAX_DELAY` (Core 0, no BT impact).  
`vTaskDelay(1)` at the end of both `loop()` paths yields Core 1 to btstack service tasks.

---

## Globals: where they live

- **Defined in `.ino`** — all mutable globals (mutexes, structs, encoder state, gamepad, relay flags, etc.)
- **`extern`'d in `types.h`** — so every `.cpp` can access them without re-declaring
- **`static` inside `loop()`** — button edge-detect state (`prev_PS`, `prev_R1`, …), motor throttle state (`sentL`, `sentR`, `lastMotorMs`), and drive logging — only `loop()` needs these
- **`static const` in `types.h`** — colour constants and `modeNames[]` — each TU gets its own 3-byte copy, which is fine

---

## Motor protocol — Cytron MDDS30 Serial Simplified

Two bytes per command (L then R), on Serial2 (GPIO 25):

```
Motor L fwd: 0x00 | map(spd,  0,100, 0,63)    rev: 0x40 | map(|spd|, 0,100, 0,63)
Motor R fwd: 0xC0 | map(spd,  0,100, 0,63)    rev: 0x80 | map(|spd|, 0,100, 0,63)
Speed range: -100..+100
```

Commands throttled to 20 Hz keepalive; fires immediately on any speed change.  
`MOTOR_L_DIR` / `MOTOR_R_DIR` defines (+1 or -1) flip motor direction without rewiring.

---

## Key tuning constants (all in `types.h`)

| Constant | Default |aqZWA Effect |
|---|---|---|
| `DEADZONE` | 20 | Stick dead band (0–511 scale) |
| `MAX_SPEED` | 100 | Motor speed ceiling sent to MDDS30 |
| `MIN_MOTOR_SPEED` | 0 | Motor speed floor (set to stall threshold, e.g. 15, when known) |
| `EXPO_BLEND` | 0.25f | Stick curve: 1.0=full cubic expo, 0.0=linear |
| `MOTOR_L_DIR` | +1 | Flip to -1 to reverse left motor without rewiring |
| `MOTOR_R_DIR` | +1 | Flip to -1 to reverse right motor without rewiring |

---

## Stick curve

```cpp
int stickToSpeed(int axis) {
  int val = -axis;                          // negate: up = positive
  if (abs(val) <= DEADZONE) return 0;
  float norm   = (float)val / STICK_MAX;
  float curved = (norm³ × EXPO_BLEND) + (norm × (1 - EXPO_BLEND));
  return (int)(constrain(curved, -1, 1) * MAX_SPEED);
}
```

Single-stick X-axis: pass `-gp->axisX()` (pre-negated at call site) so rightward push = positive turn = robot turns right.

---

## BT stability — three fixes applied in v3.1.0

1. **`Serial.setTxBufferSize(1024)` before `Serial.begin()`** — default tx_buffer_size=0 is synchronous. `debugFullStatus()` (~700 bytes) blocks `loop()` for ~61ms. DS4 HID poll is 5–10ms; a 61ms gap triggers the controller disconnect timer. Async buffer eliminates this entirely.

2. **`esp_wifi_stop()`** — WiFi and Classic BT share the 2.4GHz radio on ESP32. Even without `WiFi.begin()`, the coexistence controller steals BT air time. Stopping WiFi removes all coexistence jitter.

3. **`esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9)`** — raises Classic BT TX from default +3dBm to +9dBm. PS4 uses BR/EDR (not BLE), so `esp_bredr_*` is correct (not `esp_ble_*`).

---

## GPIO matrix fanout (Pi + Pip-Boy share Serial1 TX)

```cpp
Serial1.begin(PI_BAUD, SERIAL_8N1, PI_SERIAL_RX, PI_SERIAL_TX); // GPIO13, GPIO17
gpio_matrix_out(ESP2_SERIAL_TX, 23u, false, false); // 23 = U1TXD_OUT_IDX → GPIO4
```

Both Pi and Pip-Boy receive every line on Serial1. They filter by JSON key:
- Pi parses telemetry JSON, ignores `{"s":…}` scene messages
- Pip-Boy parses `{"s":…}` scene messages, ignores telemetry

---

## Pip-Boy scenes (`enum MowerScene`)

`IDLE=0  READY=1  DRIVING=2  MOWING=3  TURBO=4  AUTONOMOUS=5  LOW_BATTERY=6  HEAT=7  ERROR=8`

Sent as `{"s":<n>,"b1":<pct>,"b2":<pct>}` on Serial1 every 500ms from `ioTask`.

---

## Known Bluepad32 v4.1.0 gotcha — `Coll` name pollution

`btstack_hid_parser.h` defines a C enum:
```c
typedef enum { Input=8, Output, Coll, Feature, EndColl } MainItemTag;
```
`Coll` (=10) leaks into the global C++ namespace. Don't name any type or variable `Coll`.

---

## Pending hardware / tuning tasks

- [ ] Verify MDDS30 SW4 is ON (Independent Both). Earlier setting was OFF (Independent Right only).
- [ ] Test `MOTOR_L_DIR` / `MOTOR_R_DIR` — currently both +1. Flip whichever motor runs backwards.
- [ ] Measure actual stall threshold for MDDS30 and set `MIN_MOTOR_SPEED` accordingly (try 15).
