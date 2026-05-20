# ESP32 Tank Mower — Claude context

## Board
**ESP32-DevKitC-32UE** (WROOM-32UE module, external U.FL antenna).
Pinout constraints are identical to WROOM-32E.

## Active firmware
`phase1/src/32E_tank_controller_bp32_v3.2.0/`

Full technical notes (hardware, RTOS, motor protocol, BT fixes, gotchas):
→ `phase1/src/32E_tank_controller_bp32_v3.2.0/NOTES.md`

Previous monolith (all logic in one file, identical behaviour):
→ `phase1/src/32E_tank_controller_bp32_v3.1.0/32E_tank_controller_bp32_v3.1.0.ino`

---

## File structure (v3.2.0)

```
types.h              ← single include: all #defines, enums, structs, externs, prototypes
32E_tank_controller_bp32_v3.2.0.ino  ← global defs + setup() + loop()
debug.cpp            ← debugSep / debugBanner / printBTAddr / debugRelays / debugFullStatus
drive.cpp            ← LED / stick curve / motor bytes / arm state machine
bt_callbacks.cpp     ← onConnectedGamepad / onDisconnectedGamepad
io_task.cpp          ← publishCtrl2IO + ioTask + static ioXxx helpers
```

---

## Non-obvious rules — read before editing

### Arduino IDE prototype scanner
The IDE inserts auto-generated prototypes after the last `#include` in the `.ino`.
If any function is defined *before* all type definitions in the `.ino`, those types
are missing when prototypes are inserted → "does not name a type" errors.
**Fix already applied:** `.cpp` files bypass the scanner entirely. Keep new functions
in `.cpp` files, not the `.ino`. If you must add a function to the `.ino`, put it
after all type definitions.

### `publishCtrl2IO` and `ioTask` are NOT static
They look like internal helpers but are called across translation units.
Removing `static` was intentional — do not re-add it.

### Serial async buffer
`Serial.setTxBufferSize(1024)` MUST appear before `Serial.begin()`.
Default is synchronous TX → ~61ms blocking on status dump → DS4 disconnects.
Never remove this line or move it after `Serial.begin()`.

### BT power
`esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9)` — Classic BT only.
PS4 uses BR/EDR, NOT BLE. Use `esp_bredr_*`, not `esp_ble_*`.

### Single-stick X-axis
Pass `-gp->axisX()` (pre-negated) to `stickToSpeed()`.
`stickToSpeed()` negates its input internally; double negation gives correct direction.

### Motor map base
`scaleToMotor()` maps from base 0 (not 1). `map(1, 1, 100, 0, 100) = 0` silently
kills the lowest speed. This was a bug in v3.0.0; do not revert.

### `Coll` name pollution
`btstack_hid_parser.h` leaks `Coll=10` into the global namespace. Do not name
any type or variable `Coll`.

---

## Coding preferences

- `loop()` must be fully non-blocking — no `delay()` calls ever
- Use `millis()` timestamp checks for all periodic work
- Serial receive buffers: fixed `char[]` with a length counter — no `String`
- PS4 LED: flash pattern (solid mode colour idle, flash on relay activity)
  Do not add LED priority enums or GPIO 16 board LED features
- Motor direction: via `MOTOR_L_DIR` / `MOTOR_R_DIR` defines — not rewiring
- Multi-file split: `.h` + `.cpp` architecture — not multiple `.ino` files

---

## Hardware pending

- [ ] Confirm MDDS30 SW4=ON (Independent Both). Was OFF (Independent Right only).
- [ ] Test motor directions — `MOTOR_L_DIR` / `MOTOR_R_DIR` currently both +1
- [ ] Measure MDDS30 stall threshold → set `MIN_MOTOR_SPEED` (try 15)
- [ ] ESP32s must be on their own 5V supply, NOT the MDDS30 5V pin (sags under BT TX)
- [ ] External U.FL antenna routed away from MDDS30 / motor wiring

## BT escalation path (user-decided)

If BT is still unsatisfactory, the agreed order of attack:
1. **External U.FL antenna** (WROOM-32UE) — DONE on new board, this is the first test
2. **One more firmware pass** — Bluepad32 v4.2.1 upgrade, sniff-mode disable, RX-timeout
   motor stop, allowlist API, drop Serial.print volume on hot path
3. **Dedicated BT ESP32** — offload Bluepad32 onto its own chip; main controller
   becomes BT-free (no btstack, no WiFi/BT coexistence, no timing constraints from BT)

When tier 3 lands: a second ESP32 (cheap NodeMCU is fine) runs Bluepad32 only and
publishes controller state to the main controller. Simplest comm channel is I2C
(piggyback on the existing PCF8575 bus — main controller polls BT-ESP32 as an I2C
peripheral at 50–100 Hz). Failsafe: main stops motors if frames go stale >500ms.
