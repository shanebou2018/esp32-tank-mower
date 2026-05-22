# ESP32 Tank Mower

A two-phase project to build a PS4-controlled RC lawn mower (Phase 1) that evolves into a fully autonomous robot mower (Phase 2) using a Raspberry Pi, LiDAR, GPS, camera, compass, and wheel encoders.

---

## Project Roadmap

| Phase | Status | Description |
|-------|--------|-------------|
| **Phase 1** | ✅ Complete | Manual RC control via PS4 controller + ESP32 |
| **Phase 2** | 🚧 Active | Autonomous navigation via Raspberry Pi + sensors |

---

## Phase 1 — Manual RC Control

### Features
- PS4 DualShock controller over Bluetooth via **Bluepad32**
- Tank drive with two **Cytron 13A MD13S** motor drivers
- Two drive modes toggled by PS button:
  - **Dual stick** — each stick controls one track independently (LED: blue)
  - **Single stick** — left stick Y = throttle, left stick X = steering mix (LED: green)
- **Safety arm sequence** — press R1 to start:
  1. Arm relay fires (500ms)
  2. Motor relay latches ON — both stay HIGH together
  3. Press R1 again to drop both simultaneously
- **Turbo relay** — Triangle button toggles, LED flashes white while active
- **D-pad driving** — secondary control with L2-set speed ceiling:
  - Squeeze L2 to set max D-pad speed (harder squeeze = higher ceiling)
  - L1 resets the ceiling to zero
- **Gentle S-curve** on analog sticks for low-speed precision
- **Controller LED** shows system state at a glance:
  - Solid blue = dual stick mode
  - Solid green = single stick mode
  - Flashing red/blue or red/green = mower motor latched ON
  - Flashing white = turbo active (overrides motor flash)
- **Timed rumble** feedback on connect, motor latch, turbo toggle
- **Full serial debug** output at 115200 baud
- **Failsafe** — all motors and relays cut immediately on controller disconnect

### Hardware — Phase 1

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32-DevKitC-32UE (WROOM-32UE, external U.FL antenna) |
| Controller | Sony PS4 DualShock 4 |
| Motor driver | Cytron MDDS30 SmartDriveDuo (serial simplified mode) |
| Relay — Arm | GPIO 32 — safety interlock, momentary during start |
| Relay — Motor | GPIO 33 — latching mower blade relay |
| Relay — Turbo | GPIO 27 — latching turbo relay |
| Track motors | 2x DC motors via MDDS30 |
| Wheel encoders | 2x AS5600 magnetic encoders (PWM mode) |
| I2C I/O expander | PCF8575 @ 0x20 — battery levels, mower error, lights, turbo feedback |

### Pin Map — Phase 1

| Signal | GPIO | Notes |
|--------|------|-------|
| MDDS30 serial TX | 25 | Serial Simplified → MDDS30 IN1 only; disconnect AN1/AN2/IN2 |
| Arm relay | 32 | Active HIGH |
| Motor relay | 33 | Active HIGH, latching |
| Turbo relay | 27 | Active HIGH, latching (moved from GPIO 15 — strapping pin) |
| Left encoder PWM | 34 | AS5600 PWM output |
| Right encoder PWM | 35 | AS5600 PWM output |
| I2C SDA | 21 | PCF8575 |
| I2C SCL | 22 | PCF8575 |
| Pi serial TX | 17 | UART1 TX → Pi RX |
| Pi serial RX | 13 | UART1 RX ← Pi TX |
| Pip-Boy serial RX | 4 | UART1 TX also routed here via GPIO matrix |

### MDDS30 DIP Switch Settings

`11011111` — SW1=ON SW2=ON SW3=OFF SW4=ON SW5=ON SW6=ON SW7=ON SW8=ON

| Switches | Setting |
|----------|---------|
| SW1+SW2 | Serial mode |
| SW3 | Serial Simplified |
| SW4+SW5 | Independent Both |
| SW6+SW7+SW8 | 115200 baud |

### Controls — Phase 1

| Button | Action |
|--------|--------|
| PS button | Toggle tank mode (dual / single stick) |
| R1 | Start arm sequence → latch mower motor (press again to stop) |
| Triangle | Toggle turbo relay |
| L2 analog | Squeeze to set D-pad speed ceiling |
| L1 | Reset D-pad speed ceiling to zero |
| D-pad | Drive at L2-set speed |
| R2 | Reserved — Phase 2 |

### Install — Phase 1

1. In Arduino IDE, go to **File > Preferences > Additional Boards Manager URLs** and add:
   ```
   https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
   ```
2. Go to **Tools > Board > Boards Manager**, search **Bluepad32**, install.
3. Select board: **ESP32 + Bluepad32**
4. Open `phase1/src/32E_tank_controller_bp32_v3.2.0/32E_tank_controller_bp32_v3.2.0.ino`
5. Flash to your ESP32
6. Hold the PS button on your controller to pair — no extra tools needed

---

## Phase 2 — Autonomous Navigation (In Progress)

Phase 2 adds a Raspberry Pi as the high-level brain. The ESP32 firmware has been updated with a full Pi serial bridge (Serial2 on GPIO 16/17), AS5600 magnetic wheel encoders, and telemetry output at 20Hz. The ESP32 remains the low-level motor/relay controller and accepts drive commands from the Pi when no PS4 controller is connected.

### Planned Hardware — Phase 2

| Component | Purpose |
|-----------|---------|
| Raspberry Pi 4/5 | Autonomous navigation brain |
| LiDAR (e.g. RPLiDAR A1/A2) | Obstacle detection and SLAM mapping |
| GPS module (e.g. u-blox NEO-M8N) | Outdoor position tracking |
| Compass / IMU (e.g. BNO055) | Heading, orientation |
| Camera (Pi Camera or USB) | Visual obstacle detection / future CV |
| Wheel encoders (x2) | Odometry — track distance and speed per wheel |
| ESP32 (existing) | Low-level motor + relay control, sensor bridge |

### Planned Features — Phase 2

- **Roomba-style coverage** — systematic back-and-forth mowing pattern as first autonomous mode
- **Perimeter learning** — manually drive the boundary once, save to GPS waypoints
- **Obstacle avoidance** — LiDAR stops/reroutes around detected objects
- **SLAM mapping** — build and save a map of the mowing area
- **Encoder odometry** — accurate dead-reckoning between GPS fixes
- **ESP32 ↔ Pi serial bridge** — Pi sends drive commands, ESP32 reports encoder ticks and relay states
- **Web UI** — monitor and control from phone/browser over local WiFi
- **Return to base** — GPS-guided return to charging station

### ESP32 ↔ Pi Communication (Planned)

The ESP32 will expose a simple serial JSON protocol:

```
Pi → ESP32:  {"cmd":"drive","l":120,"r":115}
Pi → ESP32:  {"cmd":"relay","id":"motor","state":1}
ESP32 → Pi:  {"enc_l":1024,"enc_r":1019,"relay_motor":1,"relay_turbo":0}
```

---

## Repository Structure

```
esp32-tank-mower/
│
├── README.md
├── CLAUDE.md
├── CHANGELOG.md
├── LICENSE
├── .gitignore
│
├── docs/
│   ├── wiring/
│   │   ├── phase1_wiring.md        # Phase 1 wiring notes
│   │   └── phase2_wiring.md        # Phase 2 wiring notes (planned)
│   └── images/                     # Photos, diagrams
│
├── phase1/
│   └── src/
│       ├── 32E_tank_controller_bp32_v3.2.0/   # Active firmware (multi-file)
│       │   ├── 32E_tank_controller_bp32_v3.2.0.ino  # Global defs, setup(), loop()
│       │   ├── types.h             # All #defines, enums, structs, externs, prototypes
│       │   ├── debug.cpp           # Debug helpers
│       │   ├── drive.cpp           # LED, stick curve, motor bytes, arm state machine
│       │   ├── bt_callbacks.cpp    # Bluepad32 connect/disconnect callbacks
│       │   ├── io_task.cpp         # PCF8575 I/O task (Core 0)
│       │   └── NOTES.md            # Hardware, RTOS, motor protocol, BT notes
│       └── old/                    # Previous versions (v2.1.0 – v3.1.0)
│
└── phase2/
    ├── esp32/                      # ESP32 firmware with Pi serial bridge
    │   └── README.md
    └── pi/
        ├── src/                    # Python entry points
        ├── config/                 # Sensor calibration, area maps, waypoints
        ├── navigation/             # Coverage patterns, path planning
        ├── mapping/                # SLAM, LiDAR processing
        ├── sensors/                # GPS, compass, encoder, camera drivers
        └── README.md
```

---

## Contributing

Pull requests welcome. If you're working on Phase 2 sensors or navigation, open an issue first so we can coordinate.

---

## License

MIT License — see [LICENSE](LICENSE)
