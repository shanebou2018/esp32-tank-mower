# ESP32 Tank Mower

A two-phase project to build a PS4-controlled RC lawn mower (Phase 1) that evolves into a fully autonomous robot mower (Phase 2) using a Raspberry Pi, LiDAR, GPS, camera, compass, and wheel encoders.

---

## Project Roadmap

| Phase | Status | Description |
|-------|--------|-------------|
| **Phase 1** | ✅ Active | Manual RC control via PS4 controller + ESP32 |
| **Phase 2** | 🔲 Planned | Autonomous navigation via Raspberry Pi + sensors |

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
| Microcontroller | ESP32 (any standard dev board) |
| Controller | Sony PS4 DualShock 4 |
| Motor drivers | 2x Cytron MD13S 13A DC Motor Driver |
| Relay — Arm | Pin 32 — safety interlock, momentary during start |
| Relay — Motor | Pin 33 — latching mower blade relay |
| Relay — Turbo | Pin 15 — latching turbo relay |
| Track motors | 2x DC motors via MD13S |

### Pin Map — Phase 1

| Signal | ESP32 Pin |
|--------|-----------|
| Left track PWM | 25 |
| Left track DIR | 26 |
| Right track PWM | 27 |
| Right track DIR | 14 |
| Arm relay | 32 |
| Motor relay | 33 |
| Turbo relay | 15 |

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
4. Open `phase1/src/tank_controller_bp32.ino`
5. Flash to your ESP32
6. Hold the PS button on your controller to pair — no extra tools needed

---

## Phase 2 — Autonomous Navigation (Planned)

Phase 2 adds a Raspberry Pi as the high-level brain. The ESP32 remains as the low-level motor/relay controller and receives commands over a serial bridge from the Pi.

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
│       └── tank_controller_bp32.ino
│
└── phase2/
    ├── esp32/                      # Updated ESP32 firmware with serial bridge
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
