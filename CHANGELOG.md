# Changelog

All notable changes to this project are documented here.

---

## Phase 1 — Manual RC Control

### [1.5.0] — 2026
- Added D-pad driving with L2 analog speed ceiling
- Added L1 reset for D-pad speed ceiling
- Added gentle S-curve to analog sticks (60% linear + 40% cubic)
- D-pad takes priority over analog sticks when any direction pressed
- Serial debug reports peak L2 and calculated dpad speed

### [1.4.0] — 2026
- Remapped buttons: R1 = arm/motor sequence, Triangle = turbo, R2 = reserved
- MAX_SPEED reduced to 127 (50%) for better resolution with upgraded motor controllers

### [1.3.0] — 2026
- Added turbo relay on pin 15 (Triangle button)
- LED flashes white while turbo is active (takes priority over motor flash)
- Turbo toggles independently of mower motor

### [1.2.0] — 2026
- Replaced simple R2 relay toggle with non-blocking arm/motor sequence state machine
- Sequence: ARM relay ON → 500ms → MOTOR relay latches ON (both stay HIGH together)
- R1 press while running drops both ARM and MOTOR simultaneously
- Rumble feedback at each stage of the sequence

### [1.1.0] — 2026
- Migrated from PS4Controller library to Bluepad32
- Fixed MISC_BUTTON_SELECT → MISC_BUTTON_HOME for PS button
- Fixed btaddr printing (uint8_t[6] printed byte-by-byte as hex)
- Added timed non-blocking rumble (connect buzz + relay feedback)
- Deadzone tightened to ±6 (was 20)
- Mode toggle moved to PS button
- Added disconnect failsafe: all motors stop, all relays go LOW

### [1.0.0] — 2026
- Initial implementation
- PS4 controller via PS4Controller library
- Dual stick and single stick tank modes
- L1 relay toggle, R2 relay toggle
- LED colour reflects tank mode, flashes red when relay latched
- Serial debug output

---

## Phase 2 — Autonomous Navigation (Planned)

### [2.0.0] — TBD
- Raspberry Pi integration
- ESP32 serial bridge (JSON command protocol)
- Wheel encoder odometry
- LiDAR obstacle detection
- Roomba-style coverage pattern (first autonomous mode)
- GPS perimeter waypoints
- Compass / IMU heading
- Camera integration
- SLAM mapping
- Web UI
- Return to base
