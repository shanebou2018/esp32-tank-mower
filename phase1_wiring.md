# Phase 1 Wiring

## ESP32 → Cytron MD13S (x2)

Each MD13S has two signal pins: PWM (speed) and DIR (direction).
Connect both drivers to 3.3V-compatible logic from the ESP32.
Power the MD13S from your main battery (supports up to 30V / 13A).

| ESP32 Pin | MD13S | Track |
|-----------|-------|-------|
| 25 | PWM | Left |
| 26 | DIR | Left |
| 27 | PWM | Right |
| 14 | DIR | Right |
| GND | GND | Both |

> DIR HIGH = forward, DIR LOW = reverse (swap motor wires to invert if needed)

---

## ESP32 → Relays (x3)

All relays default **active HIGH**. If your relay module is active LOW,
swap HIGH/LOW in the sketch or add an inverter.

| ESP32 Pin | Relay | Function |
|-----------|-------|----------|
| 32 | ARM | Safety interlock — pulses during start sequence, stays HIGH with motor |
| 33 | MOTOR | Mower blade motor — latching |
| 15 | TURBO | Turbo — latching |
| GND | GND | All relays |

> Always power relay coils from a separate 5V supply if driving more than 3 relays,
> to avoid browning out the ESP32.

---

## Power

- ESP32: 5V via USB or onboard regulator from battery
- MD13S: main drive battery (match to your motor voltage)
- Relay coils: 5V (use separate supply or relay modules with onboard optocouplers)
- Ensure common GND between ESP32, MD13S, and relay supply

---

## Notes

- Keep PWM and DIR wires away from motor power wires to reduce noise
- Add a 100nF ceramic cap across each motor terminal to suppress EMI
- If the ESP32 resets under load, add a large capacitor (470–1000µF) across the 5V supply
