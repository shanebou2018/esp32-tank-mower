# ESP32 Tank Mower v2.4.0
## Pin Reference

---

### 🔵 Motors (Cytron MDDS30 — dual channel)
```
GPIO 25 → PWM1 (Left)
GPIO 26 → DIR1 (Left)
GPIO 27 → PWM2 (Right)
GPIO 14 → DIR2 (Right)
```

---

### 🔴 Relays
```
GPIO 32 → ARM relay   (momentary)
GPIO 33 → MOTOR relay (blade latch)
GPIO 15 → TURBO relay (turbo latch)
```

---

### 🟡 Encoders (AS5600 PWM)
```
GPIO 34 → Left  encoder (input)
GPIO 35 → Right encoder (input)
```

---

### 🟢 Onboard LED
```
GPIO 16 → Green LED (PWM)
```

---

### 🟣 I2C — PCF8575 (0x20)
```
GPIO 21 → SDA
GPIO 22 → SCL
```
PCF8575 inputs (HIGH = active):
```
P0  → Bat1 25%
P1  → Bat1 50%
P2  → Bat1 75%
P3  → Bat1 100%
P4  → Bat1 heat
P5  → Bat2 25%
P6  → Bat2 50%
P7  → Bat2 75%
P8  → Bat2 100%
P9  → Bat2 heat
P10 → Mower error
P11 → Turbo feedback
```
PCF8575 outputs (LOW = press):
```
P12 → Turbo button
P13 → Lights button
```

---

### 🔶 Pi Serial (Serial2)
```
GPIO 17 TX → Pi GPIO15 RX
GPIO 13 RX → Pi GPIO14 TX
```
> Pi TX was on GPIO16 in v2.1
> moved to GPIO13 in v2.2+

---

### ⚪ Pip-Boy Serial (Serial1)
```
GPIO 4 TX → Pip-Boy D0 (GPIO1)
GPIO 5 RX   (unused — TX only)
```

---

### ⚡ Power / GND
```
GND → Pi GND
GND → PCF8575 GND
GND → Encoder GND
GND → Motor driver GND
3.3V or 5V → PCF8575 VCC
     (check your module)
```

---

### 🎮 PS4 Controller
Wireless via Bluepad32 — no wiring needed.

---

### 📋 Button Map
```
PS  = Toggle drive mode
R1  = Arm / stop blade
TRI = Toggle turbo
SQR = Toggle lights
L2  = Set D-pad speed
L1  = Reset D-pad speed
```

---

### 💡 Board LED Guide
```
Off        = no controller
Dim        = dual stick
Low        = single stick
Medium     = blade running
Full       = turbo active
Fast blink = mower ERROR
Med  blink = battery heat
```
