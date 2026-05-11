# Phase 2 Wiring (Planned)

This document will cover wiring for the autonomous navigation hardware.

## Planned Connections

### Raspberry Pi → ESP32 (Serial Bridge)
- Pi TX → ESP32 RX (GPIO 16 suggested)
- Pi RX → ESP32 TX (GPIO 17 suggested)
- Common GND
- Use 3.3V logic level — Pi GPIO is 3.3V, compatible with ESP32

### Raspberry Pi → LiDAR (e.g. RPLiDAR A1/A2)
- USB connection (RPLiDAR uses USB-serial adapter)
- Power: 5V from Pi or dedicated supply

### Raspberry Pi → GPS (e.g. u-blox NEO-M8N)
- UART: Pi GPIO 14 (TX) / 15 (RX)
- Or USB module variant
- Power: 3.3V or 5V depending on module

### Raspberry Pi → Compass / IMU (e.g. BNO055)
- I2C: Pi GPIO 2 (SDA) / 3 (SCL)
- Power: 3.3V

### Raspberry Pi → Camera
- Pi Camera: CSI ribbon cable
- USB camera: USB port

### ESP32 → Wheel Encoders (x2)
- Encoder A channel → ESP32 interrupt-capable pin (e.g. GPIO 34, 35)
- Encoder B channel → ESP32 interrupt-capable pin (e.g. GPIO 36, 39)
- Power: 3.3V or 5V depending on encoder (use voltage divider if 5V output)

## Notes
- To be completed when Phase 2 hardware is selected and sourced
- Consider a PCB or terminal block breakout for the Pi hat
- EMI shielding recommended for GPS and compass near motor drivers
