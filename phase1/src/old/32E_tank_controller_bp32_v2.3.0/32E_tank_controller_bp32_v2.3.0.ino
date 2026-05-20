// ============================================================
// ESP32 Tank Mower — Phase 2 Firmware  v2.3.0
// Board: ESP32-DevKitC-32E (ESP32-WROOM-32E)
// Based on Phase 1 v1.5.0 — adds full Pi serial bridge telemetry
//
// v2.0.0 changes:
//   - Target board changed to ESP32-DevKitC-32E (WROOM-32E)
//   - Onboard WS2812 RGB LED (GPIO 16) mirrors PS4 controller LED
//     colours in real time via FastLED
//
// v2.1.0 changes:
//   - PI_SERIAL_RX moved GPIO 16 → GPIO 13 (GPIO 16 reserved for WS2812)
//   - I2C bus initialised (SDA=GPIO 21, SCL=GPIO 22) for PCF8575 expander
//   - Serial1 added (TX=GPIO 4, RX=GPIO 5) for 2nd ESP32 peer link
//
// v2.2.0 changes:
//   - PCF8575 mower I/O fully wired (12 inputs, 2 outputs)
//   - 5-tier LED priority: error(amber) > bat heat(yellow) >
//     turbo(white) > motor(red) > mode(blue/green solid)
//   - Telemetry extended: bat1/2 levels, heat, mower_err, turbo_fb, lights
//   - Pi commands: {"cmd":"pcf","id":"turbo_btn"} / "lights_btn"
//   - Controller rumble on first mower error detection
//
// v2.3.0 changes:
//   - Pip-Boy scene engine: 9 scenes sent over Serial1 at 500ms interval
//   - Scene priority: ERROR > HEAT > LOW_BAT > TURBO > MOWING >
//                     AUTONOMOUS > DRIVING > READY > IDLE
//   - Pip-Boy wiring: mower GPIO4 (TX) → Pip-Boy GPIO28 (A4 pad)
//
// Hardware:
//   2x Cytron MD13S motor driver
//   Relay: ARM   pin 32 (momentary safety arm)
//   Relay: MOTOR pin 33 (latching mower blade)
//   Relay: TURBO pin 15 (latching turbo)
//   2x AS5600 magnetic encoders (PWM mode, pins 34/35)
//   PS4 controller via Bluepad32
//   Pi 5 via Serial2 (TX=GPIO 17, RX=GPIO 13) at 115200 baud
//   2nd ESP32 via Serial1 (TX=GPIO 4,  RX=GPIO 5)  at 115200 baud
//   PCF8575 I/O expander via I2C (SDA=GPIO 21, SCL=GPIO 22, ADDR=0x20)
//   WS2812 RGB LED on GPIO 16 (onboard, 1 LED)
//
// PCF8575 pin map (mower I/O expander at 0x20)
//
//   Input pins  (HIGH = asserted)
//   P0  ( 0) : Battery 1 — 25%  level  ─┐ only HIGH during 30s window
//   P1  ( 1) : Battery 1 — 50%  level   │ after a blade state change;
//   P2  ( 2) : Battery 1 — 75%  level   │ ESP32 latches last seen value
//   P3  ( 3) : Battery 1 — 100% level  ─┘
//   P4  ( 4) : Battery 1 — heat warning  (direct pin state)
//   P5  ( 5) : Battery 2 — 25%  level  ─┐ same latch behaviour as bat1
//   P6  ( 6) : Battery 2 — 50%  level   │
//   P7  ( 7) : Battery 2 — 75%  level   │
//   P8  ( 8) : Battery 2 — 100% level  ─┘
//   P9  ( 9) : Battery 2 — heat warning  (direct pin state)
//   P10 (10) : Mower error      (hardware latched HIGH until mower resets)
//   P11 (11) : Turbo feedback   (hardware latched HIGH while turbo active)
//
//   Output pins (write LOW to activate, HIGH to release)
//   P12 (12) : Turbo button  — 200ms LOW pulse via Pi command
//   P13 (13) : Lights button — 200ms LOW pulse via Pi command
//   P14–P15  : unused (held HIGH)
//
// Pi ↔ ESP32 JSON protocol:
//   Pi → ESP32:  {"cmd":"drive","l":120,"r":115}
//   Pi → ESP32:  {"cmd":"relay","id":"motor","state":1}
//   Pi → ESP32:  {"cmd":"relay","id":"turbo","state":0}
//   Pi → ESP32:  {"cmd":"stop"}
//   Pi → ESP32:  {"cmd":"ping"}
//   Pi → ESP32:  {"cmd":"pcf","id":"turbo_btn"}   ← 200ms mower turbo press
//   Pi → ESP32:  {"cmd":"pcf","id":"lights_btn"}  ← 200ms mower lights press
//   ESP32 → Pi:  {
//     "enc_l":1024,"enc_r":1019,
//     "dist_l":0.1234,"dist_r":0.1230,
//     "spd_l":0.05,"spd_r":0.05,
//     "relay_arm":0,"relay_motor":1,"relay_turbo":0,
//     "connected":1,"mode":"DUAL","batt":85,
//     "bat1":75,"bat1_heat":0,
//     "bat2":50,"bat2_heat":0,
//     "mower_err":0,"turbo_fb":1,"lights":0
//   }
//
// Mode arbitration:
//   PS4 connected = PS4 has full control, Pi drive commands ignored
//   PS4 disconnected = Pi drive commands accepted (autonomous mode)
//   Relay commands from Pi always accepted (both modes)
//   All relays cut on PS4 disconnect (failsafe preserved)
//
// Libraries required:
//   Bluepad32  (board package — see install note below)
//   EncAS5600  (search "EncAS5600" in Library Manager)
//   FastLED    (search "FastLED" in Library Manager)
//   PCF8575    (search "PCF8575" by Renzo Mischianti in Library Manager)
//
// Install Bluepad32 for ESP32-DevKitC-32E:
//   File > Preferences > Additional Boards Manager URLs:
//   https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
//   Tools > Board > Boards Manager > search "Bluepad32" > install
//   Select board: "ESP32 + Bluepad32"
//   Tools > Board > ESP32 + Bluepad32 > ESP32 Dev Module
//   (ESP32-WROOM-32E is compatible with the generic ESP32 Dev Module target)
// ============================================================

#include <Bluepad32.h>
#include <EncAS5600.h>
#include <FastLED.h>
#include <Wire.h>
#include <PCF8575.h>

// ── ESP32 pin definitions ────────────────────────────────────
#define MOTOR_A_PWM   25
#define MOTOR_A_DIR   26
#define MOTOR_B_PWM   27
#define MOTOR_B_DIR   14

#define RELAY_ARM     32
#define RELAY_MOTOR   33
#define RELAY_TURBO   15

#define ENCODER_L_PIN 34
#define ENCODER_R_PIN 35

// ── WS2812 onboard RGB LED ───────────────────────────────────
#define WS2812_PIN    16
#define WS2812_COUNT  1
CRGB wsLeds[WS2812_COUNT];

// ── I2C / PCF8575 ────────────────────────────────────────────
#define I2C_SDA      21
#define I2C_SCL      22
#define PCF8575_ADDR 0x20

PCF8575 pcf(PCF8575_ADDR);
bool     pcfPresent = false;

// PCF8575 input pin numbers
#define PCF_BAT1_25    0
#define PCF_BAT1_50    1
#define PCF_BAT1_75    2
#define PCF_BAT1_100   3
#define PCF_BAT1_HEAT  4
#define PCF_BAT2_25    5
#define PCF_BAT2_50    6
#define PCF_BAT2_75    7
#define PCF_BAT2_100   8
#define PCF_BAT2_HEAT  9
#define PCF_MOWER_ERR  10
#define PCF_TURBO_FB   11

// PCF8575 output pin numbers (active LOW)
#define PCF_TURBO_BTN  12
#define PCF_LIGHTS_BTN 13

#define PCF_READ_MS      100   // poll PCF8575 at 10 Hz
#define PCF_BTN_PULSE_MS 200   // button press pulse width

// ── Serial1 — Pip-Boy link (TX only → Pip-Boy GPIO28 A4 pad) ────────────────
#define ESP2_SERIAL_TX  4
#define ESP2_SERIAL_RX  5       // unused — one-way TX to Pip-Boy
#define ESP2_BAUD       115200

// ── Pip-Boy scene engine ──────────────────────────────────────────────────────
// Scene IDs sent as {"s":N,"b1":NN,"b2":NN} over Serial1 every 500ms.
// The Pip-Boy maps each ID to a tab/sub animation.
enum MowerScene {
  SCENE_IDLE        = 0,  // no controller, stopped
  SCENE_READY       = 1,  // PS4 connected, idle
  SCENE_DRIVING     = 2,  // moving, no blade
  SCENE_MOWING      = 3,  // blade running
  SCENE_TURBO       = 4,  // blade + turbo active
  SCENE_AUTONOMOUS  = 5,  // Pi autonomous mode
  SCENE_LOW_BATTERY = 6,  // any battery ≤ 25%
  SCENE_HEAT        = 7,  // battery heat warning
  SCENE_ERROR       = 8,  // mower fault
};
#define PIPBOY_INTERVAL_MS 500

// ── Pi serial bridge ─────────────────────────────────────────
#define PI_SERIAL_TX      17
#define PI_SERIAL_RX      13   // moved from GPIO 16 — WS2812 conflict
#define PI_BAUD           115200
#define PI_TELEM_MS       50
#define PI_CMD_TIMEOUT_MS 500

// ── Odometry ─────────────────────────────────────────────────
#define WHEEL_DIAMETER_M  0.254f
#define GEAR_RATIO        16.0f
#define DIST_PER_DEG_M    (3.14159265f * WHEEL_DIAMETER_M / (GEAR_RATIO * 360.0f))

// ── Tuning ───────────────────────────────────────────────────
#define DEADZONE          6
#define MAX_SPEED         127
#define STICK_MAX         511
#define DPAD_SPEED_MIN    15
#define ARM_PULSE_MS      500
#define FLASH_MS          300
#define RUMBLE_MS         300

// ── Tank mode ────────────────────────────────────────────────
enum TankMode { MODE_DUAL_STICK, MODE_SINGLE_STICK };
const char* modeNames[] = { "DUAL", "SINGLE" };

// ── Arm state machine ────────────────────────────────────────
enum ArmState { ARM_IDLE, ARM_ARMING };

// ── LED colours ──────────────────────────────────────────────
struct Colour { uint8_t r, g, b; };
const Colour COL_DUAL   = { 0,   0,   255 }; // Blue   — dual stick mode
const Colour COL_SINGLE = { 0,   255, 0   }; // Green  — single stick mode
const Colour COL_RED    = { 255, 0,   0   }; // Red    — motor relay flash
const Colour COL_WHITE  = { 255, 255, 255 }; // White  — turbo flash
const Colour COL_YELLOW = { 255, 180, 0   }; // Yellow — battery heat flash
const Colour COL_AMBER  = { 255, 80,  0   }; // Amber  — mower error flash
const Colour COL_OFF    = { 0,   0,   0   }; // Off    — disconnected / error dark

// ── LED priority ─────────────────────────────────────────────
enum LedPriority {
  PRIO_MODE = 0,  // solid blue/green
  PRIO_MOTOR,     // red flash
  PRIO_TURBO,     // white flash
  PRIO_BAT_HEAT,  // yellow flash
  PRIO_ERROR      // amber flash (highest)
};

// ── Global state ─────────────────────────────────────────────
GamepadPtr  gGamepad    = nullptr;
TankMode    tankMode    = MODE_DUAL_STICK;
ArmState    armState    = ARM_IDLE;
bool        relay_motor = false;
bool        relay_turbo = false;
unsigned long armTimer  = 0;

// Button edge detection
bool prev_PS  = false;
bool prev_R1  = false;
bool prev_TRI = false;
bool prev_L1  = false;
bool prev_SQR = false;

// D-pad speed ceiling
int peakL2    = 0;
int dpadSpeed = 0;

// LED flash state
unsigned long lastFlash    = 0;
bool          flashState   = false;
LedPriority   prevLedPrio  = PRIO_MODE;

// Timed rumble
unsigned long rumbleOffTime = 0;

// Drive change detection
int prev_leftSpeed  = 9999;
int prev_rightSpeed = 9999;

// Pip-Boy scene tracking
int           scene_driveL     = 0;   // last sent drive speeds for scene detection
int           scene_driveR     = 0;
unsigned long lastPipBoySent   = 0;

// Periodic status
unsigned long lastStatusPrint = 0;
#define STATUS_INTERVAL_MS 2000

// ── Encoder state ────────────────────────────────────────────
volatile long  enc_ticks_L  = 0;
volatile long  enc_ticks_R  = 0;
volatile float enc_dist_L_m = 0.0f;
volatile float enc_dist_R_m = 0.0f;

as5600config_t cfg_L, cfg_R;
EncAS5600 *enc_L = nullptr;
EncAS5600 *enc_R = nullptr;

// ── Pi serial bridge state ───────────────────────────────────
String        piCmdBuffer   = "";
bool          piDriveActive = false;
int           pi_leftSpeed  = 0;
int           pi_rightSpeed = 0;
unsigned long lastPiCmd     = 0;
unsigned long lastTelemSent = 0;

// ── PCF8575 state ────────────────────────────────────────────
// Battery % pins pulse HIGH for ~30s on blade state change.
// We latch the last seen non-zero level so it persists after pins go LOW.
uint8_t  bat1_level    = 0;     // 0 / 25 / 50 / 75 / 100
uint8_t  bat2_level    = 0;
bool     bat1_heat     = false;
bool     bat2_heat     = false;
bool     mower_error   = false; // hardware-latched HIGH until mower resets
bool     turbo_fb      = false; // hardware-latched HIGH while turbo active
bool     pcf_lights_on = false; // software toggle — no feedback pin

bool           prev_mower_error   = false;
unsigned long  lastPCFRead        = 0;
unsigned long  turboBtnOff        = 0;
unsigned long  lightsBtnOff       = 0;

// ============================================================
// Debug helpers
// ============================================================
void debugSep()  { Serial.println(F("------------------------------------------")); }

void debugBanner(const char* msg) {
  debugSep();
  Serial.print(F("  ")); Serial.println(msg);
  debugSep();
}

void printBTAddr(const uint8_t* addr) {
  for (int i = 0; i < 6; i++) {
    if (addr[i] < 0x10) Serial.print('0');
    Serial.print(addr[i], HEX);
    if (i < 5) Serial.print(':');
  }
  Serial.println();
}

void debugRelays() {
  Serial.print(F("[RELAY] ARM=")); Serial.print(digitalRead(RELAY_ARM) ? F("ON ") : F("OFF"));
  Serial.print(F(" MOTOR="));     Serial.print(relay_motor              ? F("ON ") : F("OFF"));
  Serial.print(F(" TURBO="));     Serial.println(relay_turbo            ? F("ON")  : F("OFF"));
}

void debugFullStatus() {
  debugSep();
  Serial.println(F("  STATUS"));
  Serial.print(F("  Mode      : ")); Serial.println(modeNames[tankMode]);
  Serial.print(F("  ARM relay : ")); Serial.println(digitalRead(RELAY_ARM) ? "ON" : "OFF");
  Serial.print(F("  MOTOR     : ")); Serial.println(relay_motor  ? "ON" : "OFF");
  Serial.print(F("  TURBO     : ")); Serial.println(relay_turbo  ? "ON" : "OFF");
  Serial.print(F("  Arm state : ")); Serial.println(armState == ARM_IDLE ? "IDLE" : "ARMING");
  Serial.print(F("  dpadSpeed : ")); Serial.print(dpadSpeed);
  Serial.print(F(" (peakL2=")); Serial.print(peakL2); Serial.println(F(")"));
  Serial.print(F("  enc L     : ")); Serial.print(enc_ticks_L);
  Serial.print(F(" ticks  ")); Serial.print(enc_dist_L_m, 3); Serial.println(F("m"));
  Serial.print(F("  enc R     : ")); Serial.print(enc_ticks_R);
  Serial.print(F(" ticks  ")); Serial.print(enc_dist_R_m, 3); Serial.println(F("m"));
  Serial.print(F("  Pi drive  : ")); Serial.println(piDriveActive ? "ACTIVE" : "idle");
  Serial.print(F("  Bat1      : ")); Serial.print(bat1_level);
  Serial.print(F("% heat=")); Serial.println(bat1_heat ? "YES" : "no");
  Serial.print(F("  Bat2      : ")); Serial.print(bat2_level);
  Serial.print(F("% heat=")); Serial.println(bat2_heat ? "YES" : "no");
  Serial.print(F("  Mower err : ")); Serial.println(mower_error  ? "YES" : "no");
  Serial.print(F("  Turbo FB  : ")); Serial.println(turbo_fb     ? "ON"  : "off");
  Serial.print(F("  Lights    : ")); Serial.println(pcf_lights_on ? "ON"  : "off");
  Serial.print(F("  Free heap : ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
  debugSep();
}

// ============================================================
// LED helpers — PS4 bar + onboard WS2812
// ============================================================
void setLED(Colour c) {
  if (gGamepad) gGamepad->setColorLED(c.r, c.g, c.b);
  wsLeds[0] = CRGB(c.r, c.g, c.b);
  FastLED.show();
}

Colour modeColour() {
  return tankMode == MODE_DUAL_STICK ? COL_DUAL : COL_SINGLE;
}

LedPriority ledPriority() {
  if (mower_error)            return PRIO_ERROR;
  if (bat1_heat || bat2_heat) return PRIO_BAT_HEAT;
  if (relay_turbo)            return PRIO_TURBO;
  if (relay_motor)            return PRIO_MOTOR;
  return PRIO_MODE;
}

// Call when we want a solid colour (only sets if in mode priority)
void refreshLED() {
  if (ledPriority() == PRIO_MODE) setLED(modeColour());
}

// Called every loop() — drives flash cycling for all priority levels
void updateLED() {
  LedPriority prio = ledPriority();

  if (prio != prevLedPrio) {
    lastFlash    = 0;
    flashState   = false;
    prevLedPrio  = prio;
    if (prio == PRIO_MODE) {
      setLED(modeColour());
      return;
    }
  }

  if (prio == PRIO_MODE) return;

  unsigned long now = millis();
  if (now - lastFlash < FLASH_MS) return;
  lastFlash  = now;
  flashState = !flashState;

  switch (prio) {
    case PRIO_ERROR:    setLED(flashState ? COL_AMBER  : COL_OFF);      break;
    case PRIO_BAT_HEAT: setLED(flashState ? COL_YELLOW : modeColour()); break;
    case PRIO_TURBO:    setLED(flashState ? COL_WHITE  : modeColour()); break;
    case PRIO_MOTOR:    setLED(flashState ? COL_RED    : modeColour()); break;
    default: break;
  }
}

// ============================================================
// PCF8575 — read inputs and decode mower state
// ============================================================
static uint8_t decodeBatLevel(uint16_t pins, uint8_t p25, uint8_t p50,
                               uint8_t p75, uint8_t p100) {
  // Only update stored level while any pin is HIGH (30s window).
  // Returns 0xFF when no pin is active (caller keeps previous value).
  bool b25  = pins & (1u << p25);
  bool b50  = pins & (1u << p50);
  bool b75  = pins & (1u << p75);
  bool b100 = pins & (1u << p100);
  if (!b25 && !b50 && !b75 && !b100) return 0xFF; // window closed
  if (b100) return 100;
  if (b75)  return 75;
  if (b50)  return 50;
  return 25;
}

void readPCF() {
  uint16_t pins = pcf.read16();

  uint8_t lvl1 = decodeBatLevel(pins, PCF_BAT1_25, PCF_BAT1_50,
                                       PCF_BAT1_75, PCF_BAT1_100);
  if (lvl1 != 0xFF) bat1_level = lvl1;

  uint8_t lvl2 = decodeBatLevel(pins, PCF_BAT2_25, PCF_BAT2_50,
                                       PCF_BAT2_75, PCF_BAT2_100);
  if (lvl2 != 0xFF) bat2_level = lvl2;

  bat1_heat   = pins & (1u << PCF_BAT1_HEAT);
  bat2_heat   = pins & (1u << PCF_BAT2_HEAT);
  mower_error = pins & (1u << PCF_MOWER_ERR);
  turbo_fb    = pins & (1u << PCF_TURBO_FB);

  // Rumble and log on first error detection
  if (mower_error && !prev_mower_error) {
    Serial.println(F("[PCF ] !!! MOWER ERROR detected !!!"));
    if (gGamepad) {
      gGamepad->setRumble(0xFF, 0xFF);
      rumbleOffTime = millis() + 600;
    }
  }
  prev_mower_error = mower_error;
}

// ============================================================
// Arm / motor state machine
// ============================================================
void startArmSequence() {
  Serial.println(F("[ARM ] Sequence START — ARM relay ON"));
  digitalWrite(RELAY_ARM, HIGH);
  armState = ARM_ARMING;
  armTimer = millis() + ARM_PULSE_MS;
  debugRelays();
}

void stopMotor() {
  relay_motor = false;
  digitalWrite(RELAY_MOTOR, LOW);
  digitalWrite(RELAY_ARM,   LOW);
  armState = ARM_IDLE;
  Serial.println(F("[ARM ] MOTOR + ARM both LOW — mower stopped"));
  debugRelays();
  refreshLED();
}

void updateArmStateMachine() {
  if (armState == ARM_IDLE) return;
  if (millis() < armTimer)  return;

  if (armState == ARM_ARMING) {
    relay_motor = true;
    digitalWrite(RELAY_MOTOR, HIGH);
    armState = ARM_IDLE;
    Serial.println(F("[ARM ] MOTOR relay ON — running"));
    debugRelays();
    lastFlash  = 0;
    flashState = false;
    if (gGamepad) {
      gGamepad->setRumble(0x40, 0x40);
      rumbleOffTime = millis() + RUMBLE_MS;
    }
  }
}

// ============================================================
// Bluepad32 callbacks
// ============================================================
void onConnectedGamepad(GamepadPtr gp) {
  debugBanner("CONTROLLER CONNECTED");
  GamepadProperties props = gp->getProperties();
  Serial.print(F("[CONN] BT addr: ")); printBTAddr(props.btaddr);
  gGamepad = gp;
  refreshLED();
  gp->setRumble(0x40, 0x40);
  rumbleOffTime = millis() + RUMBLE_MS;
}

void onDisconnectedGamepad(GamepadPtr gp) {
  debugBanner("CONTROLLER DISCONNECTED — FAILSAFE");
  gGamepad = nullptr;

  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
  digitalWrite(RELAY_ARM,   LOW);
  digitalWrite(RELAY_MOTOR, LOW);
  digitalWrite(RELAY_TURBO, LOW);
  relay_motor   = false;
  relay_turbo   = false;
  armState      = ARM_IDLE;
  piDriveActive = false;
  pi_leftSpeed  = 0;
  pi_rightSpeed = 0;
  scene_driveL  = 0;
  scene_driveR  = 0;

  setLED(COL_OFF);

  Serial.println(F("[SAFE] Tracks stopped. All relays OFF."));
}

// ============================================================
// Motor drive
// ============================================================
void driveMotor(uint8_t pwmPin, uint8_t dirPin, int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, -speed);
  }
}

int stickToSpeed(int axis) {
  int val = -axis;
  if (abs(val) <= DEADZONE) return 0;
  float norm   = (float)val / (float)STICK_MAX;
  float curved = (norm * norm * norm * 0.4f) + (norm * 0.6f);
  curved = constrain(curved, -1.0f, 1.0f);
  return (int)(curved * MAX_SPEED);
}

int calcDpadSpeed(int rawL2) {
  if (rawL2 <= 0) return 0;
  return (int)map(rawL2, 0, 1023, DPAD_SPEED_MIN, MAX_SPEED);
}

// ============================================================
// Pi serial bridge — telemetry TX
// ============================================================
void sendTelemetry() {
  long  tl = enc_ticks_L;
  long  tr = enc_ticks_R;
  float dl = enc_dist_L_m;
  float dr = enc_dist_R_m;

  Serial2.print(F("{\"enc_l\":"));       Serial2.print(tl);
  Serial2.print(F(",\"enc_r\":"));       Serial2.print(tr);
  Serial2.print(F(",\"dist_l\":"));      Serial2.print(dl, 4);
  Serial2.print(F(",\"dist_r\":"));      Serial2.print(dr, 4);
  Serial2.print(F(",\"spd_l\":"));       Serial2.print(enc_L ? enc_L->getSpeed() : 0.0f, 3);
  Serial2.print(F(",\"spd_r\":"));       Serial2.print(enc_R ? enc_R->getSpeed() : 0.0f, 3);
  Serial2.print(F(",\"relay_arm\":"));   Serial2.print(digitalRead(RELAY_ARM));
  Serial2.print(F(",\"relay_motor\":")); Serial2.print(relay_motor ? 1 : 0);
  Serial2.print(F(",\"relay_turbo\":")); Serial2.print(relay_turbo ? 1 : 0);
  Serial2.print(F(",\"connected\":"));   Serial2.print(gGamepad ? 1 : 0);
  Serial2.print(F(",\"mode\":\""));      Serial2.print(modeNames[tankMode]);
  Serial2.print(F("\",\"batt\":"));      Serial2.print(gGamepad ? gGamepad->battery() : 0);
  // PCF8575 mower data
  Serial2.print(F(",\"bat1\":"));        Serial2.print(bat1_level);
  Serial2.print(F(",\"bat1_heat\":"));   Serial2.print(bat1_heat   ? 1 : 0);
  Serial2.print(F(",\"bat2\":"));        Serial2.print(bat2_level);
  Serial2.print(F(",\"bat2_heat\":"));   Serial2.print(bat2_heat   ? 1 : 0);
  Serial2.print(F(",\"mower_err\":"));   Serial2.print(mower_error ? 1 : 0);
  Serial2.print(F(",\"turbo_fb\":"));    Serial2.print(turbo_fb    ? 1 : 0);
  Serial2.print(F(",\"lights\":"));      Serial2.print(pcf_lights_on ? 1 : 0);
  Serial2.println(F("}"));
}

// ============================================================
// Pi serial bridge — command RX
// ============================================================
void parsePiCommand(const String& line) {
  if (line.length() < 5) return;
  Serial.print(F("[PI ] Rx: ")); Serial.println(line);

  if (line.indexOf(F("\"cmd\":\"ping\"")) >= 0) {
    Serial2.println(F("{\"pong\":1}"));
    return;
  }

  if (line.indexOf(F("\"cmd\":\"stop\"")) >= 0) {
    pi_leftSpeed  = 0;
    pi_rightSpeed = 0;
    piDriveActive = false;
    Serial.println(F("[PI ] Stop"));
    return;
  }

  if (line.indexOf(F("\"cmd\":\"drive\"")) >= 0) {
    if (gGamepad != nullptr) {
      Serial.println(F("[PI ] Drive ignored — PS4 connected"));
      return;
    }
    int lIdx = line.indexOf(F("\"l\":"));
    int rIdx = line.indexOf(F("\"r\":"));
    if (lIdx >= 0 && rIdx >= 0) {
      pi_leftSpeed  = constrain(line.substring(lIdx + 4).toInt(), -MAX_SPEED, MAX_SPEED);
      pi_rightSpeed = constrain(line.substring(rIdx + 4).toInt(), -MAX_SPEED, MAX_SPEED);
      piDriveActive = true;
      lastPiCmd     = millis();
      Serial.print(F("[PI ] Drive L=")); Serial.print(pi_leftSpeed);
      Serial.print(F(" R="));           Serial.println(pi_rightSpeed);
    }
    return;
  }

  if (line.indexOf(F("\"cmd\":\"relay\"")) >= 0) {
    int stateIdx = line.indexOf(F("\"state\":"));
    bool state = (stateIdx >= 0) && (line.substring(stateIdx + 8).toInt() == 1);

    if (line.indexOf(F("\"id\":\"motor\"")) >= 0) {
      if (state && !relay_motor && armState == ARM_IDLE) {
        startArmSequence();
        Serial.println(F("[PI ] Motor ON — arm sequence started"));
      } else if (!state && relay_motor) {
        stopMotor();
        Serial.println(F("[PI ] Motor OFF"));
      }
    } else if (line.indexOf(F("\"id\":\"turbo\"")) >= 0) {
      relay_turbo = state;
      digitalWrite(RELAY_TURBO, relay_turbo ? HIGH : LOW);
      Serial.print(F("[PI ] Turbo ")); Serial.println(state ? F("ON") : F("OFF"));
    }
    return;
  }

  if (line.indexOf(F("\"cmd\":\"pcf\"")) >= 0) {
    if (!pcfPresent) { Serial.println(F("[PCF ] Not connected — ignoring")); return; }
    if (line.indexOf(F("\"id\":\"turbo_btn\"")) >= 0) {
      pcf.write(PCF_TURBO_BTN, LOW);
      turboBtnOff = millis() + PCF_BTN_PULSE_MS;
      Serial.println(F("[PCF ] Turbo button press"));
    } else if (line.indexOf(F("\"id\":\"lights_btn\"")) >= 0) {
      pcf.write(PCF_LIGHTS_BTN, LOW);
      lightsBtnOff    = millis() + PCF_BTN_PULSE_MS;
      pcf_lights_on   = !pcf_lights_on;
      Serial.print(F("[PCF ] Lights button press — lights now "));
      Serial.println(pcf_lights_on ? F("ON") : F("OFF"));
    }
    return;
  }

  Serial.print(F("[PI ] Unknown: ")); Serial.println(line);
}

// ============================================================
// Pip-Boy scene engine
// ============================================================
MowerScene computeScene() {
  if (mower_error)                               return SCENE_ERROR;
  if (bat1_heat || bat2_heat)                    return SCENE_HEAT;
  if ((bat1_level > 0 && bat1_level <= 25) ||
      (bat2_level > 0 && bat2_level <= 25))      return SCENE_LOW_BATTERY;
  if (relay_turbo && relay_motor)                return SCENE_TURBO;
  if (relay_motor)                               return SCENE_MOWING;
  if (!gGamepad && piDriveActive)                return SCENE_AUTONOMOUS;
  if (scene_driveL != 0 || scene_driveR != 0)   return SCENE_DRIVING;
  if (gGamepad)                                  return SCENE_READY;
  return SCENE_IDLE;
}

void sendPipBoyUpdate() {
  MowerScene s = computeScene();
  Serial1.print(F("{\"s\":"));  Serial1.print((int)s);
  Serial1.print(F(",\"b1\":")); Serial1.print(bat1_level);
  Serial1.print(F(",\"b2\":")); Serial1.print(bat2_level);
  Serial1.println(F("}"));
}

// ============================================================
// setup()
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(400);
  debugBanner("ESP32-DevKitC-32E TANK MOWER — PHASE 2 v2.3.0");
  Serial.print(F("  Chip    : ")); Serial.println(ESP.getChipModel());
  Serial.print(F("  CPU MHz : ")); Serial.println(ESP.getCpuFreqMHz());
  Serial.print(F("  Heap    : ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
  debugSep();

  // ── WS2812 onboard LED ────────────────────────────────
  FastLED.addLeds<WS2812, WS2812_PIN, GRB>(wsLeds, WS2812_COUNT);
  FastLED.setBrightness(128);
  wsLeds[0] = CRGB::Black;
  FastLED.show();
  Serial.print(F("[INIT] WS2812 LED ready (GPIO ")); Serial.print(WS2812_PIN); Serial.println(F(")"));

  // ── Motor pins ────────────────────────────────────────
  pinMode(MOTOR_A_DIR, OUTPUT); analogWrite(MOTOR_A_PWM, 0);
  pinMode(MOTOR_B_DIR, OUTPUT); analogWrite(MOTOR_B_PWM, 0);
  Serial.println(F("[INIT] Motor pins OK"));

  // ── Relay pins — all LOW on boot ─────────────────────
  pinMode(RELAY_ARM,   OUTPUT); digitalWrite(RELAY_ARM,   LOW);
  pinMode(RELAY_MOTOR, OUTPUT); digitalWrite(RELAY_MOTOR, LOW);
  pinMode(RELAY_TURBO, OUTPUT); digitalWrite(RELAY_TURBO, LOW);
  Serial.println(F("[INIT] Relay pins OK (all LOW)"));

  // ── I2C + PCF8575 ────────────────────────────────────
  Wire.begin(I2C_SDA, I2C_SCL);
  pcfPresent = pcf.begin();
  if (pcfPresent) {
    pcf.write16(0xFFFF); // all pins HIGH: inputs float, outputs inactive
    Serial.println(F("[INIT] PCF8575 ready"));
    Serial.print(F("  SDA=GPIO")); Serial.print(I2C_SDA);
    Serial.print(F("  SCL=GPIO")); Serial.print(I2C_SCL);
    Serial.print(F("  ADDR=0x")); Serial.println(PCF8575_ADDR, HEX);
  } else {
    Serial.println(F("[INIT] PCF8575 NOT found — mower I/O disabled"));
  }

  // ── AS5600 encoders ───────────────────────────────────
  cfg_L.pwmPin = ENCODER_L_PIN;
  enc_L = new EncAS5600(modetype_t::PWM, cfg_L);
  enc_L->begin();
  enc_L->setEncHandler([](EncAS5600 &e) {
    enc_ticks_L  = e.getTicks();
    enc_dist_L_m = (float)enc_ticks_L * DIST_PER_DEG_M;
  });
  enc_L->start();

  cfg_R.pwmPin = ENCODER_R_PIN;
  enc_R = new EncAS5600(modetype_t::PWM, cfg_R);
  enc_R->begin();
  enc_R->setEncHandler([](EncAS5600 &e) {
    enc_ticks_R  = e.getTicks();
    enc_dist_R_m = (float)enc_ticks_R * DIST_PER_DEG_M;
  });
  enc_R->start();

  Serial.println(F("[INIT] AS5600 encoders started"));
  Serial.print(F("  Dist/deg = ")); Serial.print(DIST_PER_DEG_M * 1000.0f, 4); Serial.println(F("mm"));

  // ── Serial1 — Pip-Boy TX ──────────────────────────────
  Serial1.begin(ESP2_BAUD, SERIAL_8N1, ESP2_SERIAL_RX, ESP2_SERIAL_TX);
  Serial.println(F("[INIT] Serial1 (Pip-Boy TX) started"));
  Serial.print(F("  TX=GPIO")); Serial.print(ESP2_SERIAL_TX);
  Serial.print(F("  Pip-Boy RX=GPIO28 (A4 pad)"));
  Serial.print(F("  Baud=")); Serial.println(ESP2_BAUD);

  // ── Pi serial bridge (Serial2) ────────────────────────
  Serial2.begin(PI_BAUD, SERIAL_8N1, PI_SERIAL_RX, PI_SERIAL_TX);
  Serial.println(F("[INIT] Pi serial bridge (Serial2) started"));
  Serial.print(F("  TX=GPIO")); Serial.print(PI_SERIAL_TX);
  Serial.print(F("  RX=GPIO")); Serial.println(PI_SERIAL_RX);

  // ── Bluepad32 ─────────────────────────────────────────
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  debugSep();
  Serial.println(F("[BT ] Ready — hold PS to pair"));
  Serial.println(F("  PS  = toggle mode  (blue=dual / green=single)"));
  Serial.println(F("  R1  = arm → latch mower  (again to stop)"));
  Serial.println(F("  TRI = toggle turbo relay"));
  Serial.println(F("  SQR = toggle mower lights"));
  Serial.println(F("  L2  = set D-pad speed ceiling"));
  Serial.println(F("  L1  = reset D-pad speed"));
  Serial.println(F("  LED: error=amber  heat=yellow  turbo=white  motor=red  mode=solid"));
  debugSep();
}

// ============================================================
// loop()
// ============================================================
void loop() {

  // ── 1. Arm state machine ───────────────────────────────
  updateArmStateMachine();

  // ── 2. PCF8575 input poll (10 Hz) ─────────────────────
  if (pcfPresent && millis() - lastPCFRead >= PCF_READ_MS) {
    lastPCFRead = millis();
    readPCF();
  }

  // ── 3. PCF8575 output button release timers ───────────
  if (pcfPresent && turboBtnOff && millis() >= turboBtnOff) {
    turboBtnOff = 0;
    pcf.write(PCF_TURBO_BTN, HIGH);
  }
  if (pcfPresent && lightsBtnOff && millis() >= lightsBtnOff) {
    lightsBtnOff = 0;
    pcf.write(PCF_LIGHTS_BTN, HIGH);
  }

  // ── 4. Pi bridge — receive commands ───────────────────
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      parsePiCommand(piCmdBuffer);
      piCmdBuffer = "";
    } else if (piCmdBuffer.length() < 256) {
      piCmdBuffer += c;
    }
  }

  // ── 5. Pi drive timeout ────────────────────────────────
  if (piDriveActive && (millis() - lastPiCmd > PI_CMD_TIMEOUT_MS)) {
    piDriveActive = false;
    pi_leftSpeed  = 0;
    pi_rightSpeed = 0;
    Serial.println(F("[PI ] Drive timeout — stopped"));
  }

  // ── 6. Telemetry to Pi at 20 Hz ───────────────────────
  if (millis() - lastTelemSent >= PI_TELEM_MS) {
    lastTelemSent = millis();
    sendTelemetry();
  }

  // ── 7. Timed rumble off ────────────────────────────────
  if (rumbleOffTime && millis() >= rumbleOffTime) {
    rumbleOffTime = 0;
    if (gGamepad && gGamepad->isConnected()) gGamepad->setRumble(0, 0);
  }

  // ── 8. Bluepad32 update ────────────────────────────────
  BP32.update();

  if (gGamepad == nullptr || !gGamepad->isConnected()) {
    if (piDriveActive) {
      driveMotor(MOTOR_A_PWM, MOTOR_A_DIR, pi_leftSpeed);
      driveMotor(MOTOR_B_PWM, MOTOR_B_DIR, pi_rightSpeed);
    }
    updateLED();
    delay(20);
    return;
  }

  GamepadPtr gp = gGamepad;

  // ── PS button → toggle tank mode ─────────────────────
  bool curPS = (gp->miscButtons() & MISC_BUTTON_HOME) != 0;
  if (curPS && !prev_PS) {
    tankMode = (tankMode == MODE_DUAL_STICK) ? MODE_SINGLE_STICK : MODE_DUAL_STICK;
    Serial.print(F("[MODE] -> ")); Serial.println(modeNames[tankMode]);
    refreshLED();
  }
  prev_PS = curPS;

  // ── R1 → arm sequence / stop mower ───────────────────
  bool curR1 = (gp->buttons() & BUTTON_SHOULDER_R) != 0;
  if (curR1 && !prev_R1) {
    if (!relay_motor && armState == ARM_IDLE) {
      Serial.println(F("[BTN ] R1 — starting arm sequence"));
      startArmSequence();
      gp->setRumble(0x20, 0x20);
      rumbleOffTime = millis() + 150;
    } else if (relay_motor) {
      Serial.println(F("[BTN ] R1 — stopping mower"));
      stopMotor();
      gp->setRumble(0x10, 0x10);
      rumbleOffTime = millis() + 80;
    }
  }
  prev_R1 = curR1;

  // ── Triangle → toggle turbo relay ────────────────────
  bool curTRI = (gp->buttons() & BUTTON_Y) != 0;
  if (curTRI && !prev_TRI) {
    relay_turbo = !relay_turbo;
    digitalWrite(RELAY_TURBO, relay_turbo ? HIGH : LOW);
    Serial.print(F("[BTN ] TRI — TURBO ")); Serial.println(relay_turbo ? F("ON") : F("OFF"));
    debugRelays();
    if (relay_turbo) {
      lastFlash  = 0;
      flashState = false;
      gp->setRumble(0x40, 0x40);
      rumbleOffTime = millis() + RUMBLE_MS;
    } else {
      refreshLED();
      gp->setRumble(0x10, 0x10);
      rumbleOffTime = millis() + 80;
    }
  }
  prev_TRI = curTRI;

  // ── L2 analog → D-pad speed ceiling ──────────────────
  {
    int rawL2 = gp->brake();
    if (rawL2 > peakL2) {
      peakL2    = rawL2;
      dpadSpeed = calcDpadSpeed(peakL2);
      Serial.print(F("[L2 ] Peak=")); Serial.print(peakL2);
      Serial.print(F(" dpadSpeed=")); Serial.println(dpadSpeed);
    }
  }

  // ── L1 → reset D-pad speed ceiling ───────────────────
  bool curL1 = (gp->buttons() & BUTTON_SHOULDER_L) != 0;
  if (curL1 && !prev_L1) {
    peakL2    = 0;
    dpadSpeed = 0;
    Serial.println(F("[BTN ] L1 — D-pad speed RESET"));
    gp->setRumble(0x20, 0x00);
    rumbleOffTime = millis() + 100;
  }
  prev_L1 = curL1;

  // ── Square → toggle mower lights ─────────────────────
  bool curSQR = (gp->buttons() & BUTTON_X) != 0;
  if (curSQR && !prev_SQR) {
    if (pcfPresent) {
      pcf.write(PCF_LIGHTS_BTN, LOW);
      lightsBtnOff  = millis() + PCF_BTN_PULSE_MS;
      pcf_lights_on = !pcf_lights_on;
      Serial.print(F("[BTN ] SQR — lights ")); Serial.println(pcf_lights_on ? F("ON") : F("OFF"));
    } else {
      Serial.println(F("[BTN ] SQR — PCF not connected"));
    }
  }
  prev_SQR = curSQR;


  // ── Drive ─────────────────────────────────────────────
  int leftSpeed  = 0;
  int rightSpeed = 0;

  uint8_t dpad = gp->dpad();
  if (dpad & (DPAD_UP | DPAD_DOWN | DPAD_LEFT | DPAD_RIGHT)) {
    if (dpadSpeed > 0) {
      int spd = dpadSpeed;
      if      (dpad & DPAD_UP)    { leftSpeed =  spd; rightSpeed =  spd; }
      else if (dpad & DPAD_DOWN)  { leftSpeed = -spd; rightSpeed = -spd; }
      else if (dpad & DPAD_LEFT)  { leftSpeed = -spd; rightSpeed =  spd; }
      else if (dpad & DPAD_RIGHT) { leftSpeed =  spd; rightSpeed = -spd; }
    }
  } else {
    if (tankMode == MODE_DUAL_STICK) {
      leftSpeed  = stickToSpeed(gp->axisY());
      rightSpeed = stickToSpeed(gp->axisRY());
    } else {
      int throttle = stickToSpeed(gp->axisY());
      int turn     = stickToSpeed(gp->axisX());
      leftSpeed    = constrain(throttle + turn, -MAX_SPEED, MAX_SPEED);
      rightSpeed   = constrain(throttle - turn, -MAX_SPEED, MAX_SPEED);
    }
  }

  driveMotor(MOTOR_A_PWM, MOTOR_A_DIR, leftSpeed);
  driveMotor(MOTOR_B_PWM, MOTOR_B_DIR, rightSpeed);
  scene_driveL = leftSpeed;
  scene_driveR = rightSpeed;

  if (leftSpeed != prev_leftSpeed || rightSpeed != prev_rightSpeed) {
    Serial.print(F("[DRV ] L=")); Serial.print(leftSpeed);
    Serial.print(F(" R="));      Serial.println(rightSpeed);
    prev_leftSpeed  = leftSpeed;
    prev_rightSpeed = rightSpeed;
  }

  // ── Periodic status dump ──────────────────────────────
  if (millis() - lastStatusPrint >= STATUS_INTERVAL_MS) {
    lastStatusPrint = millis();
    Serial.print(F("[ENC ] L=")); Serial.print(enc_ticks_L);
    Serial.print(F(" ticks ")); Serial.print(enc_dist_L_m, 3);
    Serial.print(F("m  R=")); Serial.print(enc_ticks_R);
    Serial.print(F(" ticks ")); Serial.print(enc_dist_R_m, 3); Serial.println(F("m"));
    Serial.print(F("[STCK] LX=")); Serial.print(gp->axisX());
    Serial.print(F(" LY=")); Serial.print(gp->axisY());
    Serial.print(F(" RY=")); Serial.print(gp->axisRY());
    Serial.print(F(" | L2=")); Serial.print(gp->brake());
    Serial.print(F(" | Batt=")); Serial.print(gp->battery()); Serial.println(F("%"));
    debugFullStatus();
  }

  // ── Pip-Boy scene update at 500ms ─────────────────────
  if (millis() - lastPipBoySent >= PIPBOY_INTERVAL_MS) {
    lastPipBoySent = millis();
    sendPipBoyUpdate();
  }

  updateLED();
  delay(20);
}
