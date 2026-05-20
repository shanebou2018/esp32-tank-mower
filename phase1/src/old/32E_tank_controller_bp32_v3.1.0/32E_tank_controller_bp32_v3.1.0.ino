// ============================================================
// ESP32 Tank Mower — Phase 2 Firmware  v3.1.0
// Board: ESP32-DevKitC-32E (ESP32-WROOM-32E)
// Based on v3.0.0
//
// v3.1.0 fixes:
//   - MAX_SPEED reverted to 127 (50% duty cycle ceiling) to
//     prevent 5V rail sag under BT TX peaks causing dropouts
//   - EncAS5600::getSpeed() moved to Core 1 (publishCtrl2IO)
//     and passed via Ctrl2IO -- eliminates data race with ISR
//   - ioTask stack increased 4096 -> 6144 (320-byte snprintf
//     buf + PCF/Serial overhead was overflowing 4096)
//   - Single-stick mode turn mixing signs corrected:
//     left = throttle + turn, right = throttle - turn
//   - battery() thresholds corrected to 0-255 scale
//     (API returns 0=unknown, 1=empty, 255=full -- not %)
//   - setRumble() replaced with playDualRumble() throughout
//     (setRumble is deprecated in Bluepad32 v4.1.0)
//   - rumbleOffTime machinery removed -- playDualRumble is
//     self-terminating; no manual stop needed
//   - Drive smoothing filter removed -- added lag with no
//     benefit for MDDS30 direct-drive
//   - Switched to Cytron SmartDriveDuo Serial Simplified protocol
//     implemented directly on Serial2 (hardware UART GPIO 25)
//     avoids SoftwareSerial incompatibility on ESP32
//   - Motor output: PWM_DIR → Serial Simplified (1 wire, GPIO 25)
//     GPIO 25 → MDDS30 IN1 (serial RX). AN1/AN2/IN2 unused.
//     Motor commands throttled to 20 Hz to stay within baud budget
//     Pi bridge moved to Serial1 (shared with Pip-Boy via GPIO matrix)
//     Serial2 exclusively owns MDDS30 motor serial
//   - Serial.setTxBufferSize(1024) — async UART TX so debugFullStatus()
//     (~700 bytes every 2s) never blocks loop() / BP32.update()
//     (default tx_buffer_size=0 = synchronous writes = ~61ms stall)
//   - esp_wifi_stop() after BP32.setup() — WiFi shares the 2.4GHz radio;
//     stopping it eliminates coexistence jitter and improves range
//   - esp_bredr_tx_power_set(P9,P9) — Classic BT TX raised +3dBm→+9dBm
//   - vTaskDelay(1) at end of loop() — yields to btstack service tasks
//
// MDDS30 DIP switches (your current: 11001111)
//   SW1=ON  SW2=ON  → Serial mode
//   SW3=OFF          → Serial Simplified
//   SW4=OFF SW5=ON  → "Independent Right" only ← CHANGE to SW4=ON SW5=ON ("Independent Both")
//   SW6=ON  SW7=ON  SW8=ON → 115200 baud ✓
//   Correct setting: 11011111
//   Connect: GPIO 25 → MDDS30 IN1 only. Disconnect AN1/AN2/IN2.
//
// v3.0.0 changes (FreeRTOS split, PCF8575, Pip-Boy):
//   see v3.0.0 header.
//
// Hardware, PCF8575 pin map, and JSON protocol: see v2.5.0 header.
// ============================================================

#include <Bluepad32.h>
#include <EncAS5600.h>
#include <Wire.h>
#include <PCF8575.h>
#include "rom/gpio.h"   // gpio_matrix_out() — ROM fn, no macro conflicts
#include <esp_wifi.h>   // esp_wifi_stop() — silence the shared 2.4GHz radio
#include <esp_bt.h>     // esp_bredr_tx_power_set() — Classic BT TX power
// U1TXD_OUT_IDX = 23 on all original ESP32 silicon.

// ── Pin definitions ───────────────────────────────────────────
// Motor serial: GPIO 25 → MDDS30 IN1 (TX-only, Serial Simplified).
// GPIO 26, 27, 14 (former PWM/DIR) unused — disconnect from MDDS30.
#define MDDS30_TX_PIN  25
#define MDDS30_BAUD    115200   // DIP SW6-8=111 → 115200 baud

#define RELAY_ARM      32
#define RELAY_MOTOR    33
#define RELAY_TURBO    15

#define ENCODER_L_PIN  34
#define ENCODER_R_PIN  35

#define FLASH_MS       300

// ── I2C / PCF8575 ────────────────────────────────────────────
#define I2C_SDA        21
#define I2C_SCL        22
#define PCF8575_ADDR   0x20

PCF8575 pcf(PCF8575_ADDR);
bool    pcfPresent = false;

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
#define PCF_TURBO_BTN  12
#define PCF_LIGHTS_BTN 13

#define PCF_READ_MS      100
#define PCF_BTN_PULSE_MS 200

// ── Serial1 — Pi bridge + Pip-Boy display (shared via GPIO matrix) ──
// UART1 TX primary pin: GPIO 17 → Pi RX.   UART1 RX: GPIO 13 ← Pi TX.
// gpio_matrix_out() in setup() also routes UART1 TX to GPIO 4 (Pip-Boy).
// Both Pi and Pip-Boy receive every Serial1 line.
//   Pi firmware  : parse telemetry JSON, ignore {"s":…} scene lines.
//   Pip-Boy fw   : parse {"s":…} scene lines, ignore telemetry lines.
#define ESP2_SERIAL_TX  4       // Pip-Boy RX — routed from UART1 TX via GPIO matrix
#define ESP2_SERIAL_RX  5       // (reserved, not currently used)
#define ESP2_BAUD       115200

// ── Pip-Boy scene engine ──────────────────────────────────────
enum MowerScene {
  SCENE_IDLE        = 0,
  SCENE_READY       = 1,
  SCENE_DRIVING     = 2,
  SCENE_MOWING      = 3,
  SCENE_TURBO       = 4,
  SCENE_AUTONOMOUS  = 5,
  SCENE_LOW_BATTERY = 6,
  SCENE_HEAT        = 7,
  SCENE_ERROR       = 8,
};
#define PIPBOY_INTERVAL_MS 500

// ── Pi serial bridge ──────────────────────────────────────────
#define PI_SERIAL_TX    17      // UART1 TX primary pin → Pi RX
#define PI_SERIAL_RX    13      // UART1 RX ← Pi TX
#define PI_BAUD         115200
#define PI_TELEM_MS     50
#define PI_CMD_TIMEOUT_MS  500

// ── Odometry ─────────────────────────────────────────────────
#define WHEEL_DIAMETER_M  0.254f
#define GEAR_RATIO        16.0f
#define DIST_PER_DEG_M    (3.14159265f * WHEEL_DIAMETER_M / (GEAR_RATIO * 360.0f))

// ── Tuning ───────────────────────────────────────────────────
#define DEADZONE        20
#define MAX_SPEED       100     // Cytron SmartDriveDuo serial simplified range is -100..+100
#define MIN_MOTOR_SPEED 0       // Minimum output sent to MDDS30 for any non-zero input.
                                // Set to the lowest speed at which your motors reliably spin
                                // (e.g. 15).  0 = no floor — scaleToMotor is a straight ramp.
#define STICK_MAX       511
#define DPAD_SPEED_MIN  0       // D-pad min matches motor floor
#define ARM_PULSE_MS    500
#define RUMBLE_MS       300
#define STATUS_INTERVAL_MS 2000

// ── Stick curve ───────────────────────────────────────────────
// Blend between pure cubic (1.0) and pure linear (0.0).
//   1.0 = maximum expo: very slow near centre, snappy at edges
//   0.0 = linear:       direct 1:1 stick-to-speed
//   0.25 = light expo:  decisive response, still controllable at low speed
// Change this one number to retune — do NOT change stickToSpeed().
#define EXPO_BLEND      0.25f

// ── Motor direction ───────────────────────────────────────────
// Set to +1 (normal) or -1 (reverse) for each motor.
// Flip here instead of rewiring.  All drive math stays positive=forward.
#define MOTOR_L_DIR     (+1)
#define MOTOR_R_DIR     (+1)

// ── Types ─────────────────────────────────────────────────────
enum TankMode  { MODE_DUAL_STICK, MODE_SINGLE_STICK };
enum ArmState  { ARM_IDLE, ARM_ARMING };
struct Colour  { uint8_t r, g, b; };

const char*  modeNames[] = { "DUAL", "SINGLE" };
const Colour COL_DUAL    = {   0,   0, 255 };
const Colour COL_SINGLE  = {   0, 255,   0 };
const Colour COL_RED     = { 255,   0,   0 };
const Colour COL_WHITE   = { 255, 255, 255 };

// ============================================================
// RTOS shared state
// ============================================================

// Core 1 -> IO task
struct Ctrl2IO {
  bool     relay_motor;
  bool     relay_turbo;
  bool     connected;
  TankMode tankMode;
  int      batt;
  int      driveL;
  int      driveR;
  bool     piDriveActive;
  long     enc_ticks_L;
  long     enc_ticks_R;
  float    enc_dist_L;
  float    enc_dist_R;
  float    enc_spd_L;   // snapshotted on Core 1 — safe from ISR data race
  float    enc_spd_R;
};

// IO task -> Core 1
struct IO2Ctrl {
  bool          piDriveActive;
  int           pi_leftSpeed;
  int           pi_rightSpeed;
  unsigned long lastPiCmd;

  uint8_t bat1_level;
  uint8_t bat2_level;
  bool    bat1_heat;
  bool    bat2_heat;
  bool    mower_error;
  bool    turbo_fb;
  bool    pcf_lights_on;

  bool req_arm_start;
  bool req_motor_stop;
  bool req_turbo_set;
  bool req_turbo_val;
  bool req_lights_btn;
  bool req_rumble;
};

// ── Motor serial output ───────────────────────────────────────
// Cytron SmartDriveDuo Serial Simplified protocol, implemented directly
// on Serial2 (hardware UART) — the Cytron library uses SoftwareSerial
// which does not compile on ESP32.
// Speed range -100..+100.  Two bytes per command: L byte then R byte.
//   Motor L fwd: 0x00 | map(spd,  0,100, 0,63)   rev: 0x40 | map(|spd|,  0,100, 0,63)
//   Motor R fwd: 0xC0 | map(spd,  0,100, 0,63)   rev: 0x80 | map(|spd|,  0,100, 0,63)
inline void sendMotorBytes(int leftSpd, int rightSpd) {
  // Apply per-motor direction correction before encoding.
  // Flip MOTOR_L_DIR / MOTOR_R_DIR in the defines to reverse a motor without rewiring.
  leftSpd  = constrain(leftSpd  * MOTOR_L_DIR, -100, 100);
  rightSpd = constrain(rightSpd * MOTOR_R_DIR, -100, 100);
  uint8_t lByte = (leftSpd  >= 0)
    ? (uint8_t)(0x00 | map( leftSpd, 0, 100, 0, 63))
    : (uint8_t)(0x40 | map(-leftSpd, 0, 100, 0, 63));
  uint8_t rByte = (rightSpd >= 0)
    ? (uint8_t)(0xC0 | map( rightSpd, 0, 100, 0, 63))
    : (uint8_t)(0x80 | map(-rightSpd, 0, 100, 0, 63));
  Serial2.write(lByte);
  Serial2.write(rByte);
}

static SemaphoreHandle_t g_mutex_c2i;
static SemaphoreHandle_t g_mutex_i2c;
static Ctrl2IO  g_c2i = {};
static IO2Ctrl  g_i2c = {};

// ── Encoder state — volatile, written by ISR on Core 1 ───────
volatile long  enc_ticks_L  = 0;
volatile long  enc_ticks_R  = 0;
volatile float enc_dist_L_m = 0.0f;
volatile float enc_dist_R_m = 0.0f;

as5600config_t cfg_L, cfg_R;
EncAS5600 *enc_L = nullptr;
EncAS5600 *enc_R = nullptr;

// ============================================================
// Core 1 — local state (never accessed by IO task)
// ============================================================
GamepadPtr    gGamepad     = nullptr;
TankMode      tankMode     = MODE_DUAL_STICK;
ArmState      armState     = ARM_IDLE;
bool          relay_motor  = false;
bool          relay_turbo  = false;
unsigned long armTimer     = 0;

bool prev_PS    = false;
bool prev_R1    = false;
bool prev_TRI   = false;
bool prev_L1    = false;
bool prev_SQR   = false;
bool prev_CROSS = false;

bool showBatt  = false;
int  peakL2    = 0;
int  dpadSpeed = 0;

unsigned long lastFlash  = 0;
bool          flashState = false;

int prev_leftSpeed  = 9999;
int prev_rightSpeed = 9999;

unsigned long lastStatusPrint = 0;

// ============================================================
// Debug helpers — Core 1 only
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
  IO2Ctrl snap;
  xSemaphoreTake(g_mutex_i2c, portMAX_DELAY);
  snap = g_i2c;
  xSemaphoreGive(g_mutex_i2c);

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
  Serial.print(F("  Pi drive  : ")); Serial.println(snap.piDriveActive ? "ACTIVE" : "idle");
  Serial.print(F("  Bat1      : ")); Serial.print(snap.bat1_level);
  Serial.print(F("% heat=")); Serial.println(snap.bat1_heat ? "YES" : "no");
  Serial.print(F("  Bat2      : ")); Serial.print(snap.bat2_level);
  Serial.print(F("% heat=")); Serial.println(snap.bat2_heat ? "YES" : "no");
  Serial.print(F("  Mower err : ")); Serial.println(snap.mower_error  ? "YES" : "no");
  Serial.print(F("  Turbo FB  : ")); Serial.println(snap.turbo_fb     ? "ON"  : "off");
  Serial.print(F("  Lights    : ")); Serial.println(snap.pcf_lights_on ? "ON"  : "off");
  Serial.print(F("  Free heap : ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
  debugSep();
}

// ============================================================
// LED helpers — Core 1 only
// ============================================================
void setLED(Colour c) {
  if (gGamepad) gGamepad->setColorLED(c.r, c.g, c.b);
}

Colour modeColour() {
  return tankMode == MODE_DUAL_STICK ? COL_DUAL : COL_SINGLE;
}

void updateLED(bool motor, bool turbo) {
  if (!gGamepad) return;
  if (showBatt) return;
  if (!motor && !turbo) return;
  unsigned long now = millis();
  if (now - lastFlash < FLASH_MS) return;
  lastFlash  = now;
  flashState = !flashState;
  if (turbo) {
    setLED(flashState ? COL_WHITE : modeColour());
  } else {
    setLED(flashState ? COL_RED : modeColour());
  }
}

// Converts a raw PS4 axis value (-511..+511) to a motor speed (-100..+100).
// PS4 Y-axes are negative-up, so val = -axis to make "up" produce positive speed.
// For X-axis (single-stick turn), pass -axisX() at the call site so rightward
// push produces a positive turn value (left wheel faster = robot turns right).
int stickToSpeed(int axis) {
  int val = -axis;
  if (abs(val) <= DEADZONE) return 0;
  float norm   = (float)val / (float)STICK_MAX;
  float curved = (norm * norm * norm * EXPO_BLEND) + (norm * (1.0f - EXPO_BLEND));
  curved = constrain(curved, -1.0f, 1.0f);
  return (int)(curved * MAX_SPEED);
}

int calcDpadSpeed(int rawL2) {
  if (rawL2 <= 0) return 0;
  return (int)map(rawL2, 0, 1023, DPAD_SPEED_MIN, MAX_SPEED);
}

// Rescales any non-zero speed into MIN_MOTOR_SPEED..MAX_SPEED so
// the MDDS30 always receives a duty cycle it can act on.
// Zero passes through unchanged — motor stops cleanly.
// NOTE: map from 0 (not 1) so speed=1 produces MIN_MOTOR_SPEED, not 0.
int scaleToMotor(int speed) {
  if (speed == 0) return 0;
  int mag = (int)map(abs(speed), 0, MAX_SPEED, MIN_MOTOR_SPEED, MAX_SPEED);
  return (speed > 0) ? mag : -mag;
}

// ============================================================
// Arm state machine — Core 1 only
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
  if (!relay_turbo) setLED(modeColour());
  Serial.println(F("[ARM ] MOTOR + ARM both LOW — mower stopped"));
  debugRelays();
}

void updateArmStateMachine() {
  if (armState == ARM_IDLE) return;
  if (millis() < armTimer)  return;

  if (armState == ARM_ARMING) {
    relay_motor = true;
    digitalWrite(RELAY_MOTOR, HIGH);
    armState   = ARM_IDLE;
    lastFlash  = 0;
    flashState = false;
    Serial.println(F("[ARM ] MOTOR relay ON — running"));
    debugRelays();
    if (gGamepad) gGamepad->playDualRumble(0, RUMBLE_MS, 0x40, 0x40);
  }
}

// ============================================================
// Bluepad32 callbacks (invoked from BP32.update() on Core 1)
// ============================================================
void onConnectedGamepad(GamepadPtr gp) {
  debugBanner("CONTROLLER CONNECTED");
  GamepadProperties props = gp->getProperties();
  Serial.print(F("[CONN] BT addr: ")); printBTAddr(props.btaddr);
  gGamepad = gp;
  gp->setColorLED(COL_DUAL.r, COL_DUAL.g, COL_DUAL.b);
  gp->playDualRumble(0, RUMBLE_MS, 0x40, 0x40);
  Serial.println(F("[LED ] BLUE (dual stick)"));
}

void onDisconnectedGamepad(GamepadPtr gp) {
  debugBanner("CONTROLLER DISCONNECTED — FAILSAFE");
  gGamepad = nullptr;

  sendMotorBytes(0, 0);
  digitalWrite(RELAY_ARM,   LOW);
  digitalWrite(RELAY_MOTOR, LOW);
  digitalWrite(RELAY_TURBO, LOW);
  relay_motor = false;
  relay_turbo = false;
  armState    = ARM_IDLE;

  xSemaphoreTake(g_mutex_i2c, portMAX_DELAY);
  g_i2c.piDriveActive  = false;
  g_i2c.pi_leftSpeed   = 0;
  g_i2c.pi_rightSpeed  = 0;
  g_i2c.req_lights_btn = false;
  xSemaphoreGive(g_mutex_i2c);

  Serial.println(F("[SAFE] Tracks stopped. All relays OFF."));
}

// ============================================================
// Core 1 — publish snapshot to g_c2i each loop iteration
// ============================================================
static void publishCtrl2IO(int driveL, int driveR, bool piDriveActive) {
  noInterrupts();
  long  tl = enc_ticks_L;
  long  tr = enc_ticks_R;
  float dl = enc_dist_L_m;
  float dr = enc_dist_R_m;
  interrupts();

  // getSpeed() called here on Core 1 where the ISRs are attached — safe
  float sl = enc_L ? enc_L->getSpeed() : 0.0f;
  float sr = enc_R ? enc_R->getSpeed() : 0.0f;

  if (xSemaphoreTake(g_mutex_c2i, 0) != pdTRUE) return;
  g_c2i.relay_motor   = relay_motor;
  g_c2i.relay_turbo   = relay_turbo;
  g_c2i.connected     = (gGamepad != nullptr);
  g_c2i.tankMode      = tankMode;
  g_c2i.batt          = gGamepad ? gGamepad->battery() : 0;
  g_c2i.driveL        = driveL;
  g_c2i.driveR        = driveR;
  g_c2i.piDriveActive = piDriveActive;
  g_c2i.enc_ticks_L   = tl;
  g_c2i.enc_ticks_R   = tr;
  g_c2i.enc_dist_L    = dl;
  g_c2i.enc_dist_R    = dr;
  g_c2i.enc_spd_L     = sl;
  g_c2i.enc_spd_R     = sr;
  xSemaphoreGive(g_mutex_c2i);
}

// ============================================================
// IO task helpers — PCF8575 (Core 0 only)
// ============================================================
static uint8_t decodeBatLevel(uint16_t pins, uint8_t p25, uint8_t p50,
                               uint8_t p75, uint8_t p100) {
  bool b25  = pins & (1u << p25);
  bool b50  = pins & (1u << p50);
  bool b75  = pins & (1u << p75);
  bool b100 = pins & (1u << p100);
  if (!b25 && !b50 && !b75 && !b100) return 0xFF;
  if (b100) return 100;
  if (b75)  return 75;
  if (b50)  return 50;
  return 25;
}

static void ioReadPCF(uint8_t& bat1, uint8_t& bat2,
                      bool& heat1, bool& heat2,
                      bool& mow_err, bool& turbo,
                      bool& prev_err, bool& rumble_out) {
  uint16_t pins = pcf.read16();

  uint8_t lvl1 = decodeBatLevel(pins, PCF_BAT1_25, PCF_BAT1_50,
                                        PCF_BAT1_75, PCF_BAT1_100);
  if (lvl1 != 0xFF) bat1 = lvl1;

  uint8_t lvl2 = decodeBatLevel(pins, PCF_BAT2_25, PCF_BAT2_50,
                                        PCF_BAT2_75, PCF_BAT2_100);
  if (lvl2 != 0xFF) bat2 = lvl2;

  heat1   = pins & (1u << PCF_BAT1_HEAT);
  heat2   = pins & (1u << PCF_BAT2_HEAT);
  mow_err = pins & (1u << PCF_MOWER_ERR);
  turbo   = pins & (1u << PCF_TURBO_FB);

  if (mow_err && !prev_err) rumble_out = true;
  prev_err = mow_err;
}

// ============================================================
// IO task helpers — Pi serial bridge (Core 0 only)
// ============================================================
static void ioParsePiCommand(const char* line, bool& piDriveActive,
                              int& piL, int& piR, unsigned long& lastCmd) {
  if (strlen(line) < 5) return;

  if (strstr(line, "\"cmd\":\"ping\"")) {
    Serial1.println(F("{\"pong\":1}"));
    return;
  }

  if (strstr(line, "\"cmd\":\"stop\"")) {
    piL = piR = 0;
    piDriveActive = false;
    return;
  }

  if (strstr(line, "\"cmd\":\"drive\"")) {
    Ctrl2IO snap;
    xSemaphoreTake(g_mutex_c2i, portMAX_DELAY);
    snap = g_c2i;
    xSemaphoreGive(g_mutex_c2i);
    if (snap.connected) return;

    const char* lPtr = strstr(line, "\"l\":");
    const char* rPtr = strstr(line, "\"r\":");
    if (lPtr && rPtr) {
      piL = constrain(atoi(lPtr + 4), -MAX_SPEED, MAX_SPEED);
      piR = constrain(atoi(rPtr + 4), -MAX_SPEED, MAX_SPEED);
      piDriveActive = true;
      lastCmd = millis();
    }
    return;
  }

  if (strstr(line, "\"cmd\":\"relay\"")) {
    const char* statePtr = strstr(line, "\"state\":");
    bool state = statePtr && (atoi(statePtr + 8) == 1);

    xSemaphoreTake(g_mutex_i2c, portMAX_DELAY);
    if (strstr(line, "\"id\":\"motor\"")) {
      Ctrl2IO csnap;
      xSemaphoreGive(g_mutex_i2c);
      xSemaphoreTake(g_mutex_c2i, portMAX_DELAY);
      csnap = g_c2i;
      xSemaphoreGive(g_mutex_c2i);

      xSemaphoreTake(g_mutex_i2c, portMAX_DELAY);
      if (state && !csnap.relay_motor) {
        g_i2c.req_arm_start = true;
      } else if (!state && csnap.relay_motor) {
        g_i2c.req_motor_stop = true;
      }
      xSemaphoreGive(g_mutex_i2c);
    } else if (strstr(line, "\"id\":\"turbo\"")) {
      g_i2c.req_turbo_val = state;
      g_i2c.req_turbo_set = true;
      xSemaphoreGive(g_mutex_i2c);
    } else {
      xSemaphoreGive(g_mutex_i2c);
    }
    return;
  }

  if (strstr(line, "\"cmd\":\"pcf\"")) {
    if (!pcfPresent) return;
    if (strstr(line, "\"id\":\"turbo_btn\"")) {
      pcf.write(PCF_TURBO_BTN, LOW);
    } else if (strstr(line, "\"id\":\"lights_btn\"")) {
      pcf.write(PCF_LIGHTS_BTN, LOW);
    }
  }
}

// ============================================================
// IO task helpers — Pip-Boy scene engine (Core 0 only)
// ============================================================
static MowerScene ioComputeScene(const Ctrl2IO& c, const IO2Ctrl& io) {
  if (io.mower_error)                               return SCENE_ERROR;
  if (io.bat1_heat || io.bat2_heat)                 return SCENE_HEAT;
  if ((io.bat1_level > 0 && io.bat1_level <= 25) ||
      (io.bat2_level > 0 && io.bat2_level <= 25))   return SCENE_LOW_BATTERY;
  if (c.relay_turbo && c.relay_motor)               return SCENE_TURBO;
  if (c.relay_motor)                                return SCENE_MOWING;
  if (!c.connected && io.piDriveActive)             return SCENE_AUTONOMOUS;
  if (c.driveL != 0 || c.driveR != 0)              return SCENE_DRIVING;
  if (c.connected)                                  return SCENE_READY;
  return SCENE_IDLE;
}

static void ioSendPipBoyUpdate(const Ctrl2IO& c, const IO2Ctrl& io) {
  MowerScene s = ioComputeScene(c, io);
  Serial1.print(F("{\"s\":"));  Serial1.print((int)s);
  Serial1.print(F(",\"b1\":")); Serial1.print(io.bat1_level);
  Serial1.print(F(",\"b2\":")); Serial1.print(io.bat2_level);
  Serial1.println(F("}"));
}

// ============================================================
// IO task helpers — Pi telemetry TX (Core 0 only)
// ============================================================
static void ioSendTelemetry(const Ctrl2IO& c, const IO2Ctrl& io) {
  char buf[320];
  snprintf(buf, sizeof(buf),
    "{\"enc_l\":%ld,\"enc_r\":%ld"
    ",\"dist_l\":%.4f,\"dist_r\":%.4f"
    ",\"spd_l\":%.3f,\"spd_r\":%.3f"
    ",\"relay_arm\":%d,\"relay_motor\":%d,\"relay_turbo\":%d"
    ",\"connected\":%d,\"mode\":\"%s\",\"batt\":%d"
    ",\"bat1\":%d,\"bat1_heat\":%d"
    ",\"bat2\":%d,\"bat2_heat\":%d"
    ",\"mower_err\":%d,\"turbo_fb\":%d,\"lights\":%d}",
    c.enc_ticks_L, c.enc_ticks_R,
    c.enc_dist_L, c.enc_dist_R,
    c.enc_spd_L, c.enc_spd_R,
    digitalRead(RELAY_ARM), c.relay_motor ? 1 : 0, c.relay_turbo ? 1 : 0,
    c.connected ? 1 : 0, modeNames[c.tankMode], c.batt,
    io.bat1_level, io.bat1_heat ? 1 : 0,
    io.bat2_level, io.bat2_heat ? 1 : 0,
    io.mower_error ? 1 : 0, io.turbo_fb ? 1 : 0, io.pcf_lights_on ? 1 : 0);
  Serial1.println(buf);   // Serial1 → Pi (GPIO 17) + Pip-Boy (GPIO 4 via GPIO matrix)
}

// ============================================================
// IO task — runs on Core 0
// ============================================================
static void ioTask(void*) {
  char          piCmdBuf[256];
  uint8_t       piCmdLen       = 0;
  bool          piDriveActive  = false;
  int           pi_leftSpeed   = 0;
  int           pi_rightSpeed  = 0;
  unsigned long lastPiCmd      = 0;
  unsigned long lastTelemSent  = 0;
  unsigned long lastPipBoySent = 0;
  unsigned long lastPCFRead    = 0;
  unsigned long turboBtnOff    = 0;
  unsigned long lightsBtnOff   = 0;
  bool          prev_mow_err   = false;
  uint8_t       bat1           = 0, bat2 = 0;
  bool          heat1          = false, heat2 = false;
  bool          mow_err        = false, turbo_fb = false;
  bool          lights_on      = false;

  for (;;) {
    unsigned long now = millis();

    // ── PCF8575 input poll (10 Hz) ───────────────────────
    if (pcfPresent && now - lastPCFRead >= PCF_READ_MS) {
      lastPCFRead = now;
      bool do_rumble = false;
      ioReadPCF(bat1, bat2, heat1, heat2, mow_err, turbo_fb,
                prev_mow_err, do_rumble);

      xSemaphoreTake(g_mutex_i2c, portMAX_DELAY);
      g_i2c.bat1_level  = bat1;
      g_i2c.bat2_level  = bat2;
      g_i2c.bat1_heat   = heat1;
      g_i2c.bat2_heat   = heat2;
      g_i2c.mower_error = mow_err;
      g_i2c.turbo_fb    = turbo_fb;
      if (do_rumble) g_i2c.req_rumble = true;
      xSemaphoreGive(g_mutex_i2c);
    }

    // ── PCF8575 button release timers ────────────────────
    if (pcfPresent) {
      now = millis();
      if (turboBtnOff && now >= turboBtnOff) {
        turboBtnOff = 0;
        pcf.write(PCF_TURBO_BTN, HIGH);
      }
      if (lightsBtnOff && now >= lightsBtnOff) {
        lightsBtnOff = 0;
        pcf.write(PCF_LIGHTS_BTN, HIGH);
      }

      xSemaphoreTake(g_mutex_i2c, portMAX_DELAY);
      bool do_lights = g_i2c.req_lights_btn;
      if (do_lights) g_i2c.req_lights_btn = false;
      xSemaphoreGive(g_mutex_i2c);

      if (do_lights && !lightsBtnOff) {
        pcf.write(PCF_LIGHTS_BTN, LOW);
        lightsBtnOff = millis() + PCF_BTN_PULSE_MS;
        lights_on = !lights_on;
        xSemaphoreTake(g_mutex_i2c, portMAX_DELAY);
        g_i2c.pcf_lights_on = lights_on;
        xSemaphoreGive(g_mutex_i2c);
      }
    }

    // ── Pi bridge — receive commands (Serial1 RX, GPIO 13) ──────
    while (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') {
        piCmdBuf[piCmdLen] = '\0';
        ioParsePiCommand(piCmdBuf, piDriveActive, pi_leftSpeed,
                         pi_rightSpeed, lastPiCmd);
        piCmdLen = 0;

        xSemaphoreTake(g_mutex_i2c, portMAX_DELAY);
        g_i2c.piDriveActive = piDriveActive;
        g_i2c.pi_leftSpeed  = pi_leftSpeed;
        g_i2c.pi_rightSpeed = pi_rightSpeed;
        g_i2c.lastPiCmd     = lastPiCmd;
        xSemaphoreGive(g_mutex_i2c);
      } else if (piCmdLen < sizeof(piCmdBuf) - 1) {
        piCmdBuf[piCmdLen++] = c;
      }
    }

    // ── Pi drive timeout ──────────────────────────────────
    if (piDriveActive && (millis() - lastPiCmd > PI_CMD_TIMEOUT_MS)) {
      piDriveActive = false;
      pi_leftSpeed  = 0;
      pi_rightSpeed = 0;
      xSemaphoreTake(g_mutex_i2c, portMAX_DELAY);
      g_i2c.piDriveActive = false;
      g_i2c.pi_leftSpeed  = 0;
      g_i2c.pi_rightSpeed = 0;
      xSemaphoreGive(g_mutex_i2c);
    }

    // ── Telemetry to Pi at 20 Hz ──────────────────────────
    now = millis();
    if (now - lastTelemSent >= PI_TELEM_MS) {
      lastTelemSent = now;

      Ctrl2IO csnap;
      IO2Ctrl isnap;
      xSemaphoreTake(g_mutex_c2i, portMAX_DELAY);
      csnap = g_c2i;
      xSemaphoreGive(g_mutex_c2i);
      xSemaphoreTake(g_mutex_i2c, portMAX_DELAY);
      isnap = g_i2c;
      xSemaphoreGive(g_mutex_i2c);

      ioSendTelemetry(csnap, isnap);
    }

    // ── Pip-Boy scene update at 500ms ─────────────────────
    now = millis();
    if (now - lastPipBoySent >= PIPBOY_INTERVAL_MS) {
      lastPipBoySent = now;

      Ctrl2IO csnap;
      IO2Ctrl isnap;
      xSemaphoreTake(g_mutex_c2i, portMAX_DELAY);
      csnap = g_c2i;
      xSemaphoreGive(g_mutex_c2i);
      xSemaphoreTake(g_mutex_i2c, portMAX_DELAY);
      isnap = g_i2c;
      xSemaphoreGive(g_mutex_i2c);

      ioSendPipBoyUpdate(csnap, isnap);
    }

    vTaskDelay(1);
  }
}

// ============================================================
// setup()
// ============================================================
void setup() {
  // ── Drive relay outputs LOW before anything else ─────
  // Motor serial (GPIO 25) is initialised below with Serial2.begin().
  pinMode(RELAY_ARM,   OUTPUT); digitalWrite(RELAY_ARM,   LOW);
  pinMode(RELAY_MOTOR, OUTPUT); digitalWrite(RELAY_MOTOR, LOW);
  pinMode(RELAY_TURBO, OUTPUT); digitalWrite(RELAY_TURBO, LOW);

  // Async TX buffer MUST be set before begin().
  // Default tx_buffer_size=0 → synchronous writes → debugFullStatus()
  // blocks loop() for ~61ms at 115200 baud (700 bytes / 11520 Bps).
  // BP32.update() must fire every <10ms to keep the DS4 alive;
  // a 61ms gap is enough to trigger controller disconnect.
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(400);
  debugBanner("ESP32-DevKitC-32E TANK MOWER — PHASE 2 v3.1.0");
  Serial.print(F("  Chip    : ")); Serial.println(ESP.getChipModel());
  Serial.print(F("  CPU MHz : ")); Serial.println(ESP.getCpuFreqMHz());
  Serial.print(F("  Heap    : ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
  debugSep();

  // ── MDDS30 motor serial — Serial2, TX-only on GPIO 25 ────
  Serial2.begin(MDDS30_BAUD, SERIAL_8N1, -1, MDDS30_TX_PIN);
  sendMotorBytes(0, 0);  // explicit stop on boot
  Serial.println(F("[INIT] MDDS30 serial OK (SmartDriveDuo Simplified)"));
  Serial.print(F("  TX=GPIO")); Serial.print(MDDS30_TX_PIN);
  Serial.print(F("  Baud=")); Serial.println(MDDS30_BAUD);
  Serial.println(F("  L: Motor1/ChA  R: Motor2/ChB  (GPIO25 → IN1)"));
  Serial.println(F("  Pi bridge on Serial1 (GPIO17 TX / GPIO13 RX)"));

  // ── Relay pins — already LOW ──────────────────────────
  Serial.println(F("[INIT] Relay pins OK (all LOW)"));

  // ── I2C + PCF8575 — init here, then owned by Core 0 ──
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeout(25);
  pcfPresent = pcf.begin();
  if (pcfPresent) {
    pcf.write16(0xFFFF);
    Serial.println(F("[INIT] PCF8575 ready"));
    Serial.print(F("  SDA=GPIO")); Serial.print(I2C_SDA);
    Serial.print(F("  SCL=GPIO")); Serial.print(I2C_SCL);
    Serial.print(F("  ADDR=0x")); Serial.println(PCF8575_ADDR, HEX);
  } else {
    Serial.println(F("[INIT] PCF8575 NOT found — mower I/O disabled"));
  }

  // ── AS5600 encoders — ISRs attached to Core 1 ────────
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

  // ── Serial1 — Pi bridge + Pip-Boy (shared UART, GPIO matrix) ─
  // TX primary → GPIO 17 (Pi RX).  RX ← GPIO 13 (Pi TX).
  // GPIO matrix additionally routes UART1 TX to GPIO 4 (Pip-Boy RX).
  Serial1.begin(PI_BAUD, SERIAL_8N1, PI_SERIAL_RX, PI_SERIAL_TX);
  // Route UART1 TX additionally to Pip-Boy pin via GPIO matrix.
  // gpio_matrix_out() declared via rom/gpio.h — no extern "C" needed here.
  gpio_matrix_out(ESP2_SERIAL_TX, 23u, false, false); // 23 = U1TXD_OUT_IDX
  Serial.println(F("[INIT] Serial1: Pi bridge TX=GPIO17 RX=GPIO13 @ 115200"));
  Serial.println(F("         Pip-Boy GPIO4 also receives via GPIO matrix"));

  // (Pi bridge is on Serial1 above — Serial2 is MDDS30 motor serial)

  // ── RTOS mutexes ──────────────────────────────────────
  g_mutex_c2i = xSemaphoreCreateMutex();
  g_mutex_i2c = xSemaphoreCreateMutex();
  Serial.println(F("[INIT] RTOS mutexes created"));

  // ── Launch IO task on Core 0 ──────────────────────────
  xTaskCreatePinnedToCore(
    ioTask, "ioTask", 6144, nullptr, 1, nullptr, 0
  );
  Serial.println(F("[INIT] IO task started on Core 0 (stack 6144)"));

  // ── Bluepad32 ─────────────────────────────────────────
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  // ── BT RF optimisations ───────────────────────────────
  // WiFi and Classic BT share one 2.4GHz radio on ESP32.
  // Even without WiFi.begin(), the coexistence controller
  // can steal BT air time.  Stop it explicitly.
  esp_wifi_stop();   // harmless if WiFi never started

  // PS4 uses BR/EDR (Classic BT).  Default TX power is +3dBm.
  // Raise both min and max to +9dBm for full range.
  esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9);

  debugSep();
  Serial.println(F("[BT ] WiFi radio stopped"));
  Serial.println(F("[BT ] Classic BT TX power: +9dBm"));
  Serial.println(F("[BT ] Ready — hold PS to pair"));
  Serial.println(F("  PS    = toggle mode  (LED: blue=dual / green=single)"));
  Serial.println(F("  R1    = arm -> latch mower  (again to stop)"));
  Serial.println(F("  TRI   = toggle turbo relay"));
  Serial.println(F("  SQR   = toggle mower lights"));
  Serial.println(F("  CROSS = hold to show controller battery on LED"));
  Serial.println(F("  L2    = set D-pad speed ceiling"));
  Serial.println(F("  L1    = reset D-pad speed"));
  debugSep();
}

// ============================================================
// loop() — Core 1 (BT/motor hot path)
// ============================================================
void loop() {
  // ── Serial motor throttle state ───────────────────────
  // 115200 baud = ~11520 bytes/sec. Two motors × 1 byte = 2 bytes/cmd.
  // Cap at 20 Hz keepalive; fire immediately on any speed change.
  static int           sentL       = 9999;
  static int           sentR       = 9999;
  static unsigned long lastMotorMs = 0;

  // ── 1. BP32 update — first for minimum BT latency ────
  BP32.update();

  // ── 2. Arm state machine ──────────────────────────────
  updateArmStateMachine();

  // ── 3. Read IO2Ctrl snapshot (non-blocking) ───────────
  static IO2Ctrl io = {};
  if (xSemaphoreTake(g_mutex_i2c, 0) == pdTRUE) {
    io = g_i2c;
    g_i2c.req_arm_start  = false;
    g_i2c.req_motor_stop = false;
    g_i2c.req_turbo_set  = false;
    g_i2c.req_rumble     = false;
    xSemaphoreGive(g_mutex_i2c);
  }

  // ── 4. Service IO task relay/motor requests ───────────
  if (io.req_arm_start && !relay_motor && armState == ARM_IDLE) {
    Serial.println(F("[PI ] Motor ON — arm sequence started"));
    startArmSequence();
  }
  if (io.req_motor_stop && relay_motor) {
    Serial.println(F("[PI ] Motor OFF"));
    stopMotor();
  }
  if (io.req_turbo_set) {
    relay_turbo = io.req_turbo_val;
    digitalWrite(RELAY_TURBO, relay_turbo ? HIGH : LOW);
    Serial.print(F("[PI ] Turbo ")); Serial.println(relay_turbo ? F("ON") : F("OFF"));
    debugRelays();
    if (relay_turbo) { lastFlash = 0; flashState = false; }
    else if (!relay_motor) setLED(modeColour());
  }
  if (io.req_rumble) {
    if (gGamepad && gGamepad->isConnected()) {
      gGamepad->playDualRumble(0, 600, 0xFF, 0xFF);
      Serial.println(F("[PCF ] !!! MOWER ERROR detected !!!"));
    }
  }

  // ── 5. Pi drive timeout check ─────────────────────────
  bool piDriveActive = io.piDriveActive;
  if (piDriveActive && (millis() - io.lastPiCmd > PI_CMD_TIMEOUT_MS)) {
    piDriveActive = false;
    Serial.println(F("[PI ] Drive timeout — stopped (Core 1)"));
  }

  // ── 6. Autonomous drive (no controller) ──────────────
  if (gGamepad == nullptr || !gGamepad->isConnected()) {
    if (piDriveActive) {
      int scaledL = scaleToMotor(io.pi_leftSpeed);
      int scaledR = scaleToMotor(io.pi_rightSpeed);
      unsigned long nowM = millis();
      if (scaledL != sentL || scaledR != sentR || nowM - lastMotorMs >= 50) {
        sendMotorBytes(scaledL, scaledR);
        sentL = scaledL; sentR = scaledR; lastMotorMs = nowM;
      }
    }
    int fakeL = piDriveActive ? io.pi_leftSpeed : 0;
    int fakeR = piDriveActive ? io.pi_rightSpeed : 0;
    publishCtrl2IO(fakeL, fakeR, piDriveActive);
    updateLED(relay_motor, relay_turbo);
    vTaskDelay(1);
    return;
  }

  GamepadPtr gp = gGamepad;

  // ── PS button → toggle tank mode ─────────────────────
  bool curPS = (gp->miscButtons() & MISC_BUTTON_HOME) != 0;
  if (curPS && !prev_PS) {
    tankMode = (tankMode == MODE_DUAL_STICK) ? MODE_SINGLE_STICK : MODE_DUAL_STICK;
    Serial.print(F("[MODE] -> ")); Serial.println(modeNames[tankMode]);
    if (!relay_motor && !relay_turbo) setLED(modeColour());
  }
  prev_PS = curPS;

  // ── R1 → arm sequence / stop mower ───────────────────
  bool curR1 = (gp->buttons() & BUTTON_SHOULDER_R) != 0;
  if (curR1 && !prev_R1) {
    if (!relay_motor && armState == ARM_IDLE) {
      Serial.println(F("[BTN ] R1 — starting arm sequence"));
      startArmSequence();
      gp->playDualRumble(0, 150, 0x20, 0x20);
    } else if (relay_motor) {
      Serial.println(F("[BTN ] R1 — stopping mower"));
      stopMotor();
      gp->playDualRumble(0, 80, 0x10, 0x10);
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
      gp->playDualRumble(0, RUMBLE_MS, 0x40, 0x40);
    } else {
      if (!relay_motor) setLED(modeColour());
      gp->playDualRumble(0, 80, 0x10, 0x10);
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
    gp->playDualRumble(0, 100, 0x20, 0x20);
  }
  prev_L1 = curL1;

  // ── Square → toggle mower lights (request to IO task) ─
  bool curSQR = (gp->buttons() & BUTTON_X) != 0;
  if (curSQR && !prev_SQR) {
    if (pcfPresent) {
      xSemaphoreTake(g_mutex_i2c, portMAX_DELAY);
      g_i2c.req_lights_btn = true;
      xSemaphoreGive(g_mutex_i2c);
      Serial.println(F("[BTN ] SQR — lights toggle requested"));
    } else {
      Serial.println(F("[BTN ] SQR — PCF not connected"));
    }
  }
  prev_SQR = curSQR;

  // ── Cross (hold) → show controller battery on LED ────
  // battery() returns 0=unknown, 1=empty, 255=full (not a percentage)
  bool curCROSS = (gp->buttons() & BUTTON_A) != 0;
  if (curCROSS && !prev_CROSS) {
    showBatt = true;
    int b = gp->battery();
    Colour c;
    if      (b >= 191) c = {   0, 255,   0 };  // >= 75%
    else if (b >= 127) c = { 255, 180,   0 };  // >= 50%
    else if (b >=  64) c = { 255,  80,   0 };  // >= 25%
    else               c = { 255,   0,   0 };  // <  25%
    setLED(c);
    Serial.print(F("[BATT] Raw=")); Serial.print(b);
    Serial.print(F(" (~")); Serial.print(b * 100 / 255); Serial.println(F("%)"));
  } else if (!curCROSS && prev_CROSS) {
    showBatt   = false;
    lastFlash  = 0;
    flashState = false;
    if (!relay_motor && !relay_turbo) setLED(modeColour());
  }
  prev_CROSS = curCROSS;

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
      // axisX is positive-right on PS4.  stickToSpeed() negates its input,
      // so passing axisX directly would make right-push = negative turn,
      // meaning rightSpeed > leftSpeed = robot turns LEFT.  Pre-negate to fix.
      int turn     = stickToSpeed(-gp->axisX());
      leftSpeed    = constrain(throttle + turn, -MAX_SPEED, MAX_SPEED);
      rightSpeed   = constrain(throttle - turn, -MAX_SPEED, MAX_SPEED);
    }
  }

  {
    int scaledL = scaleToMotor(leftSpeed);
    int scaledR = scaleToMotor(rightSpeed);
    unsigned long nowM = millis();
    if (scaledL != sentL || scaledR != sentR || nowM - lastMotorMs >= 50) {
      sendMotorBytes(scaledL, scaledR);
      sentL = scaledL; sentR = scaledR; lastMotorMs = nowM;
    }
  }

  static unsigned long lastDrvLog = 0;
  if ((leftSpeed != prev_leftSpeed || rightSpeed != prev_rightSpeed)
      && millis() - lastDrvLog >= 100) {
    lastDrvLog = millis();
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
    Serial.print(F(" | Batt=")); Serial.print(gp->battery());
    Serial.print(F(" (~")); Serial.print(gp->battery() * 100 / 255); Serial.println(F("%)"));
    debugFullStatus();
  }

  // ── Publish Core 1 state to IO task ──────────────────
  publishCtrl2IO(leftSpeed, rightSpeed, piDriveActive);

  updateLED(relay_motor, relay_turbo);

  // Yield 1 FreeRTOS tick so any btstack service tasks pinned
  // to Core 1 can run between loop iterations.  No measurable
  // impact on motor/BT latency — motor is throttled to 20 Hz
  // and BP32.update() is called at the top of every iteration.
  vTaskDelay(1);
}
