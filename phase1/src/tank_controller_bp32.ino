// ============================================================
// ESP32 Tank Mower — Phase 2 Firmware
// Based on Phase 1 v1.5.0 — adds full Pi serial bridge telemetry
//
// Hardware:
//   2x Cytron MD13S motor driver
//   Relay: ARM   pin 32 (momentary safety arm)
//   Relay: MOTOR pin 33 (latching mower blade)
//   Relay: TURBO pin 15 (latching turbo)
//   2x AS5600 magnetic encoders (PWM mode, pins 34/35)
//   PS4 controller via Bluepad32
//   Pi 5 via Serial2 (GPIO16/17) at 115200 baud
//
// Pi ↔ ESP32 JSON protocol:
//   Pi → ESP32:  {"cmd":"drive","l":120,"r":115}
//   Pi → ESP32:  {"cmd":"relay","id":"motor","state":1}
//   Pi → ESP32:  {"cmd":"relay","id":"turbo","state":0}
//   Pi → ESP32:  {"cmd":"stop"}
//   Pi → ESP32:  {"cmd":"ping"}
//   ESP32 → Pi:  {"enc_l":1024,"enc_r":1019,"dist_l":0.1234,"dist_r":0.1230,
//                 "spd_l":0.05,"spd_r":0.05,
//                 "relay_arm":0,"relay_motor":1,"relay_turbo":0,
//                 "connected":1,"mode":"DUAL","batt":85}
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
//
// Install Bluepad32:
//   File > Preferences > Additional Boards Manager URLs:
//   https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
//   Tools > Board > Boards Manager > search "Bluepad32" > install
//   Select board: "ESP32 + Bluepad32"
// ============================================================

#include <Bluepad32.h>
#include <EncAS5600.h>

// ── Pin definitions ─────────────────────────────────────────
#define MOTOR_A_PWM   25    // Left track speed  (Cytron MD13S PWM)
#define MOTOR_A_DIR   26    // Left track dir    (Cytron MD13S DIR)
#define MOTOR_B_PWM   27    // Right track speed
#define MOTOR_B_DIR   14    // Right track dir

#define RELAY_ARM     32    // Arm relay   — momentary 500ms before motor
#define RELAY_MOTOR   33    // Motor relay — latching mower blade
#define RELAY_TURBO   15    // Turbo relay — latching

#define ENCODER_L_PIN 34    // AS5600 left  encoder PWM output
#define ENCODER_R_PIN 35    // AS5600 right encoder PWM output

// ── Odometry constants ───────────────────────────────────────
// Adjust WHEEL_DIAMETER_M and GEAR_RATIO to match your hardware.
// dist per encoder degree = (π × diameter) / (gear_ratio × 360)
#define WHEEL_DIAMETER_M  0.254f    // 10 inches
#define GEAR_RATIO        16.0f     // motor revs per wheel rev
#define DIST_PER_DEG_M    (3.14159265f * WHEEL_DIAMETER_M / (GEAR_RATIO * 360.0f))

// ── Tuning ───────────────────────────────────────────────────
#define DEADZONE          6         // Stick dead-zone (Bluepad32: -511..+511)
#define MAX_SPEED         127       // PWM ceiling (50% — keeps headroom for closed-loop)
#define STICK_MAX         511       // Bluepad32 full stick range
#define DPAD_SPEED_MIN    15        // Minimum d-pad speed so it always creeps
#define ARM_PULSE_MS      500       // Duration of each arm relay phase (ms)
#define FLASH_MS          300       // LED flash interval (ms)
#define RUMBLE_MS         300       // Rumble buzz duration (ms)

// ── Pi serial bridge ─────────────────────────────────────────
#define PI_SERIAL_TX      17        // ESP32 TX2 → Pi GPIO15 (RX)
#define PI_SERIAL_RX      16        // ESP32 RX2 ← Pi GPIO14 (TX)
#define PI_BAUD           115200
#define PI_TELEM_MS       50        // Telemetry to Pi every 50ms (20Hz)
#define PI_CMD_TIMEOUT_MS 500       // Stop Pi drive if no command for this long

// ── Tank mode ────────────────────────────────────────────────
enum TankMode {
  MODE_DUAL_STICK,    // Left stick = left track, Right stick = right track
  MODE_SINGLE_STICK   // Left Y = throttle, Left X = turn mix
};
const char* modeNames[] = { "DUAL", "SINGLE" };

// ── Arm state machine ────────────────────────────────────────
enum ArmState {
  ARM_IDLE,
  ARM_ARMING   // ARM relay HIGH, waiting ARM_PULSE_MS before latching MOTOR
};

// ── LED colours ──────────────────────────────────────────────
struct Colour { uint8_t r, g, b; };
const Colour COL_DUAL   = { 0,   0,   255 }; // Blue  — dual stick
const Colour COL_SINGLE = { 0,   255, 0   }; // Green — single stick
const Colour COL_RED    = { 255, 0,   0   }; // Red   — motor relay flash
const Colour COL_WHITE  = { 255, 255, 255 }; // White — turbo flash

// ── Global state ─────────────────────────────────────────────
GamepadPtr  gGamepad  = nullptr;
TankMode    tankMode  = MODE_DUAL_STICK;
ArmState    armState  = ARM_IDLE;
bool        relay_motor = false;
bool        relay_turbo = false;
unsigned long armTimer  = 0;

// Button edge detection
bool prev_PS   = false;
bool prev_R1   = false;
bool prev_TRI  = false;
bool prev_L1   = false;

// D-pad speed ceiling (set by squeezing L2, reset by L1)
int peakL2    = 0;
int dpadSpeed = 0;

// LED flash
unsigned long lastFlash = 0;
bool          flashState = false;

// Timed rumble
unsigned long rumbleOffTime = 0;

// Drive change detection (only log on change)
int prev_leftSpeed  = 9999;
int prev_rightSpeed = 9999;

// Periodic status
unsigned long lastStatusPrint = 0;
#define STATUS_INTERVAL_MS 2000

// ── Encoder state (written by EncAS5600 callbacks) ───────────
// volatile because written from library interrupt context, read from loop()
volatile long  enc_ticks_L   = 0;
volatile long  enc_ticks_R   = 0;
volatile float enc_dist_L_m  = 0.0f;
volatile float enc_dist_R_m  = 0.0f;

// EncAS5600 objects — constructed in setup() after pin config
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

// ============================================================
// Debug helpers
// ============================================================
void debugSep() { Serial.println(F("------------------------------------------")); }

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
  Serial.print(F("[RELAY] ARM=")); Serial.print(digitalRead(RELAY_ARM)  ? F("ON ") : F("OFF"));
  Serial.print(F(" MOTOR="));     Serial.print(relay_motor               ? F("ON ") : F("OFF"));
  Serial.print(F(" TURBO="));     Serial.println(relay_turbo             ? F("ON")  : F("OFF"));
}

void debugFullStatus() {
  debugSep();
  Serial.println(F("  STATUS"));
  Serial.print(F("  Mode      : ")); Serial.println(modeNames[tankMode]);
  Serial.print(F("  ARM relay : ")); Serial.println(digitalRead(RELAY_ARM) ? "ON" : "OFF");
  Serial.print(F("  MOTOR     : ")); Serial.println(relay_motor ? "ON" : "OFF");
  Serial.print(F("  TURBO     : ")); Serial.println(relay_turbo ? "ON" : "OFF");
  Serial.print(F("  Arm state : ")); Serial.println(armState == ARM_IDLE ? "IDLE" : "ARMING");
  Serial.print(F("  dpadSpeed : ")); Serial.print(dpadSpeed);
  Serial.print(F(" (peakL2=")); Serial.print(peakL2); Serial.println(F(")"));
  Serial.print(F("  enc L     : ")); Serial.print(enc_ticks_L);
  Serial.print(F(" ticks  ")); Serial.print(enc_dist_L_m, 3); Serial.println(F("m"));
  Serial.print(F("  enc R     : ")); Serial.print(enc_ticks_R);
  Serial.print(F(" ticks  ")); Serial.print(enc_dist_R_m, 3); Serial.println(F("m"));
  Serial.print(F("  Pi drive  : ")); Serial.println(piDriveActive ? "ACTIVE" : "idle");
  Serial.print(F("  Free heap : ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
  debugSep();
}

// ============================================================
// LED helpers
// ============================================================
void setLED(Colour c) {
  if (gGamepad) gGamepad->setColorLED(c.r, c.g, c.b);
}

Colour modeColour() {
  return tankMode == MODE_DUAL_STICK ? COL_DUAL : COL_SINGLE;
}

// Call every loop() iteration when relay(s) are active
void updateLED() {
  if (!gGamepad) return;
  if (!relay_motor && !relay_turbo) return; // solid colour already set
  unsigned long now = millis();
  if (now - lastFlash < FLASH_MS) return;
  lastFlash = now;
  flashState = !flashState;
  // Turbo takes priority: white ↔ mode colour
  // Motor only: red ↔ mode colour
  if (relay_turbo) {
    setLED(flashState ? COL_WHITE : modeColour());
  } else {
    setLED(flashState ? COL_RED : modeColour());
  }
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
  if (!relay_turbo) {
    setLED(modeColour());
  }
}

void updateArmStateMachine() {
  if (armState == ARM_IDLE) return;
  if (millis() < armTimer) return;

  if (armState == ARM_ARMING) {
    relay_motor = true;
    digitalWrite(RELAY_MOTOR, HIGH);
    armState = ARM_IDLE;
    Serial.println(F("[ARM ] MOTOR relay ON — ARM + MOTOR both HIGH, mower running"));
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
  gp->setColorLED(COL_DUAL.r, COL_DUAL.g, COL_DUAL.b);
  gp->setRumble(0x40, 0x40);
  rumbleOffTime = millis() + RUMBLE_MS;
  Serial.println(F("[LED ] BLUE (dual stick)"));
}

void onDisconnectedGamepad(GamepadPtr gp) {
  debugBanner("CONTROLLER DISCONNECTED — FAILSAFE");
  gGamepad = nullptr;

  // Hard stop everything
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
  digitalWrite(RELAY_ARM,   LOW);
  digitalWrite(RELAY_MOTOR, LOW);
  digitalWrite(RELAY_TURBO, LOW);
  relay_motor    = false;
  relay_turbo    = false;
  armState       = ARM_IDLE;
  piDriveActive  = false;
  pi_leftSpeed   = 0;
  pi_rightSpeed  = 0;

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

// Gentle S-curve: 60% linear + 40% cubic
// Precise at low speeds, strong ramp at high speeds
int stickToSpeed(int axis) {
  int val = -axis; // invert: up = forward
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
// Emits one JSON line every PI_TELEM_MS milliseconds.
// The Pi's serial_bridge.py reads these lines and populates _state.
// Fields used by odometry.py: enc_l, enc_r, dist_l, dist_r, spd_l, spd_r
// Fields used by web_ui.py:   relay_*, connected, mode, batt
//
// NOTE: heading is NOT sent here — BNO055 is I2C to the Pi directly (compass.py)
void sendTelemetry() {
  // Snapshot volatile encoder values once
  long  tl = enc_ticks_L;
  long  tr = enc_ticks_R;
  float dl = enc_dist_L_m;
  float dr = enc_dist_R_m;

  Serial2.print(F("{\"enc_l\":"));     Serial2.print(tl);
  Serial2.print(F(",\"enc_r\":"));     Serial2.print(tr);
  Serial2.print(F(",\"dist_l\":"));    Serial2.print(dl, 4);
  Serial2.print(F(",\"dist_r\":"));    Serial2.print(dr, 4);
  Serial2.print(F(",\"spd_l\":"));     Serial2.print(enc_L ? enc_L->getSpeed() : 0.0f, 3);
  Serial2.print(F(",\"spd_r\":"));     Serial2.print(enc_R ? enc_R->getSpeed() : 0.0f, 3);
  Serial2.print(F(",\"relay_arm\":")); Serial2.print(digitalRead(RELAY_ARM));
  Serial2.print(F(",\"relay_motor\":")); Serial2.print(relay_motor ? 1 : 0);
  Serial2.print(F(",\"relay_turbo\":")); Serial2.print(relay_turbo ? 1 : 0);
  Serial2.print(F(",\"connected\":")); Serial2.print(gGamepad ? 1 : 0);
  Serial2.print(F(",\"mode\":\""));    Serial2.print(modeNames[tankMode]);
  Serial2.print(F("\",\"batt\":"));    Serial2.print(gGamepad ? gGamepad->battery() : 0);
  Serial2.println(F("}"));
}

// ============================================================
// Pi serial bridge — command RX
// ============================================================
// Supported commands:
//   {"cmd":"drive","l":N,"r":N}  — set left/right speed -255..255
//   {"cmd":"stop"}               — zero speed, clear piDriveActive
//   {"cmd":"relay","id":"motor","state":1}
//   {"cmd":"relay","id":"turbo","state":1}
//   {"cmd":"ping"}               — responds with {"pong":1}
//
// IMPORTANT: Pi drive commands are ONLY acted on when PS4 is disconnected.
// Relay commands are always accepted so the web UI can arm/disarm.
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
    // Only accept Pi drive when PS4 is disconnected
    if (gGamepad != nullptr) {
      Serial.println(F("[PI ] Drive ignored — PS4 connected (RC has priority)"));
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
        Serial.println(F("[PI ] Motor ON via Pi — arm sequence started"));
      } else if (!state && relay_motor) {
        stopMotor();
        Serial.println(F("[PI ] Motor OFF via Pi"));
      }
    } else if (line.indexOf(F("\"id\":\"turbo\"")) >= 0) {
      relay_turbo = state;
      digitalWrite(RELAY_TURBO, relay_turbo ? HIGH : LOW);
      Serial.print(F("[PI ] Turbo ")); Serial.println(state ? F("ON") : F("OFF"));
    }
    return;
  }

  Serial.print(F("[PI ] Unknown: ")); Serial.println(line);
}

// ============================================================
// setup()
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(400);
  debugBanner("ESP32 TANK MOWER — PHASE 2");
  Serial.print(F("  Chip     : ")); Serial.println(ESP.getChipModel());
  Serial.print(F("  CPU MHz  : ")); Serial.println(ESP.getCpuFreqMHz());
  Serial.print(F("  Heap     : ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
  debugSep();

  // Track motor pins
  pinMode(MOTOR_A_DIR, OUTPUT); analogWrite(MOTOR_A_PWM, 0);
  pinMode(MOTOR_B_DIR, OUTPUT); analogWrite(MOTOR_B_PWM, 0);
  Serial.println(F("[INIT] Motor pins OK"));

  // Relay pins — all LOW on boot
  pinMode(RELAY_ARM,   OUTPUT); digitalWrite(RELAY_ARM,   LOW);
  pinMode(RELAY_MOTOR, OUTPUT); digitalWrite(RELAY_MOTOR, LOW);
  pinMode(RELAY_TURBO, OUTPUT); digitalWrite(RELAY_TURBO, LOW);
  Serial.println(F("[INIT] Relay pins OK (all LOW)"));

  // ── AS5600 encoders (PWM mode) ─────────────────────────
  // EncAS5600 library handles PWM timing internally via hardware timers.
  // Callbacks fire on each angle update and accumulate ticks + distance.
  // Do NOT use pulseIn() in parallel — it would block and corrupt readings.
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

  Serial.println(F("[INIT] AS5600 encoders started (PWM mode)"));
  Serial.print(F("  Dist/deg = ")); Serial.print(DIST_PER_DEG_M * 1000.0f, 4); Serial.println(F("mm"));
  Serial.print(F("  Left pin = ")); Serial.print(ENCODER_L_PIN);
  Serial.print(F("  Right pin = ")); Serial.println(ENCODER_R_PIN);

  // ── Pi serial bridge (Serial2) ─────────────────────────
  Serial2.begin(PI_BAUD, SERIAL_8N1, PI_SERIAL_RX, PI_SERIAL_TX);
  Serial.println(F("[INIT] Pi serial bridge started (Serial2)"));
  Serial.print(F("  TX2=GPIO")); Serial.print(PI_SERIAL_TX);
  Serial.print(F("  RX2=GPIO")); Serial.println(PI_SERIAL_RX);
  Serial.print(F("  Baud=")); Serial.println(PI_BAUD);

  // ── Bluepad32 ──────────────────────────────────────────
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  // BP32.forgetBluetoothKeys(); // uncomment to force re-pair on boot

  debugSep();
  Serial.println(F("[BT ] Ready — hold PS on controller to pair"));
  Serial.println(F("  PS  = toggle tank mode (blue=dual / green=single)"));
  Serial.println(F("  R1  = arm sequence -> latch mower (press again to stop)"));
  Serial.println(F("  TRI = toggle turbo relay (white flash while ON)"));
  Serial.println(F("  L2  = squeeze to set D-pad speed ceiling"));
  Serial.println(F("  L1  = reset D-pad speed to zero"));
  debugSep();
}

// ============================================================
// loop()
// ============================================================
void loop() {

  // ── 1. Arm state machine ───────────────────────────────
  updateArmStateMachine();

  // ── 2. Pi bridge — receive commands ───────────────────
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      parsePiCommand(piCmdBuffer);
      piCmdBuffer = "";
    } else if (piCmdBuffer.length() < 256) {
      piCmdBuffer += c;
    }
  }

  // ── 3. Pi drive timeout ────────────────────────────────
  // If Pi goes silent for PI_CMD_TIMEOUT_MS, stop wheels gracefully
  if (piDriveActive && (millis() - lastPiCmd > PI_CMD_TIMEOUT_MS)) {
    piDriveActive = false;
    pi_leftSpeed  = 0;
    pi_rightSpeed = 0;
    Serial.println(F("[PI ] Drive timeout — wheels stopped"));
  }

  // ── 4. Telemetry to Pi at 20Hz ─────────────────────────
  if (millis() - lastTelemSent >= PI_TELEM_MS) {
    lastTelemSent = millis();
    sendTelemetry();
  }

  // ── 5. Timed rumble off ────────────────────────────────
  if (rumbleOffTime && millis() >= rumbleOffTime) {
    rumbleOffTime = 0;
    if (gGamepad && gGamepad->isConnected()) {
      gGamepad->setRumble(0, 0);
    }
  }

  // ── 6. Bluepad32 update ────────────────────────────────
  BP32.update();

  if (gGamepad == nullptr || !gGamepad->isConnected()) {
    // No PS4 — Pi is in charge if piDriveActive
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
    if (!relay_motor && !relay_turbo) setLED(modeColour());
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
    // If arm sequence in progress, ignore second press
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
      if (!relay_motor) setLED(modeColour());
      gp->setRumble(0x10, 0x10);
      rumbleOffTime = millis() + 80;
    }
  }
  prev_TRI = curTRI;

  // ── L2 analog → update peak D-pad speed ceiling ──────
  {
    int rawL2 = gp->brake(); // 0-1023
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

  // ── Drive ─────────────────────────────────────────────
  // Priority: D-pad > analog sticks > Pi (Pi only active without PS4)
  int leftSpeed  = 0;
  int rightSpeed = 0;

  uint8_t dpad = gp->dpad();
  if (dpad & (DPAD_UP | DPAD_DOWN | DPAD_LEFT | DPAD_RIGHT)) {
    // D-pad active
    if (dpadSpeed > 0) {
      int spd = dpadSpeed;
      if      (dpad & DPAD_UP)    { leftSpeed =  spd; rightSpeed =  spd; }
      else if (dpad & DPAD_DOWN)  { leftSpeed = -spd; rightSpeed = -spd; }
      else if (dpad & DPAD_LEFT)  { leftSpeed = -spd; rightSpeed =  spd; }
      else if (dpad & DPAD_RIGHT) { leftSpeed =  spd; rightSpeed = -spd; }
    }
  } else {
    // Analog sticks
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
    Serial.print(F("[ENC ] spd_L=")); Serial.print(enc_L ? enc_L->getSpeed() : 0.0f, 3);
    Serial.print(F("  spd_R=")); Serial.println(enc_R ? enc_R->getSpeed() : 0.0f, 3);
    Serial.print(F("[STCK] LX=")); Serial.print(gp->axisX());
    Serial.print(F(" LY=")); Serial.print(gp->axisY());
    Serial.print(F(" RY=")); Serial.print(gp->axisRY());
    Serial.print(F(" | L2=")); Serial.print(gp->brake());
    Serial.print(F(" | Batt=")); Serial.print(gp->battery()); Serial.println(F("%"));
    debugFullStatus();
  }

  updateLED();
  delay(20); // ~50Hz main loop
}
