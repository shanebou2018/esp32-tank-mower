// ============================================================
//  ESP32 Tank Controller — Bluepad32 edition
//  Hardware:
//    2x Cytron MD13S motor driver
//    Relay: ARM   pin 32  (momentary 500ms safety arm)
//    Relay: MOTOR pin 33  (latching mower motor)
//    Relay: TURBO pin 15  (latching turbo, L2 button)
//    PS4 controller via Bluepad32
//
//  R2 ON sequence (non-blocking):
//    1. ARM relay ON
//    2. 500ms later -> MOTOR relay latches ON
//    3. 500ms later -> ARM relay OFF
//  R2 OFF: MOTOR relay off immediately, ARM forced off
//
//  L2 button: toggle TURBO relay (LED flashes white while ON)
//  PS button: toggle tank mode (blue=dual, green=single)
//
//  Install Bluepad32:
//    File > Preferences > Additional Boards Manager URLs:
//    https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
//    Tools > Board > Boards Manager > search "Bluepad32" > install
//    Select board: "ESP32 + Bluepad32"
// ============================================================

#include <Bluepad32.h>
#include <EncAS5600.h>

// -- Pin definitions -----------------------------------------
#define MOTOR_A_PWM   25   // Left  track speed  (Cytron MD13S PWM)
#define MOTOR_A_DIR   26   // Left  track dir    (Cytron MD13S DIR)
#define MOTOR_B_PWM   27   // Right track speed
#define MOTOR_B_DIR   14   // Right track dir

#define RELAY_ARM     32   // Arm relay    -- ON 500ms before motor, stays ON with motor
#define RELAY_MOTOR   33   // Motor relay  -- latching, R2 toggle
#define RELAY_TURBO   15   // Turbo relay  -- latching, L2 toggle

// -- Encoder pins --------------------------------------------
#define ENCODER_L_PIN   34   // AS5600 left  motor PWM output
#define ENCODER_R_PIN   35   // AS5600 right motor PWM output

// -- Odometry constants --------------------------------------
#define WHEEL_DIAMETER_M     0.254f   // 10 inches in meters
#define GEAR_RATIO           16.0f    // motor turns per wheel turn
// EncAS5600 counts in degrees (360 per motor rev)
// dist per degree = (PI * diameter) / (gear_ratio * 360)
#define DIST_PER_DEG_M       (3.14159265f * WHEEL_DIAMETER_M / (GEAR_RATIO * 360.0f))

// -- Tuning --------------------------------------------------
#define DEADZONE          6    // Stick dead-zone (Bluepad32: -511..+511)
#define MAX_SPEED       127    // PWM ceiling (50% power for better resolution)
#define STICK_MAX       511    // Bluepad32 full stick range
#define R1_THRESHOLD    1      // BUTTON_SHOULDER_R digital press
#define L2_THRESHOLD    400    // brake()    > this = L2 triggered (0-1023)
#define DPAD_SPEED_MIN   15     // minimum dpad speed so it always creeps a little
#define ARM_PULSE_MS    500    // how long each arm phase lasts (ms)
#define FLASH_MS        300    // LED flash interval (ms)
#define RUMBLE_MS       300    // rumble buzz duration (ms)

// -- Tank mode -----------------------------------------------
enum TankMode {
  MODE_DUAL_STICK,    // Left stick = left track, Right stick = right track
  MODE_SINGLE_STICK   // Left Y = throttle, Left X = turn
};
const char* modeNames[] = { "DUAL_STICK", "SINGLE_STICK" };

// -- Arm/motor sequence state machine ------------------------
enum ArmState {
  ARM_IDLE,       // nothing happening
  ARM_ARMING      // arm relay ON, waiting before motor latches
};

// -- LED colours ---------------------------------------------
struct Colour { uint8_t r, g, b; };
const Colour COL_DUAL   = {   0,   0, 255 };  // Blue  -- dual stick
const Colour COL_SINGLE = {   0, 255,   0 };  // Green -- single stick
const Colour COL_RED    = { 255,   0,   0 };  // Red   -- motor relay flash
const Colour COL_WHITE  = { 255, 255, 255 };  // White -- turbo flash

// -- Global state --------------------------------------------
GamepadPtr    gGamepad      = nullptr;
TankMode      tankMode      = MODE_DUAL_STICK;

bool          relay_motor   = false;
bool          relay_turbo   = false;

ArmState      armState      = ARM_IDLE;
unsigned long armTimer      = 0;

// Button edge detection
bool          prev_PS       = false;
bool          prev_R1btn    = false;
bool          prev_Tribtn    = false;
bool          prev_L1btn     = false;

// D-pad speed ceiling — set by squeezing L2, reset by L1
int           peakL2         = 0;   // highest raw L2 value seen (0-1023)
int           dpadSpeed      = 0;   // mapped speed ceiling for d-pad (0-MAX_SPEED)

// LED flash
unsigned long lastFlash     = 0;
bool          flashState    = false;

// Timed rumble
unsigned long rumbleOffTime = 0;

// Drive debug throttle
int           prev_leftSpeed  = 9999;
int           prev_rightSpeed = 9999;

// Periodic status
unsigned long lastStatusPrint = 0;
#define STATUS_INTERVAL_MS 2000

// -- Pi serial bridge (Serial2) ------------------------------
// ESP32 TX2=GPIO17 -> Pi GPIO15(RX)  |  ESP32 RX2=GPIO16 -> Pi GPIO14(TX)
#define PI_SERIAL_TX    17
#define PI_SERIAL_RX    16
#define PI_BAUD         115200
#define PI_TELEM_MS     50     // send telemetry to Pi every 50ms (~20Hz)
#define PI_CMD_TIMEOUT  500    // ms without Pi command before ignoring Pi drive

// -- Encoder objects -----------------------------------------
EncAS5600 enc_L;   // left  motor encoder
EncAS5600 enc_R;   // right motor encoder

// -- Encoder state -------------------------------------------
long  enc_ticks_L  = 0;     // cumulative ticks left
long  enc_ticks_R  = 0;     // cumulative ticks right
float enc_dist_L_m = 0.0f;  // total distance left  (m)
float enc_dist_R_m = 0.0f;  // total distance right (m)

// PWM timing for AS5600
// PWM timing handled in encoder task

// Speed calculation
unsigned long lastSpeedCalc = 0;
long          last_ticks_L  = 0;
long          last_ticks_R  = 0;
#define SPEED_CALC_MS 100   // recalculate speed every 100ms

// Pi bridge state
unsigned long lastTelemSent   = 0;
unsigned long lastPiCmd       = 0;    // millis of last valid Pi drive command
bool          piDriveActive   = false; // true when Pi is sending drive commands
int           pi_leftSpeed    = 0;
int           pi_rightSpeed   = 0;
String        piCmdBuffer     = "";   // incoming serial line buffer

// -- Debug helpers -------------------------------------------

void debugSeparator() {
  Serial.println(F("------------------------------------------"));
}

void debugBanner(const char* msg) {
  debugSeparator();
  Serial.print(F("  "));
  Serial.println(msg);
  debugSeparator();
}

void printBTAddr(const uint8_t* addr) {
  for (int i = 0; i < 6; i++) {
    if (addr[i] < 0x10) Serial.print('0');
    Serial.print(addr[i], HEX);
    if (i < 5) Serial.print(':');
  }
  Serial.println();
}

void debugRelayState() {
  Serial.print  (F("[RELAY] ARM="));
  Serial.print  (digitalRead(RELAY_ARM)  ? F("ON ") : F("OFF"));
  Serial.print  (F("  MOTOR="));
  Serial.print  (relay_motor             ? F("ON ") : F("OFF"));
  Serial.print  (F("  TURBO="));
  Serial.println(relay_turbo             ? F("ON")  : F("OFF"));
}

void debugArmState() {
  Serial.print(F("[ARM  ] State="));
  switch (armState) {
    case ARM_IDLE:     Serial.println(F("IDLE"));     break;
    case ARM_ARMING:   Serial.println(F("ARMING"));   break;
  }
}

void debugModeChange() {
  Serial.print(F("[MODE ] Tank mode -> "));
  Serial.println(modeNames[tankMode]);
  Serial.print(F("[LED  ] Colour = "));
  Serial.println(tankMode == MODE_DUAL_STICK ? F("BLUE") : F("GREEN"));
}

void debugDriveValues(int l, int r) {
  Serial.print(F("[DRIVE] Left="));
  if (l >= 0) Serial.print(' ');
  Serial.print(l);
  Serial.print(F("  Right="));
  if (r >= 0) Serial.print(' ');
  Serial.println(r);
}

void debugFullStatus() {
  debugSeparator();
  Serial.println(F("  STATUS DUMP"));
  Serial.print  (F("  Mode       : ")); Serial.println(modeNames[tankMode]);
  Serial.print  (F("  ARM  relay : ")); Serial.println(digitalRead(RELAY_ARM) ? "ON" : "OFF");
  Serial.print  (F("  MOTOR relay: ")); Serial.println(relay_motor            ? "ON" : "OFF");
  Serial.print  (F("  TURBO relay: ")); Serial.println(relay_turbo            ? "ON" : "OFF");
  Serial.print  (F("  Arm state  : "));
  switch (armState) {
    case ARM_IDLE:     Serial.println(F("IDLE"));     break;
    case ARM_ARMING:   Serial.println(F("ARMING"));   break;
  }
  Serial.print  (F("  Peak L2    : ")); Serial.print(peakL2); Serial.print(F("  dpad speed=")); Serial.println(dpadSpeed);
  Serial.print  (F("  Free heap  : ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
  debugSeparator();
}

void debugRawSticks(GamepadPtr gp) {
  Serial.print(F("[STICK] LX=")); Serial.print(gp->axisX());
  Serial.print(F(" LY="));        Serial.print(gp->axisY());
  Serial.print(F(" RX="));        Serial.print(gp->axisRX());
  Serial.print(F(" RY="));        Serial.print(gp->axisRY());
  Serial.print(F(" | L2="));      Serial.print(gp->brake());
  Serial.print(F(" R2="));        Serial.println(gp->throttle());
}

void debugButtons(GamepadPtr gp) {
  Serial.print(F("[BTN  ] "));
  uint16_t btns = gp->buttons();
  uint8_t  misc = gp->miscButtons();
  uint8_t  dpad = gp->dpad();
  if (btns & BUTTON_A)           Serial.print(F("A "));
  if (btns & BUTTON_B)           Serial.print(F("B "));
  if (btns & BUTTON_X)           Serial.print(F("X "));
  if (btns & BUTTON_Y)           Serial.print(F("Y "));
  if (btns & BUTTON_SHOULDER_L)  Serial.print(F("L1 "));
  if (btns & BUTTON_SHOULDER_R)  Serial.print(F("R1 "));
  if (btns & BUTTON_TRIGGER_L)   Serial.print(F("L2 "));
  if (btns & BUTTON_TRIGGER_R)   Serial.print(F("R2 "));
  if (btns & BUTTON_THUMB_L)     Serial.print(F("L3 "));
  if (btns & BUTTON_THUMB_R)     Serial.print(F("R3 "));
  if (misc & MISC_BUTTON_SELECT) Serial.print(F("OPTIONS "));
  if (misc & MISC_BUTTON_HOME)   Serial.print(F("PS "));
  if (misc & MISC_BUTTON_BACK)   Serial.print(F("SHARE "));
  if (dpad & DPAD_UP)            Serial.print(F("UP "));
  if (dpad & DPAD_DOWN)          Serial.print(F("DOWN "));
  if (dpad & DPAD_LEFT)          Serial.print(F("LEFT "));
  if (dpad & DPAD_RIGHT)         Serial.print(F("RIGHT "));
  Serial.println();
}

// -- LED helpers ---------------------------------------------

void setLED(Colour c) {
  if (gGamepad) gGamepad->setColorLED(c.r, c.g, c.b);
}

Colour modeColour() {
  return tankMode == MODE_DUAL_STICK ? COL_DUAL : COL_SINGLE;
}

void updateLED() {
  if (!gGamepad) return;
  if (!relay_motor && !relay_turbo) return;  // solid colour set at toggle time

  unsigned long now = millis();
  if (now - lastFlash < FLASH_MS) return;
  lastFlash  = now;
  flashState = !flashState;

  if (relay_turbo) {
    // Turbo takes priority: white <-> mode colour
    setLED(flashState ? COL_WHITE : modeColour());
  } else {
    // Motor only: red <-> mode colour
    setLED(flashState ? COL_RED : modeColour());
  }
}

// -- Arm/motor state machine ---------------------------------

void startArmSequence() {
  Serial.println(F("[ARM  ] Sequence START -- ARM relay ON"));
  digitalWrite(RELAY_ARM, HIGH);
  armState = ARM_ARMING;
  armTimer = millis() + ARM_PULSE_MS;
  debugArmState();
  debugRelayState();
}

void stopMotor() {
  relay_motor = false;
  digitalWrite(RELAY_MOTOR, LOW);
  digitalWrite(RELAY_ARM,   LOW);  // both go LOW together
  armState = ARM_IDLE;
  Serial.println(F("[ARM  ] ARM + MOTOR both LOW -- mower stopped"));
  debugRelayState();
  if (!relay_turbo) {
    setLED(modeColour());
    Serial.print(F("[LED  ] Restored "));
    Serial.println(tankMode == MODE_DUAL_STICK ? F("BLUE") : F("GREEN"));
  }
}

void updateArmStateMachine() {
  if (armState == ARM_IDLE) return;

  unsigned long now = millis();
  if (now < armTimer) return;

  if (armState == ARM_ARMING) {
    // 500ms elapsed -- latch motor ON, both ARM + MOTOR stay HIGH together
    relay_motor = true;
    digitalWrite(RELAY_MOTOR, HIGH);
    armState = ARM_IDLE;  // sequence done, both relays held by relay_motor flag
    Serial.println(F("[ARM  ] MOTOR relay ON -- ARM + MOTOR both HIGH, mower running"));
    debugArmState();
    debugRelayState();
    // Start LED flash
    lastFlash  = 0;
    flashState = false;
    // Rumble confirm
    if (gGamepad) {
      gGamepad->setRumble(0x40, 0x40);
      rumbleOffTime = now + RUMBLE_MS;
      Serial.print(F("[RUMBLE] Motor latch buzz ON for ")); Serial.print(RUMBLE_MS); Serial.println(F("ms"));
    }
  }
}

// -- Bluepad32 callbacks -------------------------------------

void onConnectedGamepad(GamepadPtr gp) {
  debugBanner("CONTROLLER CONNECTED");
  GamepadProperties props = gp->getProperties();
  Serial.print(F("[CONN] BT address : ")); printBTAddr(props.btaddr);
  Serial.print(F("[CONN] VID/PID    : 0x"));
  if (props.vendor_id  < 0x10) Serial.print('0');
  Serial.print(props.vendor_id, HEX);
  Serial.print(F(" / 0x"));
  if (props.product_id < 0x10) Serial.print('0');
  Serial.println(props.product_id, HEX);
  Serial.print(F("[CONN] HID subtype: ")); Serial.println(props.subtype);

  gGamepad = gp;
  gp->setColorLED(COL_DUAL.r, COL_DUAL.g, COL_DUAL.b);
  gp->setRumble(0x40, 0x40);
  rumbleOffTime = millis() + RUMBLE_MS;
  Serial.print(F("[RUMBLE] Connect buzz ON for ")); Serial.print(RUMBLE_MS); Serial.println(F("ms"));
  Serial.println(F("[LED  ] Set to BLUE (dual stick mode)"));
}

void onDisconnectedGamepad(GamepadPtr gp) {
  debugBanner("CONTROLLER DISCONNECTED");
  gGamepad = nullptr;
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
  digitalWrite(RELAY_ARM,   LOW);
  digitalWrite(RELAY_MOTOR, LOW);
  digitalWrite(RELAY_TURBO, LOW);
  relay_motor = false;
  relay_turbo = false;
  armState    = ARM_IDLE;
  Serial.println(F("[SAFE] Tracks stopped, all relays OFF, arm state reset"));
}

// -- Track motor drive ---------------------------------------

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
// Low stick = slow and precise, high stick = ramps up strongly
int stickToSpeed(int axis) {
  int val = -axis;                               // invert: up = forward
  if (abs(val) <= DEADZONE) val = 0;
  if (val == 0) return 0;
  float norm   = (float)val / (float)STICK_MAX; // normalise to -1.0..+1.0
  float curved = (norm * norm * norm * 0.4f) + (norm * 0.6f);
  curved = constrain(curved, -1.0f, 1.0f);
  int out = (int)(curved * MAX_SPEED);
  return out;
}

// Map peak L2 raw value (0-1023) to a dpad speed (DPAD_SPEED_MIN..MAX_SPEED)
int calcDpadSpeed(int rawL2) {
  if (rawL2 <= 0) return 0;
  return (int)map(rawL2, 0, 1023, DPAD_SPEED_MIN, MAX_SPEED);
}


// -- Pi serial bridge functions ------------------------------

// Send JSON telemetry to Pi over Serial2
// Format: {"enc_l":0,"enc_r":0,"relay_arm":0,"relay_motor":0,"relay_turbo":0,"connected":1,"mode":"DUAL","batt":0}
void sendTelemetry() {
  Serial2.print(F("{\"enc_l\":"));   Serial2.print(enc_ticks_L);
  Serial2.print(F(",\"enc_r\":"));   Serial2.print(enc_ticks_R);
  Serial2.print(F(",\"dist_l\":"));  Serial2.print(enc_dist_L_m, 4);
  Serial2.print(F(",\"dist_r\":"));  Serial2.print(enc_dist_R_m, 4);
  Serial2.print(F(",\"spd_l\":"));   Serial2.print(enc_L.getSpeed(), 3);
  Serial2.print(F(",\"spd_r\":"));   Serial2.print(enc_R.getSpeed(), 3);
  Serial2.print(F(",\"relay_arm\":"));   Serial2.print(digitalRead(RELAY_ARM));
  Serial2.print(F(",\"relay_motor\":")); Serial2.print(relay_motor  ? 1 : 0);
  Serial2.print(F(",\"relay_turbo\":")); Serial2.print(relay_turbo  ? 1 : 0);
  Serial2.print(F(",\"connected\":"));   Serial2.print(gGamepad ? 1 : 0);
  Serial2.print(F(",\"mode\":\""));
  Serial2.print(tankMode == MODE_DUAL_STICK ? F("DUAL") : F("SINGLE"));
  Serial2.print(F("\""));
  Serial2.print(F(",\"batt\":"));
  Serial2.print(gGamepad ? gGamepad->battery() : 0);
  Serial2.println(F("}"));
}

// Parse a JSON command from Pi
// Supported: {"cmd":"drive","l":100,"r":100}
//            {"cmd":"stop"}
//            {"cmd":"relay","id":"motor","state":1}
//            {"cmd":"ping"}
void parsePiCommand(String line) {
  line.trim();
  if (line.length() < 5) return;

  Serial.print(F("[PI   ] Rx: ")); Serial.println(line);

  if (line.indexOf("\"cmd\":\"ping\"") >= 0) {
    Serial2.println(F("{\"pong\":1}"));
    Serial.println(F("[PI   ] Pong sent"));
    return;
  }

  if (line.indexOf("\"cmd\":\"stop\"") >= 0) {
    pi_leftSpeed   = 0;
    pi_rightSpeed  = 0;
    piDriveActive  = false;
    Serial.println(F("[PI   ] Stop command"));
    return;
  }

  if (line.indexOf("\"cmd\":\"drive\"") >= 0) {
    // Extract l and r values
    int lIdx = line.indexOf("\"l\":") + 4;
    int rIdx = line.indexOf("\"r\":") + 4;
    if (lIdx > 4 && rIdx > 4) {
      pi_leftSpeed  = constrain(line.substring(lIdx).toInt(), -MAX_SPEED, MAX_SPEED);
      pi_rightSpeed = constrain(line.substring(rIdx).toInt(), -MAX_SPEED, MAX_SPEED);
      piDriveActive = true;
      lastPiCmd     = millis();
      Serial.print(F("[PI   ] Drive L=")); Serial.print(pi_leftSpeed);
      Serial.print(F(" R="));             Serial.println(pi_rightSpeed);
    }
    return;
  }

  if (line.indexOf("\"cmd\":\"relay\"") >= 0) {
    int stateIdx = line.indexOf("\"state\":") + 8;
    bool state   = line.substring(stateIdx).toInt() == 1;
    if (line.indexOf("\"id\":\"motor\"") >= 0) {
      if (state && !relay_motor && armState == ARM_IDLE) {
        startArmSequence();
        Serial.println(F("[PI   ] Relay motor ON via Pi"));
      } else if (!state && relay_motor) {
        stopMotor();
        Serial.println(F("[PI   ] Relay motor OFF via Pi"));
      }
    } else if (line.indexOf("\"id\":\"turbo\"") >= 0) {
      relay_turbo = state;
      digitalWrite(RELAY_TURBO, relay_turbo ? HIGH : LOW);
      Serial.print(F("[PI   ] Relay turbo ")); Serial.println(state ? F("ON") : F("OFF"));
    }
    return;
  }

  Serial.print(F("[PI   ] Unknown command: ")); Serial.println(line);
}

// -- Encoder ISRs --------------------------------------------

// AS5600 PWM mode: pulse width 1us-4097us maps to 0-360 degrees
// We detect angle and count full rotations

// Read AS5600 PWM angle using pulseIn (blocking but accurate)
// Called from encoder task running on Core 0
int readAS5600Angle(uint8_t pin) {
  unsigned long high = pulseIn(pin, HIGH, 20000);  // timeout 20ms
  if (high == 0) return -1;  // timeout
  // AS5600 PWM: high pulse 1us-4097us = 0-360deg
  int angle = (int)((high - 1) * 360 / 4096);
  return constrain(angle, 0, 359);
}

// Encoder task runs on Core 0, leaving Core 1 for Bluepad32/main loop
void encoderTask(void* pvParameters) {
  int prev_L = -1;
  int prev_R = -1;
  for (;;) {
    // Left encoder
    int angle_L = readAS5600Angle(ENCODER_L_PIN);
    if (angle_L >= 0 && prev_L >= 0) {
      int diff = angle_L - prev_L;
      if (diff >  180) diff -= 360;
      if (diff < -180) diff += 360;
      if (abs(diff) < 90) {  // sanity check -- ignore jumps > 90deg
        enc_ticks_L += diff;
      }
    }
    if (angle_L >= 0) prev_L = angle_L;

    // Right encoder
    int angle_R = readAS5600Angle(ENCODER_R_PIN);
    if (angle_R >= 0 && prev_R >= 0) {
      int diff = angle_R - prev_R;
      if (diff >  180) diff -= 360;
      if (diff < -180) diff += 360;
      if (abs(diff) < 90) {
        enc_ticks_R += diff;
      }
    }
    if (angle_R >= 0) prev_R = angle_R;

    vTaskDelay(1 / portTICK_PERIOD_MS);  // 1ms delay between reads
  }
}



// -- Encoder update ------------------------------------------
void updateOdometry() {
  enc_ticks_L  = enc_L.getTicks();
  enc_ticks_R  = enc_R.getTicks();
  enc_dist_L_m = enc_ticks_L * DIST_PER_DEG_M;
  enc_dist_R_m = enc_ticks_R * DIST_PER_DEG_M;
}

// -- Setup ---------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(400);

  debugBanner("ESP32 TANK / MOWER CONTROLLER -- BLUEPAD32");
  Serial.print(F("  Chip     : ")); Serial.println(ESP.getChipModel());
  Serial.print(F("  CPU MHz  : ")); Serial.println(ESP.getCpuFreqMHz());
  Serial.print(F("  Free heap: ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
  debugSeparator();

  Serial.println(F("[INIT] Track motor pins..."));
  pinMode(MOTOR_A_DIR, OUTPUT); analogWrite(MOTOR_A_PWM, 0);
  pinMode(MOTOR_B_DIR, OUTPUT); analogWrite(MOTOR_B_PWM, 0);
  Serial.print(F("       Motor A  PWM=")); Serial.print(MOTOR_A_PWM);
  Serial.print(F(" DIR=")); Serial.println(MOTOR_A_DIR);
  Serial.print(F("       Motor B  PWM=")); Serial.print(MOTOR_B_PWM);
  Serial.print(F(" DIR=")); Serial.println(MOTOR_B_DIR);

  Serial.println(F("[INIT] Relay pins..."));
  pinMode(RELAY_ARM,   OUTPUT); digitalWrite(RELAY_ARM,   LOW);
  pinMode(RELAY_MOTOR, OUTPUT); digitalWrite(RELAY_MOTOR, LOW);
  pinMode(RELAY_TURBO, OUTPUT); digitalWrite(RELAY_TURBO, LOW);
  Serial.print(F("       ARM   relay pin=")); Serial.println(RELAY_ARM);
  Serial.print(F("       MOTOR relay pin=")); Serial.println(RELAY_MOTOR);
  Serial.print(F("       TURBO relay pin=")); Serial.println(RELAY_TURBO);

  Serial.println(F("[INIT] Starting Bluepad32..."));

  Serial.println(F("[INIT] Encoder pins..."));
  // EncAS5600 PWM mode — one object per encoder
  enc_L.begin(modetype_t::PWM);
  enc_L.setPwmPin(ENCODER_L_PIN);
  enc_L.start();
  enc_R.begin(modetype_t::PWM);
  enc_R.setPwmPin(ENCODER_R_PIN);
  enc_R.start();
  Serial.println(F("       EncAS5600 started in PWM mode"));
  Serial.print(F("       Dist per degree=")); Serial.print(DIST_PER_DEG_M * 1000.0f, 4); Serial.println(F("mm"));
  Serial.print(F("       Left  encoder pin=")); Serial.println(ENCODER_L_PIN);
  Serial.print(F("       Right encoder pin=")); Serial.println(ENCODER_R_PIN);
  Serial.print(F("       Dist per degree=")); Serial.print(DIST_PER_DEG_M * 1000.0f, 4); Serial.println(F("mm"));

  Serial.println(F("[INIT] Starting Pi serial bridge (Serial2)..."));
  Serial2.begin(PI_BAUD, SERIAL_8N1, PI_SERIAL_RX, PI_SERIAL_TX);
  Serial.print(F("       TX2=GPIO")); Serial.print(PI_SERIAL_TX);
  Serial.print(F(" RX2=GPIO"));       Serial.println(PI_SERIAL_RX);

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  // BP32.forgetBluetoothKeys();  // uncomment to force fresh pair on boot

  Serial.println(F("[BT  ] Ready -- hold PS button on controller to pair"));
  Serial.println(F("[BT  ] PS  = toggle tank mode (blue=dual / green=single)"));
  Serial.println(F("[BT  ] R1  = arm sequence then latch mower motor (press again to stop)"));
  Serial.println(F("[BT  ] L2  = squeeze to set D-pad speed ceiling (harder = faster)"));
  Serial.println(F("[BT  ] L1  = reset D-pad speed to zero"));
  Serial.println(F("[BT  ] TRI = toggle turbo relay (white flash while ON)"));
  debugSeparator();
}

// -- Main loop -----------------------------------------------

void loop() {

  // 1. Arm state machine (non-blocking, runs every iteration)
  updateArmStateMachine();

  // 1b. Odometry update
  updateOdometry();

  // 2. Pi serial bridge — read incoming commands
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      parsePiCommand(piCmdBuffer);
      piCmdBuffer = "";
    } else {
      if (piCmdBuffer.length() < 200) piCmdBuffer += c;
    }
  }

  // 3. Pi drive timeout — if no command for PI_CMD_TIMEOUT ms, stop Pi drive
  if (piDriveActive && (millis() - lastPiCmd > PI_CMD_TIMEOUT)) {
    piDriveActive = false;
    pi_leftSpeed  = 0;
    pi_rightSpeed = 0;
    Serial.println(F("[PI   ] Drive timeout -- Pi drive stopped"));
  }

  // 4. Send telemetry to Pi at PI_TELEM_MS interval
  if (millis() - lastTelemSent >= PI_TELEM_MS) {
    lastTelemSent = millis();
    sendTelemetry();
  }

  // 5. Timed rumble off
  if (rumbleOffTime && millis() >= rumbleOffTime) {
    rumbleOffTime = 0;
    if (gGamepad && gGamepad->isConnected()) {
      gGamepad->setRumble(0, 0);
      Serial.println(F("[RUMBLE] Buzz off"));
    }
  }

  // 6. Bluepad32 update
  bool updated = BP32.update();

  if (gGamepad == nullptr || !gGamepad->isConnected()) {
    static unsigned long lastWait = 0;
    if (millis() - lastWait > 3000) {
      lastWait = millis();
      Serial.println(F("[BT  ] Waiting for controller..."));
    }
    delay(100);
    return;
  }

  if (!updated) {
    updateLED();
    return;
  }

  GamepadPtr gp = gGamepad;

  // Uncomment to flood serial with raw data every frame:
  // debugRawSticks(gp);
  // debugButtons(gp);

  // -- PS button -> toggle tank mode -------------------------
  bool curPS = (gp->miscButtons() & MISC_BUTTON_HOME) != 0;
  if (curPS && !prev_PS) {
    tankMode = (tankMode == MODE_DUAL_STICK) ? MODE_SINGLE_STICK : MODE_DUAL_STICK;
    debugModeChange();
    if (!relay_motor && !relay_turbo) setLED(modeColour());
  }
  prev_PS = curPS;

  // -- R2 -> arm sequence + latch mower motor ----------------
  bool curR1btn = (gp->buttons() & BUTTON_SHOULDER_R);
  if (curR1btn && !prev_R1btn) {
    if (!relay_motor && armState == ARM_IDLE) {
      // Start sequence
      Serial.println(F("[BTN ] R1 pressed -> starting arm sequence"));;
      startArmSequence();
      gp->setRumble(0x20, 0x20);
      rumbleOffTime = millis() + 150;
      Serial.println(F("[RUMBLE] Arm start click"));
    } else if (relay_motor) {
      // Stop mower
      Serial.println(F("[BTN ] R1 pressed -> stopping mower motor"));
      stopMotor();
      gp->setRumble(0x10, 0x10);
      rumbleOffTime = millis() + 80;
      Serial.println(F("[RUMBLE] Motor unlatch click"));
    } else {
      // Sequence in progress -- ignore
      Serial.println(F("[BTN ] R1 ignored -- arm sequence in progress"));
    }
  }
  prev_R1btn = curR1btn;

  // -- L2 -> toggle turbo relay ------------------------------
  bool curTriangle = (gp->buttons() & BUTTON_Y) != 0;
  if (curTriangle && !prev_Tribtn) {
    relay_turbo = !relay_turbo;
    digitalWrite(RELAY_TURBO, relay_turbo ? HIGH : LOW);
    Serial.print(F("[BTN ] Triangle pressed -> TURBO relay "));
    Serial.println(relay_turbo ? F("ON") : F("OFF"));
    debugRelayState();
    if (relay_turbo) {
      Serial.println(F("[LED  ] Turbo flash ON (white <-> mode colour)"));
      lastFlash  = 0;
      flashState = false;
      gp->setRumble(0x40, 0x40);
      rumbleOffTime = millis() + RUMBLE_MS;
      Serial.print(F("[RUMBLE] Turbo latch buzz ON for ")); Serial.print(RUMBLE_MS); Serial.println(F("ms"));
    } else {
      Serial.println(F("[LED  ] Turbo flash OFF"));
      if (!relay_motor) {
        setLED(modeColour());
        Serial.print(F("[LED  ] Restored "));
        Serial.println(tankMode == MODE_DUAL_STICK ? F("BLUE") : F("GREEN"));
      }
      gp->setRumble(0x10, 0x10);
      rumbleOffTime = millis() + 80;
      Serial.println(F("[RUMBLE] Turbo unlatch click"));
    }
  }
  prev_Tribtn = curTriangle;

  // -- L2 analog -> update peak dpad speed ceiling -----------
  {
    int rawL2 = gp->brake();  // 0-1023
    if (rawL2 > peakL2) {
      peakL2    = rawL2;
      dpadSpeed = calcDpadSpeed(peakL2);
      Serial.print(F("[L2   ] New peak=")); Serial.print(peakL2);
      Serial.print(F(" -> dpad speed=")); Serial.println(dpadSpeed);
    }
  }

  // -- L1 -> reset dpad speed ceiling to zero ----------------
  bool curL1btn = (gp->buttons() & BUTTON_SHOULDER_L) != 0;
  if (curL1btn && !prev_L1btn) {
    peakL2    = 0;
    dpadSpeed = 0;
    Serial.println(F("[BTN ] L1 pressed -> D-pad speed RESET to zero"));
    gp->setRumble(0x20, 0x00);
    rumbleOffTime = millis() + 100;
  }
  prev_L1btn = curL1btn;

  // -- Track drive -------------------------------------------
  // D-pad takes priority over analog sticks when any direction pressed
  int leftSpeed  = 0;
  int rightSpeed = 0;
  bool dpadActive = false;

  {
    uint8_t dpad = gp->dpad();
    if (dpad & (DPAD_UP | DPAD_DOWN | DPAD_LEFT | DPAD_RIGHT)) {
      dpadActive = true;
      if (dpadSpeed > 0) {
        int spd = dpadSpeed;
        if      (dpad & DPAD_UP)    { leftSpeed =  spd; rightSpeed =  spd; }
        else if (dpad & DPAD_DOWN)  { leftSpeed = -spd; rightSpeed = -spd; }
        else if (dpad & DPAD_LEFT)  { leftSpeed = -spd; rightSpeed =  spd; }
        else if (dpad & DPAD_RIGHT) { leftSpeed =  spd; rightSpeed = -spd; }
        static unsigned long lastDpadDbg = 0;
        if (millis() - lastDpadDbg > 300) {
          lastDpadDbg = millis();
          Serial.print(F("[DPAD ] spd=")); Serial.print(spd);
          Serial.print(F(" L="));          Serial.print(leftSpeed);
          Serial.print(F(" R="));          Serial.println(rightSpeed);
        }
      } else {
        // D-pad pressed but no speed set yet
        static unsigned long lastDpadWarn = 0;
        if (millis() - lastDpadWarn > 1500) {
          lastDpadWarn = millis();
          Serial.println(F("[DPAD ] Speed=0 -- squeeze L2 to set speed ceiling"));
        }
      }
    }
  }

  // Analog sticks only if d-pad not active
  if (!dpadActive) {
    if (tankMode == MODE_DUAL_STICK) {
      leftSpeed  = stickToSpeed(gp->axisY());
      rightSpeed = stickToSpeed(gp->axisRY());
    } else {
      int throttle = stickToSpeed(gp->axisY());
      int turn     = stickToSpeed(gp->axisX());
      leftSpeed    = constrain(throttle + turn, -MAX_SPEED, MAX_SPEED);
      rightSpeed   = constrain(throttle - turn, -MAX_SPEED, MAX_SPEED);
      if (abs(turn) > 5) {
        static unsigned long lastTurnDbg = 0;
        if (millis() - lastTurnDbg > 200) {
          lastTurnDbg = millis();
          Serial.print(F("[MIX ] Throttle=")); Serial.print(throttle);
          Serial.print(F(" Turn="));           Serial.print(turn);
          Serial.print(F(" -> L="));           Serial.print(leftSpeed);
          Serial.print(F(" R="));              Serial.println(rightSpeed);
        }
      }
    }
  }

  // Pi drive overrides RC when active
  if (piDriveActive) {
    leftSpeed  = pi_leftSpeed;
    rightSpeed = pi_rightSpeed;
  }

  driveMotor(MOTOR_A_PWM, MOTOR_A_DIR, leftSpeed);
  driveMotor(MOTOR_B_PWM, MOTOR_B_DIR, rightSpeed);

  if (leftSpeed != prev_leftSpeed || rightSpeed != prev_rightSpeed) {
    debugDriveValues(leftSpeed, rightSpeed);
    prev_leftSpeed  = leftSpeed;
    prev_rightSpeed = rightSpeed;
  }

  // -- Periodic status dump ----------------------------------
  if (millis() - lastStatusPrint > STATUS_INTERVAL_MS) {
    lastStatusPrint = millis();
    Serial.print(F("[ENC  ] L_ticks=")); Serial.print(enc_ticks_L);
    Serial.print(F(" R_ticks="));          Serial.print(enc_ticks_R);
    Serial.print(F(" L_dist="));           Serial.print(enc_dist_L_m, 3);
    Serial.print(F("m R_dist="));          Serial.print(enc_dist_R_m, 3);
    Serial.print(F("m L_spd="));           Serial.print(enc_L.getSpeed(), 3);
    Serial.print(F(" R_spd="));            Serial.println(enc_R.getSpeed(), 3);
    Serial.print(F("[ENC  ] L_dir="));     Serial.print(enc_L.getRightDir() ? "CW" : "CCW");
    Serial.print(F(" R_dir="));            Serial.println(enc_R.getRightDir() ? "CW" : "CCW");
    Serial.print(F("[TICK] LX=")); Serial.print(gp->axisX());
    Serial.print(F(" LY="));      Serial.print(gp->axisY());
    Serial.print(F(" RX="));      Serial.print(gp->axisRX());
    Serial.print(F(" RY="));      Serial.print(gp->axisRY());
    Serial.print(F(" | L2="));    Serial.print(gp->brake());
    Serial.print(F(" R2="));      Serial.print(gp->throttle());
    Serial.print(F(" | Batt="));  Serial.print(gp->battery());
    Serial.println(F("%"));
    debugFullStatus();
  }

  updateLED();

  delay(20);  // ~50 Hz
}
