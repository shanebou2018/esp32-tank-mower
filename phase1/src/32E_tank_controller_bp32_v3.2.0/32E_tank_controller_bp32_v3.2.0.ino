// ============================================================
// ESP32 Tank Mower — Phase 2 Firmware  v3.3.0
// Board: ESP32-DevKitC-32E (ESP32-WROOM-32E)
//
// v3.3.0: OTA update window (60 s WiFi at boot, hostname "tank-mower"),
//   speed category selector (Circle: 1×=Low 40%, 2×=Med 70%, 3×=Fast 100%;
//   hold 1.5 s resets to Low), deceleration smoothing (DECEL_RATE units/s).
//
// v3.2.0: refactored into multi-file sketch — no functional changes.
//   File structure:
//     32E_tank_controller_bp32_v3.2.0.ino  ← global defs, setup(), loop()
//     types.h          ← all #defines, enums, structs, externs, prototypes
//     debug.cpp        ← debugSep / debugBanner / printBTAddr / debugRelays / debugFullStatus
//     drive.cpp        ← LED / stick / motor / arm state machine
//     bt_callbacks.cpp ← onConnectedGamepad / onDisconnectedGamepad
//     io_task.cpp      ← publishCtrl2IO / ioTask / ioXxx helpers (static)
//
//   .cpp files bypass the Arduino IDE auto-prototype scanner entirely,
//   which eliminates the ordering constraint that previously required
//   functions to be placed after all type definitions.
//
// v3.1.0 changes (all preserved here):
//   Serial.setTxBufferSize(1024) — async UART so debugFullStatus() never
//     blocks loop() / BP32.update() (~61ms at 115200 baud otherwise)
//   esp_wifi_stop() — stops WiFi sharing the 2.4GHz radio with Classic BT
//   esp_bredr_tx_power_set(P9, P9) — raises Classic BT TX +3dBm → +9dBm
//   vTaskDelay(1) — yields Core 1 to btstack service tasks each iteration
//   Cytron MDDS30 Serial Simplified on Serial2 (GPIO 25)
//   Pi bridge on Serial1 (GPIO 17/13) + Pip-Boy via GPIO matrix (GPIO 4)
//   Expo curve: 0.25f blend (light cubic), single-stick X-axis sign corrected
//   scaleToMotor() map fixed (base 0, not 1)
//   MOTOR_L_DIR / MOTOR_R_DIR defines for direction without rewiring
//
// MDDS30 DIP switches: 11011111
//   SW1=ON  SW2=ON  → Serial mode
//   SW3=OFF          → Serial Simplified
//   SW4=ON  SW5=ON  → Independent Both  ← change SW4 from OFF if not done
//   SW6=ON  SW7=ON  SW8=ON → 115200 baud
//   Connect: GPIO 25 → MDDS30 IN1 only.  Disconnect AN1/AN2/IN2.
// ============================================================

#include "types.h"

// ── Global variable definitions ───────────────────────────────────────────────
// Declared extern in types.h; defined exactly once here.

SemaphoreHandle_t g_mutex_c2i;
SemaphoreHandle_t g_mutex_i2c;
Ctrl2IO  g_c2i = {};
IO2Ctrl  g_i2c = {};

volatile long  enc_ticks_L  = 0;
volatile long  enc_ticks_R  = 0;
volatile float enc_dist_L_m = 0.0f;
volatile float enc_dist_R_m = 0.0f;

as5600config_t cfg_L, cfg_R;
EncAS5600 *enc_L = nullptr;
EncAS5600 *enc_R = nullptr;

PCF8575 pcf(PCF8575_ADDR);
bool    pcfPresent = false;

GamepadPtr    gGamepad    = nullptr;
TankMode      tankMode    = MODE_DUAL_STICK;
ArmState      armState    = ARM_IDLE;
bool          relay_motor = false;
bool          relay_turbo = false;
unsigned long armTimer    = 0;

bool          showBatt    = false;
int           peakL2      = 0;
int           dpadSpeed   = 0;
unsigned long lastFlash   = 0;
bool          flashState  = false;

bool          otaWindowActive = false;
unsigned long otaWindowStart  = 0;
int           speedCat        = 3;   // 1=Low (40%), 2=Med (70%), 3=Fast (100%)

// ============================================================
// setup()
// ============================================================
void setup() {
  // ── Drive relay outputs LOW before anything else ──────────────────
  pinMode(RELAY_ARM,   OUTPUT); digitalWrite(RELAY_ARM,   LOW);
  pinMode(RELAY_MOTOR, OUTPUT); digitalWrite(RELAY_MOTOR, LOW);
  pinMode(RELAY_TURBO, OUTPUT); digitalWrite(RELAY_TURBO, LOW);

  // Async TX buffer MUST be set before Serial.begin().
  // Default tx_buffer_size=0 → synchronous writes → debugFullStatus()
  // blocks loop() for ~61ms at 115200 baud (700 bytes / 11520 Bps).
  // BP32.update() must fire every <10ms to keep the DS4 alive;
  // a 61ms gap is enough to trigger the controller disconnect timer.
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(400);
  debugBanner("ESP32-DevKitC-32E TANK MOWER — PHASE 2 v3.3.0");
  Serial.print(F("  Chip    : ")); Serial.println(ESP.getChipModel());
  Serial.print(F("  CPU MHz : ")); Serial.println(ESP.getCpuFreqMHz());
  Serial.print(F("  Heap    : ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
  debugSep();

  // ── WiFi + ArduinoOTA — 60-second update window at boot ──────────
  // WiFi connects in the background; OTA becomes available once connected.
  // loop() stops WiFi when the window expires or a controller pairs,
  // restoring full BT radio time for outdoor use.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    Serial.println(F("[OTA ] Receiving firmware..."));
  });
  ArduinoOTA.onEnd([]() {
    Serial.println(F("[OTA ] Done — rebooting"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static uint8_t lastPct = 255;
    uint8_t pct = (uint8_t)(progress * 100 / total);
    if (pct != lastPct) { lastPct = pct; Serial.print('.'); }
  });
  ArduinoOTA.onError([](ota_error_t err) {
    Serial.print(F("[OTA ] Error ")); Serial.println((int)err);
  });
  ArduinoOTA.begin();
  otaWindowActive = true;
  otaWindowStart  = millis();
  Serial.print(F("[OTA ] Window open (")); Serial.print(OTA_WINDOW_MS / 1000);
  Serial.print(F("s) — hostname: ")); Serial.println(OTA_HOSTNAME);
  Serial.print(F("  SSID: ")); Serial.println(WIFI_SSID);

  // ── MDDS30 motor serial — Serial2, TX-only on GPIO 25 ────────────
  Serial2.begin(MDDS30_BAUD, SERIAL_8N1, -1, MDDS30_TX_PIN);
  sendMotorBytes(0, 0);  // explicit stop on boot
  Serial.println(F("[INIT] MDDS30 serial OK (SmartDriveDuo Simplified)"));
  Serial.print(F("  TX=GPIO")); Serial.print(MDDS30_TX_PIN);
  Serial.print(F("  Baud=")); Serial.println(MDDS30_BAUD);
  Serial.println(F("  L: Motor1/ChA  R: Motor2/ChB  (GPIO25 → IN1)"));
  Serial.println(F("  Pi bridge on Serial1 (GPIO17 TX / GPIO13 RX)"));

  // ── I2C + PCF8575 — init here, then owned by Core 0 ──────────────
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

  // ── AS5600 encoders — ISRs attached on Core 1 ────────────────────
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

  // ── Serial1 — Pi bridge + Pip-Boy (shared UART, GPIO matrix) ─────
  // TX primary → GPIO 17 (Pi RX).  RX ← GPIO 13 (Pi TX).
  // gpio_matrix_out() additionally routes UART1 TX to GPIO 4 (Pip-Boy RX).
  // Declared via rom/gpio.h — no extern "C" needed.
  Serial1.begin(PI_BAUD, SERIAL_8N1, PI_SERIAL_RX, PI_SERIAL_TX);
  gpio_matrix_out(ESP2_SERIAL_TX, 23u, false, false); // 23 = U1TXD_OUT_IDX
  Serial.println(F("[INIT] Serial1: Pi bridge TX=GPIO17 RX=GPIO13 @ 115200"));
  Serial.println(F("         Pip-Boy GPIO4 also receives via GPIO matrix"));

  // ── RTOS mutexes ─────────────────────────────────────────────────
  g_mutex_c2i = xSemaphoreCreateMutex();
  g_mutex_i2c = xSemaphoreCreateMutex();
  Serial.println(F("[INIT] RTOS mutexes created"));

  // ── Launch IO task on Core 0 ──────────────────────────────────────
  xTaskCreatePinnedToCore(ioTask, "ioTask", 6144, nullptr, 1, nullptr, 0);
  Serial.println(F("[INIT] IO task started on Core 0 (stack 6144)"));

  // ── Bluepad32 ────────────────────────────────────────────────────
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  // ── BT RF optimisations ──────────────────────────────────────────
  // WiFi stays active during the OTA window.  loop() calls esp_wifi_stop()
  // when the window closes or a controller pairs, restoring full BT air time.

  // PS4 uses BR/EDR (Classic BT). Default TX power is +3dBm.
  // Raise both min and max to +9dBm (maximum) for full range.
  esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9);

  debugSep();
  Serial.println(F("[BT ] Classic BT TX power: +9dBm"));
  Serial.println(F("[BT ] Ready — hold PS to pair"));
  Serial.println(F("  PS    = toggle mode  (LED: blue=dual / green=single)"));
  Serial.println(F("  R1    = arm -> latch mower  (again to stop)"));
  Serial.println(F("  TRI   = toggle turbo relay"));
  Serial.println(F("  SQR   = toggle mower lights"));
  Serial.println(F("  CROSS = hold to show controller battery on LED"));
  Serial.println(F("  L2    = set D-pad speed ceiling"));
  Serial.println(F("  L1    = reset D-pad speed"));
  Serial.println(F("  CIRCLE= 1x=Low(40%) 2x=Med(70%) 3x=Fast(100%) hold=reset Low"));
  Serial.println(F("          (>3s between taps finalises; decel smoothed at stop)"));
  debugSep();
}

// ============================================================
// loop() — Core 1 (BT/motor hot path)
// ============================================================
void loop() {
  // ── Motor throttle state ──────────────────────────────────────────
  // Two-byte command at 20 Hz keepalive; fires immediately on speed change.
  static int           sentL       = 9999;
  static int           sentR       = 9999;
  static unsigned long lastMotorMs = 0;

  // ── Button edge-detect state (loop-local, not extern'd) ───────────
  static bool prev_PS    = false;
  static bool prev_R1    = false;
  static bool prev_TRI   = false;
  static bool prev_L1    = false;
  static bool prev_SQR   = false;
  static bool prev_CROSS = false;

  // ── Drive logging state (loop-local) ─────────────────────────────
  static int           prev_leftSpeed  = 9999;
  static int           prev_rightSpeed = 9999;
  static unsigned long lastStatusPrint = 0;
  static unsigned long lastDrvLog      = 0;

  // ── Deceleration smoothing state ──────────────────────────────────
  static unsigned long lastLoopMs = 0;
  static int           smoothL    = 0;
  static int           smoothR    = 0;

  // ── Speed category button state (Circle / BUTTON_B) ───────────────
  static bool          prev_B              = false;
  static int           speedBtnClicks      = 0;
  static unsigned long speedBtnLastClick   = 0;
  static unsigned long speedBtnPressStart  = 0;
  static bool          speedBtnHoldHandled = false;

  // ── 1. BP32 update — first for minimum BT latency ────────────────
  BP32.update();

  // ── 1b. OTA window ────────────────────────────────────────────────
  // Runs until 60 s timeout or a controller pairs; then WiFi stops so
  // Classic BT gets the full radio for outdoor range.
  if (otaWindowActive) {
    ArduinoOTA.handle();
    unsigned long otaNow    = millis();
    bool windowExpired      = (otaNow - otaWindowStart >= OTA_WINDOW_MS);
    bool ctrlConnected      = (gGamepad && gGamepad->isConnected());
    if (windowExpired || ctrlConnected) {
      otaWindowActive = false;
      esp_wifi_stop();
      Serial.println(windowExpired
        ? F("[OTA ] Window closed — WiFi stopped")
        : F("[OTA ] Controller paired — WiFi stopped"));
    }
  }

  // ── 2. Arm state machine ──────────────────────────────────────────
  updateArmStateMachine();

  // ── 3. Read IO2Ctrl snapshot (non-blocking) ───────────────────────
  static IO2Ctrl io = {};
  if (xSemaphoreTake(g_mutex_i2c, 0) == pdTRUE) {
    io = g_i2c;
    g_i2c.req_arm_start  = false;
    g_i2c.req_motor_stop = false;
    g_i2c.req_turbo_set  = false;
    g_i2c.req_rumble     = false;
    xSemaphoreGive(g_mutex_i2c);
  }

  // ── 4. Service IO task relay/motor requests ───────────────────────
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

  // ── 5. Pi drive timeout check ─────────────────────────────────────
  bool piDriveActive = io.piDriveActive;
  if (piDriveActive && (millis() - io.lastPiCmd > PI_CMD_TIMEOUT_MS)) {
    piDriveActive = false;
    Serial.println(F("[PI ] Drive timeout — stopped (Core 1)"));
  }

  // ── 6. Autonomous drive (no controller connected) ─────────────────
  if (gGamepad == nullptr || !gGamepad->isConnected()) {
    if (piDriveActive) {
      int scaledL = scaleToMotor(io.pi_leftSpeed);
      int scaledR = scaleToMotor(io.pi_rightSpeed);
      // Mirror Pi speeds into smooth state so handoff to gamepad starts clean.
      smoothL = io.pi_leftSpeed;
      smoothR = io.pi_rightSpeed;
      unsigned long nowM = millis();
      if (scaledL != sentL || scaledR != sentR || nowM - lastMotorMs >= 50) {
        sendMotorBytes(scaledL, scaledR);
        sentL = scaledL; sentR = scaledR; lastMotorMs = nowM;
      }
    } else {
      smoothL = 0;
      smoothR = 0;
    }
    int fakeL = piDriveActive ? io.pi_leftSpeed : 0;
    int fakeR = piDriveActive ? io.pi_rightSpeed : 0;
    publishCtrl2IO(fakeL, fakeR, piDriveActive);
    updateLED(relay_motor, relay_turbo);
    vTaskDelay(1);
    return;
  }

  GamepadPtr gp = gGamepad;

  // ── PS button → toggle tank mode ──────────────────────────────────
  bool curPS = (gp->miscButtons() & MISC_BUTTON_HOME) != 0;
  if (curPS && !prev_PS) {
    tankMode = (tankMode == MODE_DUAL_STICK) ? MODE_SINGLE_STICK : MODE_DUAL_STICK;
    Serial.print(F("[MODE] -> ")); Serial.println(modeNames[tankMode]);
    if (!relay_motor && !relay_turbo) setLED(modeColour());
  }
  prev_PS = curPS;

  // ── R1 → arm sequence / stop mower ───────────────────────────────
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

  // ── Triangle → toggle turbo relay ────────────────────────────────
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

  // ── L2 analog → D-pad speed ceiling (peak-hold) ──────────────────
  {
    int rawL2 = gp->brake();
    if (rawL2 > peakL2) {
      peakL2    = rawL2;
      dpadSpeed = calcDpadSpeed(peakL2);
      Serial.print(F("[L2 ] Peak=")); Serial.print(peakL2);
      Serial.print(F(" dpadSpeed=")); Serial.println(dpadSpeed);
    }
  }

  // ── L1 → reset D-pad speed ceiling ───────────────────────────────
  bool curL1 = (gp->buttons() & BUTTON_SHOULDER_L) != 0;
  if (curL1 && !prev_L1) {
    peakL2    = 0;
    dpadSpeed = 0;
    Serial.println(F("[BTN ] L1 — D-pad speed RESET"));
    gp->playDualRumble(0, 100, 0x20, 0x20);
  }
  prev_L1 = curL1;

  // ── Square → toggle mower lights (request to IO task) ────────────
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

  // ── Cross (hold) → show controller battery on LED ─────────────────
  // battery() returns 0=unknown, 1=empty, 255=full (not a percentage).
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

  // ── Circle (B) → speed category selector ─────────────────────────
  // 1 tap = Low (40%)   2 taps = Medium (70%)   3 taps = Fast (100%)
  // > 3 s between taps finalises the current count.
  // Hold > 1.5 s resets to Low without registering a tap.
  bool curB = (gp->buttons() & BUTTON_B) != 0;
  if (curB && !prev_B) {
    speedBtnPressStart  = millis();
    speedBtnHoldHandled = false;
  }
  if (curB && !speedBtnHoldHandled &&
      (millis() - speedBtnPressStart >= SPEED_HOLD_MS)) {
    speedCat            = 1;
    speedBtnClicks      = 0;
    speedBtnHoldHandled = true;
    gp->playDualRumble(0, 400, 0xFF, 0x40);
    Serial.println(F("[SPD ] Hold — reset to LOW (40%)"));
  }
  if (!curB && prev_B && !speedBtnHoldHandled) {
    speedBtnClicks++;
    speedBtnLastClick = millis();
    if (speedBtnClicks >= 3) {
      speedCat       = 3;
      speedBtnClicks = 0;
      gp->playDualRumble(0, 300, 0x80, 0xFF);
      Serial.println(F("[SPD ] 3 taps — FAST (100%)"));
    }
  }
  if (speedBtnClicks > 0 && !curB &&
      (millis() - speedBtnLastClick >= SPEED_BTN_TIMEOUT_MS)) {
    speedCat = (speedBtnClicks == 1) ? 1 : (speedBtnClicks == 2) ? 2 : 3;
    speedBtnClicks = 0;
    gp->playDualRumble(0, 200, 0x60, 0x60);
    Serial.print(F("[SPD ] Finalised -> "));
    Serial.println(speedCat == 1 ? F("LOW (40%)") :
                   speedCat == 2 ? F("MEDIUM (70%)") : F("FAST (100%)"));
  }
  prev_B = curB;

  // ── Drive ─────────────────────────────────────────────────────────
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
      // so passing axisX directly makes right-push = negative turn (inverted).
      // Pre-negate so rightward push = positive turn = robot turns right.
      int turn     = stickToSpeed(-gp->axisX());
      leftSpeed    = constrain(throttle + turn, -MAX_SPEED, MAX_SPEED);
      rightSpeed   = constrain(throttle - turn, -MAX_SPEED, MAX_SPEED);
    }
  }

  // Apply speed category multiplier (Circle button)
  {
    int catPct = (speedCat == 1) ? SPEED_CAT_LOW_PCT :
                 (speedCat == 2) ? SPEED_CAT_MED_PCT : 100;
    leftSpeed  = leftSpeed  * catPct / 100;
    rightSpeed = rightSpeed * catPct / 100;
  }

  // Deceleration smoothing + motor output
  {
    unsigned long nowM = millis();
    unsigned long dtMs = (lastLoopMs > 0) ? (nowM - lastLoopMs) : 10UL;
    if (dtMs > 100UL) dtMs = 100UL;
    lastLoopMs = nowM;

    smoothL = smoothDecel(smoothL, leftSpeed,  dtMs);
    smoothR = smoothDecel(smoothR, rightSpeed, dtMs);

    int scaledL = scaleToMotor(smoothL);
    int scaledR = scaleToMotor(smoothR);
    if (scaledL != sentL || scaledR != sentR || nowM - lastMotorMs >= 50) {
      sendMotorBytes(scaledL, scaledR);
      sentL = scaledL; sentR = scaledR; lastMotorMs = nowM;
    }
  }

  if ((leftSpeed != prev_leftSpeed || rightSpeed != prev_rightSpeed)
      && millis() - lastDrvLog >= 100) {
    lastDrvLog = millis();
    Serial.print(F("[DRV ] L=")); Serial.print(leftSpeed);
    Serial.print(F(" R="));      Serial.println(rightSpeed);
    prev_leftSpeed  = leftSpeed;
    prev_rightSpeed = rightSpeed;
  }

  // ── Periodic status dump ──────────────────────────────────────────
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

  // ── Publish Core 1 state to IO task ──────────────────────────────
  publishCtrl2IO(smoothL, smoothR, piDriveActive);

  updateLED(relay_motor, relay_turbo);

  // Yield 1 FreeRTOS tick so btstack service tasks pinned to Core 1
  // can run between loop() iterations.  No measurable impact on motor/BT
  // latency — motor is throttled to 20 Hz and BP32.update() is called first.
  vTaskDelay(1);
}
