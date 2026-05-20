#include "types.h"

// ── Publish Core 1 snapshot to g_c2i ─────────────────────────────────────────
// Called from loop() each iteration. Non-blocking (mutex timeout = 0):
// if the IO task is mid-read we skip this update — it will get the next one.

void publishCtrl2IO(int driveL, int driveR, bool piDriveActive) {
  noInterrupts();
  long  tl = enc_ticks_L;
  long  tr = enc_ticks_R;
  float dl = enc_dist_L_m;
  float dr = enc_dist_R_m;
  interrupts();

  // getSpeed() called here on Core 1 where the ISRs are attached — safe from data race.
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

// ── PCF8575 helpers (Core 0 only) ─────────────────────────────────────────────

static uint8_t decodeBatLevel(uint16_t pins, uint8_t p25, uint8_t p50,
                               uint8_t p75, uint8_t p100) {
  bool b25  = pins & (1u << p25);
  bool b50  = pins & (1u << p50);
  bool b75  = pins & (1u << p75);
  bool b100 = pins & (1u << p100);
  if (!b25 && !b50 && !b75 && !b100) return 0xFF;  // no valid reading
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

// ── Pi serial bridge helpers (Core 0 only) ────────────────────────────────────

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
    if (snap.connected) return;  // controller has priority

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

// ── Pip-Boy scene engine (Core 0 only) ───────────────────────────────────────

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

// ── Pi telemetry TX (Core 0 only) ────────────────────────────────────────────

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

// ── IO task — runs on Core 0 ──────────────────────────────────────────────────

void ioTask(void*) {
  char          piCmdBuf[256];
  uint8_t       piCmdLen      = 0;
  bool          piDriveActive = false;
  int           pi_leftSpeed  = 0;
  int           pi_rightSpeed = 0;
  unsigned long lastPiCmd     = 0;

  unsigned long lastTelemSent  = 0;
  unsigned long lastPipBoySent = 0;
  unsigned long lastPCFRead    = 0;
  unsigned long turboBtnOff    = 0;
  unsigned long lightsBtnOff   = 0;

  bool    prev_mow_err = false;
  uint8_t bat1 = 0, bat2 = 0;
  bool    heat1 = false, heat2 = false;
  bool    mow_err = false, turbo_fb = false;
  bool    lights_on = false;

  for (;;) {
    unsigned long now = millis();

    // ── PCF8575 input poll (10 Hz) ────────────────────────────────────
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

    // ── PCF8575 button release timers ─────────────────────────────────
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

    // ── Pi bridge — receive commands (Serial1 RX, GPIO 13) ───────────
    while (Serial1.available()) {
      char ch = Serial1.read();
      if (ch == '\n') {
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
        piCmdBuf[piCmdLen++] = ch;
      }
    }

    // ── Pi drive timeout ──────────────────────────────────────────────
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

    // ── Telemetry to Pi at 20 Hz ──────────────────────────────────────
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

    // ── Pip-Boy scene update at 500ms ─────────────────────────────────
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
