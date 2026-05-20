#include "types.h"

// ── LED helpers ───────────────────────────────────────────────────────────────

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

// ── Stick curve ───────────────────────────────────────────────────────────────
// Converts a raw PS4 axis value (-511..+511) to a motor speed (-100..+100).
// PS4 Y-axes are negative-up, so val = -axis makes "up" produce positive speed.
// For X-axis (single-stick turn), pass -axisX() at the call site so rightward
// push produces a positive turn value (left wheel faster → robot turns right).

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

// Rescales any non-zero speed into MIN_MOTOR_SPEED..MAX_SPEED so the MDDS30
// always receives a duty cycle it can act on.
// Zero passes through unchanged — motor stops cleanly.
// Maps from base 0 (not 1) so speed=1 produces MIN_MOTOR_SPEED, not 0.
int scaleToMotor(int speed) {
  if (speed == 0) return 0;
  int mag = (int)map(abs(speed), 0, MAX_SPEED, MIN_MOTOR_SPEED, MAX_SPEED);
  return (speed > 0) ? mag : -mag;
}

// Cytron SmartDriveDuo Serial Simplified — two bytes per command.
//   Motor L fwd: 0x00 | map(spd, 0,100, 0,63)   rev: 0x40 | map(|spd|, 0,100, 0,63)
//   Motor R fwd: 0xC0 | map(spd, 0,100, 0,63)   rev: 0x80 | map(|spd|, 0,100, 0,63)
void sendMotorBytes(int leftSpd, int rightSpd) {
  // Apply per-motor direction correction. Flip MOTOR_L/R_DIR to reverse without rewiring.
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

// ── Arm state machine ─────────────────────────────────────────────────────────

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
