#include "types.h"

// Invoked by BP32.update() when a gamepad connects.
void onConnectedGamepad(GamepadPtr gp) {
  debugBanner("CONTROLLER CONNECTED");
  GamepadProperties props = gp->getProperties();
  Serial.print(F("[CONN] BT addr: ")); printBTAddr(props.btaddr);
  gGamepad = gp;
  gp->setColorLED(COL_DUAL.r, COL_DUAL.g, COL_DUAL.b);
  gp->playDualRumble(0, RUMBLE_MS, 0x40, 0x40);
  Serial.println(F("[LED ] BLUE (dual stick)"));
}

// Invoked by BP32.update() when a gamepad disconnects.
// All outputs are forced safe immediately — this is the failsafe path.
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
