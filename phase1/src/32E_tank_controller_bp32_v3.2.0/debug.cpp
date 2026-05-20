#include "types.h"

void debugSep() {
  Serial.println(F("------------------------------------------"));
}

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
