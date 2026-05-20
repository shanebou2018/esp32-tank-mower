#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// types.h — shared definitions, externs, and forward declarations
//           ESP32 Tank Mower v3.2.0
//
// Every .cpp file in this sketch includes ONLY this header.
// Library includes are centralised here so there is one place to audit them.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Bluepad32.h>
#include <Wire.h>
#include <PCF8575.h>
#include <EncAS5600.h>
#include "rom/gpio.h"   // gpio_matrix_out() — ROM fn, no macro conflicts
#include <esp_wifi.h>   // esp_wifi_stop()
#include <esp_bt.h>     // esp_bredr_tx_power_set()

// ── Pin definitions ───────────────────────────────────────────────────────────
// Motor serial: GPIO 25 → MDDS30 IN1 (TX-only, Serial Simplified).
#define MDDS30_TX_PIN   25
#define MDDS30_BAUD     115200   // DIP SW6-8=111 → 115200 baud

#define RELAY_ARM       32
#define RELAY_MOTOR     33
#define RELAY_TURBO     27   // moved from GPIO 15 (strapping pin — relay coil weak-pulled
                             // it low at boot, causing silent boot mode)

#define ENCODER_L_PIN   34
#define ENCODER_R_PIN   35

#define FLASH_MS        300

// ── I2C / PCF8575 ────────────────────────────────────────────────────────────
#define I2C_SDA         21
#define I2C_SCL         22
#define PCF8575_ADDR    0x20

#define PCF_BAT1_25     0
#define PCF_BAT1_50     1
#define PCF_BAT1_75     2
#define PCF_BAT1_100    3
#define PCF_BAT1_HEAT   4
#define PCF_BAT2_25     5
#define PCF_BAT2_50     6
#define PCF_BAT2_75     7
#define PCF_BAT2_100    8
#define PCF_BAT2_HEAT   9
#define PCF_MOWER_ERR   10
#define PCF_TURBO_FB    11
#define PCF_TURBO_BTN   12
#define PCF_LIGHTS_BTN  13

#define PCF_READ_MS      100
#define PCF_BTN_PULSE_MS 200

// ── Serial1 — Pi bridge + Pip-Boy (shared via GPIO matrix) ───────────────────
// UART1 TX primary pin: GPIO 17 → Pi RX.   UART1 RX: GPIO 13 ← Pi TX.
// gpio_matrix_out() in setup() also routes UART1 TX to GPIO 4 (Pip-Boy).
// Both Pi and Pip-Boy receive every Serial1 line; both run at PI_BAUD.
#define ESP2_SERIAL_TX  4       // Pip-Boy RX — routed from UART1 TX via GPIO matrix

// ── Pi serial bridge ──────────────────────────────────────────────────────────
#define PI_SERIAL_TX    17      // UART1 TX primary pin → Pi RX
#define PI_SERIAL_RX    13      // UART1 RX ← Pi TX
#define PI_BAUD         115200
#define PI_TELEM_MS     50
#define PI_CMD_TIMEOUT_MS  500

// ── Odometry ──────────────────────────────────────────────────────────────────
#define WHEEL_DIAMETER_M  0.254f
#define GEAR_RATIO        16.0f
#define DIST_PER_DEG_M    (3.14159265f * WHEEL_DIAMETER_M / (GEAR_RATIO * 360.0f))

// ── Tuning ────────────────────────────────────────────────────────────────────
#define DEADZONE        20
#define MAX_SPEED       100     // Cytron SmartDriveDuo serial simplified range -100..+100
#define MIN_MOTOR_SPEED 0       // Lowest speed sent to MDDS30 for any non-zero input.
                                // Set to the stall threshold (e.g. 15) when known.
#define STICK_MAX       511
#define DPAD_SPEED_MIN  0
#define ARM_PULSE_MS    500
#define RUMBLE_MS       300
#define STATUS_INTERVAL_MS 2000

// Blend between pure cubic (1.0) and pure linear (0.0).
//   1.0 = maximum expo: very slow near centre, snappy at edges
//   0.0 = linear:       direct 1:1 stick-to-speed
//   0.25 = light expo:  decisive response, still controllable at low speed
// Change this one number to retune — do NOT change stickToSpeed().
#define EXPO_BLEND      0.25f

// Set +1 (normal) or -1 (reverse) to correct motor direction without rewiring.
#define MOTOR_L_DIR     (+1)
#define MOTOR_R_DIR     (+1)

#define PIPBOY_INTERVAL_MS 500

// ── Pip-Boy scene engine ──────────────────────────────────────────────────────
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

// ── Drive enums & structs ─────────────────────────────────────────────────────
enum TankMode { MODE_DUAL_STICK, MODE_SINGLE_STICK };
enum ArmState { ARM_IDLE, ARM_ARMING };

struct Colour { uint8_t r, g, b; };

// Core 1 → IO task
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

// IO task → Core 1
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

// ── Colour constants ──────────────────────────────────────────────────────────
// static → one copy per translation unit; each is 3 bytes — trivial overhead.
static const Colour COL_DUAL   = {   0,   0, 255 };
static const Colour COL_SINGLE = {   0, 255,   0 };
static const Colour COL_RED    = { 255,   0,   0 };
static const Colour COL_WHITE  = { 255, 255, 255 };

// Indexed by TankMode (MODE_DUAL_STICK=0, MODE_SINGLE_STICK=1).
static const char* const modeNames[] = { "DUAL", "SINGLE" };

// ── Shared mutable globals ────────────────────────────────────────────────────
// Defined exactly once in the .ino; extern'd here for all .cpp files.

extern SemaphoreHandle_t g_mutex_c2i;
extern SemaphoreHandle_t g_mutex_i2c;
extern Ctrl2IO  g_c2i;
extern IO2Ctrl  g_i2c;

extern volatile long  enc_ticks_L;
extern volatile long  enc_ticks_R;
extern volatile float enc_dist_L_m;
extern volatile float enc_dist_R_m;

extern as5600config_t cfg_L, cfg_R;
extern EncAS5600     *enc_L, *enc_R;

extern PCF8575 pcf;
extern bool    pcfPresent;

extern GamepadPtr    gGamepad;
extern TankMode      tankMode;
extern ArmState      armState;
extern bool          relay_motor;
extern bool          relay_turbo;
extern unsigned long armTimer;

extern bool          showBatt;
extern int           peakL2;
extern int           dpadSpeed;
extern unsigned long lastFlash;
extern bool          flashState;

// ── Cross-file function declarations ─────────────────────────────────────────

// debug.cpp
void debugSep();
void debugBanner(const char* msg);
void printBTAddr(const uint8_t* addr);
void debugRelays();
void debugFullStatus();

// drive.cpp
void   setLED(Colour c);
Colour modeColour();
void   updateLED(bool motor, bool turbo);
int    stickToSpeed(int axis);
int    calcDpadSpeed(int rawL2);
int    scaleToMotor(int speed);
void   sendMotorBytes(int leftSpd, int rightSpd);
void   startArmSequence();
void   stopMotor();
void   updateArmStateMachine();

// bt_callbacks.cpp
void onConnectedGamepad(GamepadPtr gp);
void onDisconnectedGamepad(GamepadPtr gp);

// io_task.cpp
void publishCtrl2IO(int driveL, int driveR, bool piDriveActive);
void ioTask(void*);
