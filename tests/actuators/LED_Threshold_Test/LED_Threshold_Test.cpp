/*
  TEST 4 — Non-blocking BH1750 read (minimal changes from your last fixed Test 4)
  ESP32 Arduino Core v3.x compatible (NO ledcSetup / ledcAttachPin)

  What changed vs the blocking version:
    - We keep bh1750.calibrateTiming() in setup (important)
    - In loop:
        * call bh1750.start() occasionally (if a new reading is needed)
        * only use bh1750.getLux() when bh1750.hasValue() is true
    - We hold lastLux so control/logging can continue even if a fresh sample isn't ready.
*/

#include <Arduino.h>
#include <Wire.h>
#include <hp_BH1750.h>

// ---------------------- PINS / PWM ----------------------
static const int WHITE_PWM_PIN = 25;
static const int RB_PWM_PIN    = 26;

static const uint32_t PWM_FREQ_HZ  = 200;
static const uint8_t  PWM_RES_BITS = 12;
static const uint32_t PWM_MAX      = (1UL << PWM_RES_BITS) - 1;

// Channel split
static const float WHITE_WEIGHT = 0.75f;
static const float RB_WEIGHT    = 0.25f;

// ---------------------- TEST 4 SETTINGS ----------------------
static const float LUX_ON_THRESHOLD  = 200.0f;
static const float LUX_OFF_THRESHOLD = 700.0f;

static const float FIXED_FRAC    = 0.30f;
static const float MAX_PWM_FRAC  = 0.80f;

static const uint32_t LOOP_PERIOD_MS   = 200; // control/log cadence
static const uint32_t BH1750_POLL_MS   = 120; // how often we try to start/collect a reading

// ---------------------- BH1750 ----------------------
hp_BH1750 bh1750;

// ---------------------- UTIL ----------------------
static inline uint32_t fracToDuty(float frac) {
  frac = constrain(frac, 0.0f, 1.0f);
  return (uint32_t)lroundf(frac * (float)PWM_MAX);
}

static void writeLedPWM(float overallFrac) {
  overallFrac = constrain(overallFrac, 0.0f, MAX_PWM_FRAC);

  float whiteFrac = constrain(overallFrac * WHITE_WEIGHT, 0.0f, MAX_PWM_FRAC);
  float rbFrac    = constrain(overallFrac * RB_WEIGHT,    0.0f, MAX_PWM_FRAC);

  ledcWrite(WHITE_PWM_PIN, fracToDuty(whiteFrac));
  ledcWrite(RB_PWM_PIN,    fracToDuty(rbFrac));
}

static void allOff() {
  ledcWrite(WHITE_PWM_PIN, 0);
  ledcWrite(RB_PWM_PIN, 0);
}

// ---------------------- STATE ----------------------
static bool ledsOn = false;

static uint32_t lastLoopMs = 0;
static uint32_t lastBhStartMs = 0;

static float lastLux = NAN;
static bool  haveLux = false;

// ---------------------- SETUP ----------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // Force known-good I2C pins
  Wire.begin(21, 22);

  // --- BH1750 init ---
  if (!bh1750.begin(BH1750_TO_GROUND)) {  // change to TO_VCC if ADDR tied high
    Serial.println("BH1750 begin failed!");
    while (true) delay(1000);
  }

  // Keep this fix
  bh1750.calibrateTiming();
  Serial.println("BH1750 ready (timing calibrated)");

  // --- PWM attach (ESP32 core v3.x) ---
  bool ok1 = ledcAttach(WHITE_PWM_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
  bool ok2 = ledcAttach(RB_PWM_PIN,    PWM_FREQ_HZ, PWM_RES_BITS);
  if (!ok1 || !ok2) {
    Serial.println("ERROR: ledcAttach failed!");
  }

  allOff();
  Serial.println("TEST 4 (non-blocking) started: Lux threshold -> LED ON/OFF");

  // Kick off the first measurement
  bh1750.start();
  lastBhStartMs = millis();
}

// ---------------------- LOOP ----------------------
void loop() {
  uint32_t now = millis();

  // ---- Non-blocking BH1750 sampling ----
  // Periodically start a measurement (safe even if one is already running in many libs)
  if (now - lastBhStartMs >= BH1750_POLL_MS) {
    bh1750.start();
    lastBhStartMs = now;
  }

  // If a value is ready, take it
  if (bh1750.hasValue()) {
    lastLux = bh1750.getLux();
    haveLux = true;
  }

  // ---- Control/log cadence ----
  if (now - lastLoopMs < LOOP_PERIOD_MS) return;
  lastLoopMs = now;

  if (!haveLux) {
    Serial.println("lux=NA | waiting for first reading...");
    return;
  }

  float lux = lastLux;

  // ---- Threshold control with hysteresis ----
  if (!ledsOn && lux < LUX_ON_THRESHOLD) {
    ledsOn = true;
    writeLedPWM(FIXED_FRAC);
    Serial.println("STATE CHANGE: LEDs -> ON");
  }

  if (ledsOn && lux > LUX_OFF_THRESHOLD) {
    ledsOn = false;
    allOff();
    Serial.println("STATE CHANGE: LEDs -> OFF");
  }

  // ---- Logging ----
  Serial.print("lux=");
  Serial.print(lux, 1);
  Serial.print(" | ledsOn=");
  Serial.print(ledsOn ? "YES" : "NO");
  Serial.print(" | cmdFrac=");
  Serial.println(ledsOn ? FIXED_FRAC : 0.0f, 2);
}
