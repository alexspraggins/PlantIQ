/*
  TEST 5 — Scheduler only (DAY/NIGHT) + LED output (no sensor logic)
  ESP32 Arduino Core v3.x compatible (NO ledcSetup / ledcAttachPin)

  Goal:
    - Prove the photoperiod scheduler works (state transitions + timing)
    - Prove your "hard OFF at night" rule works
    - Keep structure aligned with final code:
        * Scheduler module
        * PWM helpers (two channels + weights)

  Test-friendly timing:
    - DAY = 10 seconds
    - NIGHT = 10 seconds
*/

#include <Arduino.h>

// ---------------------- PINS / PWM ----------------------
static const int WHITE_PWM_PIN = 25;
static const int RB_PWM_PIN    = 26;

static const uint32_t PWM_FREQ_HZ  = 200;
static const uint8_t  PWM_RES_BITS = 12;
static const uint32_t PWM_MAX      = (1UL << PWM_RES_BITS) - 1;

// Channel split
static const float WHITE_WEIGHT = 0.75f;
static const float RB_WEIGHT    = 0.25f;

// ---------------------- TEST 5 SETTINGS ----------------------
static const float DAY_BRIGHTNESS_FRAC = 0.25f; // fixed brightness during DAY

// SHORT durations for test (10s/10s)
static const uint32_t ON_DURATION_MS  = 10UL * 1000UL;
static const uint32_t OFF_DURATION_MS = 10UL * 1000UL;

// Optional periodic log during state (not just transitions)
static const uint32_t HEARTBEAT_MS = 1000;

// ---------------------- UTIL ----------------------
static inline uint32_t fracToDuty(float frac) {
  frac = constrain(frac, 0.0f, 1.0f);
  return (uint32_t)lroundf(frac * (float)PWM_MAX);
}

static void writeLedPWM(float overallFrac) {
  overallFrac = constrain(overallFrac, 0.0f, 1.0f);

  float whiteFrac = constrain(overallFrac * WHITE_WEIGHT, 0.0f, 1.0f);
  float rbFrac    = constrain(overallFrac * RB_WEIGHT,    0.0f, 1.0f);

  ledcWrite(WHITE_PWM_PIN, fracToDuty(whiteFrac));
  ledcWrite(RB_PWM_PIN,    fracToDuty(rbFrac));
}

static void allOff() {
  ledcWrite(WHITE_PWM_PIN, 0);
  ledcWrite(RB_PWM_PIN, 0);
}

// ---------------------- MODULE: Scheduler ----------------------
struct LightingScheduler {
  uint32_t cycleStartMs = 0;
  bool dayOn = true;

  void begin() {
    cycleStartMs = millis();
    dayOn = true; // start in DAY at boot
    Serial.println(">> START: DAY");
  }

  // returns true if state changed
  bool update() {
    uint32_t now = millis();
    uint32_t elapsed = now - cycleStartMs;

    if (dayOn && elapsed >= ON_DURATION_MS) {
      dayOn = false;
      cycleStartMs = now;
      Serial.println(">> TRANSITION: NIGHT (LEDs forced OFF)");
      return true;
    }
    if (!dayOn && elapsed >= OFF_DURATION_MS) {
      dayOn = true;
      cycleStartMs = now;
      Serial.println(">> TRANSITION: DAY (LEDs allowed)");
      return true;
    }
    return false;
  }

  bool isDay() const { return dayOn; }

  // seconds remaining in current state (for logging)
  uint32_t secondsRemaining() const {
    uint32_t now = millis();
    uint32_t elapsed = now - cycleStartMs;
    uint32_t dur = dayOn ? ON_DURATION_MS : OFF_DURATION_MS;
    if (elapsed >= dur) return 0;
    return (dur - elapsed + 999) / 1000;
  }
};

LightingScheduler scheduler;

static uint32_t lastHeartbeatMs = 0;

// ---------------------- SETUP ----------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // PWM attach (ESP32 core v3.x)
  bool ok1 = ledcAttach(WHITE_PWM_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
  bool ok2 = ledcAttach(RB_PWM_PIN,    PWM_FREQ_HZ, PWM_RES_BITS);
  if (!ok1 || !ok2) {
    Serial.println("ERROR: ledcAttach failed!");
  }

  allOff();
  scheduler.begin();
}

// ---------------------- LOOP ----------------------
void loop() {
  // Update schedule
  bool changed = scheduler.update();

  // Enforce outputs based on current state
  if (scheduler.isDay()) {
    writeLedPWM(DAY_BRIGHTNESS_FRAC);
  } else {
    allOff();
  }

  // Heartbeat logging once per second + on transitions
  uint32_t now = millis();
  if (changed || (now - lastHeartbeatMs >= HEARTBEAT_MS)) {
    lastHeartbeatMs = now;
    Serial.print(scheduler.isDay() ? "DAY  " : "NIGHT");
    Serial.print(" | secsRemaining=");
    Serial.print(scheduler.secondsRemaining());
    Serial.print(" | whiteDuty=");
    Serial.print(fracToDuty(DAY_BRIGHTNESS_FRAC * WHITE_WEIGHT));
    Serial.print(" | rbDuty=");
    Serial.println(fracToDuty(DAY_BRIGHTNESS_FRAC * RB_WEIGHT));
  }
}
