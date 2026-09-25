/*
  PlantIQ — Modular Lux Cap Control + Option 2 Calibration (LOW -> HIGH)
  ESP32 Arduino Core v3.x compatible (NO ledcSetup / ledcAttachPin)

  Key change:
    - Use ledcAttach(pin, freq, resolution)
    - Use ledcWrite(pin, duty)   // write by PIN, not channel

  Behavior:
    - 12h ON / 12h OFF schedule (starts DAY at boot)
    - During DAY: keep canopy around TARGET_LUX (instantaneous)
    - Periodic calibration (mostly invisible):
        CALIB_LOW_FRAC  (e.g., 5%)  -> measure luxLow
        CALIB_HIGH_FRAC (e.g., 80%) -> measure luxHigh
      slopeLuxPerFrac ~= (luxHigh - luxLow)/(high-low)
    - Controller uses luxLow + slope to compute PWM command
    - Two LED channels (WHITE + RB) driven together, split by weights
*/

#include <Arduino.h>
#include <Wire.h>
#include <hp_BH1750.h>

// ---------------------- PIN / PWM CONFIG ----------------------
static const int WHITE_PWM_PIN = 25;
static const int RB_PWM_PIN    = 26;

static const uint32_t PWM_FREQ_HZ  = 20000;
static const uint8_t  PWM_RES_BITS = 12;
static const uint32_t PWM_MAX      = (1UL << PWM_RES_BITS) - 1;

// ---------------------- USER SETTINGS ----------------------
static const float TARGET_LUX   = 15000.0f; // instantaneous lux target during DAY
static const float MAX_PWM_FRAC = 0.80f;    // safety cap
static const float MIN_PWM_FRAC = 0.00f;

// Option 2 calibration levels (avoid full OFF)
static const float CALIB_LOW_FRAC  = 0.05f; // 5%
static const float CALIB_HIGH_FRAC = 0.80f; // 80% (often same as MAX_PWM_FRAC)

// LED channel split
static const float WHITE_WEIGHT = 0.75f;
static const float RB_WEIGHT    = 0.25f;

// Photoperiod (starts DAY at boot)
static const uint32_t ON_DURATION_MS  = 12UL * 60UL * 60UL * 1000UL;
static const uint32_t OFF_DURATION_MS = 12UL * 60UL * 60UL * 1000UL;

// Main control cadence
static const uint32_t CONTROL_PERIOD_MS = 500;

// Calibration cadence/timing
static const uint32_t CALIBRATE_EVERY_MS   = 5UL * 60UL * 1000UL; // every 5 min
static const uint32_t SETTLE_LOW_MS        = 300;
static const uint32_t SETTLE_HIGH_MS       = 500;
static const float    SLOPE_EMA_ALPHA      = 0.80f;
static const float    CMD_EMA_ALPHA        = 0.85f;
static const float    MIN_SLOPE_FOR_USE    = 200.0f; // lux per frac guard

// ---------------------- BH1750 ----------------------
hp_BH1750 bh1750;

// ---------------------- UTIL ----------------------
static inline uint32_t fracToDuty(float frac) {
  frac = constrain(frac, 0.0f, 1.0f);
  return (uint32_t)lroundf(frac * (float)PWM_MAX);
}

static void writeLedPWM(float overallFrac) {
  overallFrac = constrain(overallFrac, MIN_PWM_FRAC, MAX_PWM_FRAC);

  float whiteFrac = constrain(overallFrac * WHITE_WEIGHT, 0.0f, MAX_PWM_FRAC);
  float rbFrac    = constrain(overallFrac * RB_WEIGHT,    0.0f, MAX_PWM_FRAC);

  ledcWrite(WHITE_PWM_PIN, fracToDuty(whiteFrac));
  ledcWrite(RB_PWM_PIN,    fracToDuty(rbFrac));
}

static void allOff() {
  ledcWrite(WHITE_PWM_PIN, 0);
  ledcWrite(RB_PWM_PIN, 0);
}

// ---------------------- MODULE 1: Scheduler ----------------------
struct LightingScheduler {
  uint32_t cycleStartMs = 0;
  bool dayOn = true;

  void begin() {
    cycleStartMs = millis();
    dayOn = true; // start in DAY at boot
  }

  void update() {
    uint32_t now = millis();
    uint32_t elapsed = now - cycleStartMs;

    if (dayOn && elapsed >= ON_DURATION_MS) {
      dayOn = false;
      cycleStartMs = now;
      Serial.println(">> NIGHT (LEDs forced OFF)");
      allOff();
    } else if (!dayOn && elapsed >= OFF_DURATION_MS) {
      dayOn = true;
      cycleStartMs = now;
      Serial.println(">> DAY (Lux-controlled)");
    }
  }

  bool isDay() const { return dayOn; }
};

// ---------------------- MODULE 2: Lux Sensor ----------------------
struct LuxSensor {
  bool readLux(float &luxOut) {
    if (bh1750.hasValue()) {
      luxOut = bh1750.getLux();
      return true;
    }
    bh1750.start();
    return false;
  }
};

// ---------------------- MODULE 3: Option-2 Calibrator (LOW -> HIGH) ----------------------
struct LuxCalibrator {
  enum State { IDLE, LOW_WAIT, LOW_READ, HIGH_WAIT, HIGH_READ };
  State state = IDLE;

  uint32_t lastCalibMs = 0;
  uint32_t stateStartMs = 0;

  float luxLow  = NAN;
  float luxHigh = NAN;

  // slope: lux change per 1.0 PWM fraction (in the low->high region)
  float slopeLuxPerFracEma = 10000.0f; // initial guess so system can run

  bool isActive() const { return state != IDLE; }

  void begin() {
    lastCalibMs = 0;
    state = IDLE;
  }

  bool shouldStart(uint32_t now, bool isDay) {
    if (!isDay) return false;
    if (state != IDLE) return false;
    return (now - lastCalibMs) >= CALIBRATE_EVERY_MS;
  }

  void start(uint32_t now) {
    writeLedPWM(CALIB_LOW_FRAC);
    state = LOW_WAIT;
    stateStartMs = now;
  }

  bool update(uint32_t now, LuxSensor &sensor) {
    float lux;

    switch (state) {
      case IDLE:
        return false;

      case LOW_WAIT:
        if (now - stateStartMs >= SETTLE_LOW_MS) state = LOW_READ;
        return false;

      case LOW_READ:
        if (sensor.readLux(lux)) {
          luxLow = lux;
          writeLedPWM(CALIB_HIGH_FRAC);
          state = HIGH_WAIT;
          stateStartMs = now;
        }
        return false;

      case HIGH_WAIT:
        if (now - stateStartMs >= SETTLE_HIGH_MS) state = HIGH_READ;
        return false;

      case HIGH_READ:
        if (sensor.readLux(lux)) {
          luxHigh = lux;

          float luxDelta  = luxHigh - luxLow;
          float fracDelta = (CALIB_HIGH_FRAC - CALIB_LOW_FRAC);

          if (isfinite(luxDelta) && luxDelta > 0 && fracDelta > 0) {
            float slope = luxDelta / fracDelta; // lux per 1.0 frac
            if (slope > 0) {
              slopeLuxPerFracEma = SLOPE_EMA_ALPHA * slopeLuxPerFracEma
                                 + (1.0f - SLOPE_EMA_ALPHA) * slope;
            }
          }

          state = IDLE;
          lastCalibMs = now;

          Serial.print("CAL2 DONE | luxLow=");
          Serial.print(luxLow, 1);
          Serial.print(" | luxHigh=");
          Serial.print(luxHigh, 1);
          Serial.print(" | slopeLux/frac(Ema)=");
          Serial.println(slopeLuxPerFracEma, 1);

          return true;
        }
        return false;
    }

    return false;
  }

  float getLuxLow() const { return luxLow; }
  float getSlopeLuxPerFrac() const { return slopeLuxPerFracEma; }
};

// ---------------------- MODULE 4: Controller ----------------------
struct LuxController {
  float cmdFracEma = 0.0f;

  float computeCmd(float luxNow, float luxLow, float slopeLuxPerFrac) {
    float baseLux = isfinite(luxLow) ? luxLow : luxNow;
    if (!isfinite(baseLux)) return 0.0f;

    float needLux = TARGET_LUX - baseLux;
    if (needLux <= 0) return 0.0f;

    if (!isfinite(slopeLuxPerFrac) || slopeLuxPerFrac < MIN_SLOPE_FOR_USE) {
      // fallback: conservative
      float frac = CALIB_LOW_FRAC + (needLux / 8000.0f);
      return constrain(frac, MIN_PWM_FRAC, MAX_PWM_FRAC);
    }

    float addedFrac = needLux / slopeLuxPerFrac;
    float frac = CALIB_LOW_FRAC + addedFrac;

    return constrain(frac, MIN_PWM_FRAC, MAX_PWM_FRAC);
  }

  float smooth(float raw) {
    raw = constrain(raw, MIN_PWM_FRAC, MAX_PWM_FRAC);
    cmdFracEma = CMD_EMA_ALPHA * cmdFracEma + (1.0f - CMD_EMA_ALPHA) * raw;
    return cmdFracEma;
  }
};

// ---------------------- GLOBALS ----------------------
LightingScheduler scheduler;
LuxSensor sensor;
LuxCalibrator calib;
LuxController controller;

static uint32_t lastControlMs = 0;

// ---------------------- SETUP ----------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();

  if (!bh1750.begin(BH1750_TO_GROUND)) {
    Serial.println("BH1750 init failed. Check wiring/power.");
  } else {
    bh1750.setQuality(BH1750_QUALITY_HIGH);
    Serial.println("BH1750 OK.");
  }

  // ESP32 core v3.x PWM attach (pin-based)
  bool ok1 = ledcAttach(WHITE_PWM_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
  bool ok2 = ledcAttach(RB_PWM_PIN,    PWM_FREQ_HZ, PWM_RES_BITS);

  if (!ok1 || !ok2) {
    Serial.println("ERROR: ledcAttach failed for one/both LED pins.");
  }

  allOff();

  scheduler.begin();
  calib.begin();

  Serial.println("Lux control (core v3.x PWM) started.");
}

// ---------------------- LOOP ----------------------
void loop() {
  uint32_t now = millis();

  // 1) Day/night schedule
  scheduler.update();

  // 2) Night forces OFF
  if (!scheduler.isDay()) {
    allOff();
    delay(50);
    return;
  }

  // 3) Start periodic calibration
  if (calib.shouldStart(now, scheduler.isDay())) {
    calib.start(now);
  }

  // 4) Run calibration FSM
  calib.update(now, sensor);

  // 5) Control cadence
  if (now - lastControlMs < CONTROL_PERIOD_MS) return;
  lastControlMs = now;

  // Let calibration own LEDs temporarily
  if (calib.isActive()) return;

  // 6) Read current lux
  float luxNow = NAN;
  if (!sensor.readLux(luxNow)) {
    Serial.println("BH1750: waiting for reading...");
    return;
  }

  // 7) Compute + smooth command
  float rawCmd = controller.computeCmd(luxNow, calib.getLuxLow(), calib.getSlopeLuxPerFrac());
  float cmd    = controller.smooth(rawCmd);

  // 8) Apply to both channels
  writeLedPWM(cmd);

  // 9) Debug
  Serial.print("DAY | luxNow=");
  Serial.print(luxNow, 1);
  Serial.print(" | luxLowEst=");
  Serial.print(calib.getLuxLow(), 1);
  Serial.print(" | slopeLux/frac=");
  Serial.print(calib.getSlopeLuxPerFrac(), 1);
  Serial.print(" | cmdRaw=");
  Serial.print(rawCmd, 3);
  Serial.print(" | cmdEMA=");
  Serial.println(cmd, 3);
}
