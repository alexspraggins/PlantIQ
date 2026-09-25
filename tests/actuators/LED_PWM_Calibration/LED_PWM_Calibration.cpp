/*
  TEST 6 — Calibrator ONLY (Option 2: LOW -> HIGH -> MID validate)
  + Comfortable baseline + Calibrate every 10s + 1 Hz timer
  + FIXES retained:
      - EMA warm-up (initialize EMA from first slope)
      - Validation uses slopeInstant (this-cycle), EMA maintained for production use
      - Print slopeInst + slopeEma
  + NEW:
      - Compute and print an "ambient estimate" that subtracts LED contribution at LOW:
            ambientEst = luxLow - slopeEma * CALIB_LOW_FRAC
  + IMPROVEMENTS ADDED:
      (1) Minimum lux delta guard to prevent bad slope updates
      (2) Post-settle sample gating: each READ waits for a BH1750 sample timestamped
          after the settle window ends (prevents using an "old" lux value)

  ESP32 Arduino Core 2.0.17 compatible
*/

#include <Arduino.h>
#include <Wire.h>
#include <hp_BH1750.h>

// ---------------------- PINS / PWM ----------------------
static const int WHITE_PWM_PIN = 25;
static const int RB_PWM_PIN    = 26;

// ESP32 core 2.0.17 uses explicit LEDC channels
static const int WHITE_PWM_CH  = 0;
static const int RB_PWM_CH     = 1;

static const uint32_t PWM_FREQ_HZ  = 200;
static const uint8_t  PWM_RES_BITS = 12;
static const uint32_t PWM_MAX      = (1UL << PWM_RES_BITS) - 1;

// LED channel split
static const float WHITE_WEIGHT = 0.75f;
static const float RB_WEIGHT    = 0.25f;

// ---------------------- LEVELS ----------------------
static const float BASELINE_FRAC   = 0.25f; // comfortable between calibrations

// Current band (works well)
static const float CALIB_LOW_FRAC  = 0.15f; // lower to factor ambient more
static const float CALIB_HIGH_FRAC = 0.60f; // expanded high headroom

// Always keep validation at midpoint of band
static const float VALID_MID_FRAC  = (CALIB_LOW_FRAC + CALIB_HIGH_FRAC) * 0.5f;

// ---------------------- TIMING ----------------------
static const uint32_t CALIBRATE_EVERY_MS = 10000; // every 10 seconds

static const uint32_t SETTLE_LOW_MS  = 800;
static const uint32_t SETTLE_HIGH_MS = 1000;
static const uint32_t SETTLE_MID_MS  = 1000;

// EMA: alpha = "how much OLD you keep"
static const float SLOPE_EMA_ALPHA  = 0.80f;

// Non-blocking BH1750 polling
static const uint32_t BH1750_POLL_MS = 120;

// Timer prints
static uint32_t lastTimerPrintMs = 0;
static const uint32_t TIMER_PRINT_PERIOD_MS = 1000; // 1 Hz

// ---------------------- SAFETY / QUALITY GUARDS ----------------------
static const float MIN_LUX_DELTA_FOR_SLOPE = 150.0f; // tune 100–300 lux depending on setup

#if (SETTLE_LOW_MS + SETTLE_HIGH_MS + SETTLE_MID_MS) > (CALIBRATE_EVERY_MS / 2)
#warning "Calibration settle time is >50% of interval. Consider increasing CALIBRATE_EVERY_MS or reducing settle times."
#endif

// ---------------------- BH1750 ----------------------
hp_BH1750 bh1750;

// ---------------------- UTIL ----------------------
static inline uint32_t fracToDuty(float frac) {
  frac = constrain(frac, 0.0f, 1.0f);
  return (uint32_t)lroundf(frac * (float)PWM_MAX);
}

static void writeLedPWM(float overallFrac) {
  overallFrac = constrain(overallFrac, 0.0f, 1.0f);

  float whiteFrac = constrain(overallFrac * WHITE_WEIGHT, 0.0f, 1.0f);
  float rbFrac    = constrain(overallFrac * RB_WEIGHT,    0.0f, 1.0f);

  ledcWrite(WHITE_PWM_CH, fracToDuty(whiteFrac));
  ledcWrite(RB_PWM_CH,    fracToDuty(rbFrac));
}

// ---------------------- MODULE: Lux Sensor (non-blocking) ----------------------
struct LuxSensor {
  uint32_t lastStartMs = 0;

  float lastLux = NAN;
  bool haveLux = false;

  // Timestamp when the lux value was updated
  uint32_t lastLuxUpdateMs = 0;

  void begin() {
    lastStartMs = millis();
    bh1750.start(); // first conversion
  }

  void update() {
    uint32_t now = millis();

    // Kick conversions at a steady poll rate
    if (now - lastStartMs >= BH1750_POLL_MS) {
      bh1750.start();
      lastStartMs = now;
    }

    // Consume new readings when available
    if (bh1750.hasValue()) {
      lastLux = bh1750.getLux();
      haveLux = true;
      lastLuxUpdateMs = now; // approximate time of availability
    }
  }
};

LuxSensor sensor;

// ---------------------- MODULE: Calibrator FSM ----------------------
struct LuxCalibrator {
  enum State {
    IDLE,
    LOW_WAIT, LOW_READ,
    HIGH_WAIT, HIGH_READ,
    MID_WAIT, MID_READ
  } state = IDLE;

  uint32_t lastCalibMs  = 0; // last completion time
  uint32_t stateStartMs = 0;

  float luxLow  = NAN;
  float luxHigh = NAN;
  float luxMid  = NAN;

  float slopeInstant = NAN;

  float slopeEma = NAN;
  bool  haveSlopeEma = false;

  void begin() {
    lastCalibMs = millis();
    state = IDLE;
  }

  bool isActive() const { return state != IDLE; }

  bool shouldStart(uint32_t now) const {
    return (state == IDLE) && (now - lastCalibMs >= CALIBRATE_EVERY_MS);
  }

  void start(uint32_t now) {
    writeLedPWM(CALIB_LOW_FRAC);
    state = LOW_WAIT;
    stateStartMs = now;
    Serial.println("CAL START -> set LOW");
  }

  bool havePostSettleSample(const LuxSensor &s, uint32_t settleMs) const {
    return s.haveLux && (s.lastLuxUpdateMs >= (stateStartMs + settleMs));
  }

  void update(uint32_t now, const LuxSensor &s) {
    if (!s.haveLux) return;

    switch (state) {
      case IDLE:
        return;

      case LOW_WAIT:
        if ((now - stateStartMs >= SETTLE_LOW_MS) && havePostSettleSample(s, SETTLE_LOW_MS)) {
          state = LOW_READ;
        }
        return;

      case LOW_READ:
        luxLow = s.lastLux;
        writeLedPWM(CALIB_HIGH_FRAC);
        state = HIGH_WAIT;
        stateStartMs = now;
        Serial.print("CAL LOW  luxLow="); Serial.println(luxLow, 1);
        return;

      case HIGH_WAIT:
        if ((now - stateStartMs >= SETTLE_HIGH_MS) && havePostSettleSample(s, SETTLE_HIGH_MS)) {
          state = HIGH_READ;
        }
        return;

      case HIGH_READ: {
        luxHigh = s.lastLux;

        float luxDelta  = luxHigh - luxLow;
        float fracDelta = (CALIB_HIGH_FRAC - CALIB_LOW_FRAC);

        slopeInstant = NAN;
        if (isfinite(luxDelta) &&
            luxDelta > MIN_LUX_DELTA_FOR_SLOPE &&
            fracDelta > 0) {

          slopeInstant = luxDelta / fracDelta; // lux per 1.0 fraction

          if (!haveSlopeEma) {
            slopeEma = slopeInstant;
            haveSlopeEma = true;
          } else {
            slopeEma = SLOPE_EMA_ALPHA * slopeEma + (1.0f - SLOPE_EMA_ALPHA) * slopeInstant;
          }
        } else {
          Serial.print("CAL WARN | luxDelta too small or invalid: ");
          Serial.println(luxDelta, 1);
        }

        Serial.print("CAL HIGH luxHigh="); Serial.println(luxHigh, 1);

        writeLedPWM(VALID_MID_FRAC);
        state = MID_WAIT;
        stateStartMs = now;
        return;
      }

      case MID_WAIT:
        if ((now - stateStartMs >= SETTLE_MID_MS) && havePostSettleSample(s, SETTLE_MID_MS)) {
          state = MID_READ;
        }
        return;

      case MID_READ: {
        luxMid = s.lastLux;

        float luxMidPred = NAN;
        float err = NAN;
        float errPct = NAN;

        if (isfinite(slopeInstant)) {
          luxMidPred = luxLow + slopeInstant * (VALID_MID_FRAC - CALIB_LOW_FRAC);
          err = luxMid - luxMidPred;

          float expectedDelta = slopeInstant * (VALID_MID_FRAC - CALIB_LOW_FRAC);
          errPct = (fabsf(expectedDelta) > 1e-3f) ? (err / expectedDelta) * 100.0f : 0.0f;
        }

        float ambientEst = NAN;
        if (haveSlopeEma && isfinite(luxLow)) {
          ambientEst = luxLow - slopeEma * CALIB_LOW_FRAC;
          if (ambientEst < 0) ambientEst = 0;
        }

        Serial.print("CAL DONE | luxLow=");
        Serial.print(luxLow, 1);
        Serial.print(" | luxHigh=");
        Serial.print(luxHigh, 1);

        Serial.print(" | slopeInst=");
        Serial.print(isfinite(slopeInstant) ? slopeInstant : -1.0f, 1);

        Serial.print(" | slopeEma=");
        Serial.print(haveSlopeEma ? slopeEma : -1.0f, 1);

        Serial.print(" | ambientEst=");
        Serial.print(isfinite(ambientEst) ? ambientEst : -1.0f, 1);

        Serial.print(" | luxMid=");
        Serial.print(luxMid, 1);

        Serial.print(" | luxMidPred=");
        Serial.print(isfinite(luxMidPred) ? luxMidPred : -1.0f, 1);

        Serial.print(" | err=");
        Serial.print(isfinite(err) ? err : 0.0f, 1);

        Serial.print(" | errPct=");
        Serial.print(isfinite(errPct) ? errPct : 0.0f, 1);

        Serial.print(" | fracL=");
        Serial.print(CALIB_LOW_FRAC, 3);
        Serial.print(" fracH=");
        Serial.print(CALIB_HIGH_FRAC, 3);
        Serial.print(" fracM=");
        Serial.print(VALID_MID_FRAC, 3);

        Serial.println("%");

        writeLedPWM(BASELINE_FRAC);

        state = IDLE;
        lastCalibMs = now;
        return;
      }
    }
  }

  uint32_t secondsUntilNext(uint32_t now) const {
    if (state != IDLE) return 0;
    uint32_t elapsed = now - lastCalibMs;
    if (elapsed >= CALIBRATE_EVERY_MS) return 0;
    return (CALIBRATE_EVERY_MS - elapsed + 999) / 1000;
  }
};

LuxCalibrator calib;

// ---------------------- SETUP ----------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // Force I2C pins
  Wire.begin(21, 22);

  // BH1750 init
  if (!bh1750.begin(BH1750_TO_GROUND)) {
    Serial.println("BH1750 begin failed!");
    while (true) delay(1000);
  }

  bh1750.calibrateTiming();
  bh1750.setQuality(BH1750_QUALITY_HIGH);

  Serial.println("BH1750 ready (timing calibrated)");

  // ESP32 Arduino Core 2.0.17 LEDC setup
  ledcSetup(WHITE_PWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(RB_PWM_CH,    PWM_FREQ_HZ, PWM_RES_BITS);

  ledcAttachPin(WHITE_PWM_PIN, WHITE_PWM_CH);
  ledcAttachPin(RB_PWM_PIN,    RB_PWM_CH);

  // Baseline brightness
  writeLedPWM(BASELINE_FRAC);

  sensor.begin();
  calib.begin();

  Serial.println("TEST 6 started: baseline + calibration every 10s + timer + ambient estimate + guards.");
  Serial.print("Band: LOW=");
  Serial.print(CALIB_LOW_FRAC, 3);
  Serial.print(" HIGH=");
  Serial.print(CALIB_HIGH_FRAC, 3);
  Serial.print(" MID=");
  Serial.println(VALID_MID_FRAC, 3);
}

// ---------------------- LOOP ----------------------
void loop() {
  uint32_t now = millis();

  sensor.update();

  if (now - lastTimerPrintMs >= TIMER_PRINT_PERIOD_MS) {
    lastTimerPrintMs = now;

    if (calib.isActive()) {
      Serial.println("CAL TIMER | calibrating...");
    } else {
      Serial.print("CAL TIMER | next in ");
      Serial.print(calib.secondsUntilNext(now));
      Serial.println(" s");
    }
  }

  if (calib.shouldStart(now)) {
    calib.start(now);
  }

  calib.update(now, sensor);
}