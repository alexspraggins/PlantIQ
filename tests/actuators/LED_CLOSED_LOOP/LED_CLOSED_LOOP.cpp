#include <Arduino.h>
#include <Wire.h>
#include <hp_BH1750.h>

// ===================== USER TARGET =====================
static const float TARGET_LUX = 400.0f;

// ===================== PINS / PWM ======================
static const int WHITE_PWM_PIN = 25;
static const int RB_PWM_PIN    = 26;

static const uint32_t PWM_FREQ_HZ  = 200;
static const uint8_t  PWM_RES_BITS = 12;
static const uint32_t PWM_MAX      = (1UL << PWM_RES_BITS) - 1;

static const float WHITE_WEIGHT = 0.75f;
static const float RB_WEIGHT    = 0.25f;

static const float PWM_MIN_FRAC = 0.15f;
static const float PWM_MAX_FRAC = 0.60f;

// ===================== SENSOR TIMING ====================
static const uint32_t BH1750_POLL_MS = 120;
static const uint32_t LUX_STALE_MS   = 1000;

// ===================== CONTROL TIMING ===================
static const uint32_t CONTROL_PERIOD_MS = 250;

// ===================== CONTROL TUNING ===================
static const float RATE_UP_PER_SEC   = 0.03f;
static const float RATE_DOWN_PER_SEC = 0.09f;
static const float PWM_EMA_ALPHA     = 0.85f;

static const float   HOLD_ENTER_LUX       = 6.0f;
static const float   HOLD_EXIT_LUX        = 35.0f;
static const float   HOLD_ABOVE_GUARD_LUX = -2.0f;
static const float   ERR_EMA_ALPHA        = 0.80f;
static const uint8_t HOLD_ENTER_TICKS     = 2;
static const uint8_t HOLD_EXIT_TICKS      = 4;

static const float   HOLD_FAST_EXIT_ERR_LUX = 40.0f;
static const uint8_t HOLD_FAST_EXIT_TICKS  = 2;

static const float OVERSHOOT_CLAMP_LUX = 50.0f;
static const float OVERSHOOT_EXIT_LUX  = 20.0f;

// ===================== AMBIENT ==========================
static const uint32_t AMBIENT_PERIOD_MS   = 60000;
static const uint32_t AMBIENT_OFF_HOLD_MS = 250;
static const uint32_t AMBIENT_MIN_GAP_MS  = 1500;

// ---- Ambient-stale trigger (OPTION A) ----
// OPTION A = only evaluate ambient-stale while HOLDING (prevents overreaction)
static const float    PWM_STABLE_EPS    = 0.0015f;
static const uint32_t AMB_STABLE_MIN_MS = 2000;
static const float    AMB_LUX_DRIFT_LUX = 25.0f;
static const uint8_t  AMB_DRIFT_TICKS   = 3;

// ===================== CALIBRATION ======================
static const uint32_t CAL_EVERY_MS  = 30000;
static const float    CAL_LOW_FRAC  = 0.15f;
static const float    CAL_HIGH_FRAC = 0.60f;
static const float    CAL_MID_FRAC  = (CAL_LOW_FRAC + CAL_HIGH_FRAC) * 0.5f;

static const uint32_t SETTLE_LOW_MS  = 500;
static const uint32_t SETTLE_HIGH_MS = 650;
static const uint32_t SETTLE_MID_MS  = 650;

static const float SLOPE_EMA_ALPHA           = 0.80f;
static const float MIN_LED_LUX_DELTA_FOR_SLOPE = 50.0f;

// ===================== POST-STEP GUARD ==================
// Only act on lux readings after big PWM steps (cal/ambient/clamp/enter-hold).
static uint32_t lastOutputStepMs = 0;
static const uint32_t POST_STEP_GUARD_MS = 450;

// ===================== STALE SUSPEND WINDOW =============
static uint32_t suspendStaleUntilMs = 0;
static const uint32_t STALE_SUSPEND_AFTER_STEP_MS = 3000;

// ===================== BH1750 ===========================
hp_BH1750 bh1750;

// ===================== PWM HELPERS ======================
static inline uint32_t fracToDuty(float frac) {
  frac = constrain(frac, 0.0f, 1.0f);
  return (uint32_t)lroundf(frac * (float)PWM_MAX);
}

static void writeLedPWM(float overallFrac) {
  overallFrac = constrain(overallFrac, 0.0f, 1.0f);
  const float w = constrain(overallFrac * WHITE_WEIGHT, 0.0f, 1.0f);
  const float r = constrain(overallFrac * RB_WEIGHT,    0.0f, 1.0f);
  ledcWrite(WHITE_PWM_PIN, fracToDuty(w));
  ledcWrite(RB_PWM_PIN,    fracToDuty(r));
}

static float lastWrittenPwm = NAN;

static void applyPWMValidatedIfChanged(float frac) {
  frac = constrain(frac, PWM_MIN_FRAC, PWM_MAX_FRAC);
  if (!isfinite(lastWrittenPwm) || fabsf(frac - lastWrittenPwm) > 0.0005f) {
    writeLedPWM(frac);
    lastWrittenPwm = frac;
  }
}

static void applyPWMAnyImmediate(float frac) {
  frac = constrain(frac, 0.0f, 1.0f);
  writeLedPWM(frac);
  lastWrittenPwm = frac;
}

// ---------- Forward declaration ----------
struct LuxSensor;

// ===================== GLOBAL STATE =====================
static float pwmCmd = 0.25f;
static float pwmEma = 0.25f;
static float errEma = 0.0f;

static bool overshootClamped = false;
static bool holding          = false;
static uint8_t enterCount    = 0;
static uint8_t exitCount     = 0;
static uint8_t fastExitCount = 0;

static inline void resetControlFlags() {
  overshootClamped = false;
  holding = false;
  enterCount = exitCount = fastExitCount = 0;
}

// ===================== AMBIENT SAMPLER ==================
static float    ambientLux  = 0.0f;
static bool     haveAmbient = false;
static uint32_t lastAmbientMs = 0;

enum AmbientState : uint8_t { AMB_IDLE, AMB_OFF, AMB_WAIT, AMB_READ, AMB_RESTORE };
static AmbientState ambState = AMB_IDLE;
static uint32_t ambStateMs = 0;
static float pwmBeforeAmbient = 0.25f;

static inline bool ambientActive() { return ambState != AMB_IDLE; }

// Ambient-stale detector state
static bool     forceAmbient = false;
static float    pwmStableRef = NAN;
static float    luxStableRef = NAN;
static uint32_t stableSinceMs = 0;
static uint8_t  driftCount = 0;

// ===================== STARTUP SEQUENCER ================
enum BootPhase : uint8_t { BOOT_AMBIENT, BOOT_CALIB, BOOT_RUN };
static BootPhase bootPhase = BOOT_AMBIENT;

// ===================== LUX SENSOR =======================
struct LuxSensor {
  uint32_t lastStartMs = 0;
  float    lux         = NAN;
  bool     haveLux     = false;
  uint32_t luxUpdateMs = 0;

  void begin() {
    lastStartMs = millis();
    bh1750.start();
  }

  void update() {
    const uint32_t now = millis();

    if (now - lastStartMs >= BH1750_POLL_MS) {
      bh1750.start();
      lastStartMs = now;
    }
    if (bh1750.hasValue()) {
      lux = bh1750.getLux();
      haveLux = true;
      luxUpdateMs = now;
    }
  }

  bool fresh(uint32_t now) const {
    return haveLux && (now - luxUpdateMs <= LUX_STALE_MS);
  }
};

static LuxSensor sensor;

// ===================== CALIBRATOR =======================
class Calibrator {
public:
  enum State : uint8_t { IDLE, LOW_WAIT, LOW_READ, HIGH_WAIT, HIGH_READ, MID_WAIT, MID_READ };

  void begin();
  bool active() const;
  bool due(uint32_t now) const;
  void start(uint32_t now, float currentPwm);
  void update(uint32_t now, const LuxSensor& s, float ambientNow);

  bool  haveSlope = false;
  float slopeEma  = NAN;

private:
  bool postSettleSample(const LuxSensor& s, uint32_t settleMs) const;
  void finish(uint32_t now);

  State    st         = IDLE;
  uint32_t lastDoneMs  = 0;
  uint32_t stStartMs   = 0;

  float luxLow = NAN, luxHigh = NAN, luxMid = NAN;
  float ledLow = NAN, ledHigh = NAN, ledMid = NAN;

  float slopeInst    = NAN;
  float pwmBeforeCal = 0.25f;
};

static Calibrator calib;

// ===================== UTILS ============================
static inline float ledLuxNow() { return max(0.0f, sensor.lux - ambientLux); }
static inline float targetLedNow() { return max(0.0f, TARGET_LUX - ambientLux); }

static void printCtrlLine(float ledLux, float targetLed, float ledErr, const char* evt) {
  Serial.print("CTRL | lux=");   Serial.print(sensor.lux, 1);
  Serial.print(" amb=");         Serial.print(ambientLux, 1);
  Serial.print(" led=");         Serial.print(ledLux, 1);
  Serial.print(" tLed=");        Serial.print(targetLed, 1);
  Serial.print(" err=");         Serial.print(ledErr, 1);
  Serial.print(" errEma=");      Serial.print(errEma, 1);
  Serial.print(" pwm=");         Serial.print(pwmEma, 3);
  Serial.print(" slope=");       Serial.print(calib.slopeEma, 1);
  if (evt && evt[0]) { Serial.print(" | evt="); Serial.print(evt); }
  Serial.println();
}

// Big-step marker: prevents “pre-step lux” and stale detector spam after abrupt PWM changes
static void markOutputStep(uint32_t now) {
  lastOutputStepMs = now;
  suspendStaleUntilMs = now + STALE_SUSPEND_AFTER_STEP_MS;
  forceAmbient = false;
}

// ===================== OPTION A: STALE DETECTOR (HOLD ONLY) =========
static void resetAmbientStaleDetector(uint32_t now) {
  pwmStableRef  = pwmEma;
  luxStableRef  = sensor.lux;
  stableSinceMs = now;
  driftCount    = 0;
}

// OPTION A: only evaluate ambient-stale while HOLDING (reduces overreaction)
static void ambientStaleTick(uint32_t now) {
  if (!holding) return;                 // <<< OPTION A
  if (!sensor.fresh(now) || !haveAmbient) return;

  if (now < suspendStaleUntilMs) return;
  if (sensor.luxUpdateMs < (lastOutputStepMs + POST_STEP_GUARD_MS)) return;

  if (!isfinite(pwmStableRef) || !isfinite(luxStableRef)) {
    resetAmbientStaleDetector(now);
    return;
  }

  if (fabsf(pwmEma - pwmStableRef) > PWM_STABLE_EPS) {
    resetAmbientStaleDetector(now);
    return;
  }

  if (now - stableSinceMs < AMB_STABLE_MIN_MS) return;

  const float luxDrift = fabsf(sensor.lux - luxStableRef);
  if (luxDrift >= AMB_LUX_DRIFT_LUX) {
    if (++driftCount >= AMB_DRIFT_TICKS) forceAmbient = true;
  } else {
    driftCount = 0;
  }
}

static bool ambientDue(uint32_t now) {
  if (ambientActive()) return false;
  if (now - lastAmbientMs < AMBIENT_MIN_GAP_MS) return false;
  if (forceAmbient) return true;
  return (now - lastAmbientMs >= AMBIENT_PERIOD_MS);
}

// ===================== CALIBRATION GATE =================
static bool okToCalibrateNow(float ledErr) {
  if (holding) return false;
  if (overshootClamped) return false;
  if (fabsf(ledErr) > 25.0f) return false;
  return true;
}

// ===================== AMBIENT FSM ======================
static void ambientStart(uint32_t now, const char* reason) {
  pwmBeforeAmbient = pwmEma;
  resetControlFlags();
  ambState = AMB_OFF;
  ambStateMs = now;
  Serial.println(reason);
}

static void ambientTick(uint32_t now) {
  if (!ambientActive()) return;

  switch (ambState) {
    case AMB_OFF:
      applyPWMAnyImmediate(0.0f);
      ambState = AMB_WAIT;
      ambStateMs = now;
      break;

    case AMB_WAIT:
      if (now - ambStateMs >= AMBIENT_OFF_HOLD_MS) ambState = AMB_READ;
      break;

    case AMB_READ:
      if (sensor.haveLux) {
        ambientLux = max(0.0f, sensor.lux);
        haveAmbient = true;
        lastAmbientMs = now;
        Serial.print("AMB | READ ambientLux="); Serial.println(ambientLux, 1);
      }
      ambState = AMB_RESTORE;
      break;

    case AMB_RESTORE:
      pwmCmd = constrain(pwmBeforeAmbient, PWM_MIN_FRAC, PWM_MAX_FRAC);
      pwmEma = pwmCmd;
      applyPWMValidatedIfChanged(pwmEma);

      errEma = 0.0f;
      ambState = AMB_IDLE;

      markOutputStep(now);
      if (sensor.fresh(now)) resetAmbientStaleDetector(now);

      Serial.println("AMB | RESTORE");
      break;

    case AMB_IDLE:
    default:
      break;
  }
}

// ===================== CALIBRATOR DEFINITIONS ===========
void Calibrator::begin() {
  lastDoneMs = millis();
  st = IDLE;
}

bool Calibrator::active() const { return st != IDLE; }

bool Calibrator::due(uint32_t now) const {
  return (st == IDLE) && (now - lastDoneMs >= CAL_EVERY_MS);
}

bool Calibrator::postSettleSample(const LuxSensor& s, uint32_t settleMs) const {
  return s.haveLux && (s.luxUpdateMs >= (stStartMs + settleMs));
}

void Calibrator::start(uint32_t now, float currentPwm) {
  pwmBeforeCal = currentPwm;
  applyPWMValidatedIfChanged(CAL_LOW_FRAC);
  st = LOW_WAIT;
  stStartMs = now;
  Serial.println("CAL | START -> LOW");
}

void Calibrator::finish(uint32_t now) {
  st = IDLE;
  lastDoneMs = now;

  applyPWMValidatedIfChanged(constrain(pwmBeforeCal, PWM_MIN_FRAC, PWM_MAX_FRAC));

  markOutputStep(now);
  if (sensor.fresh(now)) resetAmbientStaleDetector(now);

  Serial.println("CAL | RESTORE PWM");
}

void Calibrator::update(uint32_t now, const LuxSensor& s, float ambientNow) {
  if (!s.haveLux) return;

  switch (st) {
    case IDLE: return;

    case LOW_WAIT:
      if ((now - stStartMs >= SETTLE_LOW_MS) && postSettleSample(s, SETTLE_LOW_MS)) st = LOW_READ;
      return;

    case LOW_READ:
      luxLow = s.lux;
      ledLow = max(0.0f, luxLow - ambientNow);

      Serial.print("CAL | LOW  lux="); Serial.print(luxLow, 1);
      Serial.print(" led=");           Serial.println(ledLow, 1);

      applyPWMValidatedIfChanged(CAL_HIGH_FRAC);
      st = HIGH_WAIT;
      stStartMs = now;
      return;

    case HIGH_WAIT:
      if ((now - stStartMs >= SETTLE_HIGH_MS) && postSettleSample(s, SETTLE_HIGH_MS)) st = HIGH_READ;
      return;

    case HIGH_READ: {
      luxHigh = s.lux;
      ledHigh = max(0.0f, luxHigh - ambientNow);

      Serial.print("CAL | HIGH lux="); Serial.print(luxHigh, 1);
      Serial.print(" led=");           Serial.println(ledHigh, 1);

      const float ledDelta  = ledHigh - ledLow;
      const float fracDelta = (CAL_HIGH_FRAC - CAL_LOW_FRAC);

      slopeInst = NAN;
      if (isfinite(ledDelta) && ledDelta > MIN_LED_LUX_DELTA_FOR_SLOPE && fracDelta > 1e-6f) {
        slopeInst = ledDelta / fracDelta;
        if (!haveSlope) { slopeEma = slopeInst; haveSlope = true; }
        else { slopeEma = SLOPE_EMA_ALPHA * slopeEma + (1.0f - SLOPE_EMA_ALPHA) * slopeInst; }
      } else {
        Serial.print("CAL | WARN ledDelta invalid/small: ");
        Serial.println(ledDelta, 1);
      }

      applyPWMValidatedIfChanged(CAL_MID_FRAC);
      st = MID_WAIT;
      stStartMs = now;
      return;
    }

    case MID_WAIT:
      if ((now - stStartMs >= SETTLE_MID_MS) && postSettleSample(s, SETTLE_MID_MS)) st = MID_READ;
      return;

    case MID_READ: {
      luxMid = s.lux;
      ledMid = max(0.0f, luxMid - ambientNow);

      float errPct = NAN;
      if (isfinite(slopeInst)) {
        const float ledMidPred = ledLow + slopeInst * (CAL_MID_FRAC - CAL_LOW_FRAC);
        const float err = ledMid - ledMidPred;
        const float expected = slopeInst * (CAL_MID_FRAC - CAL_LOW_FRAC);
        errPct = (fabsf(expected) > 1e-3f) ? (err / expected) * 100.0f : 0.0f;
      }

      Serial.print("CAL | DONE slopeEma=");
      Serial.print(haveSlope ? slopeEma : -1.0f, 1);
      Serial.print(" ambient=");
      Serial.print(ambientNow, 1);
      Serial.print(" errPct=");
      Serial.print(isfinite(errPct) ? errPct : 0.0f, 1);
      Serial.println("%");

      finish(now);
      return;
    }
  }
}

// ===================== SETUP ============================
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(21, 22);

  if (!bh1750.begin(BH1750_TO_GROUND)) {
    Serial.println("BH1750 init failed");
    while (true) delay(1000);
  }
  bh1750.calibrateTiming();
  bh1750.setQuality(BH1750_QUALITY_HIGH);

  const bool ok1 = ledcAttach(WHITE_PWM_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
  const bool ok2 = ledcAttach(RB_PWM_PIN,    PWM_FREQ_HZ, PWM_RES_BITS);
  if (!ok1 || !ok2) Serial.println("ERROR: ledcAttach failed!");

  pwmCmd = constrain(pwmCmd, PWM_MIN_FRAC, PWM_MAX_FRAC);
  pwmEma = constrain(pwmEma, PWM_MIN_FRAC, PWM_MAX_FRAC);
  applyPWMValidatedIfChanged(pwmEma);

  sensor.begin();
  calib.begin();

  markOutputStep(millis());

  Serial.println("TEST 8 — High/Low calib + LED-only control + ambient sampler (Option A stale trigger: HOLD-only)");
}

// ===================== LOOP =============================
void loop() {
  const uint32_t now = millis();
  sensor.update();

  // ===================== BOOT: AMBIENT FIRST =====================
  if (bootPhase == BOOT_AMBIENT) {
    if (!ambientActive() && sensor.fresh(now)) {
      ambientStart(now, "AMB | BOOT FIRST (LEDs OFF)");
    }
    ambientTick(now);

    if (!ambientActive() && sensor.fresh(now) && haveAmbient) {
      resetAmbientStaleDetector(now);
      bootPhase = BOOT_CALIB;
    }
    return;
  }

  // ===================== BOOT: FIRST CALIBRATION =====================
  if (bootPhase == BOOT_CALIB) {
    if (!calib.active() && sensor.fresh(now) && haveAmbient) {
      resetControlFlags();
      calib.start(now, pwmEma);
    }

    if (calib.active()) {
      calib.update(now, sensor, ambientLux);
      pwmEma = constrain(lastWrittenPwm, PWM_MIN_FRAC, PWM_MAX_FRAC);
      pwmCmd = pwmEma;

      if (calib.haveSlope) {
        bootPhase = BOOT_RUN;
        Serial.println("BOOT | DONE -> CONTROL RUN");
      }
    }
    return;
  }

  // ===================== RUN: AMBIENT PERIODIC / STALE TRIGGER =====================
  if (!calib.active()) {
    // OPTION A: stale trigger only runs while holding (inside ambientStaleTick)
    ambientStaleTick(now);

    if (!ambientActive() && ambientDue(now) && sensor.fresh(now)) {
      ambientStart(now, forceAmbient ? "AMB | STALE TRIGGER (LEDs OFF)" : "AMB | START (LEDs OFF)");
      forceAmbient = false;
    }

    ambientTick(now);
    if (ambientActive()) return;
  }

  // ===================== RUN: PERIODIC CALIB (GATED) =====================
  if (!ambientActive() && sensor.fresh(now) && !calib.active() && calib.due(now)) {
    const float ledLux    = ledLuxNow();
    const float targetLed = targetLedNow();
    const float ledErr    = targetLed - ledLux;

    if (okToCalibrateNow(ledErr)) {
      resetControlFlags();
      calib.start(now, pwmEma);
    }
  }

  if (calib.active()) {
    calib.update(now, sensor, ambientLux);
    pwmEma = constrain(lastWrittenPwm, PWM_MIN_FRAC, PWM_MAX_FRAC);
    pwmCmd = pwmEma;
    return;
  }

  // ===================== RUN: CONTROL TICK =====================
  static uint32_t lastCtrlMs = 0;
  if (now - lastCtrlMs < CONTROL_PERIOD_MS) return;
  lastCtrlMs = now;

  if (!sensor.fresh(now) || !haveAmbient || !calib.haveSlope) return;
  if (sensor.luxUpdateMs < (lastOutputStepMs + POST_STEP_GUARD_MS)) return;

  const float ledLux    = ledLuxNow();
  const float targetLed = targetLedNow();
  const float ledErr    = targetLed - ledLux;

  errEma = ERR_EMA_ALPHA * errEma + (1.0f - ERR_EMA_ALPHA) * ledErr;
  const float absErrEma = fabsf(errEma);

  const char* evt = "";

  // ===================== OVERSHOOT CLAMP =====================
  if (!overshootClamped) {
    if (sensor.lux >= TARGET_LUX + OVERSHOOT_CLAMP_LUX) {
      resetControlFlags();
      overshootClamped = true;

      pwmCmd = PWM_MIN_FRAC;
      pwmEma = PWM_MIN_FRAC;
      applyPWMValidatedIfChanged(pwmEma);
      markOutputStep(now);

      evt = "CLAMP";
      printCtrlLine(ledLux, targetLed, ledErr, evt);
      return;
    }
  } else {
    if (sensor.lux <= TARGET_LUX + OVERSHOOT_EXIT_LUX) {
      overshootClamped = false;
      evt = "CLAMP_REL";
    } else {
      evt = "CLAMP_HOLD";
      printCtrlLine(ledLux, targetLed, ledErr, evt);
      return;
    }
  }

  // ===================== HOLD FAST-EXIT =====================
  if (holding) {
    if (ledErr >= HOLD_FAST_EXIT_ERR_LUX) {
      if (++fastExitCount >= HOLD_FAST_EXIT_TICKS) {
        holding = false;
        enterCount = exitCount = fastExitCount = 0;
        evt = "FAST_EXIT";
      }
    } else {
      fastExitCount = 0;
    }
  } else {
    fastExitCount = 0;
  }

  // ===================== HOLD STATE MACHINE =====================
  const bool okToHold = (errEma >= HOLD_ABOVE_GUARD_LUX);

  if (holding) {
    if (absErrEma >= HOLD_EXIT_LUX) {
      if (++exitCount >= HOLD_EXIT_TICKS) {
        holding = false;
        enterCount = exitCount = 0;
        if (!evt[0]) evt = "EXIT_HOLD";
      }
    } else {
      exitCount = 0;
    }
  } else {
    if (okToHold && absErrEma <= HOLD_ENTER_LUX) {
      if (++enterCount >= HOLD_ENTER_TICKS) {
        holding = true;
        enterCount = exitCount = 0;
        fastExitCount = 0;

        pwmCmd = pwmEma; // hard freeze
        applyPWMValidatedIfChanged(pwmEma);
        markOutputStep(now);
        resetAmbientStaleDetector(now);   // important for OPTION A: baseline when HOLD begins

        if (!evt[0]) evt = "ENTER_HOLD";
      }
    } else {
      enterCount = 0;
    }
  }

  if (holding) {
    if (!evt[0]) evt = "HOLD";
    printCtrlLine(ledLux, targetLed, ledErr, evt);
    ambientStaleTick(now); // OPTION A: only meaningful while holding
    return;
  }

  // ===================== CONTROL UPDATE =====================
  float deltaFrac = ledErr / calib.slopeEma;

  const float tickUp   = RATE_UP_PER_SEC   * (CONTROL_PERIOD_MS / 1000.0f);
  const float tickDown = RATE_DOWN_PER_SEC * (CONTROL_PERIOD_MS / 1000.0f);

  if (deltaFrac >= 0.0f) deltaFrac = constrain(deltaFrac, 0.0f, tickUp);
  else                   deltaFrac = constrain(deltaFrac, -tickDown, 0.0f);

  const bool atMin = (pwmCmd <= PWM_MIN_FRAC + 0.0005f);
  if (atMin && ledErr < 0.0f) {
    deltaFrac = 0.0f;
    pwmCmd = PWM_MIN_FRAC;
  } else {
    pwmCmd = constrain(pwmCmd + deltaFrac, PWM_MIN_FRAC, PWM_MAX_FRAC);
  }

  pwmEma = PWM_EMA_ALPHA * pwmEma + (1.0f - PWM_EMA_ALPHA) * pwmCmd;
  pwmEma = constrain(pwmEma, PWM_MIN_FRAC, PWM_MAX_FRAC);

  applyPWMValidatedIfChanged(pwmEma);

  printCtrlLine(ledLux, targetLed, ledErr, evt);

  // OPTION A: this will no-op unless holding
  ambientStaleTick(now);
}