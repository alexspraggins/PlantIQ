#include <Arduino.h>
#include <Wire.h>
#include <hp_BH1750.h>

// =======================================================
// PlantIQ Lux Control + 1min ON / 1min OFF Photoperiod
// - CONTROL mode with HOLD + OVERSHOOT CLAMP (NO debounce)
// - Ambient STEP detector schedules an AMB+CAL bundle
// - Periodic AMB+CAL bundle every 30s (only when lights ON)
// - Calibration prints: LOW/HIGH/ DONE slopeEma ambient
// - Resume gate discards 3 NEW lux updates after restore (optional but kept)
// - FIX: post-restore CLAMP entry locked until we see ONE "safe" lux sample
// =======================================================

// ===================== USER TARGET =====================
static const float TARGET_LUX = 200.0f;

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

// ===================== PHOTOPERIOD (UPDATED) ============
static const uint32_t ON_MS   = 60UL * 1000UL;   // 1 minute ON
static const uint32_t OFF_MS  = 60UL * 1000UL;   // 1 minute OFF
static const uint32_t CYCLE_MS = ON_MS + OFF_MS; // 2 minutes total
static const bool     LIGHTS_ON_AT_BOOT = true;  // start ON at boot

// ===================== SENSOR TIMING ====================
static const uint32_t BH1750_POLL_MS = 120;
static const uint32_t LUX_STALE_MS   = 1000;

// ===================== CONTROL TIMING ===================
static const uint32_t CONTROL_PERIOD_MS = 250;

// ===================== CONTROL TUNING ===================
static const float RATE_UP_PER_SEC   = 0.03f;
static const float RATE_DOWN_PER_SEC = 0.09f;
static const float PWM_EMA_ALPHA     = 0.85f;

// HOLD logic
static const float   HOLD_ENTER_LUX       = 6.0f;
static const float   HOLD_EXIT_LUX        = 35.0f;
static const float   HOLD_ABOVE_GUARD_LUX = -2.0f;
static const float   ERR_EMA_ALPHA        = 0.80f;
static const uint8_t HOLD_ENTER_TICKS     = 2;
static const uint8_t HOLD_EXIT_TICKS      = 4;

static const float   HOLD_FAST_EXIT_ERR_LUX = 40.0f;
static const uint8_t HOLD_FAST_EXIT_TICKS  = 2;

// Overshoot clamp
static const float OVERSHOOT_CLAMP_LUX = 50.0f;
static const float OVERSHOOT_EXIT_LUX  = 20.0f;

// ===================== AMBIENT ==========================
static const uint32_t AMBIENT_PERIOD_MS   = 30000; // every 30s (only when lights ON)
static const uint32_t AMBIENT_MIN_GAP_MS  = 1500;
static const bool     AMBIENT_AT_BOOT     = true;

static const uint32_t AMBIENT_OFF_HOLD_MS = 800;
static const float    AMBIENT_SLOW_ALPHA  = 0.95f;

static const float AMB_STEP_LUX    = 25.0f;
static const float AMB_FAST_ALPHA  = 0.50f;

static const uint32_t RESUME_SETTLE_MS = 700;

// ===================== STEP-DELAY AMBIENT TRIGGER =======
static const float    STEP_DETECT_LUX        = 25.0f;
static const uint8_t  STEP_DETECT_TICKS_REQ  = 3;
static const uint32_t AMB_STEP_DELAY_MS      = 3000;
static const float    LUX_FAST_ALPHA         = 0.65f;
static const float    LUX_SLOW_ALPHA         = 0.97f;

static const float    AMBLIKE_K              = 1.0f;

static const uint32_t STEP_CONFIRM_MS        = 3000;
static const float    STEP_CONFIRM_LUX       = 20.0f;

// ===================== CALIBRATION ======================
static const float    CAL_LOW_FRAC  = 0.15f;
static const float    CAL_HIGH_FRAC = 0.60f;

static const uint32_t SETTLE_LOW_MS  = 900;
static const uint32_t SETTLE_HIGH_MS = 1000;

static const float SLOPE_EMA_ALPHA = 0.80f;
static const float MIN_LED_LUX_DELTA_FOR_SLOPE = 50.0f;

static const uint8_t RESUME_DISCARD_UPDATES = 3;

// ===================== BH1750 ===========================
hp_BH1750 bh1750;

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

// ===================== ROBUST SAMPLER ===================
static inline float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

static bool waitNewLuxUpdate(uint32_t timeoutMs) {
  const uint32_t start = millis();
  const uint32_t lastU = sensor.luxUpdateMs;
  while (millis() - start < timeoutMs) {
    sensor.update();
    if (sensor.haveLux && sensor.luxUpdateMs != lastU) return true;
    delay(2);
  }
  return false;
}

static bool sampleLuxMedian3AfterChange(uint32_t settleMs, float& outLux) {
  const uint32_t t0 = millis();
  while (millis() - t0 < settleMs) { sensor.update(); delay(2); }

  if (!waitNewLuxUpdate(1500)) return false;

  float a=NAN, b=NAN, c=NAN;
  if (!waitNewLuxUpdate(1500)) return false; a = sensor.lux;
  if (!waitNewLuxUpdate(1500)) return false; b = sensor.lux;
  if (!waitNewLuxUpdate(1500)) return false; c = sensor.lux;

  outLux = median3(a, b, c);
  return true;
}

// ===================== AMBIENT ESTIMATE =================
static float    ambientEstLux = 0.0f;
static bool     haveAmbient   = false;
static uint32_t lastAmbientMs = 0;

static void updateAmbientEstimate(float ambRaw) {
  ambRaw = max(0.0f, ambRaw);

  if (!haveAmbient) {
    ambientEstLux = ambRaw;
    haveAmbient = true;
    return;
  }

  const float diff = fabsf(ambRaw - ambientEstLux);
  if (diff > AMB_STEP_LUX) {
    ambientEstLux = AMB_FAST_ALPHA * ambientEstLux + (1.0f - AMB_FAST_ALPHA) * ambRaw;
  } else {
    ambientEstLux = AMBIENT_SLOW_ALPHA * ambientEstLux + (1.0f - AMBIENT_SLOW_ALPHA) * ambRaw;
  }
}

// ===================== SLOPE ============================
static bool  haveSlope = false;
static float slopeEma  = NAN;

// ===================== RUNTIME STATE ====================
static float pwmCmd = 0.25f;
static float pwmEma = 0.25f;
static float errEma = 0.0f;

static bool overshootClamped = false;
static bool holding          = false;
static uint8_t enterCount    = 0;
static uint8_t exitCount     = 0;
static uint8_t fastExitCount = 0;

static bool clampEntryLocked = false;

static inline void resetControlFlags() {
  overshootClamped = false;
  holding = false;
  enterCount = exitCount = fastExitCount = 0;
}

static inline void lockClampEntryUntilSafeSample() { clampEntryLocked = true; }

// ===================== RESUME GATE ======================
static uint32_t resumeAfterMs = 0;
static uint8_t  resumeDiscardLeft = 0;
static uint32_t resumeLastLuxUpdate = 0;

static void armResumeGate(uint8_t discardUpdates = RESUME_DISCARD_UPDATES) {
  resumeAfterMs = millis() + RESUME_SETTLE_MS;
  resumeDiscardLeft = discardUpdates;
  resumeLastLuxUpdate = sensor.luxUpdateMs;
}

static bool resumeGateAllowsControl(uint32_t now) {
  if (now < resumeAfterMs) return false;
  if (resumeDiscardLeft > 0) {
    if (sensor.luxUpdateMs != resumeLastLuxUpdate) {
      resumeLastLuxUpdate = sensor.luxUpdateMs;
      resumeDiscardLeft--;
    }
    return false;
  }
  return true;
}

// ===================== STEP-DELAY SCHEDULER =============
static float ambLikeFast = NAN;
static float ambLikeSlow = NAN;
static uint8_t stepCount = 0;

static bool ambScheduled = false;
static uint32_t ambRunAtMs = 0;

static bool     stepConfirmPending = false;
static uint32_t stepConfirmAtMs    = 0;
static float    stepBaseline       = NAN;

static uint32_t stepDetectIgnoreUntilMs = 0;

static void clearScheduledAmbient() { ambScheduled = false; ambRunAtMs = 0; }

static void armStepDetectIgnore(uint32_t ms) {
  stepDetectIgnoreUntilMs = millis() + ms;
  ambLikeFast = NAN;
  ambLikeSlow = NAN;
  stepCount = 0;
  stepConfirmPending = false;
  stepConfirmAtMs = 0;
  stepBaseline = NAN;
  clearScheduledAmbient();
}

static bool okToDetectAmbientStep() {
  return haveSlope && isfinite(slopeEma) && slopeEma > 1e-3f;
}

static void updateStepDelayDetector(uint32_t now) {
  if (!sensor.haveLux) return;
  if ((int32_t)(now - stepDetectIgnoreUntilMs) < 0) return;
  if (!okToDetectAmbientStep()) return;
  if (ambScheduled) return;

  const float ledEst  = AMBLIKE_K * slopeEma * pwmEma;
  const float ambLike = sensor.lux - ledEst;

  if (!isfinite(ambLikeFast)) {
    ambLikeFast = ambLikeSlow = ambLike;
    stepCount = 0;
    stepConfirmPending = false;
    return;
  }

  ambLikeFast = LUX_FAST_ALPHA * ambLikeFast + (1.0f - LUX_FAST_ALPHA) * ambLike;
  ambLikeSlow = LUX_SLOW_ALPHA * ambLikeSlow + (1.0f - LUX_SLOW_ALPHA) * ambLike;

  const float stepness = fabsf(ambLikeFast - ambLikeSlow);

  if (stepConfirmPending) {
    if ((int32_t)(now - stepConfirmAtMs) >= 0) {
      const float stillDiff = fabsf(ambLikeSlow - stepBaseline);
      if (stillDiff >= STEP_CONFIRM_LUX) {
        ambScheduled = true;
        ambRunAtMs = now + AMB_STEP_DELAY_MS;
        Serial.println("AMB | STEP CONFIRMED -> schedule AMB+CAL bundle");
      } else {
        Serial.println("AMB | STEP REJECTED (transient)");
      }
      stepConfirmPending = false;
      stepCount = 0;
    }
    return;
  }

  if (stepness >= STEP_DETECT_LUX) {
    if (++stepCount >= STEP_DETECT_TICKS_REQ) {
      stepConfirmPending = true;
      stepConfirmAtMs = now + STEP_CONFIRM_MS;
      stepBaseline = ambLikeSlow;
      stepCount = 0;
      Serial.println("AMB | STEP DETECTED -> confirming...");
    }
  } else {
    stepCount = 0;
  }
}

static bool scheduledAmbientDue(uint32_t now) {
  if (!ambScheduled) return false;
  return (int32_t)(now - ambRunAtMs) >= 0;
}

// ===================== CTRL PRINT =======================
static void printCtrlLine(float ledLux, float targetLed, float ledErr, const char* evt) {
  Serial.print("CTRL | lux="); Serial.print(sensor.lux, 1);
  Serial.print(" amb=");       Serial.print(ambientEstLux, 1);
  Serial.print(" led=");       Serial.print(ledLux, 1);
  Serial.print(" tLed=");      Serial.print(targetLed, 1);
  Serial.print(" err=");       Serial.print(ledErr, 1);
  Serial.print(" errEma=");    Serial.print(errEma, 1);
  Serial.print(" pwm=");       Serial.print(pwmEma, 3);
  Serial.print(" slope=");     Serial.print(slopeEma, 1);
  if (evt && evt[0]) { Serial.print(" | evt="); Serial.print(evt); }
  Serial.println();
}

// ===================== 3-MODE STATE =====================
static const uint8_t MODE_CONTROL = 0;
static const uint8_t MODE_AMBIENT = 1;
static const uint8_t MODE_CAL     = 2;

static uint8_t mode = MODE_CONTROL;
static float pwmSaved = 0.25f;

static void enterMode(uint8_t m) {
  mode = m;
  pwmSaved = pwmEma;
  resetControlFlags();
}

// ===================== TRIGGERS =========================
static bool ambientDue(uint32_t now) {
  if (now - lastAmbientMs < AMBIENT_MIN_GAP_MS) return false;
  if (!haveAmbient && AMBIENT_AT_BOOT) return true;
  return (now - lastAmbientMs >= AMBIENT_PERIOD_MS);
}

// ===================== PHOTOPERIOD HELPERS ==============
static uint32_t cycleStartMs = 0;

static bool lightsShouldBeOn(uint32_t now) {
  uint32_t t = (uint32_t)(now - cycleStartMs);
  t %= CYCLE_MS;
  if (LIGHTS_ON_AT_BOOT) return (t < ON_MS);
  else                   return (t >= OFF_MS);
}

// ===================== MODE_AMBIENT =====================
static void runAmbient() {
  Serial.println("AMB | START (LEDs OFF)");
  applyPWMAnyImmediate(0.0f);

  float amb = NAN;
  const bool ok = sampleLuxMedian3AfterChange(AMBIENT_OFF_HOLD_MS, amb);

  lastAmbientMs = millis();

  if (!ok) {
    Serial.println("AMB | WARN sample failed -> restore");
    pwmCmd = constrain(pwmSaved, PWM_MIN_FRAC, PWM_MAX_FRAC);
    pwmEma = pwmCmd;
    applyPWMValidatedIfChanged(pwmEma);
    errEma = 0.0f;

    lockClampEntryUntilSafeSample();
    armResumeGate();
    armStepDetectIgnore(RESUME_SETTLE_MS + 1500);
    mode = MODE_CONTROL;
    return;
  }

  updateAmbientEstimate(amb);
  Serial.print("AMB | READ raw="); Serial.print(amb, 1);
  Serial.print(" est=");           Serial.println(ambientEstLux, 1);

  Serial.println("BUNDLE | AMB OK -> CAL next");
  mode = MODE_CAL;
}

// ===================== MODE_CAL =========================
static void runCal() {
  if (!haveAmbient) { mode = MODE_CONTROL; return; }

  Serial.println("CAL | START");

  bool ok = true;
  float luxLow = NAN, luxHigh = NAN;

  applyPWMValidatedIfChanged(CAL_LOW_FRAC);
  if (!sampleLuxMedian3AfterChange(SETTLE_LOW_MS, luxLow)) ok = false;

  if (ok) {
    applyPWMValidatedIfChanged(CAL_HIGH_FRAC);
    if (!sampleLuxMedian3AfterChange(SETTLE_HIGH_MS, luxHigh)) ok = false;
  }

  if (!ok) {
    Serial.println("CAL | FAIL sample(s)");
  } else {
    const float ledLow  = max(0.0f, luxLow  - ambientEstLux);
    const float ledHigh = max(0.0f, luxHigh - ambientEstLux);

    Serial.print("CAL | LOW  lux="); Serial.print(luxLow, 1);
    Serial.print(" led=");           Serial.println(ledLow, 1);

    Serial.print("CAL | HIGH lux="); Serial.print(luxHigh, 1);
    Serial.print(" led=");           Serial.println(ledHigh, 1);

    const float ledDelta  = ledHigh - ledLow;
    const float fracDelta = (CAL_HIGH_FRAC - CAL_LOW_FRAC);

    if (isfinite(ledDelta) && ledDelta > MIN_LED_LUX_DELTA_FOR_SLOPE && fracDelta > 1e-6f) {
      const float slopeInst = ledDelta / fracDelta;

      if (!haveSlope || !isfinite(slopeEma)) {
        slopeEma = slopeInst;
        haveSlope = true;
      } else {
        slopeEma = SLOPE_EMA_ALPHA * slopeEma + (1.0f - SLOPE_EMA_ALPHA) * slopeInst;
      }

      Serial.print("CAL | DONE slopeEma="); Serial.print(slopeEma, 1);
      Serial.print(" ambient=");            Serial.println(ambientEstLux, 1);
    } else {
      Serial.print("CAL | WARN ledDelta invalid/small: ");
      Serial.println(ledDelta, 1);
    }
  }

  applyPWMValidatedIfChanged(constrain(pwmSaved, PWM_MIN_FRAC, PWM_MAX_FRAC));
  Serial.println("CAL | RESTORE PWM");

  lockClampEntryUntilSafeSample();

  pwmCmd = constrain(pwmSaved, PWM_MIN_FRAC, PWM_MAX_FRAC);
  pwmEma = pwmCmd;
  errEma = 0.0f;

  armResumeGate();
  armStepDetectIgnore(RESUME_SETTLE_MS + 1500);

  mode = MODE_CONTROL;
}

// ===================== MODE_CONTROL =====================
static void runControlTick(uint32_t now) {
  if (!sensor.fresh(now)) return;

  if (!haveAmbient || !haveSlope || !isfinite(slopeEma) || slopeEma < 1e-3f) {
    static uint32_t lastPrint = 0;
    if (now - lastPrint > 1000) {
      lastPrint = now;
      Serial.println("CTRL | waiting for ambient+slope");
    }
    return;
  }

  if (clampEntryLocked) {
    if (sensor.lux <= TARGET_LUX + OVERSHOOT_EXIT_LUX) clampEntryLocked = false;
  }

  const float ledLux    = max(0.0f, sensor.lux - ambientEstLux);
  const float targetLed = max(0.0f, TARGET_LUX - ambientEstLux);
  const float ledErr    = targetLed - ledLux;

  errEma = ERR_EMA_ALPHA * errEma + (1.0f - ERR_EMA_ALPHA) * ledErr;
  const float absErrEma = fabsf(errEma);

  const char* evt = "";

  if (!overshootClamped) {
    if (!clampEntryLocked && sensor.lux >= TARGET_LUX + OVERSHOOT_CLAMP_LUX) {
      overshootClamped = true;
      holding = false;
      enterCount = exitCount = fastExitCount = 0;

      pwmCmd = PWM_MIN_FRAC;
      pwmEma = PWM_MIN_FRAC;
      applyPWMValidatedIfChanged(pwmEma);

      evt = "CLAMP";
      printCtrlLine(ledLux, targetLed, ledErr, evt);
      return;
    }
  } else {
    if (sensor.lux <= TARGET_LUX + OVERSHOOT_EXIT_LUX) {
      overshootClamped = false;
      evt = "CLAMP_REL";
      printCtrlLine(ledLux, targetLed, ledErr, evt);
    } else {
      evt = "CLAMP_HOLD";
      printCtrlLine(ledLux, targetLed, ledErr, evt);
      return;
    }
  }

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

        pwmCmd = pwmEma;
        applyPWMValidatedIfChanged(pwmEma);
        if (!evt[0]) evt = "ENTER_HOLD";
      }
    } else {
      enterCount = 0;
    }
  }

  if (holding) {
    if (!evt[0]) evt = "HOLD";
    printCtrlLine(ledLux, targetLed, ledErr, evt);
    return;
  }

  float deltaFrac = ledErr / slopeEma;

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

  cycleStartMs = millis();      // photoperiod anchor
  armStepDetectIgnore(4000);

  Serial.println("READY — 1min ON / 1min OFF added. AMB+CAL bundled (step + periodic).");
}

// ===================== LOOP =============================
void loop() {
  const uint32_t now = millis();
  sensor.update();

  static uint32_t lastTick = 0;
  if (now - lastTick < CONTROL_PERIOD_MS) return;
  lastTick = now;

  // ---------- Photoperiod gate ----------
  static bool prevLightsOn = true;
  const bool lightsOn = lightsShouldBeOn(now);

  if (lightsOn != prevLightsOn) {
    prevLightsOn = lightsOn;

    if (!lightsOn) {
      Serial.println("CYCLE | LIGHTS_OFF (forcing PWM=0, pausing control/amb/cal)");
      applyPWMAnyImmediate(0.0f);

      armStepDetectIgnore(OFF_MS + 500);
      clearScheduledAmbient();
      mode = MODE_CONTROL;
      return;
    } else {
      Serial.println("CYCLE | LIGHTS_ON (run AMB->CAL bundle, then resume control)");
      overshootClamped = false;
      holding = false;
      enterCount = exitCount = fastExitCount = 0;

      clearScheduledAmbient();
      enterMode(MODE_AMBIENT);
      return;
    }
  }

  if (!lightsOn) {
    applyPWMAnyImmediate(0.0f);
    return;
  }

  // ---------- Normal system (lights ON) ----------
  if (mode == MODE_AMBIENT) { runAmbient(); return; }
  if (mode == MODE_CAL)     { runCal();     return; }

  if (!resumeGateAllowsControl(now)) return;
  if (!sensor.fresh(now)) return;

  updateStepDelayDetector(now);

  if (!haveAmbient && AMBIENT_AT_BOOT) {
    clearScheduledAmbient();
    enterMode(MODE_AMBIENT);
    return;
  }

  if (haveAmbient && (!haveSlope || !isfinite(slopeEma) || slopeEma < 1e-3f)) {
    enterMode(MODE_CAL);
    return;
  }

  const bool wantPeriodic  = ambientDue(now);
  const bool wantScheduled = scheduledAmbientDue(now);

  if (wantScheduled && (now - lastAmbientMs >= AMBIENT_MIN_GAP_MS)) {
    Serial.println("AMB | TRIGGER (scheduled step) -> bundle");
    clearScheduledAmbient();
    enterMode(MODE_AMBIENT);
    return;
  }

  if (wantPeriodic && (now - lastAmbientMs >= AMBIENT_MIN_GAP_MS) && !overshootClamped) {
    Serial.println("AMB | TRIGGER (periodic) -> bundle");
    enterMode(MODE_AMBIENT);
    return;
  }

  runControlTick(now);
}