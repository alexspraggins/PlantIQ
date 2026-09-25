#include <Arduino.h>
#include <Wire.h>
#include <hp_BH1750.h>

// =======================================================
// PlantIQ Light Control - FULLY NON-BLOCKING
// ESP32 Arduino Core 2.0.17
//
// - Uses ledcSetup() + ledcAttachPin()
// - Fully non-blocking ambient sampling + calibration
// - 1 min ON / 1 min OFF photoperiod
// - CONTROL mode with HOLD + OVERSHOOT CLAMP
// - Ambient STEP detector schedules AMB+CAL bundle
// - Periodic AMB+CAL bundle every 30s (lights ON only)
// - Calibration: LOW/HIGH/DONE slopeEma ambient
// - Resume gate after mode switches
// - Clamp-entry lock after restore until safe sample
// - Option A:
//   LEDs go OFF only when ambient >= TARGET
//   If targetPwmRaw < PWM_MIN_FRAC, clamp to min instead of forcing OFF
// =======================================================


// =======================================================
//                 TIME PREFIXED PRINTS
// =======================================================
static void printTimePrefix() {
  uint32_t ms = millis();
  uint32_t totalSeconds = ms / 1000;
  uint32_t hours   = (totalSeconds / 3600) % 24;
  uint32_t minutes = (totalSeconds / 60) % 60;
  uint32_t seconds = totalSeconds % 60;
  uint32_t mmm     = ms % 1000;

  if (hours < 10)   Serial.print('0');
  Serial.print(hours); Serial.print(':');
  if (minutes < 10) Serial.print('0');
  Serial.print(minutes); Serial.print(':');
  if (seconds < 10) Serial.print('0');
  Serial.print(seconds); Serial.print('.');
  if (mmm < 100) Serial.print('0');
  if (mmm < 10)  Serial.print('0');
  Serial.print(mmm);
  Serial.print(" -> ");
}

#define TPRINT(x)    do { printTimePrefix(); Serial.print(x); } while(0)
#define TPRINTLN(x)  do { printTimePrefix(); Serial.println(x); } while(0)


// =======================================================
//                        CONFIG
// =======================================================
static const float TARGET_LUX = 350.0f;

// PWM pins/channels
static const int WHITE_PWM_PIN = 25;
static const int RB_PWM_PIN    = 26;
static const int WHITE_PWM_CH  = 0;
static const int RB_PWM_CH     = 1;

static const uint32_t PWM_FREQ_HZ  = 200;
static const uint8_t  PWM_RES_BITS = 12;
static const uint32_t PWM_MAX      = (1UL << PWM_RES_BITS) - 1;

static const float WHITE_WEIGHT = 0.75f;
static const float RB_WEIGHT    = 0.25f;

static const float PWM_MIN_FRAC = 0.10f;
static const float PWM_MAX_FRAC = 0.60f;

// Photoperiod
static const uint32_t ON_MS    = 60UL * 1000UL;
static const uint32_t OFF_MS   = 60UL * 1000UL;
static const uint32_t CYCLE_MS = ON_MS + OFF_MS;
static const bool     LIGHTS_ON_AT_BOOT = true;

// Sensor timing
static const uint32_t BH1750_POLL_MS = 120;
static const uint32_t LUX_STALE_MS   = 1000;

// Control timing
static const uint32_t CONTROL_PERIOD_MS = 250;

// Control tuning
static const float RATE_UP_PER_SEC   = 0.03f;
static const float RATE_DOWN_PER_SEC = 0.09f;
static const float PWM_EMA_ALPHA     = 0.85f;

// HOLD tuning
static const float   HOLD_ENTER_LUX       = 5.0f;
static const float   HOLD_EXIT_LUX        = 15.0f;
static const float   HOLD_ABOVE_GUARD_LUX = -2.0f;
static const float   ERR_EMA_ALPHA        = 0.80f;
static const uint8_t HOLD_ENTER_TICKS     = 2;
static const uint8_t HOLD_EXIT_TICKS      = 4;

static const float   HOLD_FAST_EXIT_ERR_LUX = 40.0f;
static const uint8_t HOLD_FAST_EXIT_TICKS   = 2;

// Overshoot clamp
static const float OVERSHOOT_CLAMP_LUX = 50.0f;
static const float OVERSHOOT_EXIT_LUX  = 20.0f;

// Ambient/calibration
static const uint32_t AMBIENT_PERIOD_MS   = 30000;
static const uint32_t AMBIENT_MIN_GAP_MS  = 1500;
static const bool     AMBIENT_AT_BOOT     = true;

static const uint32_t AMBIENT_OFF_HOLD_MS = 800;
static const float    AMBIENT_SLOW_ALPHA  = 0.95f;

static const float AMB_STEP_LUX   = 25.0f;
static const float AMB_FAST_ALPHA = 0.50f;

static const uint32_t RESUME_SETTLE_MS = 700;

// Step-delay ambient trigger
static const float    STEP_DETECT_LUX       = 25.0f;
static const uint8_t  STEP_DETECT_TICKS_REQ = 3;
static const uint32_t AMB_STEP_DELAY_MS     = 3000;
static const float    LUX_FAST_ALPHA        = 0.65f;
static const float    LUX_SLOW_ALPHA        = 0.97f;
static const float    AMBLIKE_K             = 1.0f;
static const uint32_t STEP_CONFIRM_MS       = 3000;
static const float    STEP_CONFIRM_LUX      = 20.0f;

// Calibration span
static const float    CAL_LOW_FRAC  = PWM_MIN_FRAC;
static const float    CAL_HIGH_FRAC = 0.60f;

static const uint32_t SETTLE_LOW_MS  = 900;
static const uint32_t SETTLE_HIGH_MS = 1000;

static const float SLOPE_EMA_ALPHA = 0.80f;
static const float MIN_LED_LUX_DELTA_FOR_SLOPE = 50.0f;

// Resume gate
static const uint8_t RESUME_DISCARD_UPDATES = 3;

// Non-blocking sample timeout
static const uint32_t SAMPLE_PHASE_TIMEOUT_MS = 1500;


// =======================================================
//                         TYPES
// =======================================================
enum LightMode : uint8_t {
  MODE_CONTROL = 0,
  MODE_AMBIENT_SAMPLE,
  MODE_CALIBRATION
};

enum SamplePhase : uint8_t {
  SAMPLE_IDLE = 0,
  SAMPLE_SETTLE,
  SAMPLE_FLUSH_WAIT,
  SAMPLE_COLLECT_1,
  SAMPLE_COLLECT_2,
  SAMPLE_COLLECT_3,
  SAMPLE_DONE,
  SAMPLE_FAIL
};

enum AmbientPhase : uint8_t {
  AMB_IDLE = 0,
  AMB_START,
  AMB_WAIT_SAMPLE,
  AMB_RESTORE_FAIL,
  AMB_TO_CAL
};

enum CalPhase : uint8_t {
  CAL_IDLE = 0,
  CAL_START,
  CAL_SET_LOW,
  CAL_WAIT_LOW,
  CAL_SET_HIGH,
  CAL_WAIT_HIGH,
  CAL_PROCESS,
  CAL_RESTORE,
  CAL_FINISH
};

struct LuxSensorState {
  uint32_t lastStartMs;
  float lux;
  bool haveLux;
  uint32_t luxUpdateMs;
};

struct OutputState {
  float lastWrittenPwm;
};

struct AmbientState {
  float estLux;
  bool haveAmbient;
  uint32_t lastAmbientMs;
};

struct CalibrationState {
  bool haveSlope;
  float slopeEma;
};

struct ControlState {
  float pwmCmd;
  float pwmEma;
  float errEma;

  bool overshootClamped;
  bool holding;
  uint8_t enterCount;
  uint8_t exitCount;
  uint8_t fastExitCount;
  bool clampEntryLocked;
};

struct ResumeGateState {
  uint32_t resumeAfterMs;
  uint8_t  discardLeft;
  uint32_t lastSeenLuxUpdate;
};

struct StepSchedulerState {
  float ambLikeFast;
  float ambLikeSlow;
  uint8_t stepCount;

  bool     ambScheduled;
  uint32_t ambRunAtMs;

  bool     stepConfirmPending;
  uint32_t stepConfirmAtMs;
  float    stepBaseline;

  uint32_t detectIgnoreUntilMs;
};

struct PhotoperiodState {
  uint32_t cycleStartMs;
  bool prevLightsOn;
};

struct LightRuntime {
  uint8_t mode;
  float pwmSaved;
};

struct SampleJob {
  uint8_t phase;
  uint32_t settleUntilMs;
  uint32_t timeoutAtMs;
  uint32_t refUpdateMs;
  float a;
  float b;
  float c;
  float result;
};

struct AmbientModeState {
  uint8_t phase;
};

struct CalModeState {
  uint8_t phase;
  float luxLow;
  float luxHigh;
};

struct LightSystem {
  LuxSensorState sensor;
  OutputState output;
  AmbientState ambient;
  CalibrationState cal;
  ControlState ctrl;
  ResumeGateState resume;
  StepSchedulerState step;
  PhotoperiodState photo;
  LightRuntime rt;
  SampleJob sample;
  AmbientModeState ambMode;
  CalModeState calMode;
};


// =======================================================
//                    GLOBAL OBJECTS
// =======================================================
static hp_BH1750 bh1750;

static LightSystem gLight = {
  {0, NAN, false, 0},         // sensor
  {NAN},                      // output
  {0.0f, false, 0},           // ambient
  {false, NAN},               // cal
  {0.25f, 0.25f, 0.0f, false, false, 0, 0, 0, false}, // ctrl
  {0, 0, 0},                  // resume
  {NAN, NAN, 0, false, 0, false, 0, NAN, 0}, // step
  {0, true},                  // photo
  {MODE_CONTROL, 0.25f},      // rt
  {SAMPLE_IDLE, 0, 0, 0, NAN, NAN, NAN, NAN}, // sample
  {AMB_IDLE},                 // ambMode
  {CAL_IDLE, NAN, NAN}        // calMode
};


// =======================================================
//                   GENERAL HELPERS
// =======================================================
static inline float clamp01(float v) {
  return constrain(v, 0.0f, 1.0f);
}

static inline float clampPwmWindow(float v) {
  return constrain(v, PWM_MIN_FRAC, PWM_MAX_FRAC);
}

static inline uint32_t fracToDuty(float frac) {
  frac = clamp01(frac);
  return (uint32_t)lroundf(frac * (float)PWM_MAX);
}

static inline float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

static void resetControlFlags() {
  gLight.ctrl.overshootClamped = false;
  gLight.ctrl.holding = false;
  gLight.ctrl.enterCount = 0;
  gLight.ctrl.exitCount = 0;
  gLight.ctrl.fastExitCount = 0;
}

static void lockClampEntryUntilSafeSample() {
  gLight.ctrl.clampEntryLocked = true;
}


// =======================================================
//                    SENSOR MANAGER
// =======================================================
static void sensorBegin() {
  gLight.sensor.lastStartMs = millis();
  bh1750.start();
}

static void sensorUpdate() {
  const uint32_t now = millis();

  if (now - gLight.sensor.lastStartMs >= BH1750_POLL_MS) {
    bh1750.start();
    gLight.sensor.lastStartMs = now;
  }

  if (bh1750.hasValue()) {
    gLight.sensor.lux = bh1750.getLux();
    gLight.sensor.haveLux = true;
    gLight.sensor.luxUpdateMs = now;
  }
}

static bool sensorFresh(uint32_t now) {
  return gLight.sensor.haveLux && (now - gLight.sensor.luxUpdateMs <= LUX_STALE_MS);
}


// =======================================================
//              NON-BLOCKING SAMPLE JOB MANAGER
// =======================================================
static void startSampleJob(uint32_t settleMs) {
  gLight.sample.phase = SAMPLE_SETTLE;
  gLight.sample.settleUntilMs = millis() + settleMs;
  gLight.sample.timeoutAtMs = 0;
  gLight.sample.refUpdateMs = gLight.sensor.luxUpdateMs;
  gLight.sample.a = NAN;
  gLight.sample.b = NAN;
  gLight.sample.c = NAN;
  gLight.sample.result = NAN;
}

static void failSampleJob() {
  gLight.sample.phase = SAMPLE_FAIL;
  gLight.sample.result = NAN;
}

static void finishSampleJob() {
  gLight.sample.result = median3(gLight.sample.a, gLight.sample.b, gLight.sample.c);
  gLight.sample.phase = SAMPLE_DONE;
}

static bool sampleSawNewLux() {
  return gLight.sensor.haveLux && (gLight.sensor.luxUpdateMs != gLight.sample.refUpdateMs);
}

static void armSampleTimeout() {
  gLight.sample.timeoutAtMs = millis() + SAMPLE_PHASE_TIMEOUT_MS;
}

static bool sampleTimedOut(uint32_t now) {
  return ((int32_t)(now - gLight.sample.timeoutAtMs) >= 0);
}

static void updateSampleJob(uint32_t now) {
  switch (gLight.sample.phase) {
    case SAMPLE_IDLE:
    case SAMPLE_DONE:
    case SAMPLE_FAIL:
      return;

    case SAMPLE_SETTLE:
      if ((int32_t)(now - gLight.sample.settleUntilMs) >= 0) {
        gLight.sample.phase = SAMPLE_FLUSH_WAIT;
        gLight.sample.refUpdateMs = gLight.sensor.luxUpdateMs;
        armSampleTimeout();
      }
      return;

    case SAMPLE_FLUSH_WAIT:
      if (sampleSawNewLux()) {
        gLight.sample.refUpdateMs = gLight.sensor.luxUpdateMs;
        gLight.sample.phase = SAMPLE_COLLECT_1;
        armSampleTimeout();
      } else if (sampleTimedOut(now)) {
        failSampleJob();
      }
      return;

    case SAMPLE_COLLECT_1:
      if (sampleSawNewLux()) {
        gLight.sample.a = gLight.sensor.lux;
        gLight.sample.refUpdateMs = gLight.sensor.luxUpdateMs;
        gLight.sample.phase = SAMPLE_COLLECT_2;
        armSampleTimeout();
      } else if (sampleTimedOut(now)) {
        failSampleJob();
      }
      return;

    case SAMPLE_COLLECT_2:
      if (sampleSawNewLux()) {
        gLight.sample.b = gLight.sensor.lux;
        gLight.sample.refUpdateMs = gLight.sensor.luxUpdateMs;
        gLight.sample.phase = SAMPLE_COLLECT_3;
        armSampleTimeout();
      } else if (sampleTimedOut(now)) {
        failSampleJob();
      }
      return;

    case SAMPLE_COLLECT_3:
      if (sampleSawNewLux()) {
        gLight.sample.c = gLight.sensor.lux;
        gLight.sample.refUpdateMs = gLight.sensor.luxUpdateMs;
        finishSampleJob();
      } else if (sampleTimedOut(now)) {
        failSampleJob();
      }
      return;
  }
}


// =======================================================
//                    OUTPUT MANAGER
// =======================================================
static void writeLedPWM(float overallFrac) {
  overallFrac = clamp01(overallFrac);

  const float w = clamp01(overallFrac * WHITE_WEIGHT);
  const float r = clamp01(overallFrac * RB_WEIGHT);

  ledcWrite(WHITE_PWM_CH, fracToDuty(w));
  ledcWrite(RB_PWM_CH,    fracToDuty(r));
}

static void applyPWMAnyImmediate(float frac) {
  frac = clamp01(frac);
  writeLedPWM(frac);
  gLight.output.lastWrittenPwm = frac;
}

static void applyPWMValidatedIfChanged(float frac) {
  frac = clampPwmWindow(frac);

  if (!isfinite(gLight.output.lastWrittenPwm) ||
      fabsf(frac - gLight.output.lastWrittenPwm) > 0.0005f) {
    writeLedPWM(frac);
    gLight.output.lastWrittenPwm = frac;
  }
}


// =======================================================
//                   AMBIENT MANAGER
// =======================================================
static void updateAmbientEstimate(float ambRaw) {
  ambRaw = max(0.0f, ambRaw);

  if (!gLight.ambient.haveAmbient) {
    gLight.ambient.estLux = ambRaw;
    gLight.ambient.haveAmbient = true;
    return;
  }

  const float diff = fabsf(ambRaw - gLight.ambient.estLux);
  if (diff > AMB_STEP_LUX) {
    gLight.ambient.estLux = AMB_FAST_ALPHA * gLight.ambient.estLux
                          + (1.0f - AMB_FAST_ALPHA) * ambRaw;
  } else {
    gLight.ambient.estLux = AMBIENT_SLOW_ALPHA * gLight.ambient.estLux
                          + (1.0f - AMBIENT_SLOW_ALPHA) * ambRaw;
  }
}

static bool ambientDue(uint32_t now) {
  if (now - gLight.ambient.lastAmbientMs < AMBIENT_MIN_GAP_MS) return false;
  if (!gLight.ambient.haveAmbient && AMBIENT_AT_BOOT) return true;
  return (now - gLight.ambient.lastAmbientMs >= AMBIENT_PERIOD_MS);
}


// =======================================================
//                  RESUME GATE MANAGER
// =======================================================
static void armResumeGate(uint8_t discardUpdates = RESUME_DISCARD_UPDATES) {
  gLight.resume.resumeAfterMs = millis() + RESUME_SETTLE_MS;
  gLight.resume.discardLeft = discardUpdates;
  gLight.resume.lastSeenLuxUpdate = gLight.sensor.luxUpdateMs;
}

static bool resumeGateReady(uint32_t now) {
  if (now < gLight.resume.resumeAfterMs) return false;

  if (gLight.resume.discardLeft > 0) {
    if (gLight.sensor.luxUpdateMs != gLight.resume.lastSeenLuxUpdate) {
      gLight.resume.lastSeenLuxUpdate = gLight.sensor.luxUpdateMs;
      gLight.resume.discardLeft--;
    }
    return false;
  }

  return true;
}


// =======================================================
//                 STEP SCHEDULER MANAGER
// =======================================================
static void clearScheduledAmbient() {
  gLight.step.ambScheduled = false;
  gLight.step.ambRunAtMs = 0;
}

static void armStepDetectIgnore(uint32_t ms) {
  gLight.step.detectIgnoreUntilMs = millis() + ms;
  gLight.step.ambLikeFast = NAN;
  gLight.step.ambLikeSlow = NAN;
  gLight.step.stepCount = 0;
  gLight.step.stepConfirmPending = false;
  gLight.step.stepConfirmAtMs = 0;
  gLight.step.stepBaseline = NAN;
  clearScheduledAmbient();
}

static bool scheduledAmbientDue(uint32_t now) {
  return gLight.step.ambScheduled && ((int32_t)(now - gLight.step.ambRunAtMs) >= 0);
}

static bool okToDetectAmbientStep() {
  return gLight.cal.haveSlope && isfinite(gLight.cal.slopeEma) && gLight.cal.slopeEma > 1e-3f;
}

static void updateStepDelayDetector(uint32_t now) {
  if (!gLight.sensor.haveLux) return;
  if ((int32_t)(now - gLight.step.detectIgnoreUntilMs) < 0) return;
  if (!okToDetectAmbientStep()) return;
  if (gLight.step.ambScheduled) return;

  const float ledEst  = AMBLIKE_K * gLight.cal.slopeEma * gLight.ctrl.pwmEma;
  const float ambLike = gLight.sensor.lux - ledEst;

  if (!isfinite(gLight.step.ambLikeFast)) {
    gLight.step.ambLikeFast = ambLike;
    gLight.step.ambLikeSlow = ambLike;
    gLight.step.stepCount = 0;
    gLight.step.stepConfirmPending = false;
    return;
  }

  gLight.step.ambLikeFast = LUX_FAST_ALPHA * gLight.step.ambLikeFast
                          + (1.0f - LUX_FAST_ALPHA) * ambLike;
  gLight.step.ambLikeSlow = LUX_SLOW_ALPHA * gLight.step.ambLikeSlow
                          + (1.0f - LUX_SLOW_ALPHA) * ambLike;

  const float stepness = fabsf(gLight.step.ambLikeFast - gLight.step.ambLikeSlow);

  if (gLight.step.stepConfirmPending) {
    if ((int32_t)(now - gLight.step.stepConfirmAtMs) >= 0) {
      const float stillDiff = fabsf(gLight.step.ambLikeSlow - gLight.step.stepBaseline);
      if (stillDiff >= STEP_CONFIRM_LUX) {
        gLight.step.ambScheduled = true;
        gLight.step.ambRunAtMs = now + AMB_STEP_DELAY_MS;
        TPRINTLN("AMB | STEP CONFIRMED -> schedule AMB+CAL bundle");
      } else {
        TPRINTLN("AMB | STEP REJECTED (transient)");
      }
      gLight.step.stepConfirmPending = false;
      gLight.step.stepCount = 0;
    }
    return;
  }

  if (stepness >= STEP_DETECT_LUX) {
    if (++gLight.step.stepCount >= STEP_DETECT_TICKS_REQ) {
      gLight.step.stepConfirmPending = true;
      gLight.step.stepConfirmAtMs = now + STEP_CONFIRM_MS;
      gLight.step.stepBaseline = gLight.step.ambLikeSlow;
      gLight.step.stepCount = 0;
      TPRINTLN("AMB | STEP DETECTED -> confirming...");
    }
  } else {
    gLight.step.stepCount = 0;
  }
}


// =======================================================
//                        LOGGING
// =======================================================
static void printCtrlLine(float ledLux, float targetLed, float ledErr,
                          float targetPwmClamped, float targetPwmRaw,
                          const char* evt) {
  printTimePrefix();
  Serial.print("CTRL | target=");   Serial.print(TARGET_LUX, 1);
  Serial.print(" lux=");            Serial.print(gLight.sensor.lux, 1);
  Serial.print(" amb=");            Serial.print(gLight.ambient.estLux, 1);
  Serial.print(" led=");            Serial.print(ledLux, 1);
  Serial.print(" tLed=");           Serial.print(targetLed, 1);
  Serial.print(" err=");            Serial.print(ledErr, 1);
  Serial.print(" errEma=");         Serial.print(gLight.ctrl.errEma, 1);
  Serial.print(" pwm=");            Serial.print(gLight.ctrl.pwmEma, 3);
  Serial.print(" tPwm=");           Serial.print(targetPwmClamped, 3);
  Serial.print(" tPwmRaw=");        Serial.print(targetPwmRaw, 3);
  Serial.print(" slope=");          Serial.print(gLight.cal.slopeEma, 1);
  if (evt && evt[0]) {
    Serial.print(" | evt=");
    Serial.print(evt);
  }
  Serial.println();
}

static void printAmbRead(float raw, float est) {
  printTimePrefix();
  Serial.print("AMB | READ raw="); Serial.print(raw, 1);
  Serial.print(" est=");           Serial.println(est, 1);
}

static void printCalLow(float luxLow, float ledLow) {
  printTimePrefix();
  Serial.print("CAL | LOW  lux="); Serial.print(luxLow, 1);
  Serial.print(" led=");           Serial.println(ledLow, 1);
}

static void printCalHigh(float luxHigh, float ledHigh) {
  printTimePrefix();
  Serial.print("CAL | HIGH lux="); Serial.print(luxHigh, 1);
  Serial.print(" led=");           Serial.println(ledHigh, 1);
}

static void printCalDone(float slope, float amb) {
  printTimePrefix();
  Serial.print("CAL | DONE slopeEma="); Serial.print(slope, 1);
  Serial.print(" ambient=");            Serial.println(amb, 1);
}

static void printCalWarnLedDelta(float ledDelta) {
  printTimePrefix();
  Serial.print("CAL | WARN ledDelta invalid/small: ");
  Serial.println(ledDelta, 1);
}


// =======================================================
//                  PHOTOPERIOD MANAGER
// =======================================================
static bool lightsShouldBeOn(uint32_t now) {
  uint32_t t = (uint32_t)(now - gLight.photo.cycleStartMs);
  t %= CYCLE_MS;
  return LIGHTS_ON_AT_BOOT ? (t < ON_MS) : (t >= OFF_MS);
}


// =======================================================
//               TOP-LEVEL MODE TRANSITIONS
// =======================================================
static void enterMode(uint8_t mode) {
  gLight.rt.mode = mode;
  gLight.rt.pwmSaved = gLight.ctrl.pwmEma;
  resetControlFlags();

  if (mode == MODE_AMBIENT_SAMPLE) {
    gLight.ambMode.phase = AMB_START;
  } else if (mode == MODE_CALIBRATION) {
    gLight.calMode.phase = CAL_START;
  }
}


// =======================================================
//                  MODE: AMBIENT SAMPLE
// =======================================================
static void updateAmbientMode(uint32_t now) {
  updateSampleJob(now);

  switch (gLight.ambMode.phase) {
    case AMB_IDLE:
      return;

    case AMB_START:
      TPRINTLN("AMB | START (LEDs OFF)");
      applyPWMAnyImmediate(0.0f);
      startSampleJob(AMBIENT_OFF_HOLD_MS);
      gLight.ambMode.phase = AMB_WAIT_SAMPLE;
      return;

    case AMB_WAIT_SAMPLE:
      if (gLight.sample.phase == SAMPLE_DONE) {
        const float amb = gLight.sample.result;
        gLight.ambient.lastAmbientMs = now;

        updateAmbientEstimate(amb);
        printAmbRead(amb, gLight.ambient.estLux);

        TPRINTLN("BUNDLE | AMB OK -> CAL next");
        gLight.rt.mode = MODE_CALIBRATION;
        gLight.calMode.phase = CAL_START;
        gLight.ambMode.phase = AMB_IDLE;
      } else if (gLight.sample.phase == SAMPLE_FAIL) {
        gLight.ambient.lastAmbientMs = now;
        gLight.ambMode.phase = AMB_RESTORE_FAIL;
      }
      return;

    case AMB_RESTORE_FAIL:
      TPRINTLN("AMB | WARN sample failed -> restore");

      gLight.ctrl.pwmCmd = clampPwmWindow(gLight.rt.pwmSaved);
      gLight.ctrl.pwmEma = gLight.ctrl.pwmCmd;
      applyPWMValidatedIfChanged(gLight.ctrl.pwmEma);
      gLight.ctrl.errEma = 0.0f;

      lockClampEntryUntilSafeSample();
      armResumeGate();
      armStepDetectIgnore(RESUME_SETTLE_MS + 1500);

      gLight.rt.mode = MODE_CONTROL;
      gLight.ambMode.phase = AMB_IDLE;
      return;

    case AMB_TO_CAL:
      return;
  }
}


// =======================================================
//                  MODE: CALIBRATION
// =======================================================
static void updateCalibrationMode(uint32_t now) {
  updateSampleJob(now);

  switch (gLight.calMode.phase) {
    case CAL_IDLE:
      return;

    case CAL_START:
      if (!gLight.ambient.haveAmbient) {
        gLight.rt.mode = MODE_CONTROL;
        gLight.calMode.phase = CAL_IDLE;
        return;
      }
      TPRINTLN("CAL | START");
      gLight.calMode.phase = CAL_SET_LOW;
      return;

    case CAL_SET_LOW:
      applyPWMValidatedIfChanged(CAL_LOW_FRAC);
      startSampleJob(SETTLE_LOW_MS);
      gLight.calMode.phase = CAL_WAIT_LOW;
      return;

    case CAL_WAIT_LOW:
      if (gLight.sample.phase == SAMPLE_DONE) {
        gLight.calMode.luxLow = gLight.sample.result;
        gLight.calMode.phase = CAL_SET_HIGH;
      } else if (gLight.sample.phase == SAMPLE_FAIL) {
        TPRINTLN("CAL | FAIL sample(s)");
        gLight.calMode.phase = CAL_RESTORE;
      }
      return;

    case CAL_SET_HIGH:
      applyPWMValidatedIfChanged(CAL_HIGH_FRAC);
      startSampleJob(SETTLE_HIGH_MS);
      gLight.calMode.phase = CAL_WAIT_HIGH;
      return;

    case CAL_WAIT_HIGH:
      if (gLight.sample.phase == SAMPLE_DONE) {
        gLight.calMode.luxHigh = gLight.sample.result;
        gLight.calMode.phase = CAL_PROCESS;
      } else if (gLight.sample.phase == SAMPLE_FAIL) {
        TPRINTLN("CAL | FAIL sample(s)");
        gLight.calMode.phase = CAL_RESTORE;
      }
      return;

    case CAL_PROCESS: {
      const float ledLow  = max(0.0f, gLight.calMode.luxLow  - gLight.ambient.estLux);
      const float ledHigh = max(0.0f, gLight.calMode.luxHigh - gLight.ambient.estLux);

      printCalLow(gLight.calMode.luxLow, ledLow);
      printCalHigh(gLight.calMode.luxHigh, ledHigh);

      const float ledDelta  = ledHigh - ledLow;
      const float fracDelta = (CAL_HIGH_FRAC - CAL_LOW_FRAC);

      if (isfinite(ledDelta) && ledDelta > MIN_LED_LUX_DELTA_FOR_SLOPE && fracDelta > 1e-6f) {
        const float slopeInst = ledDelta / fracDelta;

        if (!gLight.cal.haveSlope || !isfinite(gLight.cal.slopeEma)) {
          gLight.cal.slopeEma = slopeInst;
          gLight.cal.haveSlope = true;
        } else {
          gLight.cal.slopeEma = SLOPE_EMA_ALPHA * gLight.cal.slopeEma
                              + (1.0f - SLOPE_EMA_ALPHA) * slopeInst;
        }

        printCalDone(gLight.cal.slopeEma, gLight.ambient.estLux);
      } else {
        printCalWarnLedDelta(ledDelta);
      }

      gLight.calMode.phase = CAL_RESTORE;
      return;
    }

    case CAL_RESTORE:
      applyPWMValidatedIfChanged(clampPwmWindow(gLight.rt.pwmSaved));
      TPRINTLN("CAL | RESTORE PWM");

      lockClampEntryUntilSafeSample();

      gLight.ctrl.pwmCmd = clampPwmWindow(gLight.rt.pwmSaved);
      gLight.ctrl.pwmEma = gLight.ctrl.pwmCmd;
      gLight.ctrl.errEma = 0.0f;

      armResumeGate();
      armStepDetectIgnore(RESUME_SETTLE_MS + 1500);

      gLight.calMode.phase = CAL_FINISH;
      return;

    case CAL_FINISH:
      gLight.rt.mode = MODE_CONTROL;
      gLight.calMode.phase = CAL_IDLE;
      return;
  }
}


// =======================================================
//                 MODE: CONTROL TICK
// =======================================================
static void runControlTick(uint32_t now) {
  if (!sensorFresh(now)) return;

  if (!gLight.ambient.haveAmbient || !gLight.cal.haveSlope ||
      !isfinite(gLight.cal.slopeEma) || gLight.cal.slopeEma < 1e-3f) {
    static uint32_t lastPrint = 0;
    if (now - lastPrint > 1000) {
      lastPrint = now;
      TPRINTLN("CTRL | waiting for ambient+slope");
    }
    return;
  }

  const float ledLux    = max(0.0f, gLight.sensor.lux - gLight.ambient.estLux);
  const float targetLed = max(0.0f, TARGET_LUX - gLight.ambient.estLux);
  const float ledErr    = targetLed - ledLux;

  float targetPwmRaw = (targetLed <= 0.0f) ? 0.0f : (targetLed / gLight.cal.slopeEma);
  float targetPwmClamped = constrain(targetPwmRaw, PWM_MIN_FRAC, PWM_MAX_FRAC);

  if (targetLed <= 0.0f) {
    applyPWMAnyImmediate(0.0f);
    gLight.ctrl.pwmCmd = 0.0f;
    gLight.ctrl.pwmEma = 0.0f;
    gLight.ctrl.errEma = 0.0f;
    resetControlFlags();
    printCtrlLine(0.0f, targetLed, 0.0f, 0.0f, targetPwmRaw, "AMBIENT_OK_OFF");
    return;
  }

  if (targetPwmRaw < PWM_MIN_FRAC) {
    targetPwmClamped = PWM_MIN_FRAC;
  }

  if (gLight.ctrl.clampEntryLocked) {
    if (gLight.sensor.lux <= TARGET_LUX + OVERSHOOT_EXIT_LUX) {
      gLight.ctrl.clampEntryLocked = false;
    }
  }

  gLight.ctrl.errEma = ERR_EMA_ALPHA * gLight.ctrl.errEma + (1.0f - ERR_EMA_ALPHA) * ledErr;
  const float absErrEma = fabsf(gLight.ctrl.errEma);

  const char* evt = "";

  if (!gLight.ctrl.overshootClamped) {
    if (!gLight.ctrl.clampEntryLocked && gLight.sensor.lux >= TARGET_LUX + OVERSHOOT_CLAMP_LUX) {
      gLight.ctrl.overshootClamped = true;
      gLight.ctrl.holding = false;
      gLight.ctrl.enterCount = 0;
      gLight.ctrl.exitCount = 0;
      gLight.ctrl.fastExitCount = 0;

      gLight.ctrl.pwmCmd = PWM_MIN_FRAC;
      gLight.ctrl.pwmEma = PWM_MIN_FRAC;
      applyPWMValidatedIfChanged(gLight.ctrl.pwmEma);

      evt = "CLAMP";
      printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
      return;
    }
  } else {
    if (gLight.sensor.lux <= TARGET_LUX + OVERSHOOT_EXIT_LUX) {
      gLight.ctrl.overshootClamped = false;
      evt = "CLAMP_REL";
      printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
    } else {
      evt = "CLAMP_HOLD";
      printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
      return;
    }
  }

  if (gLight.ctrl.holding) {
    if (ledErr >= HOLD_FAST_EXIT_ERR_LUX) {
      if (++gLight.ctrl.fastExitCount >= HOLD_FAST_EXIT_TICKS) {
        gLight.ctrl.holding = false;
        gLight.ctrl.enterCount = 0;
        gLight.ctrl.exitCount = 0;
        gLight.ctrl.fastExitCount = 0;
        evt = "FAST_EXIT";
      }
    } else {
      gLight.ctrl.fastExitCount = 0;
    }
  } else {
    gLight.ctrl.fastExitCount = 0;
  }

  const bool okToHold = (gLight.ctrl.errEma >= HOLD_ABOVE_GUARD_LUX);

  if (gLight.ctrl.holding) {
    if (absErrEma >= HOLD_EXIT_LUX) {
      if (++gLight.ctrl.exitCount >= HOLD_EXIT_TICKS) {
        gLight.ctrl.holding = false;
        gLight.ctrl.enterCount = 0;
        gLight.ctrl.exitCount = 0;
        if (!evt[0]) evt = "EXIT_HOLD";
      }
    } else {
      gLight.ctrl.exitCount = 0;
    }
  } else {
    if (okToHold && absErrEma <= HOLD_ENTER_LUX) {
      if (++gLight.ctrl.enterCount >= HOLD_ENTER_TICKS) {
        gLight.ctrl.holding = true;
        gLight.ctrl.enterCount = 0;
        gLight.ctrl.exitCount = 0;
        gLight.ctrl.fastExitCount = 0;

        gLight.ctrl.pwmCmd = gLight.ctrl.pwmEma;
        applyPWMValidatedIfChanged(gLight.ctrl.pwmEma);

        if (!evt[0]) evt = "ENTER_HOLD";
      }
    } else {
      gLight.ctrl.enterCount = 0;
    }
  }

  if (gLight.ctrl.holding) {
    if (!evt[0]) evt = "HOLD";
    printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
    return;
  }

  float deltaFrac = ledErr / gLight.cal.slopeEma;

  const float tickUp   = RATE_UP_PER_SEC   * (CONTROL_PERIOD_MS / 1000.0f);
  const float tickDown = RATE_DOWN_PER_SEC * (CONTROL_PERIOD_MS / 1000.0f);

  if (deltaFrac >= 0.0f) deltaFrac = constrain(deltaFrac, 0.0f, tickUp);
  else                   deltaFrac = constrain(deltaFrac, -tickDown, 0.0f);

  const bool atMin = (gLight.ctrl.pwmCmd <= PWM_MIN_FRAC + 0.0005f);
  if (atMin && ledErr < 0.0f) {
    deltaFrac = 0.0f;
    gLight.ctrl.pwmCmd = PWM_MIN_FRAC;
  } else {
    gLight.ctrl.pwmCmd = constrain(gLight.ctrl.pwmCmd + deltaFrac, PWM_MIN_FRAC, PWM_MAX_FRAC);
  }

  gLight.ctrl.pwmEma = PWM_EMA_ALPHA * gLight.ctrl.pwmEma + (1.0f - PWM_EMA_ALPHA) * gLight.ctrl.pwmCmd;
  gLight.ctrl.pwmEma = constrain(gLight.ctrl.pwmEma, PWM_MIN_FRAC, PWM_MAX_FRAC);

  applyPWMValidatedIfChanged(gLight.ctrl.pwmEma);
  printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
}


// =======================================================
//                  TOP-LEVEL LIGHT LOOP
// =======================================================
static void updateLightSystem(uint32_t now) {
  sensorUpdate();

  const bool lightsOn = lightsShouldBeOn(now);

  if (lightsOn != gLight.photo.prevLightsOn) {
    gLight.photo.prevLightsOn = lightsOn;

    if (!lightsOn) {
      TPRINTLN("CYCLE | LIGHTS_OFF (forcing PWM=0, pausing control/amb/cal)");
      applyPWMAnyImmediate(0.0f);

      armStepDetectIgnore(OFF_MS + 500);
      clearScheduledAmbient();

      gLight.sample.phase = SAMPLE_IDLE;
      gLight.ambMode.phase = AMB_IDLE;
      gLight.calMode.phase = CAL_IDLE;
      gLight.rt.mode = MODE_CONTROL;
      return;
    } else {
      TPRINTLN("CYCLE | LIGHTS_ON (run AMB->CAL bundle, then resume control)");
      resetControlFlags();
      clearScheduledAmbient();
      enterMode(MODE_AMBIENT_SAMPLE);
      return;
    }
  }

  if (!lightsOn) {
    applyPWMAnyImmediate(0.0f);
    return;
  }

  if (gLight.rt.mode == MODE_AMBIENT_SAMPLE) {
    updateAmbientMode(now);
    return;
  }

  if (gLight.rt.mode == MODE_CALIBRATION) {
    updateCalibrationMode(now);
    return;
  }

  if (!resumeGateReady(now)) return;
  if (!sensorFresh(now)) return;

  updateStepDelayDetector(now);

  if (!gLight.ambient.haveAmbient && AMBIENT_AT_BOOT) {
    clearScheduledAmbient();
    enterMode(MODE_AMBIENT_SAMPLE);
    return;
  }

  if (gLight.ambient.haveAmbient &&
      (!gLight.cal.haveSlope || !isfinite(gLight.cal.slopeEma) || gLight.cal.slopeEma < 1e-3f)) {
    enterMode(MODE_CALIBRATION);
    return;
  }

  const bool wantPeriodic  = ambientDue(now);
  const bool wantScheduled = scheduledAmbientDue(now);

  if (wantScheduled && (now - gLight.ambient.lastAmbientMs >= AMBIENT_MIN_GAP_MS)) {
    TPRINTLN("AMB | TRIGGER (scheduled step) -> bundle");
    clearScheduledAmbient();
    enterMode(MODE_AMBIENT_SAMPLE);
    return;
  }

  if (wantPeriodic &&
      (now - gLight.ambient.lastAmbientMs >= AMBIENT_MIN_GAP_MS) &&
      !gLight.ctrl.overshootClamped) {
    TPRINTLN("AMB | TRIGGER (periodic) -> bundle");
    enterMode(MODE_AMBIENT_SAMPLE);
    return;
  }

  static uint32_t lastControlTickMs = 0;
  if (now - lastControlTickMs >= CONTROL_PERIOD_MS) {
    lastControlTickMs = now;
    runControlTick(now);
  }
}


// =======================================================
//                         SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(21, 22);

  if (!bh1750.begin(BH1750_TO_GROUND)) {
    TPRINTLN("BH1750 init failed");
    while (true) delay(1000);
  }

  bh1750.calibrateTiming();
  bh1750.setQuality(BH1750_QUALITY_HIGH);

  ledcSetup(WHITE_PWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(RB_PWM_CH,    PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(WHITE_PWM_PIN, WHITE_PWM_CH);
  ledcAttachPin(RB_PWM_PIN,    RB_PWM_CH);

  gLight.ctrl.pwmCmd = clampPwmWindow(gLight.ctrl.pwmCmd);
  gLight.ctrl.pwmEma = clampPwmWindow(gLight.ctrl.pwmEma);
  applyPWMValidatedIfChanged(gLight.ctrl.pwmEma);

  sensorBegin();

  gLight.photo.cycleStartMs = millis();
  gLight.photo.prevLightsOn = LIGHTS_ON_AT_BOOT;
  armStepDetectIgnore(4000);

  TPRINTLN("READY — fully non-blocking light module. 1min ON / 1min OFF. AMB+CAL bundled.");
}


// =======================================================
//                          LOOP
// =======================================================
void loop() {
  updateLightSystem(millis());
}