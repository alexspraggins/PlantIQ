#include <Arduino.h>
#include <math.h>

/*
  ==========================================================
  TEST5 – Option B Core (Pulse + Diffusion) + Structured Logs
  ==========================================================

  This test:
    - Reads VH400 -> calibrated voltage -> VWC (piecewise)
    - EMA filters VWC
    - Requires LOW_CONSECUTIVE samples below LOWER_VWC to start a cycle
    - Waters using:
        PULSE_ON (pump ON for PULSE_ON_MS)
        DIFFUSE  (pump OFF for DIFFUSE_WAIT_MS)
        CHECK    (stop if >= UPPER_VWC else pulse again)

  Fixes added:
    - If VWC is invalid (NaN), controller will NOT keep pulsing forever.
      It will hold safely (pump OFF) and return to IDLE.
    - CHECK waits for a recent sensor update (prevents stale decisions).

  Logging format:
    [millis] TYPE | key=value key=value ...

  TYPE meanings:
    STAT : periodic snapshot
    EVT  : state transitions / decisions
*/

static const int MOISTURE_ADC_PIN = 34;
static const int PUMP_PIN         = 25;

// ADC constants
static const float ADC_REF_V = 3.30f;
static const int   ADC_MAX   = 4095;

// Voltage calibration (from your multimeter)
static const float V_GAIN = 1.03202105f;
static const float V_BIAS = 0.11009852f;

// ADC floor handling (adc≈0 at very low V -> force 0V)
static const int ADC_FLOOR_THRESHOLD = 10;

// Sampling / Filter
static const int   SAMPLES    = 12;
static const float EMA_ALPHA  = 0.20f;

// Targets / Trigger (your requested values)
static float LOWER_VWC = 1.0f;
static float UPPER_VWC = 9.0f;
static const int LOW_CONSEC_REQUIRED = 4;

// Pulse / Diffusion (your requested values)
static const uint32_t PULSE_ON_MS     = 3000;
static const uint32_t DIFFUSE_WAIT_MS = 5000;

// Timing
static const uint32_t SENSOR_POLL_MS  = 500;
static const uint32_t CTRL_TICK_MS    = 100;
static const uint32_t STATUS_PRINT_MS = 500;

// Require sensor freshness before making CHECK decisions
static const uint32_t SENSOR_FRESH_MS = SENSOR_POLL_MS * 2; // 1s window

// State Machine
enum class CtrlState : uint8_t { IDLE, PULSE_ON, DIFFUSE_WAIT, CHECK };
static CtrlState state = CtrlState::IDLE;

static uint32_t stateEnterMs = 0;
static uint32_t lastSensorMs = 0;
static uint32_t lastCtrlMs   = 0;
static uint32_t lastPrintMs  = 0;

static float vwcRaw = NAN;
static float vwcEma = NAN;
static float vCal   = NAN;

static int lowCount = 0;
static int pulsesThisCycle = 0;
static bool pumpOn = false;

// ---------------- Utility ----------------
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void pumpSet(bool on) {
  pumpOn = on;
  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
}

static const char* stateName(CtrlState s) {
  switch (s) {
    case CtrlState::IDLE:         return "IDLE";
    case CtrlState::PULSE_ON:     return "PULSE_ON";
    case CtrlState::DIFFUSE_WAIT: return "DIFFUSE";
    case CtrlState::CHECK:        return "CHECK";
    default: return "?";
  }
}

static void enterState(CtrlState s) {
  state = s;
  stateEnterMs = millis();
}

// -------- Structured logging helpers --------
static void logLine(const char* type) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] ");
  Serial.print(type);
  Serial.print(" | ");
}

static void logEvt(const char* msg) {
  logLine("EVT");
  Serial.println(msg);
}

static void printStatus() {
  logLine("STAT");

  Serial.print("state=");
  Serial.print(stateName(state));

  Serial.print(" pump=");
  Serial.print(pumpOn ? "ON" : "OFF");

  Serial.print(" vCal=");
  if (isnan(vCal)) Serial.print("nan");
  else Serial.print(vCal, 3);

  Serial.print(" vwcRaw=");
  if (isnan(vwcRaw)) Serial.print("nan");
  else Serial.print(vwcRaw, 1);

  Serial.print(" vwcEma=");
  if (isnan(vwcEma)) Serial.print("nan");
  else Serial.print(vwcEma, 1);

  Serial.print(" lowCnt=");
  Serial.print(lowCount);

  Serial.print(" pulses=");
  Serial.print(pulsesThisCycle);

  Serial.print(" lower=");
  Serial.print(LOWER_VWC, 1);

  Serial.print(" upper=");
  Serial.print(UPPER_VWC, 1);

  Serial.print(" pulseMs=");
  Serial.print(PULSE_ON_MS);

  Serial.print(" diffuseMs=");
  Serial.print(DIFFUSE_WAIT_MS);

  Serial.println();
}

// ---------------- Sensor pipeline ----------------
static float adcToVraw(int adc) {
  return (float)adc * (ADC_REF_V / (float)ADC_MAX);
}

static int readAdcAvg() {
  uint32_t sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(MOISTURE_ADC_PIN);
    delayMicroseconds(1500);
  }
  return (int)(sum / (uint32_t)SAMPLES);
}

static float calibrateVoltageFromAdc(int adcAvg) {
  if (adcAvg <= ADC_FLOOR_THRESHOLD) return 0.0f;
  float vRaw = adcToVraw(adcAvg);
  float v = V_GAIN * vRaw + V_BIAS;
  return clampf(v, 0.0f, 3.60f);
}

// VH400 datasheet piecewise voltage -> VWC
static float vh400VoltageToVWC(float v) {
  float vwc;
  if (v <= 1.10f)      vwc = 10.0f * v - 1.0f;
  else if (v <= 1.30f) vwc = 25.0f * v - 17.5f;
  else if (v <= 1.82f) vwc = 48.08f * v - 47.5f;
  else if (v <= 2.20f) vwc = 26.32f * v - 7.89f;
  else if (v <= 3.00f) vwc = 62.5f  * v - 87.5f;
  else                 vwc = 62.5f  * v - 87.5f; // extend
  return clampf(vwc, 0.0f, 100.0f);
}

static bool sensorFresh(uint32_t now) {
  return (now - lastSensorMs) <= SENSOR_FRESH_MS;
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(PUMP_PIN, OUTPUT);
  pumpSet(false);

  Serial.println("\nTEST5 | Option B: Pulse + Diffusion controller (structured logs)");
  logEvt("event=BOOT");
  enterState(CtrlState::IDLE);
}

void loop() {
  const uint32_t now = millis();

  // (1) Sensor update loop
  if (now - lastSensorMs >= SENSOR_POLL_MS) {
    lastSensorMs = now;

    int adcAvg = readAdcAvg();
    vCal = calibrateVoltageFromAdc(adcAvg);
    vwcRaw = vh400VoltageToVWC(vCal);

    if (isnan(vwcEma)) vwcEma = vwcRaw;
    else vwcEma = EMA_ALPHA * vwcRaw + (1.0f - EMA_ALPHA) * vwcEma;

    // Only count consecutive-low in IDLE (don’t retrigger while watering)
    if (state == CtrlState::IDLE) {
      if (!isnan(vwcEma) && vwcEma < LOWER_VWC) lowCount++;
      else lowCount = 0;
    } else {
      lowCount = 0;
    }
  }

  // (2) Status print loop
  if (now - lastPrintMs >= STATUS_PRINT_MS) {
    lastPrintMs = now;
    printStatus();
  }

  // (3) Control tick loop
  if (now - lastCtrlMs < CTRL_TICK_MS) return;
  lastCtrlMs = now;

  switch (state) {
    case CtrlState::IDLE: {
      if (lowCount >= LOW_CONSEC_REQUIRED) {
        logEvt("event=START_CYCLE reason=LOW_CONSEC");
        lowCount = 0;
        pulsesThisCycle = 0;

        pumpSet(true);
        enterState(CtrlState::PULSE_ON);
      }
      break;
    }

    case CtrlState::PULSE_ON: {
      if (now - stateEnterMs >= PULSE_ON_MS) {
        pumpSet(false);
        pulsesThisCycle++;

        logEvt("event=PULSE_DONE next=DIFFUSE_WAIT");
        enterState(CtrlState::DIFFUSE_WAIT);
      }
      break;
    }

    case CtrlState::DIFFUSE_WAIT: {
      if (now - stateEnterMs >= DIFFUSE_WAIT_MS) {
        logEvt("event=DIFFUSE_DONE next=CHECK");
        enterState(CtrlState::CHECK);
      }
      break;
    }

    case CtrlState::CHECK: {
      // Don’t decide using stale/unknown sensor values
      if (!sensorFresh(now) || isnan(vwcEma)) {
        pumpSet(false);
        logEvt("event=CHECK_HOLD reason=SENSOR_STALE_OR_NAN action=IDLE");
        pulsesThisCycle = 0;
        enterState(CtrlState::IDLE);
        break;
      }

      if (vwcEma >= UPPER_VWC) {
        pumpSet(false);
        logEvt("event=STOP_CYCLE reason=UPPER_REACHED");

        pulsesThisCycle = 0;
        enterState(CtrlState::IDLE);
      } else {
        logEvt("event=NEXT_PULSE reason=BELOW_UPPER");
        pumpSet(true);
        enterState(CtrlState::PULSE_ON);
      }
      break;
    }
  }
}