#include <Arduino.h>
#include <math.h>

/*
  ==========================================================
  TEST6B – Option B Controller + Safety + Fault + Immediate Force-Start
  ==========================================================

  Adds to TEST5:
    - MAX_PULSES_PER_CYCLE
    - MAX_TOTAL_PUMP_ON_MS (per cycle)
    - NO_RISE detection (VWC not increasing after N pulses)
    - FAULT state (pump forced OFF)

  Serial test controls:
    - 's' : FORCE START immediately (interrupts any active non-FAULT state)
    - 'r' : Reset FAULT -> IDLE

  Logging format:
    [millis] TYPE | key=value key=value ...

  TYPE:
    STAT : periodic snapshot
    EVT  : state transitions / decisions
    FLT  : fault events
*/

static const int MOISTURE_ADC_PIN = 34;
static const int PUMP_PIN         = 25;

// ---------------- ADC constants ----------------
static const float ADC_REF_V = 3.30f;
static const int   ADC_MAX   = 4095;

// Voltage calibration (from your multimeter)
static const float V_GAIN = 1.03202105f;
static const float V_BIAS = 0.11009852f;

// ADC floor handling
static const int ADC_FLOOR_THRESHOLD = 10;

// Sampling / Filter
static const int   SAMPLES    = 12;
static const float EMA_ALPHA  = 0.20f;

// Targets / Trigger
static float LOWER_VWC = 10.4f;   // start cycle if vwcEma < LOWER for N samples
static float UPPER_VWC = 11.0f;  // stop cycle if vwcEma >= UPPER
static const int LOW_CONSEC_REQUIRED = 4;

// Pulse / Diffusion
static const uint32_t PULSE_ON_MS     = 750;
static const uint32_t DIFFUSE_WAIT_MS = 5000;

// Timing
static const uint32_t SENSOR_POLL_MS  = 500;
static const uint32_t CTRL_TICK_MS    = 100;
static const uint32_t STATUS_PRINT_MS = 500;

// Sensor freshness gate
static const uint32_t SENSOR_FRESH_MS = SENSOR_POLL_MS * 2; // 1s

// ---------------- Safety limits ----------------
static const int      MAX_PULSES_PER_CYCLE  = 12;
static const uint32_t MAX_TOTAL_PUMP_ON_MS  = 30000; // 30s ON max per cycle

// “No rise” detection
static const int   RISE_CHECK_AFTER_PULSES = 3;     // evaluate after this many pulses
static const float MIN_EXPECTED_RISE_VWC   = 0.8f;  // expected minimum increase from cycle start

// ---------------- State Machine ----------------
enum class CtrlState : uint8_t { IDLE, PULSE_ON, DIFFUSE_WAIT, CHECK, FAULT };
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
static uint32_t totalPumpOnMs = 0;
static float cycleStartVwc = NAN;

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
    case CtrlState::FAULT:        return "FAULT";
    default: return "?";
  }
}

static void enterState(CtrlState s) {
  state = s;
  stateEnterMs = millis();
}

// -------- Structured logging --------
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

static void logFlt(const char* msg) {
  logLine("FLT");
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

  Serial.print(" onMs=");
  Serial.print(totalPumpOnMs);

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

// VH400 piecewise voltage -> VWC
static float vh400VoltageToVWC(float v) {
  float vwc;
  if (v <= 1.10f)      vwc = 10.0f * v - 1.0f;
  else if (v <= 1.30f) vwc = 25.0f * v - 17.5f;
  else if (v <= 1.82f) vwc = 48.08f * v - 47.5f;
  else if (v <= 2.20f) vwc = 26.32f * v - 7.89f;
  else if (v <= 3.00f) vwc = 62.5f  * v - 87.5f;
  else                 vwc = 62.5f  * v - 87.5f;
  return clampf(vwc, 0.0f, 100.0f);
}

static bool sensorFresh(uint32_t now) {
  return (now - lastSensorMs) <= SENSOR_FRESH_MS;
}

// ---------------- Cycle control helpers ----------------
static void resetCycleVars() {
  lowCount = 0;
  pulsesThisCycle = 0;
  totalPumpOnMs = 0;
  cycleStartVwc = vwcEma; // baseline for rise detection (can be NAN if sensor not ready)
}

static void startCycleNow(const char* reasonMsg) {
  logEvt(reasonMsg);
  resetCycleVars();
  pumpSet(true);
  enterState(CtrlState::PULSE_ON);
}

static void enterFault(const char* faultMsg) {
  pumpSet(false);
  logFlt(faultMsg);
  enterState(CtrlState::FAULT);
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(PUMP_PIN, OUTPUT);
  pumpSet(false);

  Serial.println("\nTEST6B | Option B + Safety + Fault + Immediate Force-Start");
  Serial.println("Serial: 's' force start NOW, 'r' reset FAULT\n");

  logEvt("event=BOOT");
  enterState(CtrlState::IDLE);
}

void loop() {
  const uint32_t now = millis();

  // Serial controls (non-blocking)
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == 's' || c == 'S') {
      if (state != CtrlState::FAULT) {
        // Interrupt current cycle/state and restart immediately
        startCycleNow("event=START_CYCLE reason=FORCED_IMMEDIATE");
      } else {
        logEvt("event=START_IGNORED reason=IN_FAULT");
      }
    }

    if (c == 'r' || c == 'R') {
      if (state == CtrlState::FAULT) {
        logEvt("event=FAULT_RESET action=IDLE");
        pumpSet(false);
        pulsesThisCycle = 0;
        totalPumpOnMs = 0;
        cycleStartVwc = NAN;
        lowCount = 0;
        enterState(CtrlState::IDLE);
      }
    }
  }

  // (1) Sensor update loop
  if (now - lastSensorMs >= SENSOR_POLL_MS) {
    lastSensorMs = now;

    int adcAvg = readAdcAvg();
    vCal = calibrateVoltageFromAdc(adcAvg);
    vwcRaw = vh400VoltageToVWC(vCal);

    if (isnan(vwcEma)) vwcEma = vwcRaw;
    else vwcEma = EMA_ALPHA * vwcRaw + (1.0f - EMA_ALPHA) * vwcEma;

    // Only count consecutive-low in IDLE
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
        startCycleNow("event=START_CYCLE reason=LOW_CONSEC");
      }
      break;
    }

    case CtrlState::PULSE_ON: {
      // Safety: total ON time cap
      if (totalPumpOnMs >= MAX_TOTAL_PUMP_ON_MS) {
        enterFault("fault=MAX_TOTAL_ON_TIME action=PUMP_OFF");
        break;
      }

      if (now - stateEnterMs >= PULSE_ON_MS) {
        pumpSet(false);
        pulsesThisCycle++;
        totalPumpOnMs += PULSE_ON_MS;

        // Safety: max pulses cap
        if (pulsesThisCycle >= MAX_PULSES_PER_CYCLE) {
          enterFault("fault=MAX_PULSES action=PUMP_OFF");
          break;
        }

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
      // Don’t decide on stale/unknown values
      if (!sensorFresh(now) || isnan(vwcEma) || isnan(cycleStartVwc)) {
        enterFault("fault=SENSOR_STALE_OR_NAN action=PUMP_OFF");
        break;
      }

      // Stop condition
      if (vwcEma >= UPPER_VWC) {
        pumpSet(false);
        logEvt("event=STOP_CYCLE reason=UPPER_REACHED");
        pulsesThisCycle = 0;
        totalPumpOnMs = 0;
        cycleStartVwc = NAN;
        enterState(CtrlState::IDLE);
        break;
      }

      // No-rise detection after N pulses
      if (pulsesThisCycle >= RISE_CHECK_AFTER_PULSES) {
        float rise = vwcEma - cycleStartVwc;
        if (rise < MIN_EXPECTED_RISE_VWC) {
          enterFault("fault=NO_RISE action=PUMP_OFF");
          break;
        }
      }

      // Continue watering
      logEvt("event=NEXT_PULSE reason=BELOW_UPPER");
      pumpSet(true);
      enterState(CtrlState::PULSE_ON);
      break;
    }

    case CtrlState::FAULT: {
      pumpSet(false);
      // Stay here until 'r' reset
      break;
    }
  }
}