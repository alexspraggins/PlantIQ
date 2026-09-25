#include <Arduino.h>
#include <math.h>

/*
  ==========================================================
  TEST4 – VH400 VWC Filtering + Low-Consecutive Trigger
  ==========================================================

  PURPOSE:
  --------
  Build the "decision layer" that will later trigger watering.

  This test:
    1) Reads VH400 using ADC averaging
    2) Applies your voltage calibration (matches multimeter)
    3) Handles ADC floor (adc≈0 at very low voltage -> force 0V)
    4) Converts calibrated voltage to VWC (%) using VH400 piecewise curve
    5) Applies EMA smoothing on VWC (spike reduction)
    6) Implements LOW-CONSECUTIVE trigger logic:
         - if VWC_ema stays below LOWER_VWC for N consecutive samples,
           print TRIGGER_READY

  WHY THIS MATTERS:
  -----------------
  Soil moisture is noisy + delayed. EMA + consecutive-low prevents
  false triggers from bumps, brief ADC spikes, or transient changes.

  OUTPUT:
  -------
  Prints:
    adcAvg, vCal, vwcRaw, vwcEma, lowCount, and optional TRIGGER_READY
*/

// ---------------- Hardware ----------------
static const int MOISTURE_ADC_PIN = 34;

// ---------------- ADC Constants ----------------
static const float ADC_REF_V = 3.30f;
static const int   ADC_MAX   = 4095;

// ---------------- Voltage Calibration ----------------
// V_true ≈ V_GAIN * V_raw + V_BIAS (from multimeter)
static const float V_GAIN = 1.03202105f;
static const float V_BIAS = 0.11009852f;

// ---------------- ADC Floor Handling ----------------
static const int ADC_FLOOR_THRESHOLD = 10;

// ---------------- Sampling / Timing ----------------
static const int      SAMPLES = 12;
static const uint32_t PRINT_MS = 500;

// ---------------- Filter / Trigger Params ----------------
static const float EMA_ALPHA = 0.20f;   // 0..1 (higher = faster, noisier)
static const float LOWER_VWC = 32.0f;   // trigger threshold (example)
static const int   LOW_CONSEC_REQUIRED = 4;

// ---------------- Runtime ----------------
static float vwcEma = NAN;
static int   lowCount = 0;
static uint32_t lastPrintMs = 0;

// ---------------- Utility ----------------
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

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
  // Floor handling: prevent V_BIAS from creating a fake ~0.11V at adc≈0
  if (adcAvg <= ADC_FLOOR_THRESHOLD) return 0.0f;

  float vRaw = adcToVraw(adcAvg);
  float vCal = V_GAIN * vRaw + V_BIAS;
  return clampf(vCal, 0.0f, 3.60f);
}

// VH400 datasheet piecewise voltage -> VWC
static float vh400VoltageToVWC(float v) {
  float vwc;

  if (v <= 1.10f) {
    vwc = 10.0f * v - 1.0f;
  } else if (v <= 1.30f) {
    vwc = 25.0f * v - 17.5f;
  } else if (v <= 1.82f) {
    vwc = 48.08f * v - 47.5f;
  } else if (v <= 2.20f) {
    vwc = 26.32f * v - 7.89f;
  } else if (v <= 3.00f) {
    vwc = 62.5f * v - 87.5f;
  } else {
    vwc = 62.5f * v - 87.5f; // extend
  }

  return clampf(vwc, 0.0f, 100.0f);
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  Serial.println("TEST4 | VH400: VWC EMA + low-consecutive trigger");
}

// ---------------- Loop ----------------
void loop() {
  const uint32_t now = millis();
  if (now - lastPrintMs < PRINT_MS) return;
  lastPrintMs = now;

  // 1) Read sensor -> calibrated voltage -> raw VWC
  int adcAvg = readAdcAvg();
  float vCal = calibrateVoltageFromAdc(adcAvg);
  float vwcRaw = vh400VoltageToVWC(vCal);

  // 2) EMA filter on VWC
  if (isnan(vwcEma)) vwcEma = vwcRaw;
  else vwcEma = EMA_ALPHA * vwcRaw + (1.0f - EMA_ALPHA) * vwcEma;

  // 3) Low-consecutive trigger logic (based on filtered VWC)
  if (!isnan(vwcEma) && vwcEma < LOWER_VWC) lowCount++;
  else lowCount = 0;

  // 4) Print
  Serial.print("TEST4 | adcAvg=");
  Serial.print(adcAvg);

  Serial.print(" vCal=");
  Serial.print(vCal, 3);

  Serial.print(" vwcRaw=");
  Serial.print(vwcRaw, 1);

  Serial.print(" vwcEma=");
  Serial.print(vwcEma, 1);

  Serial.print(" lowCount=");
  Serial.print(lowCount);

  if (lowCount >= LOW_CONSEC_REQUIRED) {
    Serial.print(" TRIGGER_READY");
  }

  Serial.println();
}