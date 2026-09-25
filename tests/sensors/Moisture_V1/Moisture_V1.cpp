#include <Arduino.h>

static const int MOISTURE_ADC_PIN = 34;

// ADC constants
static const float ADC_REF_V = 3.30f;
static const int   ADC_MAX   = 4095;

// Voltage calibration from your multimeter data
// V_true ≈ V_GAIN * V_raw + V_BIAS
static const float V_GAIN = 1.03202105f;
static const float V_BIAS = 0.11009852f;

// Averaging
static const int SAMPLES = 12;

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float adcToVraw(int adc) {
  return (float)adc * (ADC_REF_V / (float)ADC_MAX);
}

static float calibrateVoltage(float vRaw) {
  float v = V_GAIN * vRaw + V_BIAS;
  return clampf(v, 0.0f, 3.60f);
}

static int readAdcAvg() {
  uint32_t sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(MOISTURE_ADC_PIN);
    delayMicroseconds(1500); // slight spacing reduces correlated noise
  }
  return (int)(sum / (uint32_t)SAMPLES);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  Serial.println("TEST2 | ADC single vs avg (with voltage calibration)");
}

void loop() {
  // single sample
  int adc1 = analogRead(MOISTURE_ADC_PIN);
  float vRaw1 = adcToVraw(adc1);
  float vCal1 = calibrateVoltage(vRaw1);

  // averaged
  int adcAvg = readAdcAvg();
  float vRawAvg = adcToVraw(adcAvg);
  float vCalAvg = calibrateVoltage(vRawAvg);

  Serial.print("TEST2 | adc1=");
  Serial.print(adc1);
  Serial.print(" vRaw1=");
  Serial.print(vRaw1, 3);
  Serial.print(" vCal1=");
  Serial.print(vCal1, 3);

  Serial.print(" | adcAvg=");
  Serial.print(adcAvg);
  Serial.print(" vRawAvg=");
  Serial.print(vRawAvg, 3);
  Serial.print(" vCalAvg=");
  Serial.println(vCalAvg, 3);

  delay(500);
}