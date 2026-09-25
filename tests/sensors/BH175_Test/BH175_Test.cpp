#include <Wire.h>
#include <hp_BH1750.h>

hp_BH1750 bh1750;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);        // ESP32 default I2C pins SDA/SCL

  if (!bh1750.begin(BH1750_TO_GROUND)) {
    Serial.println("BH1750 not found!");
    while (1);
  }

  bh1750.setQuality(BH1750_QUALITY_HIGH);
  bh1750.start();            // start first measurement
}

void loop() {
  if (bh1750.hasValue()) {   // non-blocking check
    float lux = bh1750.getLux();
    Serial.print("Lux: ");
    Serial.println(lux);

    bh1750.start();          // trigger next measurement
  }

  // 🚀 Your LED PWM / pump logic runs freely here
} 

/*
#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;

// EMA state
float lux_ema = 0.0f;
bool ema_init = false;
const float ALPHA = 0.10f;   // EMA smoothing factor

// Timing
const unsigned long PHASE_TIME_MS = 10000; // 10 seconds
unsigned long phaseStartMs = 0;
bool emaPhase = false;  // false = raw only, true = raw + EMA

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 init failed");
    while (true) delay(1000);
  }

  phaseStartMs = millis();

  Serial.println("===== START =====");
  Serial.println("First 10s: RAW only");
}

void loop() {
  unsigned long now = millis();

  // Switch phases every 10 seconds
  if (now - phaseStartMs >= PHASE_TIME_MS) {
    phaseStartMs = now;
    emaPhase = !emaPhase;

    if (emaPhase) {
      Serial.println("----- SWITCH: RAW + EMA -----");
      ema_init = false; // reset EMA so comparison is clean
    } else {
      Serial.println("----- SWITCH: RAW ONLY -----");
    }
  }

  // Read sensor
  float lux_raw = lightMeter.readLightLevel();

  if (!emaPhase) {
    // Phase 1: raw only
    Serial.printf("[RAW ] lux = %.1f\n", lux_raw);
  } else {
    // Phase 2: raw + EMA
    if (!ema_init) {
      lux_ema = lux_raw;
      ema_init = true;
    } else {
      lux_ema = (1.0f - ALPHA) * lux_ema + ALPHA * lux_raw;
    }

    Serial.printf("[EMA ] raw = %.1f | ema = %.1f\n",
                  lux_raw, lux_ema);
  }

  delay(200); // ~5 Hz logging
}*/

