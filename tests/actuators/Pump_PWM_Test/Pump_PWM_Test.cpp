#include <Arduino.h>

static const int PUMP_PWM_PIN = 27;   // GPIO driving MOSFET gate

// PWM settings for pump
static const int PUMP_PWM_FREQ_HZ  = 25000; // 25 kHz
static const int PUMP_PWM_RES_BITS = 10;   // 0–1023

static const int MAX_DUTY = (1 << PUMP_PWM_RES_BITS) - 1; // 1023
static const float MIN_DUTY_FRAC = 0.30f; // avoid stall
static const float MAX_DUTY_FRAC = 1.00f;

void setPumpSpeed(float speed01) {
  // speed01: 0.0 (off) → 1.0 (full speed)
  speed01 = constrain(speed01, 0.0f, 1.0f);

  // OFF is special (fully off)
  if (speed01 == 0.0f) {
    ledcWrite(PUMP_PWM_PIN, 0);
    return;
  }

  // Enforce minimum duty to avoid stalling
  float dutyFrac =
      MIN_DUTY_FRAC + speed01 * (MAX_DUTY_FRAC - MIN_DUTY_FRAC);

  int duty = (int)(dutyFrac * MAX_DUTY);
  ledcWrite(PUMP_PWM_PIN, duty);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("Initializing pump PWM...");

  bool ok = ledcAttach(PUMP_PWM_PIN,
                       PUMP_PWM_FREQ_HZ,
                       PUMP_PWM_RES_BITS);

  if (!ok) {
    Serial.println("Pump PWM attach FAILED");
    while (true) delay(1000);
  }

  Serial.println("Pump PWM ready");
  setPumpSpeed(0.0f);  // pump OFF at boot
}

void loop() {
  // Demo: run pump at ~60% speed
  setPumpSpeed(0.6f);
  delay(10000);

  // Turn pump off
  setPumpSpeed(0.0f);
  delay(5000);
}
