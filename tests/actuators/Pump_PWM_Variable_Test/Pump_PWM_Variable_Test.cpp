#include <Arduino.h>

static const int PUMP_PWM_PIN = 27;

// PWM configuration
static const int PUMP_PWM_FREQ_HZ = 25000; // 25 kHz
static const int PUMP_PWM_RES_BITS = 8;    // 0–255

static const int MAX_DUTY = (1 << PUMP_PWM_RES_BITS) - 1; // 255
static const float MIN_DUTY_FRAC = 0.30f;  // avoid stall

// Current speed state (0.0–1.0)
static float currentSpeed = 0.0f;

// ---------- Low-level PWM write ----------
void applyPumpDuty(float speed01) {
  if (speed01 <= 0.0f) {
    ledcWrite(PUMP_PWM_PIN, 0);
    return;
  }

  float dutyFrac =
      MIN_DUTY_FRAC + speed01 * (1.0f - MIN_DUTY_FRAC);

  int duty = (int)(dutyFrac * MAX_DUTY);
  ledcWrite(PUMP_PWM_PIN, duty);
}

// ---------- Smooth ramp function ----------
void rampPumpTo(float targetSpeed,
                float step = 0.02f,
                int stepDelayMs = 20) {
  targetSpeed = constrain(targetSpeed, 0.0f, 1.0f);

  while (fabs(currentSpeed - targetSpeed) > step) {
    if (currentSpeed < targetSpeed)
      currentSpeed += step;
    else
      currentSpeed -= step;

    currentSpeed = constrain(currentSpeed, 0.0f, 1.0f);
    applyPumpDuty(currentSpeed);
    delay(stepDelayMs);
  }

  // Final snap to target
  currentSpeed = targetSpeed;
  applyPumpDuty(currentSpeed);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("Initializing pump PWM (25 kHz)");

  bool ok = ledcAttach(PUMP_PWM_PIN,
                       PUMP_PWM_FREQ_HZ,
                       PUMP_PWM_RES_BITS);

  if (!ok) {
    Serial.println("ERROR: PWM attach failed");
    while (true) delay(1000);
  }

  rampPumpTo(0.0f); // ensure OFF
}

// ---------- Demo loop ----------
void loop() {
  Serial.println("Ramp UP");
  rampPumpTo(1.0f, 0.02f, 20);  // smooth acceleration
  delay(3000);

  Serial.println("Ramp DOWN");
  rampPumpTo(0.0f, 0.02f, 20);  // smooth deceleration
  delay(5000);
}
