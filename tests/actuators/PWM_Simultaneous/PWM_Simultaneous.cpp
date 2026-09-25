#include <Arduino.h>

static const int WHITE_PIN = 25;
static const int RB_PIN    = 26;

static const int PWM_FREQ_HZ  = 200;
static const int PWM_RES_BITS = 12;
static const int PWM_MAX      = (1 << PWM_RES_BITS) - 1;

void writePWM(int pin, float duty01) {
  duty01 = constrain(duty01, 0.0f, 1.0f);
  int duty = (int)(duty01 * PWM_MAX + 0.5f);
  ledcWrite(pin, duty);   // ✅ write by PIN (new API)
}

void setup() {
  Serial.begin(115200);

  // ✅ attach PWM directly to each pin (new API)
  if (!ledcAttach(WHITE_PIN, PWM_FREQ_HZ, PWM_RES_BITS)) {
    Serial.println("ledcAttach failed for WHITE_PIN");
    while (true) delay(1000);
  }
  if (!ledcAttach(RB_PIN, PWM_FREQ_HZ, PWM_RES_BITS)) {
    Serial.println("ledcAttach failed for RB_PIN");
    while (true) delay(1000);
  }

  writePWM(WHITE_PIN, 0.0f);
  writePWM(RB_PIN, 0.0f);

  Serial.println("Step 0 (pin-based): send W0.5 or R0.2");
}

void loop() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() < 2) return;

  char which = toupper(s[0]);
  float v = s.substring(1).toFloat();

  if (which == 'W') writePWM(WHITE_PIN, v);
  else if (which == 'R') writePWM(RB_PIN, v);

  Serial.printf("Set %c=%.3f\n", which, v);
} 
