#include <Arduino.h>

static const int LED_PWM_PIN   = 25;
static const int LEDC_CHANNEL  = 0;
static const int LEDC_FREQ_HZ  = 200;
static const int LEDC_RES_BITS = 12;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("Booting...");

  // Configure PWM channel
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RES_BITS);

  // Attach pin to channel
  ledcAttachPin(LED_PWM_PIN, LEDC_CHANNEL);

  Serial.println("LEDC configured.");
}

void loop() {

  static int duty = 0;

  Serial.print("alive, duty=");
  Serial.println(duty);

  // Write duty cycle to the channel
  ledcWrite(LEDC_CHANNEL, duty);

  duty += 400;
  if (duty > 4095) duty = 0;

  delay(1000);
}