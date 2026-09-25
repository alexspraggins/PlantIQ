#include <Arduino.h>


static const int LED_PWM_PIN = 26;


// PWM settings
static const int LEDC_FREQ_HZ   = 200;  // your max
static const int LEDC_RES_BITS  = 12;   // smooth dimming
static const float MAX_BRIGHTNESS = 0.80f;


static const int MAX_DUTY = (1 << LEDC_RES_BITS) - 1; // 4095


void setLedBrightness(float brightness01) {
 brightness01 = constrain(brightness01, 0.0f, 1.0f);
 brightness01 *= MAX_BRIGHTNESS;


 int duty = (int)(brightness01 * MAX_DUTY);
 ledcWrite(LED_PWM_PIN, duty);   // NOTE: new API writes by PIN
}


void setup() {
 Serial.begin(115200);


 // NEW API: attach PWM to a pin with freq + resolution
 bool ok = ledcAttach(LED_PWM_PIN, LEDC_FREQ_HZ, LEDC_RES_BITS);
 if (!ok) {
   Serial.println("ledcAttach failed (bad pin or unsupported config)");
   while (true) delay(1000);
 }
 else {
   Serial.println("Works");
 }


 setLedBrightness(0.0f);
}


void loop() {
 setLedBrightness(0.80f);
 delay(5000);
}


