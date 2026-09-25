/*#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bme.begin(0x77)) {  // Many boards use 0x76
    Serial.println("Could not find BME280!");
    while (1);
  }
}

void loop() {
  Serial.print("Temp: ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.print("Pressure: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.println();
  delay(2000);
}*/

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Serial.println("Scanning...");
}

void loop() {
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(addr, HEX);
    }
  }
  delay(5000);
}

