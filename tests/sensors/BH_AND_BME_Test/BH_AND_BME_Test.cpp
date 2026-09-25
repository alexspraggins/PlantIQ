#include <Wire.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>

// ==========================
// I2C PINS
// ==========================
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

// ==========================
// SENSOR OBJECTS
// ==========================
Adafruit_BME280 bme;
BH1750 lightMeter;

// ==========================
// TIMING
// ==========================
unsigned long lastReadMs = 0;
const unsigned long READ_INTERVAL_MS = 2000;

// ==========================
// SETUP
// ==========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Starting BME280 + BH1750 test...");

  // Start I2C bus
  Wire.begin(SDA_PIN, SCL_PIN);

  // -------- BME280 --------
  // Common I2C addresses: 0x76 or 0x77
  bool bmeOk = bme.begin(0x76);
  if (!bmeOk) {
    bmeOk = bme.begin(0x77);
  }

  if (bmeOk) {
    Serial.println("BME280 found.");
  } else {
    Serial.println("BME280 NOT found. Check wiring/address.");
  }

  // -------- BH1750 --------
  bool bhOk = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  if (bhOk) {
    Serial.println("BH1750 found.");
  } else {
    Serial.println("BH1750 NOT found. Check wiring/address.");
  }

  Serial.println("Setup complete.");
}

// ==========================
// LOOP
// ==========================
void loop() {
  unsigned long now = millis();

  if (now - lastReadMs >= READ_INTERVAL_MS) {
    lastReadMs = now;

    // ----- Read BME280 -----
    float temperatureC = bme.readTemperature();
    float humidityRH   = bme.readHumidity();
    float pressurehPa  = bme.readPressure() / 100.0F;

    // ----- Read BH1750 -----
    float lux = lightMeter.readLightLevel();

    // ----- Print -----
    Serial.println("----------------------------------------");

    if (!isnan(temperatureC) && !isnan(humidityRH) && !isnan(pressurehPa)) {
      Serial.print("Temperature: ");
      Serial.print(temperatureC);
      Serial.println(" °C");

      Serial.print("Humidity: ");
      Serial.print(humidityRH);
      Serial.println(" %");

      Serial.print("Pressure: ");
      Serial.print(pressurehPa);
      Serial.println(" hPa");
    } else {
      Serial.println("BME280 read failed.");
    }

    if (lux >= 0) {
      Serial.print("Light: ");
      Serial.print(lux);
      Serial.println(" lux");
    } else {
      Serial.println("BH1750 read failed.");
    }
  }
}