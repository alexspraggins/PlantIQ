#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// =========================
// USER CONFIG
// =========================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* DEVICE_ID  = "esp32-01";
const char* DEVICE_KEY = "YOUR_DEVICE_KEY";

const char* POST_URL =
  "https://wnyrngcvgsadhlymrohv.supabase.co/functions/v1/post_sensor_readings";

// Send every 15 seconds for testing
const unsigned long POST_INTERVAL_MS = 15000;

unsigned long lastPostMs = 0;

// =========================
// WIFI
// =========================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// =========================
// POST TEST DATA
// =========================
bool postSensorReadings(float soilVwc, float lux, float tempC, float humidityRh) {
  WiFiClientSecure client;
  client.setInsecure();  // MVP/testing only

  HTTPClient https;
  if (!https.begin(client, POST_URL)) {
    Serial.println("HTTPS begin failed");
    return false;
  }

  https.addHeader("Content-Type", "application/json");
  https.addHeader("x-device-id", DEVICE_ID);
  https.addHeader("x-device-key", DEVICE_KEY);

  String payload = "{";
  payload += "\"soil_moisture_vwc\":" + String(soilVwc, 3) + ",";
  payload += "\"light_lux\":" + String(lux, 1) + ",";
  payload += "\"temperature_c\":" + String(tempC, 2) + ",";
  payload += "\"humidity_rh\":" + String(humidityRh, 2);
  payload += "}";

  Serial.println("POST payload:");
  Serial.println(payload);

  int httpCode = https.POST(payload);
  String response = https.getString();

  Serial.print("HTTP code: ");
  Serial.println(httpCode);
  Serial.print("Response: ");
  Serial.println(response);

  https.end();

  return (httpCode >= 200 && httpCode < 300);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  connectWiFi();

  // Send one packet immediately on boot
  bool ok = postSensorReadings(0.23, 450.0, 23.50, 52.10);
  Serial.println(ok ? "Initial post succeeded" : "Initial post failed");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    connectWiFi();
  }

  unsigned long now = millis();
  if (now - lastPostMs >= POST_INTERVAL_MS) {
    lastPostMs = now;

    // Fake test values; replace with real sensor reads later
    static float soil = 0.20;
    static float lux  = 400.0;
    static float temp = 23.0;
    static float hum  = 50.0;

    // Small changing values so you can see new rows clearly
    soil += 0.01;
    if (soil > 0.35) soil = 0.20;

    lux += 25.0;
    if (lux > 700.0) lux = 400.0;

    temp += 0.2;
    if (temp > 26.0) temp = 23.0;

    hum += 0.5;
    if (hum > 60.0) hum = 50.0;

    bool ok = postSensorReadings(soil, lux, temp, hum);
    Serial.println(ok ? "Post succeeded" : "Post failed");
  }

  delay(100);
}
