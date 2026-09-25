#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char* URL =
"https://wnyrngcvgsadhlymrohv.supabase.co/functions/v1/get_assignment_status";

const char* DEVICE_ID = "esp32-01";
const char* DEVICE_KEY = "YOUR_DEVICE_KEY";

void setup() {

  Serial.begin(115200);
  delay(2000);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.println("Connecting...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;

  https.begin(client, URL);

  https.addHeader("x-device-id", DEVICE_ID);
  https.addHeader("x-device-key", DEVICE_KEY);
  https.addHeader("Content-Type", "application/json");
  https.addHeader("User-Agent", "PlantIQ-ESP32/1.0");
  https.addHeader("Connection", "close");

  int httpCode = https.GET();

  Serial.print("HTTP code: ");
  Serial.println(httpCode);

  if (httpCode > 0) {

    String payload = https.getString();

    Serial.println("Response:");
    Serial.println(payload);

  } else {

    Serial.println("Request failed");
    Serial.println(https.errorToString(httpCode));

  }

  https.end();
}

void loop() {}
