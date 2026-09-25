#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>

Preferences prefs;

// Change these for first-time setup
const char* DEFAULT_WIFI_SSID = "YOUR_WIFI_SSID";
const char* DEFAULT_WIFI_PASS = "YOUR_WIFI_PASSWORD";

// Test endpoint (safe public endpoint)
const char* TEST_URL = "http://httpbin.org/get";


// ===============================
// FACTORY RESET
// ===============================

void factoryReset() {

  Serial.println("\nFACTORY RESET INITIATED");

  prefs.begin("wifi", false);
  prefs.clear();
  prefs.end();

  Serial.println("Stored WiFi credentials erased");

  WiFi.disconnect(true, true);

  delay(1000);

  Serial.println("Restarting device...");
  ESP.restart();
}


// ===============================
// SAVE / LOAD WIFI
// ===============================

void saveWiFiCredentials(const char* ssid, const char* pass) {

  prefs.begin("wifi", false);

  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);

  prefs.end();
}


void loadWiFiCredentials(String &ssid, String &pass) {

  prefs.begin("wifi", true);

  ssid = prefs.getString("ssid", "");
  pass = prefs.getString("pass", "");

  prefs.end();
}


// ===============================
// WIFI CONNECT
// ===============================

bool connectWiFi(String ssid, String pass) {

  Serial.println("Connecting to WiFi...");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());

  int retries = 0;

  while (WiFi.status() != WL_CONNECTED && retries < 20) {

    delay(500);
    Serial.print(".");
    retries++;

  }

  if (WiFi.status() == WL_CONNECTED) {

    Serial.println("\nWiFi connected!");

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    return true;
  }

  Serial.println("\nWiFi connection FAILED");

  return false;
}


// ===============================
// INTERNET TEST
// ===============================

void testInternet() {

  HTTPClient http;

  Serial.println("Testing internet connection...");

  http.begin(TEST_URL);

  int httpCode = http.GET();

  if (httpCode > 0) {

    Serial.print("HTTP response code: ");
    Serial.println(httpCode);

    String payload = http.getString();

    Serial.println("Response received.");
  }

  else {

    Serial.print("HTTP request failed: ");
    Serial.println(httpCode);
  }

  http.end();
}


// ===============================
// SETUP
// ===============================

void setup() {

  Serial.begin(115200);

  delay(2000);

  Serial.println("ESP32 WiFi Test");
  Serial.println("Press 'r' for factory reset");

  String ssid, pass;

  loadWiFiCredentials(ssid, pass);


  // If nothing stored yet, save default credentials
  if (ssid == "") {

    Serial.println("No WiFi credentials stored. Saving defaults.");

    saveWiFiCredentials(DEFAULT_WIFI_SSID, DEFAULT_WIFI_PASS);

    ssid = DEFAULT_WIFI_SSID;
    pass = DEFAULT_WIFI_PASS;

  }
  else {

    Serial.println("Credentials Saved");

  }


  if (connectWiFi(ssid, pass)) {

    testInternet();

  }

}


// ===============================
// LOOP
// ===============================

void loop() {

  if (Serial.available()) {

    char cmd = Serial.read();

    if (cmd == 'r' || cmd == 'R') {

      factoryReset();

    }

  }

  delay(10000);

}
