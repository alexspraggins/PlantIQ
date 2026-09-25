#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// =====================================================
// WIFI / BACKEND
// =====================================================
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char* DEVICE_ID  = "esp32-01";
const char* DEVICE_KEY = "YOUR_DEVICE_KEY";

const char* IDEAL_CARE_URL =
  "https://wnyrngcvgsadhlymrohv.supabase.co/functions/v1/get_ideal_config";

// =====================================================
// GLOBAL IDEAL CARE CONFIG
// =====================================================
struct CareRange {
  float minVal = 0.0f;
  float maxVal = 0.0f;
  float target = 0.0f;
  String unit = "";
  bool valid = false;
};

struct IdealCareConfig {
  String plant = "";
  CareRange soilMoisture;
  CareRange lightLux;
  CareRange temperatureC;
  CareRange humidityRH;
  bool valid = false;
  unsigned long fetchedAtMs = 0;
};

IdealCareConfig gCare;

// =====================================================
// CARE FETCH TIMING
// =====================================================
unsigned long lastCareFetchMs = 0;
const unsigned long CARE_REFRESH_MS = 6UL * 60UL * 60UL * 1000UL; // 6 hours

// =====================================================
// WIFI
// =====================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("[WIFI] Connecting");
  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WIFI] Connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WIFI] Connection failed");
  }
}

// =====================================================
// CARE PARSE HELPERS
// =====================================================
bool parseRange(JsonVariant obj, CareRange &range) {
  if (!obj.is<JsonObject>()) return false;
  if (!obj["min"].is<float>() && !obj["min"].is<int>()) return false;
  if (!obj["max"].is<float>() && !obj["max"].is<int>()) return false;
  if (!obj["target"].is<float>() && !obj["target"].is<int>()) return false;

  range.minVal = obj["min"].as<float>();
  range.maxVal = obj["max"].as<float>();
  range.target = obj["target"].as<float>();
  range.unit   = obj["unit"] | "";
  range.valid  = true;
  return true;
}

void clearCareConfig() {
  gCare = IdealCareConfig();
}

// =====================================================
// FETCH IDEAL CARE PARAMETERS
// =====================================================
bool fetchIdealCareParameters() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[CARE] Wi-Fi not connected");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  if (!http.begin(client, IDEAL_CARE_URL)) {
    Serial.println("[CARE] HTTP begin failed");
    return false;
  }

  http.addHeader("x-device-id", DEVICE_ID);
  http.addHeader("x-device-key", DEVICE_KEY);

  Serial.println("[CARE] Fetching ideal care parameters...");
  int httpCode = http.GET();
  String payload = http.getString();
  http.end();

  Serial.print("[CARE] HTTP status: ");
  Serial.println(httpCode);

  if (httpCode != 200) {
    Serial.println("[CARE] Request failed");
    Serial.println(payload);
    return false;
  }

  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("[CARE] JSON parse failed: ");
    Serial.println(err.c_str());
    return false;
  }

  IdealCareConfig newCfg;
  newCfg.plant = doc["plant"] | "";

  bool okSoil  = parseRange(doc["soil_moisture_vwc"], newCfg.soilMoisture);
  bool okLight = parseRange(doc["light_lux"],         newCfg.lightLux);
  bool okTemp  = parseRange(doc["temperature_c"],     newCfg.temperatureC);
  bool okHum   = parseRange(doc["humidity_rh"],       newCfg.humidityRH);

  if (!(okSoil && okLight && okTemp && okHum)) {
    Serial.println("[CARE] Missing or invalid parameter block");
    return false;
  }

  newCfg.valid = true;
  newCfg.fetchedAtMs = millis();

  gCare = newCfg;

  Serial.println("[CARE] Ideal care parameters updated");
  return true;
}

// =====================================================
// PRINT HELPERS
// =====================================================
void printRange(const char* label, const CareRange& r) {
  Serial.print(label);
  Serial.print(" min=");
  Serial.print(r.minVal, 3);
  Serial.print(" target=");
  Serial.print(r.target, 3);
  Serial.print(" max=");
  Serial.print(r.maxVal, 3);
  Serial.print(" unit=");
  Serial.print(r.unit);
  Serial.print(" valid=");
  Serial.println(r.valid ? "true" : "false");
}

void printIdealCare() {
  Serial.println("--------------------------------------------------");
  Serial.println("[IDEAL CARE]");
  Serial.print("Plant: ");
  Serial.println(gCare.plant);
  Serial.print("Config valid: ");
  Serial.println(gCare.valid ? "true" : "false");
  printRange("Soil Moisture", gCare.soilMoisture);
  printRange("Light",         gCare.lightLux);
  printRange("Temperature",   gCare.temperatureC);
  printRange("Humidity",      gCare.humidityRH);
  Serial.print("Fetched at ms: ");
  Serial.println(gCare.fetchedAtMs);
  Serial.println("--------------------------------------------------");
}

// =====================================================
// REFRESH POLICY
// =====================================================
void maybeRefreshCareParameters() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (!gCare.valid || millis() - lastCareFetchMs >= CARE_REFRESH_MS) {
    if (fetchIdealCareParameters()) {
      lastCareFetchMs = millis();
      printIdealCare();
    }
  }
}

// Call this after successful reassignment / assignment change
void onAssignmentChanged() {
  Serial.println("[ASSIGNMENT] Changed -> refreshing ideal care");
  clearCareConfig();

  if (fetchIdealCareParameters()) {
    lastCareFetchMs = millis();
    printIdealCare();
  } else {
    Serial.println("[CARE] Refresh after assignment change failed");
  }
}

// =====================================================
// SETUP / LOOP
// =====================================================
unsigned long lastStatusPrintMs = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  clearCareConfig();
  connectWiFi();

  if (fetchIdealCareParameters()) {
    lastCareFetchMs = millis();
    printIdealCare();
  } else {
    Serial.println("[CARE] Initial fetch failed");
  }
}

void loop() {
  maybeRefreshCareParameters();

  if (millis() - lastStatusPrintMs >= 10000) {
    lastStatusPrintMs = millis();
    Serial.println("[STATUS] Running");
    Serial.print("[STATUS] Wi-Fi connected: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "true" : "false");
    Serial.print("[STATUS] Care config valid: ");
    Serial.println(gCare.valid ? "true" : "false");
  }

  delay(100);
}
