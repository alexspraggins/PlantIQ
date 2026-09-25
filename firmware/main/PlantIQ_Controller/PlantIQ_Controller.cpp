
/*
===========================================================
PlantIQ ESP32 BLE Provisioning + Pairing + Full Control
Merged from:
- Connection_Final_Test.ino
- FULL_CONRTOL_LOGIC_V4.ino

Behavior
--------
- BLE provisioning / key refresh / Wi-Fi reprovision / assignment refresh
- Moisture control
- Light control
- BME280 + BH1750 sensing
- Ideal care fetch after assignment
- Telemetry post using saved device key + current plant assignment
===========================================================
*/

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <ArduinoJson.h>
#include <hp_BH1750.h>
#include <Adafruit_BME280.h>
#include "mbedtls/aes.h"


// Forward declarations used across merged BLE/control sections
static void moistureStartCycleNow(const char* reasonMsg, const char* jsonReason);
static void maybeHandleAssignmentTransition();


// =======================================================
// CONFIG
// =======================================================

static const char* DEVICE_ID = "esp32-01";

#define SERVICE_UUID      "12345678-1234-1234-1234-1234567890ab"
#define DEVICE_INFO_UUID  "abcd1234-1234-1234-1234-abcdefabcdef"
#define PROVISION_UUID    "dcba4321-4321-4321-4321-fedcbafedcba"

static const char* ASSIGNMENT_HOST = "wnyrngcvgsadhlymrohv.supabase.co";
static const uint16_t ASSIGNMENT_PORT = 443;
static const char* ASSIGNMENT_PATH = "/functions/v1/get_assignment_status";

static const unsigned long ASSIGNMENT_CHECK_MS      = 15000;
static const unsigned long FIRST_HTTPS_DELAY_MS     = 10000;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS  = 10000;
static const unsigned long NET_TIMEOUT_MS           = 15000;

static const char* IDEAL_CARE_URL =
  "https://wnyrngcvgsadhlymrohv.supabase.co/functions/v1/get_ideal_config";

static const char* TELEMETRY_URL =
  "https://wnyrngcvgsadhlymrohv.supabase.co/functions/v1/post_sensor_readings";

static const uint32_t TELEMETRY_INTERVAL_MS = 30000;
static uint32_t gLastTelemetryMs = 0;

// Intentionally blank in the public repository. Configure the matching
// provisioning key locally before using encrypted BLE provisioning.
static const uint8_t AES_KEY[16] = {};

// =======================================================
// STATE
// =======================================================

enum BleState {
  BLE_STATE_SETUP,
  BLE_STATE_READY_TO_PAIR,
  BLE_STATE_PAIRED
};

BleState bleState = BLE_STATE_SETUP;

// =======================================================
// GLOBALS
// =======================================================

Preferences prefs;

BLEServer* server = nullptr;
BLEAdvertising* advertising = nullptr;
BLECharacteristic* deviceInfoChar = nullptr;
BLECharacteristic* provisionChar = nullptr;

bool bleClientConnected = false;
bool bleAdvertising = true;

bool wifiConnected = false;
bool assigned = false;
bool provisioningInProgress = false;
bool requestInProgress = false;

String assignedPlantId = "";

String savedDeviceKey = "";
String savedWifiSsid  = "";
String savedWifiPass  = "";

unsigned long lastAssignmentCheckMs = 0;
unsigned long wifiConnectedAtMs = 0;

// State-change logging memory
String lastPublishedDeviceInfo = "";

// =======================================================
// FORWARD DECLARATIONS
// =======================================================

const char* bleStateToString(BleState s);
void refreshPublicState();
void updateDeviceInfoCharacteristic();
void startBLE();
void stopBLE();
bool hasDeviceKey();
bool hasWiFiCredentials();
bool connectWiFiOnce();
String makeDeviceInfoJson();
void logPublishedStateIfChanged();
void logWaitForWifiReprovision();
void logWaitForRegistration();
bool fetchAssignmentStatus(bool& assignedOut, String& plantIdOut);
void forceImmediateAssignmentRefresh();

// =======================================================
// LOG HELPERS
// =======================================================

void logDivider() {
  Serial.println("--------------------------------------------------");
}

void logSection(const char* title) {
  logDivider();
  Serial.print("[");
  Serial.print(title);
  Serial.println("]");
}

void logKV(const char* key, const String& value) {
  Serial.print("  ");
  Serial.print(key);
  Serial.print(": ");
  Serial.println(value);
}

void logKV(const char* key, const char* value) {
  Serial.print("  ");
  Serial.print(key);
  Serial.print(": ");
  Serial.println(value);
}

void logKV(const char* key, bool value) {
  Serial.print("  ");
  Serial.print(key);
  Serial.print(": ");
  Serial.println(value ? "true" : "false");
}

void logKV(const char* key, int value) {
  Serial.print("  ");
  Serial.print(key);
  Serial.print(": ");
  Serial.println(value);
}

void logKV(const char* key, unsigned long value) {
  Serial.print("  ");
  Serial.print(key);
  Serial.print(": ");
  Serial.println(value);
}

void logIP(const char* key, IPAddress ip) {
  Serial.print("  ");
  Serial.print(key);
  Serial.print(": ");
  Serial.println(ip);
}

void logWaitForWifiReprovision() {
  logDivider();
  Serial.println("[WAITING FOR APP]");
  Serial.println("Device requires WiFi reprovision.");
  Serial.println("App should send WiFi-only payload:");
  Serial.println("{\"wifi_ssid\":\"...\",\"wifi_pass\":\"...\"}");
  logDivider();
}

void logWaitForRegistration() {
  logDivider();
  Serial.println("[WAITING FOR APP]");
  Serial.println("Device requires registration.");
  Serial.println("App should register/create a new backend device,");
  Serial.println("then send either:");
  Serial.println("{\"device_key\":\"...\"}");
  Serial.println("or");
  Serial.println("{\"device_key\":\"...\",\"wifi_ssid\":\"...\",\"wifi_pass\":\"...\"}");
  logDivider();
}

// =======================================================
// HELPERS
// =======================================================

const char* bleStateToString(BleState s) {
  switch (s) {
    case BLE_STATE_SETUP:         return "SETUP";
    case BLE_STATE_READY_TO_PAIR: return "READY_TO_PAIR";
    case BLE_STATE_PAIRED:        return "PAIRED";
    default:                      return "UNKNOWN";
  }
}

void sanitizeString(String& s) {
  s.trim();

  while (s.length() > 0) {
    char c = s[s.length() - 1];
    if (c == '\r' || c == '\n' || c == '\0' || c == ' ') {
      s.remove(s.length() - 1);
    } else {
      break;
    }
  }

  while (s.length() > 0) {
    char c = s[0];
    if (c == '\r' || c == '\n' || c == '\0' || c == ' ') {
      s.remove(0, 1);
    } else {
      break;
    }
  }
}

bool hasWiFiCredentials() {
  return savedWifiSsid.length() > 0;
}

bool hasDeviceKey() {
  return savedDeviceKey.length() > 0;
}

bool needsRegistration() {
  return !hasDeviceKey();
}

bool needsWiFiReprovision() {
  return hasDeviceKey() && !hasWiFiCredentials();
}

void setBleState(BleState s) {
  if (bleState == s) return;

  logSection("STATE CHANGE");
  logKV("BLE state", String(bleStateToString(bleState)) + " -> " + bleStateToString(s));
  bleState = s;
}

void stopBLE() {
  if (!advertising || !bleAdvertising) return;
  advertising->stop();
  bleAdvertising = false;

  logSection("BLE");
  logKV("Advertising", "stopped");
}

void startBLE() {
  if (!advertising || bleAdvertising) return;
  advertising->start();
  bleAdvertising = true;

  logSection("BLE");
  logKV("Advertising", "started");
}

void applyBleState() {
  if (bleState == BLE_STATE_PAIRED) {
    stopBLE();
  } else {
    startBLE();
  }
}

String makeDeviceInfoJson() {
  StaticJsonDocument<384> doc;
  doc["device_id"] = DEVICE_ID;
  doc["state"] = bleStateToString(bleState);
  doc["wifi_connected"] = wifiConnected;
  doc["assigned"] = assigned;
  doc["plant_id"] = assignedPlantId;
  doc["busy"] = provisioningInProgress || requestInProgress;
  doc["device_key_present"] = hasDeviceKey();
  doc["wifi_credentials_present"] = hasWiFiCredentials();
  doc["needs_registration"] = needsRegistration();
  doc["needs_wifi_reprovision"] = needsWiFiReprovision();

  String out;
  serializeJson(doc, out);
  return out;
}

void logPublishedStateIfChanged() {
  String payload = makeDeviceInfoJson();
  if (payload == lastPublishedDeviceInfo) return;

  lastPublishedDeviceInfo = payload;

  logSection("PUBLIC STATE");
  Serial.println(payload);
}

void updateDeviceInfoCharacteristic() {
  if (!deviceInfoChar) return;
  String payload = makeDeviceInfoJson();
  deviceInfoChar->setValue(payload.c_str());
}

void refreshPublicState() {
  if (!hasDeviceKey() || !wifiConnected) {
    assigned = false;
    assignedPlantId = "";
    setBleState(BLE_STATE_SETUP);
  } else if (assigned) {
    setBleState(BLE_STATE_PAIRED);
  } else {
    setBleState(BLE_STATE_READY_TO_PAIR);
  }

  applyBleState();
  updateDeviceInfoCharacteristic();
  logPublishedStateIfChanged();
}

// =======================================================
// NVS
// =======================================================

void loadCredentials() {
  prefs.begin("plantiq", true);
  savedDeviceKey = prefs.getString("device_key", "");
  savedWifiSsid  = prefs.getString("wifi_ssid", "");
  savedWifiPass  = prefs.getString("wifi_pass", "");
  prefs.end();

  sanitizeString(savedDeviceKey);
  sanitizeString(savedWifiSsid);
  sanitizeString(savedWifiPass);

  logSection("LOADED CREDENTIALS");
  logKV("Device key", savedDeviceKey);
  logKV("WiFi SSID", savedWifiSsid);
  logKV("WiFi Pass", savedWifiPass);
}

void saveWiFiCredentialsOnly(const String& ssidIn, const String& passIn) {
  String ssid = ssidIn;
  String pass = passIn;

  sanitizeString(ssid);
  sanitizeString(pass);

  prefs.begin("plantiq", false);
  prefs.putString("wifi_ssid", ssid);
  prefs.putString("wifi_pass", pass);
  prefs.end();

  savedWifiSsid = ssid;
  savedWifiPass = pass;

  logSection("CREDENTIALS");
  logKV("WiFi credentials", "saved");
}

void saveDeviceKeyOnly(const String& keyIn) {
  String key = keyIn;
  sanitizeString(key);

  prefs.begin("plantiq", false);
  prefs.putString("device_key", key);
  prefs.end();

  savedDeviceKey = key;

  logSection("REGISTRATION");
  logKV("Device key", "saved");
}

void saveCredentials(const String& keyIn, const String& ssidIn, const String& passIn) {
  String key = keyIn;
  String ssid = ssidIn;
  String pass = passIn;

  sanitizeString(key);
  sanitizeString(ssid);
  sanitizeString(pass);

  prefs.begin("plantiq", false);
  prefs.putString("device_key", key);
  prefs.putString("wifi_ssid", ssid);
  prefs.putString("wifi_pass", pass);
  prefs.end();

  savedDeviceKey = key;
  savedWifiSsid = ssid;
  savedWifiPass = pass;

  logSection("CREDENTIALS");
  logKV("Saved", "device key + WiFi");
}

void clearCredentials() {
  prefs.begin("plantiq", false);
  prefs.clear();
  prefs.end();

  savedDeviceKey = "";
  savedWifiSsid = "";
  savedWifiPass = "";

  wifiConnected = false;
  assigned = false;
  assignedPlantId = "";

  provisioningInProgress = false;
  requestInProgress = false;

  logSection("CREDENTIALS");
  logKV("Cleared", "all");
}

void clearWiFiCredentialsOnly() {
  prefs.begin("plantiq", false);
  prefs.remove("wifi_ssid");
  prefs.remove("wifi_pass");
  prefs.end();

  savedWifiSsid = "";
  savedWifiPass = "";

  wifiConnected = false;
  assigned = false;
  assignedPlantId = "";
  requestInProgress = false;

  logSection("CREDENTIALS");
  logKV("WiFi credentials", "cleared");
  logKV("Device key", "kept");
}

void clearDeviceRegistrationOnly() {
  prefs.begin("plantiq", false);
  prefs.remove("device_key");
  prefs.end();

  savedDeviceKey = "";
  assigned = false;
  assignedPlantId = "";
  requestInProgress = false;

  logSection("REGISTRATION");
  logKV("Device key", "cleared");
  logKV("WiFi credentials", "kept");
}

void resetToUnregisteredState() {
  logSection("REGISTRATION");
  logKV("Action", "reset to unregistered state");

  clearDeviceRegistrationOnly();
  refreshPublicState();

  if (!bleClientConnected) {
    startBLE();
  }

  logWaitForRegistration();
}

void resetToWifiReprovisionState() {
  logSection("WIFI");
  logKV("Action", "reset to WiFi reprovision state");

  clearWiFiCredentialsOnly();
  refreshPublicState();

  if (!bleClientConnected) {
    startBLE();
  }

  logWaitForWifiReprovision();
}

// =======================================================
// AES DECRYPTION
// =======================================================

bool pkcs7Unpad(uint8_t* data, size_t& len) {
  if (len == 0) return false;

  uint8_t pad = data[len - 1];
  if (pad == 0 || pad > 16 || pad > len) return false;

  for (size_t i = 0; i < pad; i++) {
    if (data[len - 1 - i] != pad) return false;
  }

  len -= pad;
  return true;
}

bool decryptProvisionPayload(const uint8_t* input, size_t inputLen, String& plaintextOut) {
  if (inputLen < 32) {
    logSection("PROVISION");
    logKV("Error", "encrypted payload too short");
    return false;
  }

  const size_t ivLen = 16;
  const size_t cipherLen = inputLen - ivLen;

  if (cipherLen == 0 || (cipherLen % 16) != 0) {
    logSection("PROVISION");
    logKV("Error", "ciphertext length invalid");
    return false;
  }

  uint8_t iv[16];
  memcpy(iv, input, 16);

  uint8_t* decrypted = (uint8_t*)malloc(cipherLen);
  if (!decrypted) {
    logSection("PROVISION");
    logKV("Error", "memory allocation failed");
    return false;
  }

  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);

  int rc = mbedtls_aes_setkey_dec(&ctx, AES_KEY, 128);
  if (rc != 0) {
    mbedtls_aes_free(&ctx);
    free(decrypted);
    logSection("PROVISION");
    logKV("Error", String("AES key setup failed: ") + rc);
    return false;
  }

  rc = mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_DECRYPT, cipherLen, iv, input + ivLen, decrypted);
  mbedtls_aes_free(&ctx);

  if (rc != 0) {
    free(decrypted);
    logSection("PROVISION");
    logKV("Error", String("AES decrypt failed: ") + rc);
    return false;
  }

  size_t plainLen = cipherLen;
  if (!pkcs7Unpad(decrypted, plainLen)) {
    free(decrypted);
    logSection("PROVISION");
    logKV("Error", "PKCS7 unpad failed");
    return false;
  }

  plaintextOut = "";
  plaintextOut.reserve(plainLen);
  for (size_t i = 0; i < plainLen; i++) {
    plaintextOut += (char)decrypted[i];
  }

  free(decrypted);
  sanitizeString(plaintextOut);
  return true;
}

// =======================================================
// WIFI
// =======================================================

bool connectWiFiOnce() {
  sanitizeString(savedWifiSsid);
  sanitizeString(savedWifiPass);

  if (savedWifiSsid.length() == 0) {
    wifiConnected = false;
    return false;
  }

  logSection("WIFI");
  logKV("Action", "connecting");
  logKV("SSID", savedWifiSsid);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(300);
  WiFi.begin(savedWifiSsid.c_str(), savedWifiPass.c_str());

  unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED &&
         (millis() - startMs) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  wifiConnected = (WiFi.status() == WL_CONNECTED);

  if (wifiConnected) {
    wifiConnectedAtMs = millis();

    logSection("WIFI");
    logKV("Connected", true);
    logIP("IP", WiFi.localIP());
    logIP("Gateway", WiFi.gatewayIP());
    logIP("DNS", WiFi.dnsIP());
  } else {
    logSection("WIFI");
    logKV("Connected", false);
  }

  refreshPublicState();
  return wifiConnected;
}

// =======================================================
// BACKEND ASSIGNMENT CHECK
// =======================================================

bool fetchAssignmentStatus(bool& assignedOut, String& plantIdOut) {
  assignedOut = false;
  plantIdOut = "";

  if (requestInProgress) {
    logSection("ASSIGNMENT");
    logKV("Skipped", "request already in progress");
    return false;
  }

  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    logSection("ASSIGNMENT");
    logKV("Skipped", "no WiFi");
    return false;
  }

  sanitizeString(savedDeviceKey);
  if (savedDeviceKey.length() == 0) {
    logSection("ASSIGNMENT");
    logKV("Skipped", "no device key");
    return false;
  }

  requestInProgress = true;
  refreshPublicState();

  bool ok = false;

  logSection("ASSIGNMENT");
  logKV("Action", "checking status");

  bool shouldRestartBle = false;
  if (bleState != BLE_STATE_PAIRED && bleAdvertising) {
    stopBLE();
    shouldRestartBle = true;
    delay(250);
  }

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(NET_TIMEOUT_MS);
  client.setHandshakeTimeout(15);

  HTTPClient https;
  https.setTimeout(NET_TIMEOUT_MS);
  https.setConnectTimeout(NET_TIMEOUT_MS);
  https.setReuse(false);

  if (!https.begin(client, ASSIGNMENT_HOST, ASSIGNMENT_PORT, ASSIGNMENT_PATH, true)) {
    logSection("ASSIGNMENT");
    logKV("Error", "HTTPS begin failed");

    if (shouldRestartBle && !assigned && !bleClientConnected) {
      delay(100);
      startBLE();
    }

    requestInProgress = false;
    refreshPublicState();
    return false;
  }

  https.addHeader("x-device-id", DEVICE_ID);
  https.addHeader("x-device-key", savedDeviceKey);

  int httpCode = https.GET();

  logSection("ASSIGNMENT");
  logKV("HTTP code", httpCode);

  String response = "";
  if (httpCode > 0) {
    response = https.getString();
    logKV("Response", response);
  } else {
    logKV("Error", https.errorToString(httpCode));
  }

  if (httpCode == 401) {
    logSection("ASSIGNMENT");
    logKV("Result", "device unauthorized");

    https.end();
    client.stop();

    resetToUnregisteredState();

    requestInProgress = false;
    refreshPublicState();
    return false;
  }

  if (httpCode == 404) {
    logSection("ASSIGNMENT");
    logKV("Result", "device not registered");

    https.end();
    client.stop();

    resetToUnregisteredState();

    requestInProgress = false;
    refreshPublicState();
    return false;
  }

  if (httpCode >= 200 && httpCode < 300) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, response);

    if (err) {
      logSection("ASSIGNMENT");
      logKV("JSON parse error", err.c_str());
    } else {
      assignedOut = doc["assigned"] | false;

      if (doc["plant_id"].isNull()) {
        plantIdOut = "";
      } else {
        plantIdOut = doc["plant_id"].as<String>();
        sanitizeString(plantIdOut);
      }

      ok = true;
    }
  }

  https.end();
  client.stop();

  if (shouldRestartBle && !assignedOut && !bleClientConnected && (hasWiFiCredentials() || needsRegistration())) {
    delay(100);
    startBLE();
  }

  requestInProgress = false;
  refreshPublicState();
  return ok;
}

void forceImmediateAssignmentRefresh() {
  logSection("ASSIGNMENT");
  logKV("Action", "forced refresh requested by app");

  if (provisioningInProgress) {
    logKV("Result", "skipped: provisioning in progress");
    return;
  }

  if (requestInProgress) {
    logKV("Result", "skipped: request already in progress");
    return;
  }

  if (!hasDeviceKey()) {
    logKV("Result", "skipped: no device key");
    refreshPublicState();
    return;
  }

  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    logKV("Result", "skipped: no WiFi");
    refreshPublicState();
    return;
  }

  bool backendAssigned = false;
  String backendPlantId = "";

  bool ok = fetchAssignmentStatus(backendAssigned, backendPlantId);
  if (!ok) {
    logKV("Result", "forced refresh failed");
    refreshPublicState();
    return;
  }

  assigned = backendAssigned;
  assignedPlantId = backendPlantId;

  logKV("Assigned", assigned);
  logKV("Plant ID", assignedPlantId);

  lastAssignmentCheckMs = millis();
  refreshPublicState();
}

void checkAssignmentIfDue() {
  if (!wifiConnected) return;
  if (provisioningInProgress) return;
  if (requestInProgress) return;
  if (!hasDeviceKey()) return;

  if (millis() - wifiConnectedAtMs < FIRST_HTTPS_DELAY_MS) return;
  if (millis() - lastAssignmentCheckMs < ASSIGNMENT_CHECK_MS) return;

  lastAssignmentCheckMs = millis();

  bool backendAssigned = false;
  String backendPlantId = "";

  bool ok = fetchAssignmentStatus(backendAssigned, backendPlantId);
  if (!ok) {
    logSection("ASSIGNMENT");
    logKV("Result", "check failed");
    return;
  }

  assigned = backendAssigned;
  assignedPlantId = backendPlantId;

  logSection("ASSIGNMENT");
  logKV("Assigned", assigned);
  logKV("Plant ID", assignedPlantId);

  refreshPublicState();
}

// =======================================================
// FACTORY RESET
// =======================================================

void factoryReset() {
  logSection("FACTORY RESET");
  logKV("Action", "clearing all credentials and state");

  clearCredentials();
  WiFi.disconnect(true, true);
  delay(200);
  refreshPublicState();
}

// =======================================================
// SERIAL
// =======================================================

void printState() {
  logDivider();
  Serial.println("PLANTIQ DEVICE STATE");
  logDivider();

  logKV("Device ID", DEVICE_ID);
  logKV("BLE state", bleStateToString(bleState));
  logKV("WiFi connected", wifiConnected);
  logKV("BLE client connected", bleClientConnected);
  logKV("BLE advertising", bleAdvertising);
  logKV("Assigned", assigned);
  logKV("Plant ID", assignedPlantId);
  logKV("Provisioning busy", provisioningInProgress);
  logKV("Request busy", requestInProgress);

  logDivider();
  Serial.println("STORED CREDENTIALS");
  logDivider();

  logKV("Device key (raw)", savedDeviceKey);
  logKV("Device key present", hasDeviceKey());
  logKV("WiFi SSID", savedWifiSsid);
  logKV("WiFi password", savedWifiPass);
  logKV("WiFi credentials present", hasWiFiCredentials());

  logDivider();
  Serial.println("APP HINTS");
  logDivider();

  logKV("Needs registration", needsRegistration());
  logKV("Needs WiFi reprovision", needsWiFiReprovision());

  logDivider();
  Serial.println("NETWORK");
  logDivider();

  if (WiFi.status() == WL_CONNECTED) {
    logIP("IP", WiFi.localIP());
    logIP("Gateway", WiFi.gatewayIP());
    logIP("DNS", WiFi.dnsIP());
    logKV("RSSI", WiFi.RSSI());
  } else {
    logKV("IP", "not connected");
  }

  logDivider();
  Serial.println("CURRENT DEVICE_INFO");
  logDivider();
  Serial.println(makeDeviceInfoJson());
  logDivider();
}

void handleSerial() {
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == 'p' || c == 'P') {
    printState();
  } else if (c == 'r' || c == 'R') {
    factoryReset();
  } else if (c == 'c' || c == 'C') {
    bool backendAssigned = false;
    String backendPlantId = "";
    bool ok = fetchAssignmentStatus(backendAssigned, backendPlantId);

    logSection("MANUAL CHECK");
    logKV("Result", ok ? "success" : "failed");

    if (ok) {
      assigned = backendAssigned;
      assignedPlantId = backendPlantId;
      refreshPublicState();
      maybeHandleAssignmentTransition();
    }
  } else if (c == 's' || c == 'S') {
    moistureStartCycleNow("event=START_CYCLE reason=FORCED_IMMEDIATE", "FORCED_IMMEDIATE");
  }
}

// =======================================================
// BLE CALLBACKS
// =======================================================

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleClientConnected = true;
    logSection("BLE");
    logKV("Client", "connected");
    updateDeviceInfoCharacteristic();
    logPublishedStateIfChanged();
  }

  void onDisconnect(BLEServer* pServer) override {
    bleClientConnected = false;
    logSection("BLE");
    logKV("Client", "disconnected");

    delay(100);
    if (bleAdvertising && advertising) {
      advertising->start();
    }

    updateDeviceInfoCharacteristic();
    logPublishedStateIfChanged();
  }
};

class DeviceInfoCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic* characteristic) override {
    String payload = makeDeviceInfoJson();
    characteristic->setValue(payload.c_str());
    // Intentionally no serial logging here.
    // The app may poll frequently; we only log when public state changes.
  }
};

class ProvisionCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    if (provisioningInProgress) {
      logSection("PROVISION");
      logKV("Skipped", "already in progress");
      return;
    }

    provisioningInProgress = true;
    refreshPublicState();

    do {
      size_t rawLen = characteristic->getLength();
      uint8_t* rawData = characteristic->getData();

      logSection("PROVISION");
      logKV("Write bytes", (int)rawLen);

      if (rawData == nullptr || rawLen == 0) {
        logKV("Error", "empty payload");
        break;
      }

      String decryptedJson;
      if (!decryptProvisionPayload(rawData, rawLen, decryptedJson)) {
        logKV("Error", "decryption failed");
        break;
      }

      logKV("Decrypted JSON", decryptedJson);

      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, decryptedJson);
      if (err) {
        logKV("JSON parse error", err.c_str());
        break;
      }

      bool hasAction = doc.containsKey("action");
      bool hasKey    = doc.containsKey("device_key");
      bool hasSsid   = doc.containsKey("wifi_ssid");
      bool hasPass   = doc.containsKey("wifi_pass");

      if (hasSsid != hasPass) {
        logKV("Error", "wifi_ssid and wifi_pass must both be present");
        break;
      }

      // ===================================================
      // ACTION: FORCE ASSIGNMENT REFRESH
      // { "action": "refresh_assignment" }
      // ===================================================
      if (hasAction && !hasKey && !hasSsid && !hasPass) {
        String action = doc["action"].as<String>();
        sanitizeString(action);

        if (action != "refresh_assignment") {
          logKV("Error", "unknown action");
          break;
        }

        logKV("Mode", "action");
        logKV("Action", action);

        forceImmediateAssignmentRefresh();
        break;
      }

      // ===================================================
      // MODE 1: FULL PROVISIONING
      // { device_key, wifi_ssid, wifi_pass }
      // ===================================================
      if (hasKey && hasSsid && hasPass) {
        String key  = doc["device_key"].as<String>();
        String ssid = doc["wifi_ssid"].as<String>();
        String pass = doc["wifi_pass"].as<String>();

        sanitizeString(key);
        sanitizeString(ssid);
        sanitizeString(pass);

        if (key.length() == 0) {
          logKV("Error", "invalid device_key");
          break;
        }

        if (ssid.length() == 0) {
          logKV("Error", "invalid WiFi SSID");
          break;
        }

        logKV("Mode", "full provisioning");
        logKV("Device key", key);
        logKV("WiFi SSID", ssid);
        logKV("WiFi Pass", pass);

        saveCredentials(key, ssid, pass);

        assigned = false;
        assignedPlantId = "";
        lastAssignmentCheckMs = 0;

        if (!connectWiFiOnce()) {
          logKV("Result", "WiFi failed after full provisioning");
          WiFi.disconnect(true, true);
          delay(200);
          resetToWifiReprovisionState();
          break;
        }

        logKV("Result", "full provisioning success");
      }

      // ===================================================
      // MODE 2: KEY-ONLY REFRESH
      // { device_key }
      // ===================================================
      else if (hasKey && !hasSsid && !hasPass) {
        String key = doc["device_key"].as<String>();
        sanitizeString(key);

        if (key.length() == 0) {
          logKV("Error", "invalid device_key");
          break;
        }

        logKV("Mode", "key-only refresh");
        logKV("New device key", key);

        saveDeviceKeyOnly(key);

        assigned = false;
        assignedPlantId = "";
        lastAssignmentCheckMs = 0;

        if (WiFi.status() == WL_CONNECTED) {
          wifiConnected = true;
          wifiConnectedAtMs = millis();
          logKV("WiFi", "keeping existing connection");
        } else if (hasWiFiCredentials()) {
          logKV("WiFi", "reconnecting with saved credentials");
          if (!connectWiFiOnce()) {
            logKV("Result", "WiFi reconnect failed after key refresh");
            WiFi.disconnect(true, true);
            delay(200);
            resetToWifiReprovisionState();
            break;
          }
        } else {
          wifiConnected = false;
          logKV("WiFi", "no saved WiFi credentials; registration saved, waiting for WiFi reprovision");
          refreshPublicState();
          logWaitForWifiReprovision();
        }

        logKV("Result", "key-only refresh success");
      }

      // ===================================================
      // MODE 3: WIFI-ONLY REPROVISION
      // { wifi_ssid, wifi_pass }
      // ===================================================
      else if (!hasKey && hasSsid && hasPass) {
        String ssid = doc["wifi_ssid"].as<String>();
        String pass = doc["wifi_pass"].as<String>();

        sanitizeString(ssid);
        sanitizeString(pass);

        if (!hasDeviceKey()) {
          logKV("Error", "wifi-only reprovision rejected: no saved device key");
          break;
        }

        if (ssid.length() == 0) {
          logKV("Error", "invalid WiFi SSID");
          break;
        }

        logKV("Mode", "wifi-only reprovision");
        logKV("Existing device key", savedDeviceKey);
        logKV("WiFi SSID", ssid);
        logKV("WiFi Pass", pass);

        saveWiFiCredentialsOnly(ssid, pass);

        assigned = false;
        assignedPlantId = "";
        lastAssignmentCheckMs = 0;

        if (!connectWiFiOnce()) {
          logKV("Result", "WiFi failed after WiFi-only reprovision");
          WiFi.disconnect(true, true);
          delay(200);
          resetToWifiReprovisionState();
          break;
        }

        logKV("Result", "WiFi-only reprovision success");
      }

      // ===================================================
      // INVALID
      // ===================================================
      else {
        logKV("Error", "invalid payload shape");
        logKV("Expected", "full provisioning, key-only refresh, WiFi-only reprovision, or refresh_assignment action");
        break;
      }

    } while (false);

    provisioningInProgress = false;
    refreshPublicState();
  }
};

// =======================================================
// BLE SETUP
// =======================================================

void setupBLE() {
  String name = "PlantIQ-";
  name += DEVICE_ID;

  logSection("BLE");
  logKV("Device name", name);

  BLEDevice::init(name.c_str());

  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);

  deviceInfoChar = service->createCharacteristic(
    DEVICE_INFO_UUID,
    BLECharacteristic::PROPERTY_READ
  );
  deviceInfoChar->setCallbacks(new DeviceInfoCallbacks());

  provisionChar = service->createCharacteristic(
    PROVISION_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  provisionChar->setCallbacks(new ProvisionCallbacks());

  service->start();

  advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  bleAdvertising = true;

  logSection("BLE");
  logKV("Advertising", "started");
  logKV("Serial command", "p = print state");
  logKV("Serial command", "r = factory reset");
  logKV("Serial command", "c = force assignment check");
}

// =======================================================
// BOOT
// =======================================================

void bootResolve() {
  if (!hasWiFiCredentials()) {
    logSection("BOOT");
    if (hasDeviceKey()) {
      logKV("Result", "device key exists but WiFi missing; waiting for WiFi-only reprovision");
      refreshPublicState();
      logWaitForWifiReprovision();
    } else {
      logKV("Result", "no saved WiFi credentials; waiting for BLE provisioning");
      refreshPublicState();
      logWaitForRegistration();
    }
    return;
  }

  logSection("BOOT");
  logKV("Result", "saved WiFi credentials found; attempting WiFi");

  if (!connectWiFiOnce()) {
    logKV("Result", "boot WiFi failed; clearing WiFi only and keeping device key");
    WiFi.disconnect(true, true);
    delay(200);
    resetToWifiReprovisionState();
  }
}



struct CareRange;
struct IdealCareConfig;

static bool parseRange(JsonVariant obj, CareRange &range);
static void printRange(const char* label, const CareRange& r);
static void clearCareConfig();

/*
==========================================================
PlantIQ Combined Controller + Config Fetch + Telemetry Post
ESP32 Arduino Core 2.0.17

Includes
--------
- Moisture control
- Light control
- BME280 climate sensing
- Wi-Fi
- Fetch ideal care config from backend
- Apply fetched care config to control targets
- Post real telemetry to backend

Control target mapping
----------------------
- soil_moisture_vwc.min    -> MOIST_LOWER_VWC
- soil_moisture_vwc.max    -> MOIST_UPPER_VWC
- light_lux.target         -> dynamic light target

Telemetry post payload
----------------------
{
  "device_id": "...",
  "plant_id": null,
  "soil_moisture_vwc": ...,
  "light_lux": ...,
  "temperature_c": ...,
  "humidity_rh": ...
}
==========================================================
*/


// =======================================================
//                 SHARED TIME PREFIXED PRINTS
// =======================================================
static void printTimePrefix() {
  uint32_t ms = millis();
  uint32_t totalSeconds = ms / 1000;
  uint32_t hours   = (totalSeconds / 3600) % 24;
  uint32_t minutes = (totalSeconds / 60) % 60;
  uint32_t seconds = totalSeconds % 60;
  uint32_t mmm     = ms % 1000;

  if (hours < 10)   Serial.print('0');
  Serial.print(hours); Serial.print(':');
  if (minutes < 10) Serial.print('0');
  Serial.print(minutes); Serial.print(':');
  if (seconds < 10) Serial.print('0');
  Serial.print(seconds); Serial.print('.');
  if (mmm < 100) Serial.print('0');
  if (mmm < 10)  Serial.print('0');
  Serial.print(mmm);
  Serial.print(" -> ");
}

#define TPRINT(x)    do { printTimePrefix(); Serial.print(x); } while (0)
#define TPRINTLN(x)  do { printTimePrefix(); Serial.println(x); } while (0)

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}



// =======================================================
//                 IDEAL CARE CONFIG
// =======================================================
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

static IdealCareConfig gCare;
static unsigned long lastCareFetchMs = 0;
static const unsigned long CARE_REFRESH_MS = 6UL * 60UL * 60UL * 1000UL; // 6 hr


// =======================================================
//                    CONFIG HELPERS
// =======================================================
static bool parseRange(JsonVariant obj, CareRange &range) {
  if (!obj.is<JsonObject>()) return false;
  if (!obj["min"].is<float>()    && !obj["min"].is<int>()) return false;
  if (!obj["max"].is<float>()    && !obj["max"].is<int>()) return false;
  if (!obj["target"].is<float>() && !obj["target"].is<int>()) return false;

  range.minVal = obj["min"].as<float>();
  range.maxVal = obj["max"].as<float>();
  range.target = obj["target"].as<float>();
  range.unit   = obj["unit"] | "";
  range.valid  = true;
  return true;
}

static void clearCareConfig() {
  gCare = IdealCareConfig();
}


// =======================================================
//                  MOISTURE CONTROLLER
// =======================================================

// ---------------- Pins ----------------
static const int MOISTURE_ADC_PIN = 34;
static const int PUMP_PIN         = 27;

// ---------------- ADC constants ----------------
static const float MOIST_ADC_REF_V = 3.30f;
static const int   MOIST_ADC_MAX   = 4095;

// Voltage calibration
static const float MOIST_V_GAIN = 1.03202105f;
static const float MOIST_V_BIAS = 0.11009852f;

// ADC floor handling
static const int MOIST_ADC_FLOOR_THRESHOLD = 10;

// Sampling / Filter
static const int   MOIST_SAMPLES   = 12;
static const float MOIST_EMA_ALPHA = 0.20f;

// Targets / Trigger
static float MOIST_LOWER_VWC = 9.0f;
static float MOIST_UPPER_VWC = 20.0f;
static const int MOIST_LOW_CONSEC_REQUIRED = 4;

// Pulse / Diffusion
static const uint32_t MOIST_PULSE_ON_MS     = 1500UL;
static const uint32_t MOIST_DIFFUSE_WAIT_MS = 3UL * 60000UL;

// Timing
static const uint32_t MOIST_SENSOR_POLL_MS  = 500;
static const uint32_t MOIST_CTRL_TICK_MS    = 100;
static const uint32_t MOIST_STATUS_PRINT_MS = 500;

// Sensor freshness gate
static const uint32_t MOIST_SENSOR_FRESH_MS = MOIST_SENSOR_POLL_MS * 2;

// Safety limits
static const int      MOIST_MAX_PULSES_PER_CYCLE = 12;
static const uint32_t MOIST_MAX_TOTAL_PUMP_ON_MS = 30000;

// “No rise” detection
static const int   MOIST_RISE_CHECK_AFTER_PULSES = 3;
static const float MOIST_MIN_EXPECTED_RISE_VWC   = 0.8f;

// ---------------- States ----------------
static const uint8_t MOIST_STATE_IDLE         = 0;
static const uint8_t MOIST_STATE_PULSE_ON     = 1;
static const uint8_t MOIST_STATE_DIFFUSE_WAIT = 2;
static const uint8_t MOIST_STATE_CHECK        = 3;

struct MoistureSystem {
  uint8_t state;

  uint32_t stateEnterMs;
  uint32_t lastSensorMs;
  uint32_t lastCtrlMs;
  uint32_t lastPrintMs;

  float vwcRaw;
  float vwcEma;
  float vCal;

  int lowCount;
  int pulsesThisCycle;
  uint32_t totalPumpOnMs;
  float cycleStartVwc;

  bool pumpOn;
};

static MoistureSystem gMoist = {
  MOIST_STATE_IDLE,
  0, 0, 0, 0,
  NAN, NAN, NAN,
  0, 0, 0, NAN,
  false
};

static const char* moistureStateName(uint8_t s) {
  switch (s) {
    case MOIST_STATE_IDLE:         return "IDLE";
    case MOIST_STATE_PULSE_ON:     return "PULSE_ON";
    case MOIST_STATE_DIFFUSE_WAIT: return "DIFFUSE";
    case MOIST_STATE_CHECK:        return "CHECK";
    default: return "?";
  }
}

static void moisturePumpSet(bool on) {
  gMoist.pumpOn = on;
  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
}

static void moistureEnterState(uint8_t s) {
  gMoist.state = s;
  gMoist.stateEnterMs = millis();
}

static void moistureLogLine(const char* type) {
  printTimePrefix();
  Serial.print("MOIST ");
  Serial.print(type);
  Serial.print(" | ");
}

static void logEventJson(const char* subsystem, const char* event, const char* reason = nullptr) {
  printTimePrefix();
  Serial.print("EVT ");
  Serial.print("{\"device_id\":\"");
  Serial.print(DEVICE_ID);
  Serial.print("\",\"ms\":");
  Serial.print(millis());
  Serial.print(",\"subsystem\":\"");
  Serial.print(subsystem);
  Serial.print("\",\"event\":\"");
  Serial.print(event);
  Serial.print("\"");
  if (reason && reason[0]) {
    Serial.print(",\"reason\":\"");
    Serial.print(reason);
    Serial.print("\"");
  }
  Serial.println("}");
}

static void moistureLogEvt(const char* msg) {
  moistureLogLine("EVT");
  Serial.println(msg);
}

static void moisturePrintStatus() {
  moistureLogLine("STAT");

  Serial.print("state=");
  Serial.print(moistureStateName(gMoist.state));

  Serial.print(" pump=");
  Serial.print(gMoist.pumpOn ? "ON" : "OFF");

  Serial.print(" vCal=");
  if (isnan(gMoist.vCal)) Serial.print("nan");
  else Serial.print(gMoist.vCal, 3);

  Serial.print(" vwcRaw=");
  if (isnan(gMoist.vwcRaw)) Serial.print("nan");
  else Serial.print(gMoist.vwcRaw, 1);

  Serial.print(" vwcEma=");
  if (isnan(gMoist.vwcEma)) Serial.print("nan");
  else Serial.print(gMoist.vwcEma, 1);

  Serial.print(" lowCnt=");
  Serial.print(gMoist.lowCount);

  Serial.print(" pulses=");
  Serial.print(gMoist.pulsesThisCycle);

  Serial.print(" onMs=");
  Serial.print(gMoist.totalPumpOnMs);

  Serial.print(" lower=");
  Serial.print(MOIST_LOWER_VWC, 1);

  Serial.print(" upper=");
  Serial.print(MOIST_UPPER_VWC, 1);

  Serial.print(" pulseMs=");
  Serial.print(MOIST_PULSE_ON_MS);

  Serial.print(" diffuseMs=");
  Serial.print(MOIST_DIFFUSE_WAIT_MS);

  Serial.println();
}

static float moistureAdcToVraw(int adc) {
  return (float)adc * (MOIST_ADC_REF_V / (float)MOIST_ADC_MAX);
}

static int moistureReadAdcAvg() {
  uint32_t sum = 0;
  for (int i = 0; i < MOIST_SAMPLES; i++) {
    sum += analogRead(MOISTURE_ADC_PIN);
    delayMicroseconds(1500);
  }
  return (int)(sum / (uint32_t)MOIST_SAMPLES);
}

static float moistureCalibrateVoltageFromAdc(int adcAvg) {
  if (adcAvg <= MOIST_ADC_FLOOR_THRESHOLD) return 0.0f;
  float vRaw = moistureAdcToVraw(adcAvg);
  float v = MOIST_V_GAIN * vRaw + MOIST_V_BIAS;
  return clampf(v, 0.0f, 3.60f);
}

static float moistureVoltageToVWC(float v) {
  float vwc;
  if (v <= 1.10f)      vwc = 10.0f * v - 1.0f;
  else if (v <= 1.30f) vwc = 25.0f * v - 17.5f;
  else if (v <= 1.82f) vwc = 48.08f * v - 47.5f;
  else if (v <= 2.20f) vwc = 26.32f * v - 7.89f;
  else if (v <= 3.00f) vwc = 62.5f  * v - 87.5f;
  else                 vwc = 62.5f  * v - 87.5f;
  return clampf(vwc, 0.0f, 100.0f);
}

static bool moistureSensorFresh(uint32_t now) {
  return (now - gMoist.lastSensorMs) <= MOIST_SENSOR_FRESH_MS;
}

static void moistureResetCycleVars() {
  gMoist.lowCount = 0;
  gMoist.pulsesThisCycle = 0;
  gMoist.totalPumpOnMs = 0;
  gMoist.cycleStartVwc = gMoist.vwcEma;
}

static void moistureStopCycleToIdle(const char* reasonMsg, const char* jsonReason) {
  moisturePumpSet(false);
  moistureLogEvt(reasonMsg);
  logEventJson("moisture", "SAFE_STOP", jsonReason);
  gMoist.pulsesThisCycle = 0;
  gMoist.totalPumpOnMs = 0;
  gMoist.cycleStartVwc = NAN;
  gMoist.lowCount = 0;
  moistureEnterState(MOIST_STATE_IDLE);
}

static void moistureStartCycleNow(const char* reasonMsg, const char* jsonReason) {
  moistureLogEvt(reasonMsg);
  logEventJson("moisture", "START_CYCLE", jsonReason);
  moistureResetCycleVars();
  moisturePumpSet(true);
  moistureEnterState(MOIST_STATE_PULSE_ON);
}

static void updateMoistureSystem(uint32_t now) {
  if (now - gMoist.lastSensorMs >= MOIST_SENSOR_POLL_MS) {
    gMoist.lastSensorMs = now;

    int adcAvg = moistureReadAdcAvg();
    gMoist.vCal = moistureCalibrateVoltageFromAdc(adcAvg);
    gMoist.vwcRaw = moistureVoltageToVWC(gMoist.vCal);

    if (isnan(gMoist.vwcEma)) gMoist.vwcEma = gMoist.vwcRaw;
    else gMoist.vwcEma = MOIST_EMA_ALPHA * gMoist.vwcRaw + (1.0f - MOIST_EMA_ALPHA) * gMoist.vwcEma;

    if (gMoist.state == MOIST_STATE_IDLE) {
      if (!isnan(gMoist.vwcEma) && gMoist.vwcEma < MOIST_LOWER_VWC) gMoist.lowCount++;
      else gMoist.lowCount = 0;
    } else {
      gMoist.lowCount = 0;
    }
  }

  if (now - gMoist.lastPrintMs >= MOIST_STATUS_PRINT_MS) {
    gMoist.lastPrintMs = now;
    moisturePrintStatus();
  }

  if (now - gMoist.lastCtrlMs < MOIST_CTRL_TICK_MS) return;
  gMoist.lastCtrlMs = now;

  switch (gMoist.state) {
    case MOIST_STATE_IDLE: {
      if (gMoist.lowCount >= MOIST_LOW_CONSEC_REQUIRED) {
        moistureStartCycleNow("event=START_CYCLE reason=LOW_CONSEC", "LOW_CONSEC");
      }
      break;
    }

    case MOIST_STATE_PULSE_ON: {
      if (gMoist.totalPumpOnMs >= MOIST_MAX_TOTAL_PUMP_ON_MS) {
        moistureStopCycleToIdle("event=SAFE_STOP reason=MAX_TOTAL_ON_TIME action=PUMP_OFF", "MAX_TOTAL_ON_TIME");
        break;
      }

      if (now - gMoist.stateEnterMs >= MOIST_PULSE_ON_MS) {
        moisturePumpSet(false);
        gMoist.pulsesThisCycle++;
        gMoist.totalPumpOnMs += MOIST_PULSE_ON_MS;

        if (gMoist.pulsesThisCycle >= MOIST_MAX_PULSES_PER_CYCLE) {
          moistureStopCycleToIdle("event=SAFE_STOP reason=MAX_PULSES action=PUMP_OFF", "MAX_PULSES");
          break;
        }

        moistureLogEvt("event=PULSE_DONE next=DIFFUSE_WAIT");
        moistureEnterState(MOIST_STATE_DIFFUSE_WAIT);
      }
      break;
    }

    case MOIST_STATE_DIFFUSE_WAIT: {
      if (now - gMoist.stateEnterMs >= MOIST_DIFFUSE_WAIT_MS) {
        moistureLogEvt("event=DIFFUSE_DONE next=CHECK");
        moistureEnterState(MOIST_STATE_CHECK);
      }
      break;
    }

    case MOIST_STATE_CHECK: {
      if (!moistureSensorFresh(now) || isnan(gMoist.vwcEma) || isnan(gMoist.cycleStartVwc)) {
        moistureStopCycleToIdle("event=SAFE_STOP reason=SENSOR_STALE_OR_NAN action=PUMP_OFF", "SENSOR_STALE_OR_NAN");
        break;
      }

      if (gMoist.vwcEma >= MOIST_UPPER_VWC) {
        moisturePumpSet(false);
        moistureLogEvt("event=STOP_CYCLE reason=UPPER_REACHED");
        logEventJson("moisture", "STOP_CYCLE", "UPPER_REACHED");
        gMoist.pulsesThisCycle = 0;
        gMoist.totalPumpOnMs = 0;
        gMoist.cycleStartVwc = NAN;
        moistureEnterState(MOIST_STATE_IDLE);
        break;
      }

      if (gMoist.pulsesThisCycle >= MOIST_RISE_CHECK_AFTER_PULSES) {
        float rise = gMoist.vwcEma - gMoist.cycleStartVwc;
        if (rise < MOIST_MIN_EXPECTED_RISE_VWC) {
          moistureStopCycleToIdle("event=SAFE_STOP reason=NO_RISE action=PUMP_OFF", "NO_RISE");
          break;
        }
      }

      moistureLogEvt("event=NEXT_PULSE reason=BELOW_UPPER");
      moisturePumpSet(true);
      moistureEnterState(MOIST_STATE_PULSE_ON);
      break;
    }
  }
}


// =======================================================
//                    BME280 SENSING
// =======================================================
static Adafruit_BME280 bme;

static const uint32_t BME_POLL_MS  = 2000;
static const uint32_t BME_STALE_MS = 10000;

struct ClimateSystem {
  bool present;
  bool haveReading;
  uint32_t lastReadMs;
  float temperatureC;
  float humidityRH;
  float pressureHpa;
};

static ClimateSystem gClimate = {
  false, false, 0, NAN, NAN, NAN
};

static bool controlAllowed();

static void updateClimateSystem(uint32_t now) {
  if (!gClimate.present) return;

  // No plant-specific climate sensing/logging should run until a plant is
  // actively assigned and valid care config has been loaded.
  if (!controlAllowed()) {
    gClimate.haveReading = false;
    gClimate.temperatureC = NAN;
    gClimate.humidityRH = NAN;
    gClimate.pressureHpa = NAN;
    return;
  }

  if (now - gClimate.lastReadMs < BME_POLL_MS) return;

  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0f;

  gClimate.lastReadMs = now;

  if (isfinite(t) && isfinite(h) && isfinite(p)) {
    gClimate.temperatureC = t;
    gClimate.humidityRH   = h;
    gClimate.pressureHpa  = p;
    gClimate.haveReading  = true;

    printTimePrefix();
    Serial.print("CLIMATE DBG | tempC=");
    Serial.print(gClimate.temperatureC, 2);
    Serial.print(" humidityRH=");
    Serial.print(gClimate.humidityRH, 1);
    Serial.print(" pressureHpa=");
    Serial.println(gClimate.pressureHpa, 1);
  } else {
    printTimePrefix();
    Serial.println("CLIMATE DBG | WARN invalid BME280 reading");
  }
}

static bool climateFresh(uint32_t now) {
  return gClimate.present && gClimate.haveReading && ((now - gClimate.lastReadMs) <= BME_STALE_MS);
}


// =======================================================
//                    LIGHT CONTROLLER
// =======================================================
static hp_BH1750 bh1750;

// Dynamic target; default used until config fetch succeeds
static float gTargetLux = 350.0f;

// PWM pins/channels
static const int WHITE_PWM_PIN = 25;
static const int RB_PWM_PIN    = 26;
static const int WHITE_PWM_CH  = 0;
static const int RB_PWM_CH     = 1;

static const uint32_t PWM_FREQ_HZ  = 200;
static const uint8_t  PWM_RES_BITS = 12;
static const uint32_t PWM_MAX      = (1UL << PWM_RES_BITS) - 1;

static const float WHITE_WEIGHT = 0.75f;
static const float RB_WEIGHT    = 0.25f;

static const float PWM_MIN_FRAC = 0.10f;
static const float PWM_MAX_FRAC = 0.90f;

// Photoperiod
static const uint32_t ON_MS    = 5UL * 60UL * 1000UL;
static const uint32_t OFF_MS   = 60UL * 1000UL;
static const uint32_t CYCLE_MS = ON_MS + OFF_MS;
static const bool     LIGHTS_ON_AT_BOOT = true;

// Sensor timing
static const uint32_t BH1750_POLL_MS = 120;
static const uint32_t LUX_STALE_MS   = 1000;

// Control timing
static const uint32_t CONTROL_PERIOD_MS = 250;

// Control tuning
static const float RATE_UP_PER_SEC   = 0.03f;
static const float RATE_DOWN_PER_SEC = 0.09f;
static const float PWM_EMA_ALPHA     = 0.85f;

// HOLD tuning
static const float   HOLD_ENTER_LUX       = 5.0f;
static const float   HOLD_EXIT_LUX        = 15.0f;
static const float   HOLD_ABOVE_GUARD_LUX = -2.0f;
static const float   ERR_EMA_ALPHA        = 0.80f;
static const uint8_t HOLD_ENTER_TICKS     = 2;
static const uint8_t HOLD_EXIT_TICKS      = 4;

static const float   HOLD_FAST_EXIT_ERR_LUX = 40.0f;
static const uint8_t HOLD_FAST_EXIT_TICKS   = 2;

// Overshoot clamp
static const float OVERSHOOT_CLAMP_LUX = 50.0f;
static const float OVERSHOOT_EXIT_LUX  = 20.0f;

// Ambient/calibration
static const uint32_t AMBIENT_PERIOD_MS   = 3UL * 60000UL;
static const uint32_t AMBIENT_MIN_GAP_MS  = 1500;
static const bool     AMBIENT_AT_BOOT     = true;

static const uint32_t AMBIENT_OFF_HOLD_MS = 800;
static const float    AMBIENT_SLOW_ALPHA  = 0.95f;

static const float AMB_STEP_LUX   = 25.0f;
static const float AMB_FAST_ALPHA = 0.50f;

static const uint32_t RESUME_SETTLE_MS = 700;

// Step-delay ambient trigger
static const float    STEP_DETECT_LUX       = 1000.0f;
static const uint8_t  STEP_DETECT_TICKS_REQ = 3;
static const uint32_t AMB_STEP_DELAY_MS     = 10000UL;
static const float    LUX_FAST_ALPHA        = 0.65f;
static const float    LUX_SLOW_ALPHA        = 0.97f;
static const float    AMBLIKE_K             = 1.0f;
static const uint32_t STEP_CONFIRM_MS       = 3000;
static const float    STEP_CONFIRM_LUX      = 20.0f;

// Calibration span
static const float    CAL_LOW_FRAC  = PWM_MIN_FRAC;
static const float    CAL_HIGH_FRAC = 0.60f;

static const uint32_t SETTLE_LOW_MS  = 900;
static const uint32_t SETTLE_HIGH_MS = 1000;

static const float SLOPE_EMA_ALPHA = 0.80f;
static const float MIN_LED_LUX_DELTA_FOR_SLOPE = 50.0f;

// Resume gate
static const uint8_t RESUME_DISCARD_UPDATES = 3;

// Non-blocking sample timeout
static const uint32_t SAMPLE_PHASE_TIMEOUT_MS = 1500;


// =======================================================
//                         TYPES
// =======================================================
static const uint8_t MODE_CONTROL        = 0;
static const uint8_t MODE_AMBIENT_SAMPLE = 1;
static const uint8_t MODE_CALIBRATION    = 2;

static const uint8_t SAMPLE_IDLE       = 0;
static const uint8_t SAMPLE_SETTLE     = 1;
static const uint8_t SAMPLE_FLUSH_WAIT = 2;
static const uint8_t SAMPLE_COLLECT_1  = 3;
static const uint8_t SAMPLE_COLLECT_2  = 4;
static const uint8_t SAMPLE_COLLECT_3  = 5;
static const uint8_t SAMPLE_DONE       = 6;
static const uint8_t SAMPLE_FAIL       = 7;

static const uint8_t AMB_IDLE         = 0;
static const uint8_t AMB_START        = 1;
static const uint8_t AMB_WAIT_SAMPLE  = 2;
static const uint8_t AMB_RESTORE_FAIL = 3;
static const uint8_t AMB_TO_CAL       = 4;

static const uint8_t CAL_IDLE      = 0;
static const uint8_t CAL_START     = 1;
static const uint8_t CAL_SET_LOW   = 2;
static const uint8_t CAL_WAIT_LOW  = 3;
static const uint8_t CAL_SET_HIGH  = 4;
static const uint8_t CAL_WAIT_HIGH = 5;
static const uint8_t CAL_PROCESS   = 6;
static const uint8_t CAL_RESTORE   = 7;
static const uint8_t CAL_FINISH    = 8;

struct LuxSensorState {
  uint32_t lastStartMs;
  float lux;
  bool haveLux;
  uint32_t luxUpdateMs;
};

struct OutputState {
  float lastWrittenPwm;
};

struct AmbientState {
  float estLux;
  bool haveAmbient;
  uint32_t lastAmbientMs;
};

struct CalibrationState {
  bool haveSlope;
  float slopeEma;
};

struct ControlState {
  float pwmCmd;
  float pwmEma;
  float errEma;

  bool overshootClamped;
  bool holding;
  uint8_t enterCount;
  uint8_t exitCount;
  uint8_t fastExitCount;
  bool clampEntryLocked;
};

struct ResumeGateState {
  uint32_t resumeAfterMs;
  uint8_t  discardLeft;
  uint32_t lastSeenLuxUpdate;
};

struct StepSchedulerState {
  float ambLikeFast;
  float ambLikeSlow;
  uint8_t stepCount;

  bool     ambScheduled;
  uint32_t ambRunAtMs;

  bool     stepConfirmPending;
  uint32_t stepConfirmAtMs;
  float    stepBaseline;

  uint32_t detectIgnoreUntilMs;
};

struct PhotoperiodState {
  uint32_t cycleStartMs;
  bool prevLightsOn;
};

struct LightRuntime {
  uint8_t mode;
  float pwmSaved;
};

struct SampleJob {
  uint8_t phase;
  uint32_t settleUntilMs;
  uint32_t timeoutAtMs;
  uint32_t refUpdateMs;
  float a;
  float b;
  float c;
  float result;
};

struct AmbientModeState {
  uint8_t phase;
};

struct CalModeState {
  uint8_t phase;
  float luxLow;
  float luxHigh;
};

struct LightSystem {
  LuxSensorState sensor;
  OutputState output;
  AmbientState ambient;
  CalibrationState cal;
  ControlState ctrl;
  ResumeGateState resume;
  StepSchedulerState step;
  PhotoperiodState photo;
  LightRuntime rt;
  SampleJob sample;
  AmbientModeState ambMode;
  CalModeState calMode;
};

static LightSystem gLight = {
  {0, NAN, false, 0},
  {NAN},
  {0.0f, false, 0},
  {false, NAN},
  {0.25f, 0.25f, 0.0f, false, false, 0, 0, 0, false},
  {0, 0, 0},
  {NAN, NAN, 0, false, 0, false, 0, NAN, 0},
  {0, true},
  {MODE_CONTROL, 0.25f},
  {SAMPLE_IDLE, 0, 0, 0, NAN, NAN, NAN, NAN},
  {AMB_IDLE},
  {CAL_IDLE, NAN, NAN}
};

static const char* lightModeName(uint8_t mode) {
  switch (mode) {
    case MODE_CONTROL:        return "CONTROL";
    case MODE_AMBIENT_SAMPLE: return "AMBIENT_SAMPLE";
    case MODE_CALIBRATION:    return "CALIBRATION";
    default: return "?";
  }
}


// =======================================================
//                   GENERAL HELPERS
// =======================================================
static inline float clamp01(float v) {
  return constrain(v, 0.0f, 1.0f);
}

static inline float clampPwmWindow(float v) {
  return constrain(v, PWM_MIN_FRAC, PWM_MAX_FRAC);
}

static inline uint32_t fracToDuty(float frac) {
  frac = clamp01(frac);
  return (uint32_t)lroundf(frac * (float)PWM_MAX);
}

static inline float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

static void resetControlFlags() {
  gLight.ctrl.overshootClamped = false;
  gLight.ctrl.holding = false;
  gLight.ctrl.enterCount = 0;
  gLight.ctrl.exitCount = 0;
  gLight.ctrl.fastExitCount = 0;
}

static void lockClampEntryUntilSafeSample() {
  gLight.ctrl.clampEntryLocked = true;
}

static float currentTargetLux() {
  return gTargetLux;
}


// =======================================================
//               APPLY FETCHED CARE TO CONTROL
// =======================================================
static void applyCareTargetsToControllers() {
  if (!gCare.valid) return;

  if (gCare.soilMoisture.valid) {
    float lo = gCare.soilMoisture.minVal;
    float hi = gCare.soilMoisture.maxVal;

    if (isfinite(lo) && isfinite(hi) && hi > lo) {
      MOIST_LOWER_VWC = lo;
      MOIST_UPPER_VWC = hi;
    }
  }

  if (gCare.lightLux.valid) {
    float tgt = gCare.lightLux.target;
    if (isfinite(tgt) && tgt > 0.0f) {
      gTargetLux = tgt;
    }
  }

  printTimePrefix();
  Serial.print("[CARE APPLY] moisture lower=");
  Serial.print(MOIST_LOWER_VWC, 1);
  Serial.print(" upper=");
  Serial.print(MOIST_UPPER_VWC, 1);
  Serial.print(" lightTarget=");
  Serial.println(gTargetLux, 1);
}


// =======================================================
//              FETCH IDEAL CARE PARAMETERS
// =======================================================
static bool fetchIdealCareParameters() {
  if (!hasDeviceKey()) {
    TPRINTLN("[CARE] Device key missing");
    return false;
  }

  if (!assigned) {
    TPRINTLN("[CARE] Device not assigned");
    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    TPRINTLN("[CARE] Wi-Fi not connected");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  if (!http.begin(client, IDEAL_CARE_URL)) {
    TPRINTLN("[CARE] HTTP begin failed");
    return false;
  }

  http.addHeader("x-device-id", DEVICE_ID);
  http.addHeader("x-device-key", savedDeviceKey.c_str());

  TPRINTLN("[CARE] Fetching ideal care parameters...");
  int httpCode = http.GET();
  String payload = http.getString();
  http.end();

  printTimePrefix();
  Serial.print("[CARE] HTTP status: ");
  Serial.println(httpCode);

  if (httpCode != 200) {
    TPRINTLN("[CARE] Request failed");
    Serial.println(payload);
    return false;
  }

  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    printTimePrefix();
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
    TPRINTLN("[CARE] Missing or invalid parameter block");
    return false;
  }

  newCfg.valid = true;
  newCfg.fetchedAtMs = millis();
  gCare = newCfg;

  TPRINTLN("[CARE] Ideal care parameters updated");
  applyCareTargetsToControllers();
  return true;
}

static void printRange(const char* label, const CareRange& r) {
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

static void printIdealCare() {
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

static void maybeRefreshCareParameters() {
  if (!assigned) return;
  if (WiFi.status() != WL_CONNECTED) return;

  if (!gCare.valid || millis() - lastCareFetchMs >= CARE_REFRESH_MS) {
    if (fetchIdealCareParameters()) {
      lastCareFetchMs = millis();
      printIdealCare();
    }
  }
}

static void onAssignmentChanged() {
  TPRINTLN("[ASSIGNMENT] Changed -> refreshing ideal care");
  clearCareConfig();

  if (fetchIdealCareParameters()) {
    lastCareFetchMs = millis();
    printIdealCare();
  } else {
    TPRINTLN("[CARE] Refresh after assignment change failed");
  }
}


// =======================================================
//                    SENSOR MANAGER
// =======================================================
static void lightSensorBegin() {
  gLight.sensor.lastStartMs = millis();
  bh1750.start();
}

static void lightSensorUpdate() {
  const uint32_t now = millis();

  if (now - gLight.sensor.lastStartMs >= BH1750_POLL_MS) {
    bh1750.start();
    gLight.sensor.lastStartMs = now;
  }

  if (bh1750.hasValue()) {
    gLight.sensor.lux = bh1750.getLux();
    gLight.sensor.haveLux = true;
    gLight.sensor.luxUpdateMs = now;
  }
}

static bool lightSensorFresh(uint32_t now) {
  return gLight.sensor.haveLux && (now - gLight.sensor.luxUpdateMs <= LUX_STALE_MS);
}


// =======================================================
//              NON-BLOCKING SAMPLE JOB MANAGER
// =======================================================
static void startSampleJob(uint32_t settleMs) {
  gLight.sample.phase = SAMPLE_SETTLE;
  gLight.sample.settleUntilMs = millis() + settleMs;
  gLight.sample.timeoutAtMs = 0;
  gLight.sample.refUpdateMs = gLight.sensor.luxUpdateMs;
  gLight.sample.a = NAN;
  gLight.sample.b = NAN;
  gLight.sample.c = NAN;
  gLight.sample.result = NAN;
}

static void failSampleJob() {
  gLight.sample.phase = SAMPLE_FAIL;
  gLight.sample.result = NAN;
}

static void finishSampleJob() {
  gLight.sample.result = median3(gLight.sample.a, gLight.sample.b, gLight.sample.c);
  gLight.sample.phase = SAMPLE_DONE;
}

static bool sampleSawNewLux() {
  return gLight.sensor.haveLux && (gLight.sensor.luxUpdateMs != gLight.sample.refUpdateMs);
}

static void armSampleTimeout() {
  gLight.sample.timeoutAtMs = millis() + SAMPLE_PHASE_TIMEOUT_MS;
}

static bool sampleTimedOut(uint32_t now) {
  return ((int32_t)(now - gLight.sample.timeoutAtMs) >= 0);
}

static void updateSampleJob(uint32_t now) {
  switch (gLight.sample.phase) {
    case SAMPLE_IDLE:
    case SAMPLE_DONE:
    case SAMPLE_FAIL:
      return;

    case SAMPLE_SETTLE:
      if ((int32_t)(now - gLight.sample.settleUntilMs) >= 0) {
        gLight.sample.phase = SAMPLE_FLUSH_WAIT;
        gLight.sample.refUpdateMs = gLight.sensor.luxUpdateMs;
        armSampleTimeout();
      }
      return;

    case SAMPLE_FLUSH_WAIT:
      if (sampleSawNewLux()) {
        gLight.sample.refUpdateMs = gLight.sensor.luxUpdateMs;
        gLight.sample.phase = SAMPLE_COLLECT_1;
        armSampleTimeout();
      } else if (sampleTimedOut(now)) {
        failSampleJob();
      }
      return;

    case SAMPLE_COLLECT_1:
      if (sampleSawNewLux()) {
        gLight.sample.a = gLight.sensor.lux;
        gLight.sample.refUpdateMs = gLight.sensor.luxUpdateMs;
        gLight.sample.phase = SAMPLE_COLLECT_2;
        armSampleTimeout();
      } else if (sampleTimedOut(now)) {
        failSampleJob();
      }
      return;

    case SAMPLE_COLLECT_2:
      if (sampleSawNewLux()) {
        gLight.sample.b = gLight.sensor.lux;
        gLight.sample.refUpdateMs = gLight.sensor.luxUpdateMs;
        gLight.sample.phase = SAMPLE_COLLECT_3;
        armSampleTimeout();
      } else if (sampleTimedOut(now)) {
        failSampleJob();
      }
      return;

    case SAMPLE_COLLECT_3:
      if (sampleSawNewLux()) {
        gLight.sample.c = gLight.sensor.lux;
        gLight.sample.refUpdateMs = gLight.sensor.luxUpdateMs;
        finishSampleJob();
      } else if (sampleTimedOut(now)) {
        failSampleJob();
      }
      return;
  }
}


// =======================================================
//                    OUTPUT MANAGER
// =======================================================
static void writeLedPWM(float overallFrac) {
  overallFrac = clamp01(overallFrac);

  const float w = clamp01(overallFrac * WHITE_WEIGHT);
  const float r = clamp01(overallFrac * RB_WEIGHT);

  ledcWrite(WHITE_PWM_CH, fracToDuty(w));
  ledcWrite(RB_PWM_CH,    fracToDuty(r));
}

static void applyPWMAnyImmediate(float frac) {
  frac = clamp01(frac);
  writeLedPWM(frac);
  gLight.output.lastWrittenPwm = frac;
}

static void applyPWMValidatedIfChanged(float frac) {
  frac = clampPwmWindow(frac);

  if (!isfinite(gLight.output.lastWrittenPwm) ||
      fabsf(frac - gLight.output.lastWrittenPwm) > 0.0005f) {
    writeLedPWM(frac);
    gLight.output.lastWrittenPwm = frac;
  }
}


// =======================================================
//                   AMBIENT MANAGER
// =======================================================
static void updateAmbientEstimate(float ambRaw) {
  ambRaw = max(0.0f, ambRaw);

  if (!gLight.ambient.haveAmbient) {
    gLight.ambient.estLux = ambRaw;
    gLight.ambient.haveAmbient = true;
    return;
  }

  const float diff = fabsf(ambRaw - gLight.ambient.estLux);
  if (diff > AMB_STEP_LUX) {
    gLight.ambient.estLux = AMB_FAST_ALPHA * gLight.ambient.estLux
                          + (1.0f - AMB_FAST_ALPHA) * ambRaw;
  } else {
    gLight.ambient.estLux = AMBIENT_SLOW_ALPHA * gLight.ambient.estLux
                          + (1.0f - AMBIENT_SLOW_ALPHA) * ambRaw;
  }
}

static bool ambientDue(uint32_t now) {
  if (now - gLight.ambient.lastAmbientMs < AMBIENT_MIN_GAP_MS) return false;
  if (!gLight.ambient.haveAmbient && AMBIENT_AT_BOOT) return true;
  return (now - gLight.ambient.lastAmbientMs >= AMBIENT_PERIOD_MS);
}


// =======================================================
//                  RESUME GATE MANAGER
// =======================================================
static void armResumeGate(uint8_t discardUpdates = RESUME_DISCARD_UPDATES) {
  gLight.resume.resumeAfterMs = millis() + RESUME_SETTLE_MS;
  gLight.resume.discardLeft = discardUpdates;
  gLight.resume.lastSeenLuxUpdate = gLight.sensor.luxUpdateMs;
}

static bool resumeGateReady(uint32_t now) {
  if (now < gLight.resume.resumeAfterMs) return false;

  if (gLight.resume.discardLeft > 0) {
    if (gLight.sensor.luxUpdateMs != gLight.resume.lastSeenLuxUpdate) {
      gLight.resume.lastSeenLuxUpdate = gLight.sensor.luxUpdateMs;
      gLight.resume.discardLeft--;
    }
    return false;
  }

  return true;
}


// =======================================================
//                 STEP SCHEDULER MANAGER
// =======================================================
static void clearScheduledAmbient() {
  gLight.step.ambScheduled = false;
  gLight.step.ambRunAtMs = 0;
}

static void armStepDetectIgnore(uint32_t ms) {
  gLight.step.detectIgnoreUntilMs = millis() + ms;
  gLight.step.ambLikeFast = NAN;
  gLight.step.ambLikeSlow = NAN;
  gLight.step.stepCount = 0;
  gLight.step.stepConfirmPending = false;
  gLight.step.stepConfirmAtMs = 0;
  gLight.step.stepBaseline = NAN;
  clearScheduledAmbient();
}

static bool scheduledAmbientDue(uint32_t now) {
  return gLight.step.ambScheduled && ((int32_t)(now - gLight.step.ambRunAtMs) >= 0);
}

static bool okToDetectAmbientStep() {
  return gLight.cal.haveSlope && isfinite(gLight.cal.slopeEma) && gLight.cal.slopeEma > 1e-3f;
}

static void updateStepDelayDetector(uint32_t now) {
  if (!gLight.sensor.haveLux) return;
  if ((int32_t)(now - gLight.step.detectIgnoreUntilMs) < 0) return;
  if (!okToDetectAmbientStep()) return;
  if (gLight.step.ambScheduled) return;

  const float ledEst  = AMBLIKE_K * gLight.cal.slopeEma * gLight.ctrl.pwmEma;
  const float ambLike = gLight.sensor.lux - ledEst;

  if (!isfinite(gLight.step.ambLikeFast)) {
    gLight.step.ambLikeFast = ambLike;
    gLight.step.ambLikeSlow = ambLike;
    gLight.step.stepCount = 0;
    gLight.step.stepConfirmPending = false;
    return;
  }

  gLight.step.ambLikeFast = LUX_FAST_ALPHA * gLight.step.ambLikeFast
                          + (1.0f - LUX_FAST_ALPHA) * ambLike;
  gLight.step.ambLikeSlow = LUX_SLOW_ALPHA * gLight.step.ambLikeSlow
                          + (1.0f - LUX_SLOW_ALPHA) * ambLike;

  const float stepness = fabsf(gLight.step.ambLikeFast - gLight.step.ambLikeSlow);

  if (gLight.step.stepConfirmPending) {
    if ((int32_t)(now - gLight.step.stepConfirmAtMs) >= 0) {
      const float stillDiff = fabsf(gLight.step.ambLikeSlow - gLight.step.stepBaseline);
      if (stillDiff >= STEP_CONFIRM_LUX) {
        gLight.step.ambScheduled = true;
        gLight.step.ambRunAtMs = now + AMB_STEP_DELAY_MS;
        TPRINTLN("LIGHT AMB | STEP CONFIRMED -> schedule AMB+CAL bundle");
        logEventJson("light", "STEP_CONFIRMED");
      } else {
        TPRINTLN("LIGHT AMB | STEP REJECTED (transient)");
      }
      gLight.step.stepConfirmPending = false;
      gLight.step.stepCount = 0;
    }
    return;
  }

  if (stepness >= STEP_DETECT_LUX) {
    if (++gLight.step.stepCount >= STEP_DETECT_TICKS_REQ) {
      gLight.step.stepConfirmPending = true;
      gLight.step.stepConfirmAtMs = now + STEP_CONFIRM_MS;
      gLight.step.stepBaseline = gLight.step.ambLikeSlow;
      gLight.step.stepCount = 0;
      TPRINTLN("LIGHT AMB | STEP DETECTED -> confirming...");
    }
  } else {
    gLight.step.stepCount = 0;
  }
}


// =======================================================
//                        LOGGING
// =======================================================
static void printCtrlLine(float ledLux, float targetLed, float ledErr,
                          float targetPwmClamped, float targetPwmRaw,
                          const char* evt) {
  printTimePrefix();
  Serial.print("LIGHT CTRL | mode=");
  Serial.print(lightModeName(gLight.rt.mode));
  Serial.print(" target=");   Serial.print(currentTargetLux(), 1);
  Serial.print(" lux=");      Serial.print(gLight.sensor.lux, 1);
  Serial.print(" amb=");      Serial.print(gLight.ambient.estLux, 1);
  Serial.print(" led=");      Serial.print(ledLux, 1);
  Serial.print(" tLed=");     Serial.print(targetLed, 1);
  Serial.print(" err=");      Serial.print(ledErr, 1);
  Serial.print(" errEma=");   Serial.print(gLight.ctrl.errEma, 1);
  Serial.print(" pwm=");      Serial.print(gLight.ctrl.pwmEma, 3);
  Serial.print(" tPwm=");     Serial.print(targetPwmClamped, 3);
  Serial.print(" tPwmRaw=");  Serial.print(targetPwmRaw, 3);
  Serial.print(" slope=");    Serial.print(gLight.cal.slopeEma, 1);
  if (evt && evt[0]) {
    Serial.print(" | evt=");
    Serial.print(evt);
  }
  Serial.println();
}

static void printAmbRead(float raw, float est) {
  printTimePrefix();
  Serial.print("LIGHT AMB | READ raw=");
  Serial.print(raw, 1);
  Serial.print(" est=");
  Serial.println(est, 1);
}

static void printCalLow(float luxLow, float ledLow) {
  printTimePrefix();
  Serial.print("LIGHT CAL | LOW lux=");
  Serial.print(luxLow, 1);
  Serial.print(" led=");
  Serial.println(ledLow, 1);
}

static void printCalHigh(float luxHigh, float ledHigh) {
  printTimePrefix();
  Serial.print("LIGHT CAL | HIGH lux=");
  Serial.print(luxHigh, 1);
  Serial.print(" led=");
  Serial.println(ledHigh, 1);
}

static void printCalDone(float slope, float amb) {
  printTimePrefix();
  Serial.print("LIGHT CAL | DONE slopeEma=");
  Serial.print(slope, 1);
  Serial.print(" ambient=");
  Serial.println(amb, 1);
}

static void printCalWarnLedDelta(float ledDelta) {
  printTimePrefix();
  Serial.print("LIGHT CAL | WARN ledDelta invalid/small: ");
  Serial.println(ledDelta, 1);
}


// =======================================================
//                  PHOTOPERIOD MANAGER
// =======================================================
static bool lightsShouldBeOn(uint32_t now) {
  uint32_t t = (uint32_t)(now - gLight.photo.cycleStartMs);
  t %= CYCLE_MS;
  return LIGHTS_ON_AT_BOOT ? (t < ON_MS) : (t >= OFF_MS);
}


// =======================================================
//               TOP-LEVEL MODE TRANSITIONS
// =======================================================
static void enterMode(uint8_t mode) {
  gLight.rt.mode = mode;
  gLight.rt.pwmSaved = gLight.ctrl.pwmEma;
  resetControlFlags();

  if (mode == MODE_AMBIENT_SAMPLE) {
    gLight.ambMode.phase = AMB_START;
  } else if (mode == MODE_CALIBRATION) {
    gLight.calMode.phase = CAL_START;
  }
}


// =======================================================
//                  MODE: AMBIENT SAMPLE
// =======================================================
static void updateAmbientMode(uint32_t now) {
  updateSampleJob(now);

  switch (gLight.ambMode.phase) {
    case AMB_IDLE:
      return;

    case AMB_START:
      TPRINTLN("LIGHT AMB | START (LEDs OFF)");
      applyPWMAnyImmediate(0.0f);
      startSampleJob(AMBIENT_OFF_HOLD_MS);
      gLight.ambMode.phase = AMB_WAIT_SAMPLE;
      return;

    case AMB_WAIT_SAMPLE:
      if (gLight.sample.phase == SAMPLE_DONE) {
        const float amb = gLight.sample.result;
        gLight.ambient.lastAmbientMs = now;

        updateAmbientEstimate(amb);
        printAmbRead(amb, gLight.ambient.estLux);

        TPRINTLN("LIGHT BUNDLE | AMB OK -> CAL next");
        gLight.rt.mode = MODE_CALIBRATION;
        gLight.calMode.phase = CAL_START;
        gLight.ambMode.phase = AMB_IDLE;
      } else if (gLight.sample.phase == SAMPLE_FAIL) {
        gLight.ambient.lastAmbientMs = now;
        gLight.ambMode.phase = AMB_RESTORE_FAIL;
      }
      return;

    case AMB_RESTORE_FAIL:
      TPRINTLN("LIGHT AMB | WARN sample failed -> restore");

      gLight.ctrl.pwmCmd = clampPwmWindow(gLight.rt.pwmSaved);
      gLight.ctrl.pwmEma = gLight.ctrl.pwmCmd;
      applyPWMValidatedIfChanged(gLight.ctrl.pwmEma);
      gLight.ctrl.errEma = 0.0f;

      lockClampEntryUntilSafeSample();
      armResumeGate();
      armStepDetectIgnore(RESUME_SETTLE_MS + 1500);

      gLight.rt.mode = MODE_CONTROL;
      gLight.ambMode.phase = AMB_IDLE;
      return;

    case AMB_TO_CAL:
      return;
  }
}


// =======================================================
//                  MODE: CALIBRATION
// =======================================================
static void updateCalibrationMode(uint32_t now) {
  updateSampleJob(now);

  switch (gLight.calMode.phase) {
    case CAL_IDLE:
      return;

    case CAL_START:
      if (!gLight.ambient.haveAmbient) {
        gLight.rt.mode = MODE_CONTROL;
        gLight.calMode.phase = CAL_IDLE;
        return;
      }
      TPRINTLN("LIGHT CAL | START");
      gLight.calMode.phase = CAL_SET_LOW;
      return;

    case CAL_SET_LOW:
      applyPWMValidatedIfChanged(CAL_LOW_FRAC);
      startSampleJob(SETTLE_LOW_MS);
      gLight.calMode.phase = CAL_WAIT_LOW;
      return;

    case CAL_WAIT_LOW:
      if (gLight.sample.phase == SAMPLE_DONE) {
        gLight.calMode.luxLow = gLight.sample.result;
        gLight.calMode.phase = CAL_SET_HIGH;
      } else if (gLight.sample.phase == SAMPLE_FAIL) {
        TPRINTLN("LIGHT CAL | FAIL sample(s)");
        gLight.calMode.phase = CAL_RESTORE;
      }
      return;

    case CAL_SET_HIGH:
      applyPWMValidatedIfChanged(CAL_HIGH_FRAC);
      startSampleJob(SETTLE_HIGH_MS);
      gLight.calMode.phase = CAL_WAIT_HIGH;
      return;

    case CAL_WAIT_HIGH:
      if (gLight.sample.phase == SAMPLE_DONE) {
        gLight.calMode.luxHigh = gLight.sample.result;
        gLight.calMode.phase = CAL_PROCESS;
      } else if (gLight.sample.phase == SAMPLE_FAIL) {
        TPRINTLN("LIGHT CAL | FAIL sample(s)");
        gLight.calMode.phase = CAL_RESTORE;
      }
      return;

    case CAL_PROCESS: {
      const float ledLow  = max(0.0f, gLight.calMode.luxLow  - gLight.ambient.estLux);
      const float ledHigh = max(0.0f, gLight.calMode.luxHigh - gLight.ambient.estLux);

      printCalLow(gLight.calMode.luxLow, ledLow);
      printCalHigh(gLight.calMode.luxHigh, ledHigh);

      const float ledDelta  = ledHigh - ledLow;
      const float fracDelta = (CAL_HIGH_FRAC - CAL_LOW_FRAC);

      if (isfinite(ledDelta) && ledDelta > MIN_LED_LUX_DELTA_FOR_SLOPE && fracDelta > 1e-6f) {
        const float slopeInst = ledDelta / fracDelta;

        if (!gLight.cal.haveSlope || !isfinite(gLight.cal.slopeEma)) {
          gLight.cal.slopeEma = slopeInst;
          gLight.cal.haveSlope = true;
        } else {
          gLight.cal.slopeEma = SLOPE_EMA_ALPHA * gLight.cal.slopeEma
                              + (1.0f - SLOPE_EMA_ALPHA) * slopeInst;
        }

        printCalDone(gLight.cal.slopeEma, gLight.ambient.estLux);
      } else {
        printCalWarnLedDelta(ledDelta);
      }

      gLight.calMode.phase = CAL_RESTORE;
      return;
    }

    case CAL_RESTORE:
      applyPWMValidatedIfChanged(clampPwmWindow(gLight.rt.pwmSaved));
      TPRINTLN("LIGHT CAL | RESTORE PWM");

      lockClampEntryUntilSafeSample();

      gLight.ctrl.pwmCmd = clampPwmWindow(gLight.rt.pwmSaved);
      gLight.ctrl.pwmEma = gLight.ctrl.pwmCmd;
      gLight.ctrl.errEma = 0.0f;

      armResumeGate();
      armStepDetectIgnore(RESUME_SETTLE_MS + 1500);

      gLight.calMode.phase = CAL_FINISH;
      return;

    case CAL_FINISH:
      gLight.rt.mode = MODE_CONTROL;
      gLight.calMode.phase = CAL_IDLE;
      return;
  }
}


// =======================================================
//                 MODE: CONTROL TICK
// =======================================================
static void runControlTick(uint32_t now) {
  if (!lightSensorFresh(now)) return;

  if (!gLight.ambient.haveAmbient || !gLight.cal.haveSlope ||
      !isfinite(gLight.cal.slopeEma) || gLight.cal.slopeEma < 1e-3f) {
    static uint32_t lastPrint = 0;
    if (now - lastPrint > 1000) {
      lastPrint = now;
      TPRINTLN("LIGHT CTRL | waiting for ambient+slope");
    }
    return;
  }

  const float targetLux = currentTargetLux();

  const float ledLux    = max(0.0f, gLight.sensor.lux - gLight.ambient.estLux);
  const float targetLed = max(0.0f, targetLux - gLight.ambient.estLux);
  const float ledErr    = targetLed - ledLux;

  float targetPwmRaw = (targetLed <= 0.0f) ? 0.0f : (targetLed / gLight.cal.slopeEma);
  float targetPwmClamped = constrain(targetPwmRaw, PWM_MIN_FRAC, PWM_MAX_FRAC);

  if (targetLed <= 0.0f) {
    applyPWMAnyImmediate(0.0f);
    gLight.ctrl.pwmCmd = 0.0f;
    gLight.ctrl.pwmEma = 0.0f;
    gLight.ctrl.errEma = 0.0f;
    resetControlFlags();
    printCtrlLine(0.0f, targetLed, 0.0f, 0.0f, targetPwmRaw, "AMBIENT_OK_OFF");
    return;
  }

  if (targetPwmRaw < PWM_MIN_FRAC) {
    targetPwmClamped = PWM_MIN_FRAC;
  }

  if (gLight.ctrl.clampEntryLocked) {
    if (gLight.sensor.lux <= targetLux + OVERSHOOT_EXIT_LUX) {
      gLight.ctrl.clampEntryLocked = false;
    }
  }

  gLight.ctrl.errEma = ERR_EMA_ALPHA * gLight.ctrl.errEma + (1.0f - ERR_EMA_ALPHA) * ledErr;
  const float absErrEma = fabsf(gLight.ctrl.errEma);

  const char* evt = "";

  if (!gLight.ctrl.overshootClamped) {
    if (!gLight.ctrl.clampEntryLocked && gLight.sensor.lux >= targetLux + OVERSHOOT_CLAMP_LUX) {
      gLight.ctrl.overshootClamped = true;
      gLight.ctrl.holding = false;
      gLight.ctrl.enterCount = 0;
      gLight.ctrl.exitCount = 0;
      gLight.ctrl.fastExitCount = 0;

      gLight.ctrl.pwmCmd = PWM_MIN_FRAC;
      gLight.ctrl.pwmEma = PWM_MIN_FRAC;
      applyPWMValidatedIfChanged(gLight.ctrl.pwmEma);

      evt = "CLAMP";
      logEventJson("light", "CLAMP");
      printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
      return;
    }
  } else {
    if (gLight.sensor.lux <= targetLux + OVERSHOOT_EXIT_LUX) {
      gLight.ctrl.overshootClamped = false;
      evt = "CLAMP_REL";
      printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
    } else {
      evt = "CLAMP_HOLD";
      printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
      return;
    }
  }

  if (gLight.ctrl.holding) {
    if (ledErr >= HOLD_FAST_EXIT_ERR_LUX) {
      if (++gLight.ctrl.fastExitCount >= HOLD_FAST_EXIT_TICKS) {
        gLight.ctrl.holding = false;
        gLight.ctrl.enterCount = 0;
        gLight.ctrl.exitCount = 0;
        gLight.ctrl.fastExitCount = 0;
        evt = "FAST_EXIT";
      }
    } else {
      gLight.ctrl.fastExitCount = 0;
    }
  } else {
    gLight.ctrl.fastExitCount = 0;
  }

  const bool okToHold = (gLight.ctrl.errEma >= HOLD_ABOVE_GUARD_LUX);

  if (gLight.ctrl.holding) {
    if (absErrEma >= HOLD_EXIT_LUX) {
      if (++gLight.ctrl.exitCount >= HOLD_EXIT_TICKS) {
        gLight.ctrl.holding = false;
        gLight.ctrl.enterCount = 0;
        gLight.ctrl.exitCount = 0;
        if (!evt[0]) evt = "EXIT_HOLD";
      }
    } else {
      gLight.ctrl.exitCount = 0;
    }
  } else {
    if (okToHold && absErrEma <= HOLD_ENTER_LUX) {
      if (++gLight.ctrl.enterCount >= HOLD_ENTER_TICKS) {
        gLight.ctrl.holding = true;
        gLight.ctrl.enterCount = 0;
        gLight.ctrl.exitCount = 0;
        gLight.ctrl.fastExitCount = 0;

        gLight.ctrl.pwmCmd = gLight.ctrl.pwmEma;
        applyPWMValidatedIfChanged(gLight.ctrl.pwmEma);

        if (!evt[0]) evt = "ENTER_HOLD";
      }
    } else {
      gLight.ctrl.enterCount = 0;
    }
  }

  if (gLight.ctrl.holding) {
    if (!evt[0]) evt = "HOLD";
    printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
    return;
  }

  float deltaFrac = ledErr / gLight.cal.slopeEma;

  const float tickUp   = RATE_UP_PER_SEC   * (CONTROL_PERIOD_MS / 1000.0f);
  const float tickDown = RATE_DOWN_PER_SEC * (CONTROL_PERIOD_MS / 1000.0f);

  if (deltaFrac >= 0.0f) deltaFrac = constrain(deltaFrac, 0.0f, tickUp);
  else                   deltaFrac = constrain(deltaFrac, -tickDown, 0.0f);

  const bool atMin = (gLight.ctrl.pwmCmd <= PWM_MIN_FRAC + 0.0005f);
  if (atMin && ledErr < 0.0f) {
    deltaFrac = 0.0f;
    gLight.ctrl.pwmCmd = PWM_MIN_FRAC;
  } else {
    gLight.ctrl.pwmCmd = constrain(gLight.ctrl.pwmCmd + deltaFrac, PWM_MIN_FRAC, PWM_MAX_FRAC);
  }

  gLight.ctrl.pwmEma = PWM_EMA_ALPHA * gLight.ctrl.pwmEma + (1.0f - PWM_EMA_ALPHA) * gLight.ctrl.pwmCmd;
  gLight.ctrl.pwmEma = constrain(gLight.ctrl.pwmEma, PWM_MIN_FRAC, PWM_MAX_FRAC);

  applyPWMValidatedIfChanged(gLight.ctrl.pwmEma);
  printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
}


// =======================================================
//                  TOP-LEVEL LIGHT LOOP
// =======================================================
static void updateLightSystem(uint32_t now) {
  lightSensorUpdate();

  const bool lightsOn = lightsShouldBeOn(now);

  if (lightsOn != gLight.photo.prevLightsOn) {
    gLight.photo.prevLightsOn = lightsOn;

    if (!lightsOn) {
      TPRINTLN("LIGHT CYCLE | LIGHTS_OFF (forcing PWM=0, pausing control/amb/cal)");
      logEventJson("light", "LIGHTS_OFF");
      applyPWMAnyImmediate(0.0f);

      armStepDetectIgnore(OFF_MS + 500);
      clearScheduledAmbient();

      gLight.sample.phase = SAMPLE_IDLE;
      gLight.ambMode.phase = AMB_IDLE;
      gLight.calMode.phase = CAL_IDLE;
      gLight.rt.mode = MODE_CONTROL;
      return;
    } else {
      TPRINTLN("LIGHT CYCLE | LIGHTS_ON (run AMB->CAL bundle, then resume control)");
      logEventJson("light", "LIGHTS_ON");
      resetControlFlags();
      clearScheduledAmbient();
      enterMode(MODE_AMBIENT_SAMPLE);
      return;
    }
  }

  if (!lightsOn) {
    applyPWMAnyImmediate(0.0f);
    return;
  }

  if (gLight.rt.mode == MODE_AMBIENT_SAMPLE) {
    updateAmbientMode(now);
    return;
  }

  if (gLight.rt.mode == MODE_CALIBRATION) {
    updateCalibrationMode(now);
    return;
  }

  if (!resumeGateReady(now)) return;
  if (!lightSensorFresh(now)) return;

  updateStepDelayDetector(now);

  if (!gLight.ambient.haveAmbient && AMBIENT_AT_BOOT) {
    clearScheduledAmbient();
    enterMode(MODE_AMBIENT_SAMPLE);
    return;
  }

  if (gLight.ambient.haveAmbient &&
      (!gLight.cal.haveSlope || !isfinite(gLight.cal.slopeEma) || gLight.cal.slopeEma < 1e-3f)) {
    enterMode(MODE_CALIBRATION);
    return;
  }

  const bool wantPeriodic  = ambientDue(now);
  const bool wantScheduled = scheduledAmbientDue(now);

  if (wantScheduled && (now - gLight.ambient.lastAmbientMs >= AMBIENT_MIN_GAP_MS)) {
    TPRINTLN("LIGHT AMB | TRIGGER (scheduled step) -> bundle");
    clearScheduledAmbient();
    enterMode(MODE_AMBIENT_SAMPLE);
    return;
  }

  if (wantPeriodic &&
      (now - gLight.ambient.lastAmbientMs >= AMBIENT_MIN_GAP_MS) &&
      !gLight.ctrl.overshootClamped) {
    TPRINTLN("LIGHT AMB | TRIGGER (periodic) -> bundle");
    enterMode(MODE_AMBIENT_SAMPLE);
    return;
  }

  static uint32_t lastControlTickMs = 0;
  if (now - lastControlTickMs >= CONTROL_PERIOD_MS) {
    lastControlTickMs = now;
    runControlTick(now);
  }
}


// =======================================================
//                 TELEMETRY LOGGING / POST
// =======================================================
static void printJsonNumberOrNull(float v, int digits) {
  if (isnan(v) || !isfinite(v)) Serial.print("null");
  else Serial.print(v, digits);
}

static void printTelemetryJson(uint32_t now) {
  printTimePrefix();
  Serial.print("TEL ");
  Serial.print("{\"device_id\":\"");
  Serial.print(DEVICE_ID);
  Serial.print("\"");

  Serial.print(",\"plant_id\":");
  if (assignedPlantId.length() > 0) {
    Serial.print("\"");
    Serial.print(assignedPlantId);
    Serial.print("\"");
  } else {
    Serial.print("null");
  }

  Serial.print(",\"soil_moisture_vwc\":");
  printJsonNumberOrNull(gMoist.vwcEma, 1);

  Serial.print(",\"light_lux\":");
  if (gLight.sensor.haveLux) Serial.print(gLight.sensor.lux, 1);
  else Serial.print("null");

  Serial.print(",\"temperature_c\":");
  if (climateFresh(now)) Serial.print(gClimate.temperatureC, 2);
  else Serial.print("null");

  Serial.print(",\"humidity_rh\":");
  if (climateFresh(now)) Serial.print(gClimate.humidityRH, 1);
  else Serial.print("null");

  Serial.println("}");

  printTimePrefix();
  Serial.print("DBG ");
  Serial.print("{\"ms\":");
  Serial.print(now);
  Serial.print(",\"light\":{\"mode\":\"");
  Serial.print(lightModeName(gLight.rt.mode));
  Serial.print("\",\"pwm\":");
  printJsonNumberOrNull(gLight.ctrl.pwmEma, 3);
  Serial.print(",\"ambient_lux\":");
  printJsonNumberOrNull(gLight.ambient.estLux, 1);
  Serial.print(",\"slope\":");
  printJsonNumberOrNull(gLight.cal.slopeEma, 1);
  Serial.print("},\"moisture\":{\"state\":\"");
  Serial.print(moistureStateName(gMoist.state));
  Serial.print("\",\"pump_on\":");
  Serial.print(gMoist.pumpOn ? "true" : "false");
  Serial.print(",\"pulses\":");
  Serial.print(gMoist.pulsesThisCycle);
  Serial.print("},\"climate\":{\"pressure_hpa\":");
  if (gClimate.haveReading) Serial.print(gClimate.pressureHpa, 1);
  else Serial.print("null");
  Serial.println("}}");
}

static bool telemetryPlantIdKnown() {
  return assignedPlantId.length() > 0;
}

static bool telemetryBlockedByLightMode() {
  return gLight.rt.mode != MODE_CONTROL;
}

static bool postTelemetryToBackend(uint32_t now) {
  if (!hasDeviceKey()) {
    TPRINTLN("[TEL] Skip POST: device key missing");
    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    TPRINTLN("[TEL] Skip POST: Wi-Fi not connected");
    return false;
  }

  if (!telemetryPlantIdKnown()) {
    TPRINTLN("[TEL] Skip POST: plant_id unknown");
    return false;
  }

  if (telemetryBlockedByLightMode()) {
    printTimePrefix();
    Serial.print("[TEL] Skip POST: light busy in mode ");
    Serial.println(lightModeName(gLight.rt.mode));
    return false;
  }

  DynamicJsonDocument doc(512);
  doc["device_id"] = DEVICE_ID;
  doc["plant_id"] = assignedPlantId;

  if (isfinite(gMoist.vwcEma))          doc["soil_moisture_vwc"] = roundf(gMoist.vwcEma * 10.0f) / 10.0f;
  else                                  doc["soil_moisture_vwc"] = nullptr;

  if (gLight.sensor.haveLux)            doc["light_lux"] = roundf(gLight.sensor.lux * 10.0f) / 10.0f;
  else                                  doc["light_lux"] = nullptr;

  if (climateFresh(now))                doc["temperature_c"] = roundf(gClimate.temperatureC * 100.0f) / 100.0f;
  else                                  doc["temperature_c"] = nullptr;

  if (climateFresh(now))                doc["humidity_rh"] = roundf(gClimate.humidityRH * 10.0f) / 10.0f;
  else                                  doc["humidity_rh"] = nullptr;

  String body;
  serializeJson(doc, body);

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  if (!http.begin(client, TELEMETRY_URL)) {
    TPRINTLN("[TEL] HTTP begin failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-device-id", DEVICE_ID);
  http.addHeader("x-device-key", savedDeviceKey.c_str());

  int httpCode = http.POST(body);
  String resp = http.getString();
  http.end();

  printTimePrefix();
  Serial.print("[TEL] POST status: ");
  Serial.println(httpCode);

  if (httpCode < 200 || httpCode >= 300) {
    TPRINTLN("[TEL] POST failed");
    Serial.println(resp);
    return false;
  }

  TPRINTLN("[TEL] POST ok");
  return true;
}

static void maybeSendTelemetry(uint32_t now) {
  if (now - gLastTelemetryMs < TELEMETRY_INTERVAL_MS) return;
  gLastTelemetryMs = now;

  if (!telemetryPlantIdKnown()) {
    TPRINTLN("[TEL] Interval reached, but plant_id is unknown; not sending");
    return;
  }

  if (telemetryBlockedByLightMode()) {
    printTimePrefix();
    Serial.print("[TEL] Interval reached, but light is busy in mode ");
    Serial.print(lightModeName(gLight.rt.mode));
    Serial.println("; not sending");
    return;
  }

  printTelemetryJson(now);
  postTelemetryToBackend(now);
}





// =======================================================
// EXTRA FORWARD DECLARATIONS FOR MERGED CONTROL
// =======================================================
static void setupControlHardware();
static void maybeReconnectWiFiFromSaved(uint32_t now);
static void maybeHandleAssignmentTransition();
static bool controlPlantAssigned();
static bool controlAllowed();
static void forceControlSafeOff();
static void maybeHandleControlGateTransition();
static bool prevAssignedState = false;
static String prevAssignedPlantId = "";
static uint32_t gLastWifiRetryMs = 0;
static const uint32_t WIFI_RETRY_MS = 10000;
static bool gPrevControlAllowed = false;

// =======================================================
// WIFI RECONNECT FOR RUNTIME DROPS
// =======================================================
static void maybeReconnectWiFiFromSaved(uint32_t now) {
  if (WiFi.status() == WL_CONNECTED) return;
  if (provisioningInProgress || requestInProgress) return;
  if (!hasWiFiCredentials()) return;
  if (now - gLastWifiRetryMs < WIFI_RETRY_MS) return;

  gLastWifiRetryMs = now;

  logSection("WIFI");
  logKV("Action", "runtime reconnect attempt");
  connectWiFiOnce();
}

// =======================================================
// ASSIGNMENT TRANSITION HOOK
// =======================================================
static void maybeHandleAssignmentTransition() {
  bool changed = (assigned != prevAssignedState) || (assignedPlantId != prevAssignedPlantId);
  if (!changed) return;

  prevAssignedState = assigned;
  prevAssignedPlantId = assignedPlantId;

  printTimePrefix();
  Serial.print("[ASSIGNMENT] transition assigned=");
  Serial.print(assigned ? "true" : "false");
  Serial.print(" plant_id=");
  Serial.println(assignedPlantId);

  clearCareConfig();

  if (assigned) {
    if (fetchIdealCareParameters()) {
      lastCareFetchMs = millis();
      printIdealCare();
    } else {
      TPRINTLN("[CARE] Refresh after assignment change failed");
    }
  } else {
    TPRINTLN("[CARE] Assignment cleared; plant-specific control disabled");
  }
}

static bool controlPlantAssigned() {
  return assigned && assignedPlantId.length() > 0;
}

static bool controlAllowed() {
  return controlPlantAssigned() && gCare.valid;
}

static void forceControlSafeOff() {
  moisturePumpSet(false);
  gMoist.lowCount = 0;
  gMoist.pulsesThisCycle = 0;
  gMoist.totalPumpOnMs = 0;
  gMoist.cycleStartVwc = NAN;
  moistureEnterState(MOIST_STATE_IDLE);

  applyPWMAnyImmediate(0.0f);
  gLight.ctrl.pwmCmd = PWM_MIN_FRAC;
  gLight.ctrl.pwmEma = PWM_MIN_FRAC;
  gLight.ctrl.errEma = 0.0f;
  resetControlFlags();
  clearScheduledAmbient();
  gLight.sample.phase = SAMPLE_IDLE;
  gLight.ambMode.phase = AMB_IDLE;
  gLight.calMode.phase = CAL_IDLE;
  gLight.rt.mode = MODE_CONTROL;
}

static void maybeHandleControlGateTransition() {
  bool nowAllowed = controlAllowed();
  if (nowAllowed == gPrevControlAllowed) return;

  gPrevControlAllowed = nowAllowed;

  if (nowAllowed) {
    // Restart the photoperiod when plant-specific control becomes valid.
    // This prevents the ON/OFF cycle from silently advancing while the
    // device is unassigned or missing valid care parameters.
    gLight.photo.cycleStartMs = millis();
    gLight.photo.prevLightsOn = !LIGHTS_ON_AT_BOOT; // force transition on next light tick

    resetControlFlags();
    clearScheduledAmbient();
    gLight.sample.phase = SAMPLE_IDLE;
    gLight.ambMode.phase = AMB_IDLE;
    gLight.calMode.phase = CAL_IDLE;
    gLight.rt.mode = MODE_CONTROL;

    printTimePrefix();
    Serial.print("[CONTROL] enabled for plant_id=");
    Serial.print(assignedPlantId);
    Serial.println(" (photoperiod restarted)");
    return;
  }

  printTimePrefix();
  Serial.print("[CONTROL] disabled; assigned=");
  Serial.print(assigned ? "true" : "false");
  Serial.print(" plant_id=");
  Serial.print(assignedPlantId);
  Serial.print(" care_valid=");
  Serial.println(gCare.valid ? "true" : "false");

  forceControlSafeOff();
}

// =======================================================
// CONTROL HARDWARE SETUP
// =======================================================
static void setupControlHardware() {
  clearCareConfig();

  analogReadResolution(12);
  pinMode(PUMP_PIN, OUTPUT);
  moisturePumpSet(false);
  moistureEnterState(MOIST_STATE_IDLE);

  Wire.begin(21, 22);

  if (!bh1750.begin(BH1750_TO_GROUND)) {
    TPRINTLN("BH1750 init failed");
    while (true) delay(1000);
  }

  bh1750.calibrateTiming();
  bh1750.setQuality(BH1750_QUALITY_HIGH);

  if (bme.begin(0x76, &Wire)) {
    gClimate.present = true;
    TPRINTLN("BME280 init OK @ 0x76");
  } else if (bme.begin(0x77, &Wire)) {
    gClimate.present = true;
    TPRINTLN("BME280 init OK @ 0x77");
  } else {
    gClimate.present = false;
    TPRINTLN("BME280 init failed");
  }

  ledcSetup(WHITE_PWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(RB_PWM_CH,    PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(WHITE_PWM_PIN, WHITE_PWM_CH);
  ledcAttachPin(RB_PWM_PIN,    RB_PWM_CH);

  gLight.ctrl.pwmCmd = clampPwmWindow(gLight.ctrl.pwmCmd);
  gLight.ctrl.pwmEma = clampPwmWindow(gLight.ctrl.pwmEma);
  applyPWMValidatedIfChanged(gLight.ctrl.pwmEma);

  lightSensorBegin();

  gLight.photo.cycleStartMs = millis();
  gLight.photo.prevLightsOn = LIGHTS_ON_AT_BOOT;
  armStepDetectIgnore(4000);
}



// =======================================================
// SETUP / LOOP
// =======================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  setupControlHardware();

  loadCredentials();
  setupBLE();
  refreshPublicState();
  bootResolve();

  if (wifiConnected && hasDeviceKey()) {
    bool wasAssigned = assigned;
    String wasPlantId = assignedPlantId;

    forceImmediateAssignmentRefresh();

    if (assigned != wasAssigned || assignedPlantId != wasPlantId) {
      maybeHandleAssignmentTransition();
    } else if (assigned && fetchIdealCareParameters()) {
      lastCareFetchMs = millis();
      printIdealCare();
    }
  }

  TPRINTLN("READY — BLE provisioning + moisture + light + climate + config fetch + telemetry");
  TPRINTLN("SERIAL: p=print state, r=factory reset, c=assignment check, s=force watering cycle");
  moistureLogEvt("event=BOOT");
  logEventJson("system", "BOOT");
}

void loop() {
  const uint32_t now = millis();

  handleSerial();

  bool nowWifi = (WiFi.status() == WL_CONNECTED);
  if (nowWifi != wifiConnected) {
    wifiConnected = nowWifi;

    if (wifiConnected) {
      wifiConnectedAtMs = millis();
      logSection("WIFI");
      logKV("Status", "became connected");
    } else {
      logSection("WIFI");
      logKV("Status", "became disconnected");
      assigned = false;
      assignedPlantId = "";
      refreshPublicState();
      maybeHandleAssignmentTransition();
    }

    refreshPublicState();
  }

  maybeReconnectWiFiFromSaved(now);
  checkAssignmentIfDue();
  maybeHandleAssignmentTransition();
  maybeRefreshCareParameters();
  maybeHandleControlGateTransition();

  updateClimateSystem(now);

  if (controlAllowed()) {
    updateMoistureSystem(now);
    updateLightSystem(now);
  } else {
    forceControlSafeOff();
  }

  maybeSendTelemetry(now);
}
