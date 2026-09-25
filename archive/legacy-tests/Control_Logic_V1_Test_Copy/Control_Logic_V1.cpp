/*
===========================================================
PlantIQ ESP32 BLE Provisioning + Pairing Firmware
Merged with:
- Moisture control
- Light control

Updated to support:
1. Full provisioning:
   {
     "device_key":"...",
     "wifi_ssid":"...",
     "wifi_pass":"..."
   }

2. Key-only refresh:
   {
     "device_key":"..."
   }

3. Wi-Fi-only reprovision:
   {
     "wifi_ssid":"...",
     "wifi_pass":"..."
   }

4. Force assignment refresh:
   {
     "action":"refresh_assignment"
   }

Behavior
--------
1. BLE advertises the device.
2. App reads device_info.
3. App writes encrypted provisioning/control payload.
4. ESP32 decrypts and processes one of the supported payload modes.
5. Wi-Fi handling:
   - On full provisioning Wi-Fi failure:
       clear Wi-Fi credentials only
       keep device_key
       return to SETUP
       print a single "waiting for app" Wi-Fi reprovision message
   - On Wi-Fi-only reprovision failure:
       clear Wi-Fi credentials only
       keep device_key
       return to SETUP
       print a single "waiting for app" Wi-Fi reprovision message
   - On boot Wi-Fi failure:
       clear Wi-Fi credentials only
       keep device_key
       return to SETUP
       print a single "waiting for app" Wi-Fi reprovision message
6. Backend registration handling:
   - If backend returns 401 or 404 during assignment check:
       clear stored device_key
       clear assignment state
       keep Wi-Fi credentials
       return to SETUP so app can register again
7. App-triggered assignment sync:
   - App can write {"action":"refresh_assignment"}
   - ESP32 immediately checks backend assignment status
8. BLE:
   - SETUP         -> BLE ON
   - READY_TO_PAIR -> BLE ON, paused during HTTPS check
   - PAIRED        -> BLE OFF

device_info extras
------------------
The app can inspect:
- device_key_present
- wifi_credentials_present
- needs_registration
- needs_wifi_reprovision

Logging
-------
- No BLE-read spam while the app polls device_info
- A state snapshot is printed only when public device_info fields change
- Single-shot waiting messages are printed when the device is waiting
  for app action (registration or Wi-Fi reprovision)

Serial
------
p -> print state
r -> factory reset
c -> force assignment check
s -> force moisture cycle
===========================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <ArduinoJson.h>
#include "mbedtls/aes.h"

#include <math.h>
#include <Wire.h>
#include <hp_BH1750.h>

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

static const uint8_t AES_KEY[16] = {
  0x00, 0x11, 0x22, 0x33,
  0x44, 0x55, 0x66, 0x77,
  0x88, 0x99, 0xaa, 0xbb,
  0xcc, 0xdd, 0xee, 0xff
};

// =======================================================
// STATE / ENUMS
// =======================================================

enum BleState {
  BLE_STATE_SETUP,
  BLE_STATE_READY_TO_PAIR,
  BLE_STATE_PAIRED
};

enum class MoistureState : uint8_t {
  IDLE,
  PULSE_ON,
  DIFFUSE_WAIT,
  CHECK
};

enum class SamplePhase : uint8_t {
  IDLE,
  WAIT_SETTLE,
  WAIT_FLUSH,
  TAKE_A,
  TAKE_B,
  TAKE_C,
  DONE_OK,
  DONE_FAIL
};

enum class AmbientSubState : uint8_t {
  IDLE,
  SAMPLING
};

enum class CalSubState : uint8_t {
  IDLE,
  START_LOW,
  SAMPLING_LOW,
  START_HIGH,
  SAMPLING_HIGH,
  FINALIZE
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
void checkAssignmentIfDue();
void handleMergedSerial();

// moisture forward declarations used before moisture section
static void startCycleNow(const char* reasonMsg);
static void moistureLogEvt(const char* msg);

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

void handleMergedSerial() {
  while (Serial.available()) {
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
      }
    } else if (c == 's' || c == 'S') {
      startCycleNow("event=START_CYCLE reason=FORCED_IMMEDIATE");
    }
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
  logKV("Serial command", "s = force moisture cycle");
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

// =======================================================
//                 TIME PREFIXED PRINTS
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

#define TPRINT(x)    do { printTimePrefix(); Serial.print(x); } while(0)
#define TPRINTLN(x)  do { printTimePrefix(); Serial.println(x); } while(0)

// =======================================================
//                   MOISTURE CONTROL
// =======================================================

static const int MOISTURE_ADC_PIN = 34;
static const int PUMP_PIN         = 27;

static const float ADC_REF_V = 3.30f;
static const int   ADC_MAX   = 4095;

static const float V_GAIN = 1.03202105f;
static const float V_BIAS = 0.11009852f;

static const int ADC_FLOOR_THRESHOLD = 10;

static const int   MOISTURE_SAMPLES = 12;
static const float MOISTURE_EMA_ALPHA = 0.20f;

static float LOWER_VWC = 9.0f;
static float UPPER_VWC = 20.0f;
static const int LOW_CONSEC_REQUIRED = 4;

static const uint32_t PULSE_ON_MS     = 750;
static const uint32_t DIFFUSE_WAIT_MS = 60000;

static const uint32_t SENSOR_POLL_MS         = 500;
static const uint32_t MOISTURE_CTRL_TICK_MS  = 100;
static const uint32_t STATUS_PRINT_MS        = 500;
static const uint32_t SENSOR_FRESH_MS        = SENSOR_POLL_MS * 2;

static const int      MAX_PULSES_PER_CYCLE = 12;
static const uint32_t MAX_TOTAL_PUMP_ON_MS = 30000;

static const int   RISE_CHECK_AFTER_PULSES = 3;
static const float MIN_EXPECTED_RISE_VWC   = 0.8f;

static MoistureState gMoistureState = MoistureState::IDLE;

static uint32_t moistureStateEnterMs = 0;
static uint32_t lastMoistureSensorMs = 0;
static uint32_t lastMoistureCtrlMs   = 0;
static uint32_t lastMoisturePrintMs  = 0;

static float vwcRaw = NAN;
static float vwcEma = NAN;
static float vCal   = NAN;

static int lowCount = 0;
static int pulsesThisCycle = 0;
static uint32_t totalPumpOnMs = 0;
static float cycleStartVwc = NAN;

static bool pumpOn = false;

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void pumpSet(bool on) {
  pumpOn = on;
  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
}

static const char* moistureStateToString(MoistureState s) {
  switch (s) {
    case MoistureState::IDLE:         return "IDLE";
    case MoistureState::PULSE_ON:     return "PULSE_ON";
    case MoistureState::DIFFUSE_WAIT: return "DIFFUSE";
    case MoistureState::CHECK:        return "CHECK";
    default:                          return "?";
  }
}

static void setMoistureState(MoistureState s) {
  gMoistureState = s;
  moistureStateEnterMs = millis();
}

static void moistureLogLine(const char* type) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] ");
  Serial.print(type);
  Serial.print(" | ");
}

static void moistureLogEvt(const char* msg) {
  moistureLogLine("EVT");
  Serial.println(msg);
}

static void printMoistureStatus() {
  moistureLogLine("STAT");

  Serial.print("state=");
  Serial.print(moistureStateToString(gMoistureState));

  Serial.print(" pump=");
  Serial.print(pumpOn ? "ON" : "OFF");

  Serial.print(" vCal=");
  if (isnan(vCal)) Serial.print("nan");
  else Serial.print(vCal, 3);

  Serial.print(" vwcRaw=");
  if (isnan(vwcRaw)) Serial.print("nan");
  else Serial.print(vwcRaw, 1);

  Serial.print(" vwcEma=");
  if (isnan(vwcEma)) Serial.print("nan");
  else Serial.print(vwcEma, 1);

  Serial.print(" lowCnt=");
  Serial.print(lowCount);

  Serial.print(" pulses=");
  Serial.print(pulsesThisCycle);

  Serial.print(" onMs=");
  Serial.print(totalPumpOnMs);

  Serial.print(" lower=");
  Serial.print(LOWER_VWC, 1);

  Serial.print(" upper=");
  Serial.print(UPPER_VWC, 1);

  Serial.print(" pulseMs=");
  Serial.print(PULSE_ON_MS);

  Serial.print(" diffuseMs=");
  Serial.print(DIFFUSE_WAIT_MS);

  Serial.println();
}

static float adcToVraw(int adc) {
  return (float)adc * (ADC_REF_V / (float)ADC_MAX);
}

static int readMoistureAdcAvg() {
  uint32_t sum = 0;
  for (int i = 0; i < MOISTURE_SAMPLES; i++) {
    sum += analogRead(MOISTURE_ADC_PIN);
    delayMicroseconds(1500);
  }
  return (int)(sum / (uint32_t)MOISTURE_SAMPLES);
}

static float calibrateVoltageFromAdc(int adcAvg) {
  if (adcAvg <= ADC_FLOOR_THRESHOLD) return 0.0f;
  float vRaw = adcToVraw(adcAvg);
  float v = V_GAIN * vRaw + V_BIAS;
  return clampf(v, 0.0f, 3.60f);
}

static float vh400VoltageToVWC(float v) {
  float vwc;
  if (v <= 1.10f)      vwc = 10.0f * v - 1.0f;
  else if (v <= 1.30f) vwc = 25.0f * v - 17.5f;
  else if (v <= 1.82f) vwc = 48.08f * v - 47.5f;
  else if (v <= 2.20f) vwc = 26.32f * v - 7.89f;
  else                 vwc = 62.5f  * v - 87.5f;
  return clampf(vwc, 0.0f, 100.0f);
}

static bool moistureSensorFresh(uint32_t now) {
  return (now - lastMoistureSensorMs) <= SENSOR_FRESH_MS;
}

static void resetCycleVars() {
  lowCount = 0;
  pulsesThisCycle = 0;
  totalPumpOnMs = 0;
  cycleStartVwc = vwcEma;
}

static void stopCycleToIdle(const char* reasonMsg) {
  pumpSet(false);
  moistureLogEvt(reasonMsg);
  pulsesThisCycle = 0;
  totalPumpOnMs = 0;
  cycleStartVwc = NAN;
  lowCount = 0;
  setMoistureState(MoistureState::IDLE);
}

static void startCycleNow(const char* reasonMsg) {
  if (gMoistureState != MoistureState::IDLE) {
    moistureLogEvt("event=FORCED_START_IGNORED reason=STATE_NOT_IDLE");
    return;
  }

  moistureLogEvt(reasonMsg);
  resetCycleVars();
  pumpSet(true);
  setMoistureState(MoistureState::PULSE_ON);
}

static void updateMoistureSensor(uint32_t now) {
  if (now - lastMoistureSensorMs < SENSOR_POLL_MS) return;
  lastMoistureSensorMs = now;

  int adcAvg = readMoistureAdcAvg();
  vCal = calibrateVoltageFromAdc(adcAvg);
  vwcRaw = vh400VoltageToVWC(vCal);

  if (isnan(vwcEma)) vwcEma = vwcRaw;
  else vwcEma = MOISTURE_EMA_ALPHA * vwcRaw + (1.0f - MOISTURE_EMA_ALPHA) * vwcEma;

  if (gMoistureState == MoistureState::IDLE) {
    if (!isnan(vwcEma) && vwcEma < LOWER_VWC) lowCount++;
    else lowCount = 0;
  } else {
    lowCount = 0;
  }
}

static void updateMoistureStatus(uint32_t now) {
  if (now - lastMoisturePrintMs < STATUS_PRINT_MS) return;
  lastMoisturePrintMs = now;
  printMoistureStatus();
}

static void updateMoistureControl(uint32_t now) {
  if (now - lastMoistureCtrlMs < MOISTURE_CTRL_TICK_MS) return;
  lastMoistureCtrlMs = now;

  switch (gMoistureState) {
    case MoistureState::IDLE:
      if (lowCount >= LOW_CONSEC_REQUIRED) {
        startCycleNow("event=START_CYCLE reason=LOW_CONSEC");
      }
      break;

    case MoistureState::PULSE_ON:
      if (totalPumpOnMs >= MAX_TOTAL_PUMP_ON_MS) {
        stopCycleToIdle("event=SAFE_STOP reason=MAX_TOTAL_ON_TIME action=PUMP_OFF");
        break;
      }

      if (now - moistureStateEnterMs >= PULSE_ON_MS) {
        pumpSet(false);
        pulsesThisCycle++;
        totalPumpOnMs += PULSE_ON_MS;

        if (pulsesThisCycle >= MAX_PULSES_PER_CYCLE) {
          stopCycleToIdle("event=SAFE_STOP reason=MAX_PULSES action=PUMP_OFF");
          break;
        }

        moistureLogEvt("event=PULSE_DONE next=DIFFUSE_WAIT");
        setMoistureState(MoistureState::DIFFUSE_WAIT);
      }
      break;

    case MoistureState::DIFFUSE_WAIT:
      if (now - moistureStateEnterMs >= DIFFUSE_WAIT_MS) {
        moistureLogEvt("event=DIFFUSE_DONE next=CHECK");
        setMoistureState(MoistureState::CHECK);
      }
      break;

    case MoistureState::CHECK:
      if (!moistureSensorFresh(now) || isnan(vwcEma) || isnan(cycleStartVwc)) {
        stopCycleToIdle("event=SAFE_STOP reason=SENSOR_STALE_OR_NAN action=PUMP_OFF");
        break;
      }

      if (vwcEma >= UPPER_VWC) {
        pumpSet(false);
        moistureLogEvt("event=STOP_CYCLE reason=UPPER_REACHED");
        pulsesThisCycle = 0;
        totalPumpOnMs = 0;
        cycleStartVwc = NAN;
        setMoistureState(MoistureState::IDLE);
        break;
      }

      if (pulsesThisCycle >= RISE_CHECK_AFTER_PULSES) {
        float rise = vwcEma - cycleStartVwc;
        if (rise < MIN_EXPECTED_RISE_VWC) {
          stopCycleToIdle("event=SAFE_STOP reason=NO_RISE action=PUMP_OFF");
          break;
        }
      }

      moistureLogEvt("event=NEXT_PULSE reason=BELOW_UPPER");
      pumpSet(true);
      setMoistureState(MoistureState::PULSE_ON);
      break;
  }
}

// =======================================================
//                     LIGHT CONTROL
// =======================================================

static const float TARGET_LUX = 350.0f;

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
static const float PWM_MAX_FRAC = 0.60f;

static const uint32_t ON_MS    = 60UL * 1000UL;
static const uint32_t OFF_MS   = 60UL * 1000UL;
static const uint32_t CYCLE_MS = ON_MS + OFF_MS;
static const bool     LIGHTS_ON_AT_BOOT = true;

static const uint32_t BH1750_POLL_MS = 120;
static const uint32_t LUX_STALE_MS   = 1000;
static const uint32_t LIGHT_CONTROL_PERIOD_MS = 250;

static const float RATE_UP_PER_SEC   = 0.03f;
static const float RATE_DOWN_PER_SEC = 0.09f;
static const float PWM_EMA_ALPHA     = 0.85f;

static const float   HOLD_ENTER_LUX         = 5.0f;
static const float   HOLD_EXIT_LUX          = 15.0f;
static const float   HOLD_ABOVE_GUARD_LUX   = -2.0f;
static const float   ERR_EMA_ALPHA          = 0.80f;
static const uint8_t HOLD_ENTER_TICKS       = 2;
static const uint8_t HOLD_EXIT_TICKS        = 4;
static const float   HOLD_FAST_EXIT_ERR_LUX = 40.0f;
static const uint8_t HOLD_FAST_EXIT_TICKS   = 2;

static const float OVERSHOOT_CLAMP_LUX = 50.0f;
static const float OVERSHOOT_EXIT_LUX  = 20.0f;

static const uint32_t AMBIENT_PERIOD_MS   = 30000;
static const uint32_t AMBIENT_MIN_GAP_MS  = 1500;
static const bool     AMBIENT_AT_BOOT     = true;
static const uint32_t AMBIENT_OFF_HOLD_MS = 800;
static const float    AMBIENT_SLOW_ALPHA  = 0.95f;
static const float    AMB_STEP_LUX        = 25.0f;
static const float    AMB_FAST_ALPHA      = 0.50f;

static const uint32_t RESUME_SETTLE_MS       = 700;
static const float    STEP_DETECT_LUX        = 25.0f;
static const uint8_t  STEP_DETECT_TICKS_REQ  = 3;
static const uint32_t AMB_STEP_DELAY_MS      = 3000;
static const float    LUX_FAST_ALPHA         = 0.65f;
static const float    LUX_SLOW_ALPHA         = 0.97f;
static const float    AMBLIKE_K              = 1.0f;
static const uint32_t STEP_CONFIRM_MS        = 3000;
static const float    STEP_CONFIRM_LUX       = 20.0f;

static const float    CAL_LOW_FRAC  = PWM_MIN_FRAC;
static const float    CAL_HIGH_FRAC = 0.60f;
static const uint32_t SETTLE_LOW_MS  = 900;
static const uint32_t SETTLE_HIGH_MS = 1000;
static const float    SLOPE_EMA_ALPHA = 0.80f;
static const float    MIN_LED_LUX_DELTA_FOR_SLOPE = 50.0f;

static const uint8_t  RESUME_DISCARD_UPDATES    = 3;
static const uint32_t SAMPLE_UPDATE_TIMEOUT_MS  = 1500;

hp_BH1750 bh1750;

struct LuxSensor {
  uint32_t lastStartMs = 0;
  float lux = NAN;
  bool haveLux = false;
  uint32_t luxUpdateMs = 0;

  void begin() {
    lastStartMs = millis();
    bh1750.start();
  }

  void update() {
    const uint32_t now = millis();
    if (now - lastStartMs >= BH1750_POLL_MS) {
      bh1750.start();
      lastStartMs = now;
    }
    if (bh1750.hasValue()) {
      lux = bh1750.getLux();
      haveLux = true;
      luxUpdateMs = now;
    }
  }

  bool fresh(uint32_t now) const {
    return haveLux && (now - luxUpdateMs <= LUX_STALE_MS);
  }
};

static LuxSensor sensor;

static inline uint32_t fracToDuty(float frac) {
  frac = constrain(frac, 0.0f, 1.0f);
  return (uint32_t)lroundf(frac * (float)PWM_MAX);
}

static void writeLedPWM(float overallFrac) {
  overallFrac = constrain(overallFrac, 0.0f, 1.0f);

  const float w = constrain(overallFrac * WHITE_WEIGHT, 0.0f, 1.0f);
  const float r = constrain(overallFrac * RB_WEIGHT,    0.0f, 1.0f);

  ledcWrite(WHITE_PWM_CH, fracToDuty(w));
  ledcWrite(RB_PWM_CH,    fracToDuty(r));
}

static float lastWrittenPwm = NAN;

static void applyPWMValidatedIfChanged(float frac) {
  frac = constrain(frac, PWM_MIN_FRAC, PWM_MAX_FRAC);
  if (!isfinite(lastWrittenPwm) || fabsf(frac - lastWrittenPwm) > 0.0005f) {
    writeLedPWM(frac);
    lastWrittenPwm = frac;
  }
}

static void applyPWMAnyImmediate(float frac) {
  frac = constrain(frac, 0.0f, 1.0f);
  writeLedPWM(frac);
  lastWrittenPwm = frac;
}

static inline float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

struct AsyncMedianSampler {
  bool active = false;
  SamplePhase phase = SamplePhase::IDLE;
  uint32_t settleUntilMs = 0;
  uint32_t phaseStartMs = 0;
  uint32_t refLuxUpdateMs = 0;
  float a = NAN, b = NAN, c = NAN;
  float result = NAN;

  void start(uint32_t settleMs) {
    active = true;
    phase = SamplePhase::WAIT_SETTLE;
    settleUntilMs = millis() + settleMs;
    phaseStartMs = millis();
    refLuxUpdateMs = sensor.luxUpdateMs;
    a = b = c = result = NAN;
  }

  bool newLuxArrived() const {
    return sensor.haveLux && sensor.luxUpdateMs != refLuxUpdateMs;
  }

  void advanceForNextPhase(SamplePhase next) {
    phase = next;
    phaseStartMs = millis();
    refLuxUpdateMs = sensor.luxUpdateMs;
  }

  void fail() {
    active = false;
    phase = SamplePhase::DONE_FAIL;
  }

  void finish() {
    result = median3(a, b, c);
    active = false;
    phase = SamplePhase::DONE_OK;
  }

  void update(uint32_t now) {
    if (!active) return;

    switch (phase) {
      case SamplePhase::WAIT_SETTLE:
        if ((int32_t)(now - settleUntilMs) >= 0) {
          advanceForNextPhase(SamplePhase::WAIT_FLUSH);
        }
        break;

      case SamplePhase::WAIT_FLUSH:
        if (newLuxArrived()) {
          advanceForNextPhase(SamplePhase::TAKE_A);
        } else if (now - phaseStartMs > SAMPLE_UPDATE_TIMEOUT_MS) {
          fail();
        }
        break;

      case SamplePhase::TAKE_A:
        if (newLuxArrived()) {
          a = sensor.lux;
          advanceForNextPhase(SamplePhase::TAKE_B);
        } else if (now - phaseStartMs > SAMPLE_UPDATE_TIMEOUT_MS) {
          fail();
        }
        break;

      case SamplePhase::TAKE_B:
        if (newLuxArrived()) {
          b = sensor.lux;
          advanceForNextPhase(SamplePhase::TAKE_C);
        } else if (now - phaseStartMs > SAMPLE_UPDATE_TIMEOUT_MS) {
          fail();
        }
        break;

      case SamplePhase::TAKE_C:
        if (newLuxArrived()) {
          c = sensor.lux;
          finish();
        } else if (now - phaseStartMs > SAMPLE_UPDATE_TIMEOUT_MS) {
          fail();
        }
        break;

      default:
        break;
    }
  }

  bool doneOk() const   { return phase == SamplePhase::DONE_OK; }
  bool doneFail() const { return phase == SamplePhase::DONE_FAIL; }

  void clearDone() {
    if (phase == SamplePhase::DONE_OK || phase == SamplePhase::DONE_FAIL) {
      phase = SamplePhase::IDLE;
    }
  }
};

static AsyncMedianSampler sampleAmbient;
static AsyncMedianSampler sampleCalLow;
static AsyncMedianSampler sampleCalHigh;

static float ambientEstLux = 0.0f;
static bool  haveAmbient   = false;
static uint32_t lastAmbientMs = 0;

static void updateAmbientEstimate(float ambRaw) {
  ambRaw = max(0.0f, ambRaw);

  if (!haveAmbient) {
    ambientEstLux = ambRaw;
    haveAmbient = true;
    return;
  }

  const float diff = fabsf(ambRaw - ambientEstLux);
  if (diff > AMB_STEP_LUX) {
    ambientEstLux = AMB_FAST_ALPHA * ambientEstLux + (1.0f - AMB_FAST_ALPHA) * ambRaw;
  } else {
    ambientEstLux = AMBIENT_SLOW_ALPHA * ambientEstLux + (1.0f - AMBIENT_SLOW_ALPHA) * ambRaw;
  }
}

static bool  haveSlope = false;
static float slopeEma  = NAN;

static float pwmCmd = 0.25f;
static float pwmEma = 0.25f;
static float errEma = 0.0f;

static bool overshootClamped = false;
static bool holding          = false;
static uint8_t enterCount    = 0;
static uint8_t exitCount     = 0;
static uint8_t fastExitCount = 0;
static bool clampEntryLocked = false;

static inline void resetControlFlags() {
  overshootClamped = false;
  holding = false;
  enterCount = exitCount = fastExitCount = 0;
}

static inline void lockClampEntryUntilSafeSample() {
  clampEntryLocked = true;
}

static uint32_t resumeAfterMs = 0;
static uint8_t  resumeDiscardLeft = 0;
static uint32_t resumeLastLuxUpdate = 0;

static void armResumeGate(uint8_t discardUpdates = RESUME_DISCARD_UPDATES) {
  resumeAfterMs = millis() + RESUME_SETTLE_MS;
  resumeDiscardLeft = discardUpdates;
  resumeLastLuxUpdate = sensor.luxUpdateMs;
}

static bool resumeGateAllowsControl(uint32_t now) {
  if (now < resumeAfterMs) return false;

  if (resumeDiscardLeft > 0) {
    if (sensor.luxUpdateMs != resumeLastLuxUpdate) {
      resumeLastLuxUpdate = sensor.luxUpdateMs;
      resumeDiscardLeft--;
    }
    return false;
  }

  return true;
}

static float ambLikeFast = NAN;
static float ambLikeSlow = NAN;
static uint8_t stepCount = 0;

static bool     ambScheduled = false;
static uint32_t ambRunAtMs   = 0;

static bool     stepConfirmPending = false;
static uint32_t stepConfirmAtMs    = 0;
static float    stepBaseline       = NAN;

static uint32_t stepDetectIgnoreUntilMs = 0;

static void clearScheduledAmbient() {
  ambScheduled = false;
  ambRunAtMs = 0;
}

static void armStepDetectIgnore(uint32_t ms) {
  stepDetectIgnoreUntilMs = millis() + ms;
  ambLikeFast = NAN;
  ambLikeSlow = NAN;
  stepCount = 0;
  stepConfirmPending = false;
  stepConfirmAtMs = 0;
  stepBaseline = NAN;
  clearScheduledAmbient();
}

static bool okToDetectAmbientStep() {
  return haveSlope && isfinite(slopeEma) && slopeEma > 1e-3f;
}

static void updateStepDelayDetector(uint32_t now) {
  if (!sensor.haveLux) return;
  if ((int32_t)(now - stepDetectIgnoreUntilMs) < 0) return;
  if (!okToDetectAmbientStep()) return;
  if (ambScheduled) return;

  const float ledEst  = AMBLIKE_K * slopeEma * pwmEma;
  const float ambLike = sensor.lux - ledEst;

  if (!isfinite(ambLikeFast)) {
    ambLikeFast = ambLikeSlow = ambLike;
    stepCount = 0;
    stepConfirmPending = false;
    return;
  }

  ambLikeFast = LUX_FAST_ALPHA * ambLikeFast + (1.0f - LUX_FAST_ALPHA) * ambLike;
  ambLikeSlow = LUX_SLOW_ALPHA * ambLikeSlow + (1.0f - LUX_SLOW_ALPHA) * ambLike;

  const float stepness = fabsf(ambLikeFast - ambLikeSlow);

  if (stepConfirmPending) {
    if ((int32_t)(now - stepConfirmAtMs) >= 0) {
      const float stillDiff = fabsf(ambLikeSlow - stepBaseline);
      if (stillDiff >= STEP_CONFIRM_LUX) {
        ambScheduled = true;
        ambRunAtMs = now + AMB_STEP_DELAY_MS;
        TPRINTLN("AMB | STEP CONFIRMED -> schedule AMB+CAL bundle");
      } else {
        TPRINTLN("AMB | STEP REJECTED (transient)");
      }
      stepConfirmPending = false;
      stepCount = 0;
    }
    return;
  }

  if (stepness >= STEP_DETECT_LUX) {
    if (++stepCount >= STEP_DETECT_TICKS_REQ) {
      stepConfirmPending = true;
      stepConfirmAtMs = now + STEP_CONFIRM_MS;
      stepBaseline = ambLikeSlow;
      stepCount = 0;
      TPRINTLN("AMB | STEP DETECTED -> confirming...");
    }
  } else {
    stepCount = 0;
  }
}

static bool scheduledAmbientDue(uint32_t now) {
  return ambScheduled && ((int32_t)(now - ambRunAtMs) >= 0);
}

static void printCtrlLine(float ledLux, float targetLed, float ledErr,
                          float targetPwmClamped, float targetPwmRaw,
                          const char* evt) {
  printTimePrefix();
  Serial.print("CTRL | target="); Serial.print(TARGET_LUX, 1);
  Serial.print(" lux=");          Serial.print(sensor.lux, 1);
  Serial.print(" amb=");          Serial.print(ambientEstLux, 1);
  Serial.print(" led=");          Serial.print(ledLux, 1);
  Serial.print(" tLed=");         Serial.print(targetLed, 1);
  Serial.print(" err=");          Serial.print(ledErr, 1);
  Serial.print(" errEma=");       Serial.print(errEma, 1);
  Serial.print(" pwm=");          Serial.print(pwmEma, 3);
  Serial.print(" tPwm=");         Serial.print(targetPwmClamped, 3);
  Serial.print(" tPwmRaw=");      Serial.print(targetPwmRaw, 3);
  Serial.print(" slope=");        Serial.print(slopeEma, 1);
  if (evt && evt[0]) {
    Serial.print(" | evt=");
    Serial.print(evt);
  }
  Serial.println();
}

static void printAmbRead(float raw, float est) {
  printTimePrefix();
  Serial.print("AMB | READ raw="); Serial.print(raw, 1);
  Serial.print(" est=");           Serial.println(est, 1);
}

static void printCalLow(float luxLow, float ledLow) {
  printTimePrefix();
  Serial.print("CAL | LOW  lux="); Serial.print(luxLow, 1);
  Serial.print(" led=");           Serial.println(ledLow, 1);
}

static void printCalHigh(float luxHigh, float ledHigh) {
  printTimePrefix();
  Serial.print("CAL | HIGH lux="); Serial.print(luxHigh, 1);
  Serial.print(" led=");           Serial.println(ledHigh, 1);
}

static void printCalDone(float slope, float amb) {
  printTimePrefix();
  Serial.print("CAL | DONE slopeEma="); Serial.print(slope, 1);
  Serial.print(" ambient=");            Serial.println(amb, 1);
}

static void printCalWarnLedDelta(float ledDelta) {
  printTimePrefix();
  Serial.print("CAL | WARN ledDelta invalid/small: ");
  Serial.println(ledDelta, 1);
}

static const uint8_t MODE_CONTROL = 0;
static const uint8_t MODE_AMBIENT = 1;
static const uint8_t MODE_CAL     = 2;

static uint8_t mode = MODE_CONTROL;
static float pwmSaved = 0.25f;

static void enterMode(uint8_t m) {
  mode = m;
  pwmSaved = pwmEma;
  resetControlFlags();
}

static bool ambientDue(uint32_t now) {
  if (now - lastAmbientMs < AMBIENT_MIN_GAP_MS) return false;
  if (!haveAmbient && AMBIENT_AT_BOOT) return true;
  return (now - lastAmbientMs >= AMBIENT_PERIOD_MS);
}

static uint32_t cycleStartMs = 0;

static bool lightsShouldBeOn(uint32_t now) {
  uint32_t t = (uint32_t)(now - cycleStartMs);
  t %= CYCLE_MS;
  return LIGHTS_ON_AT_BOOT ? (t < ON_MS) : (t >= OFF_MS);
}

static AmbientSubState ambientSub = AmbientSubState::IDLE;

static void startAmbientMode() {
  TPRINTLN("AMB | START (LEDs OFF)");
  applyPWMAnyImmediate(0.0f);
  sampleAmbient.start(AMBIENT_OFF_HOLD_MS);
  ambientSub = AmbientSubState::SAMPLING;
}

static void updateAmbientMode(uint32_t now) {
  (void)now;

  if (ambientSub == AmbientSubState::IDLE) {
    startAmbientMode();
    return;
  }

  sampleAmbient.update(millis());

  if (sampleAmbient.doneFail()) {
    lastAmbientMs = millis();
    TPRINTLN("AMB | WARN sample failed -> restore");

    pwmCmd = constrain(pwmSaved, PWM_MIN_FRAC, PWM_MAX_FRAC);
    pwmEma = pwmCmd;
    applyPWMValidatedIfChanged(pwmEma);
    errEma = 0.0f;

    lockClampEntryUntilSafeSample();
    armResumeGate();
    armStepDetectIgnore(RESUME_SETTLE_MS + 1500);

    sampleAmbient.clearDone();
    ambientSub = AmbientSubState::IDLE;
    mode = MODE_CONTROL;
    return;
  }

  if (sampleAmbient.doneOk()) {
    const float amb = sampleAmbient.result;
    lastAmbientMs = millis();

    updateAmbientEstimate(amb);
    printAmbRead(amb, ambientEstLux);

    TPRINTLN("BUNDLE | AMB OK -> CAL next");

    sampleAmbient.clearDone();
    ambientSub = AmbientSubState::IDLE;
    mode = MODE_CAL;
  }
}

static CalSubState calSub = CalSubState::IDLE;
static float luxLowSample = NAN;
static float luxHighSample = NAN;

static void restoreAfterCalibration() {
  applyPWMValidatedIfChanged(constrain(pwmSaved, PWM_MIN_FRAC, PWM_MAX_FRAC));
  TPRINTLN("CAL | RESTORE PWM");

  lockClampEntryUntilSafeSample();

  pwmCmd = constrain(pwmSaved, PWM_MIN_FRAC, PWM_MAX_FRAC);
  pwmEma = pwmCmd;
  errEma = 0.0f;

  armResumeGate();
  armStepDetectIgnore(RESUME_SETTLE_MS + 1500);

  calSub = CalSubState::IDLE;
  mode = MODE_CONTROL;
}

static void updateCalMode(uint32_t now) {
  (void)now;

  switch (calSub) {
    case CalSubState::IDLE:
      if (!haveAmbient) {
        mode = MODE_CONTROL;
        return;
      }
      TPRINTLN("CAL | START");
      calSub = CalSubState::START_LOW;
      break;

    case CalSubState::START_LOW:
      applyPWMValidatedIfChanged(CAL_LOW_FRAC);
      sampleCalLow.start(SETTLE_LOW_MS);
      calSub = CalSubState::SAMPLING_LOW;
      break;

    case CalSubState::SAMPLING_LOW:
      sampleCalLow.update(millis());
      if (sampleCalLow.doneFail()) {
        TPRINTLN("CAL | FAIL sample(s)");
        sampleCalLow.clearDone();
        restoreAfterCalibration();
      } else if (sampleCalLow.doneOk()) {
        luxLowSample = sampleCalLow.result;
        sampleCalLow.clearDone();
        calSub = CalSubState::START_HIGH;
      }
      break;

    case CalSubState::START_HIGH:
      applyPWMValidatedIfChanged(CAL_HIGH_FRAC);
      sampleCalHigh.start(SETTLE_HIGH_MS);
      calSub = CalSubState::SAMPLING_HIGH;
      break;

    case CalSubState::SAMPLING_HIGH:
      sampleCalHigh.update(millis());
      if (sampleCalHigh.doneFail()) {
        TPRINTLN("CAL | FAIL sample(s)");
        sampleCalHigh.clearDone();
        restoreAfterCalibration();
      } else if (sampleCalHigh.doneOk()) {
        luxHighSample = sampleCalHigh.result;
        sampleCalHigh.clearDone();
        calSub = CalSubState::FINALIZE;
      }
      break;

    case CalSubState::FINALIZE: {
      const float ledLow  = max(0.0f, luxLowSample  - ambientEstLux);
      const float ledHigh = max(0.0f, luxHighSample - ambientEstLux);

      printCalLow(luxLowSample, ledLow);
      printCalHigh(luxHighSample, ledHigh);

      const float ledDelta  = ledHigh - ledLow;
      const float fracDelta = (CAL_HIGH_FRAC - CAL_LOW_FRAC);

      if (isfinite(ledDelta) && ledDelta > MIN_LED_LUX_DELTA_FOR_SLOPE && fracDelta > 1e-6f) {
        const float slopeInst = ledDelta / fracDelta;

        if (!haveSlope || !isfinite(slopeEma)) {
          slopeEma = slopeInst;
          haveSlope = true;
        } else {
          slopeEma = SLOPE_EMA_ALPHA * slopeEma + (1.0f - SLOPE_EMA_ALPHA) * slopeInst;
        }

        printCalDone(slopeEma, ambientEstLux);
      } else {
        printCalWarnLedDelta(ledDelta);
      }

      restoreAfterCalibration();
      break;
    }
  }
}

static void runLightControlTick(uint32_t now) {
  if (!sensor.fresh(now)) return;

  if (!haveAmbient || !haveSlope || !isfinite(slopeEma) || slopeEma < 1e-3f) {
    static uint32_t lastPrint = 0;
    if (now - lastPrint > 1000) {
      lastPrint = now;
      TPRINTLN("CTRL | waiting for ambient+slope");
    }
    return;
  }

  const float ledLux    = max(0.0f, sensor.lux - ambientEstLux);
  const float targetLed = max(0.0f, TARGET_LUX - ambientEstLux);
  const float ledErr    = targetLed - ledLux;

  float targetPwmRaw = (targetLed <= 0.0f) ? 0.0f : (targetLed / slopeEma);
  float targetPwmClamped = constrain(targetPwmRaw, PWM_MIN_FRAC, PWM_MAX_FRAC);

  if (targetLed <= 0.0f) {
    applyPWMAnyImmediate(0.0f);
    pwmCmd = 0.0f;
    pwmEma = 0.0f;
    errEma = 0.0f;
    resetControlFlags();
    printCtrlLine(0.0f, targetLed, 0.0f, 0.0f, targetPwmRaw, "AMBIENT_OK_OFF");
    return;
  }

  if (targetPwmRaw < PWM_MIN_FRAC) targetPwmClamped = PWM_MIN_FRAC;

  if (clampEntryLocked) {
    if (sensor.lux <= TARGET_LUX + OVERSHOOT_EXIT_LUX) clampEntryLocked = false;
  }

  errEma = ERR_EMA_ALPHA * errEma + (1.0f - ERR_EMA_ALPHA) * ledErr;
  const float absErrEma = fabsf(errEma);

  const char* evt = "";

  if (!overshootClamped) {
    if (!clampEntryLocked && sensor.lux >= TARGET_LUX + OVERSHOOT_CLAMP_LUX) {
      overshootClamped = true;
      holding = false;
      enterCount = exitCount = fastExitCount = 0;

      pwmCmd = PWM_MIN_FRAC;
      pwmEma = PWM_MIN_FRAC;
      applyPWMValidatedIfChanged(pwmEma);

      evt = "CLAMP";
      printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
      return;
    }
  } else {
    if (sensor.lux <= TARGET_LUX + OVERSHOOT_EXIT_LUX) {
      overshootClamped = false;
      evt = "CLAMP_REL";
      printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
    } else {
      evt = "CLAMP_HOLD";
      printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
      return;
    }
  }

  if (holding) {
    if (ledErr >= HOLD_FAST_EXIT_ERR_LUX) {
      if (++fastExitCount >= HOLD_FAST_EXIT_TICKS) {
        holding = false;
        enterCount = exitCount = fastExitCount = 0;
        evt = "FAST_EXIT";
      }
    } else {
      fastExitCount = 0;
    }
  } else {
    fastExitCount = 0;
  }

  const bool okToHold = (errEma >= HOLD_ABOVE_GUARD_LUX);

  if (holding) {
    if (absErrEma >= HOLD_EXIT_LUX) {
      if (++exitCount >= HOLD_EXIT_TICKS) {
        holding = false;
        enterCount = exitCount = 0;
        if (!evt[0]) evt = "EXIT_HOLD";
      }
    } else {
      exitCount = 0;
    }
  } else {
    if (okToHold && absErrEma <= HOLD_ENTER_LUX) {
      if (++enterCount >= HOLD_ENTER_TICKS) {
        holding = true;
        enterCount = exitCount = 0;
        fastExitCount = 0;

        pwmCmd = pwmEma;
        applyPWMValidatedIfChanged(pwmEma);
        if (!evt[0]) evt = "ENTER_HOLD";
      }
    } else {
      enterCount = 0;
    }
  }

  if (holding) {
    if (!evt[0]) evt = "HOLD";
    printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
    return;
  }

  float deltaFrac = ledErr / slopeEma;

  const float tickUp   = RATE_UP_PER_SEC   * (LIGHT_CONTROL_PERIOD_MS / 1000.0f);
  const float tickDown = RATE_DOWN_PER_SEC * (LIGHT_CONTROL_PERIOD_MS / 1000.0f);

  if (deltaFrac >= 0.0f) deltaFrac = constrain(deltaFrac, 0.0f, tickUp);
  else                   deltaFrac = constrain(deltaFrac, -tickDown, 0.0f);

  const bool atMin = (pwmCmd <= PWM_MIN_FRAC + 0.0005f);
  if (atMin && ledErr < 0.0f) {
    deltaFrac = 0.0f;
    pwmCmd = PWM_MIN_FRAC;
  } else {
    pwmCmd = constrain(pwmCmd + deltaFrac, PWM_MIN_FRAC, PWM_MAX_FRAC);
  }

  pwmEma = PWM_EMA_ALPHA * pwmEma + (1.0f - PWM_EMA_ALPHA) * pwmCmd;
  pwmEma = constrain(pwmEma, PWM_MIN_FRAC, PWM_MAX_FRAC);

  applyPWMValidatedIfChanged(pwmEma);
  printCtrlLine(ledLux, targetLed, ledErr, targetPwmClamped, targetPwmRaw, evt);
}

static uint32_t lastLightTick = 0;

static void updateLightSystem(uint32_t now) {
  sensor.update();

  if (mode == MODE_AMBIENT) updateAmbientMode(now);
  if (mode == MODE_CAL)     updateCalMode(now);

  if (now - lastLightTick < LIGHT_CONTROL_PERIOD_MS) return;
  lastLightTick = now;

  static bool prevLightsOn = LIGHTS_ON_AT_BOOT;
  const bool lightsOn = lightsShouldBeOn(now);

  if (lightsOn != prevLightsOn) {
    prevLightsOn = lightsOn;

    if (!lightsOn) {
      TPRINTLN("CYCLE | LIGHTS_OFF (forcing PWM=0, pausing control/amb/cal)");
      applyPWMAnyImmediate(0.0f);
      armStepDetectIgnore(OFF_MS + 500);
      clearScheduledAmbient();
      ambientSub = AmbientSubState::IDLE;
      calSub = CalSubState::IDLE;
      mode = MODE_CONTROL;
      return;
    } else {
      TPRINTLN("CYCLE | LIGHTS_ON (run AMB->CAL bundle, then resume control)");
      resetControlFlags();
      clearScheduledAmbient();
      ambientSub = AmbientSubState::IDLE;
      calSub = CalSubState::IDLE;
      enterMode(MODE_AMBIENT);
      return;
    }
  }

  if (!lightsOn) {
    applyPWMAnyImmediate(0.0f);
    return;
  }

  if (mode == MODE_AMBIENT || mode == MODE_CAL) return;

  if (!resumeGateAllowsControl(now)) return;
  if (!sensor.fresh(now)) return;

  updateStepDelayDetector(now);

  if (!haveAmbient && AMBIENT_AT_BOOT) {
    clearScheduledAmbient();
    ambientSub = AmbientSubState::IDLE;
    enterMode(MODE_AMBIENT);
    return;
  }

  if (haveAmbient && (!haveSlope || !isfinite(slopeEma) || slopeEma < 1e-3f)) {
    calSub = CalSubState::IDLE;
    enterMode(MODE_CAL);
    return;
  }

  const bool wantPeriodic  = ambientDue(now);
  const bool wantScheduled = scheduledAmbientDue(now);

  if (wantScheduled && (now - lastAmbientMs >= AMBIENT_MIN_GAP_MS)) {
    TPRINTLN("AMB | TRIGGER (scheduled step) -> bundle");
    clearScheduledAmbient();
    ambientSub = AmbientSubState::IDLE;
    enterMode(MODE_AMBIENT);
    return;
  }

  if (wantPeriodic && (now - lastAmbientMs >= AMBIENT_MIN_GAP_MS) && !overshootClamped) {
    TPRINTLN("AMB | TRIGGER (periodic) -> bundle");
    ambientSub = AmbientSubState::IDLE;
    enterMode(MODE_AMBIENT);
    return;
  }

  runLightControlTick(now);
}

// =======================================================
// SETUP
// =======================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  loadCredentials();
  setupBLE();
  refreshPublicState();
  bootResolve();

  analogReadResolution(12);
  pinMode(PUMP_PIN, OUTPUT);
  pumpSet(false);
  moistureLogEvt("event=BOOT");
  setMoistureState(MoistureState::IDLE);

  Wire.begin(21, 22);

  if (!bh1750.begin(BH1750_TO_GROUND)) {
    TPRINTLN("BH1750 init failed");
    while (true) delay(1000);
  }
  bh1750.calibrateTiming();
  bh1750.setQuality(BH1750_QUALITY_HIGH);

  ledcSetup(WHITE_PWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(RB_PWM_CH,    PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(WHITE_PWM_PIN, WHITE_PWM_CH);
  ledcAttachPin(RB_PWM_PIN,    RB_PWM_CH);

  pwmCmd = constrain(pwmCmd, PWM_MIN_FRAC, PWM_MAX_FRAC);
  pwmEma = constrain(pwmEma, PWM_MIN_FRAC, PWM_MAX_FRAC);
  applyPWMValidatedIfChanged(pwmEma);

  sensor.begin();

  cycleStartMs = millis();
  armStepDetectIgnore(4000);

  TPRINTLN("READY — merged BLE + provisioning + moisture + light control");
  TPRINTLN("Serial: 'p' print state, 'r' factory reset, 'c' assignment check, 's' force moisture cycle");
}

// =======================================================
// LOOP
// =======================================================

void loop() {
  const uint32_t now = millis();

  handleMergedSerial();
  checkAssignmentIfDue();

  updateMoistureSensor(now);
  updateMoistureStatus(now);
  updateMoistureControl(now);

  updateLightSystem(now);
}