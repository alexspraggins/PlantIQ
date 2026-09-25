/*
===========================================================
PlantIQ ESP32 BLE Provisioning + Pairing Firmware
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

// =======================================================
// SETUP / LOOP
// =======================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  loadCredentials();
  setupBLE();
  refreshPublicState();
  bootResolve();
}

void loop() {
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
    }
    refreshPublicState();
  }

  checkAssignmentIfDue();
}