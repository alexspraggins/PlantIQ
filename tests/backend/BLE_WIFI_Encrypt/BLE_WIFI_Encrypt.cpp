/*
===========================================================
PlantIQ ESP32 BLE Provisioning + WiFi Firmware
===========================================================

BLE is used only to provision WiFi credentials.

Flow
----
1. ESP32 boots and advertises BLE.
2. Phone connects and writes provisioning JSON.
3. ESP32 decrypts payload and parses JSON credentials.
4. Credentials stored in NVS.
5. ESP32 connects to WiFi.
6. Internet connectivity checked via HTTP.
7. If WiFi succeeds:
   - provisioned = true
   - BLE advertising stops
8. If WiFi fails:
   - provisioned remains false
   - BLE stays on for reprovisioning

BLE Service
-----------
Service UUID
12345678-1234-1234-1234-1234567890ab

Read Characteristic
abcd1234-1234-1234-1234-abcdefabcdef

Write Characteristic
dcba4321-4321-4321-4321-fedcbafedcba

Serial Commands
---------------
p  → print device state
r  → factory reset

*/

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <ArduinoJson.h>
#include "mbedtls/aes.h"

// =======================================================
// DEVICE CONFIG
// =======================================================

static const char* DEVICE_ID = "esp32-01";

#define SERVICE_UUID      "12345678-1234-1234-1234-1234567890ab"
#define DEVICE_INFO_UUID  "abcd1234-1234-1234-1234-abcdefabcdef"
#define PROVISION_UUID    "dcba4321-4321-4321-4321-fedcbafedcba"

static const unsigned long WIFI_CHECK_INTERVAL_MS = 10000;

// Intentionally blank in the public repository. Configure the matching
// provisioning key locally before using encrypted BLE provisioning.
static const uint8_t AES_KEY[16] = {};

// =======================================================
// GLOBAL STATE
// =======================================================

bool provisioned = false;
bool wifiConnected = false;
bool bleStopped = false;
bool bleClientConnected = false;

String savedDeviceKey = "";
String savedWifiSsid  = "";
String savedWifiPass  = "";

Preferences prefs;

BLEServer* server = nullptr;
BLEAdvertising* advertising = nullptr;
BLECharacteristic* deviceInfoChar = nullptr;
BLECharacteristic* provisionChar = nullptr;

unsigned long lastWifiCheckMs = 0;

// =======================================================
// JSON STATUS
// =======================================================

String makeDeviceInfoJson() {
  StaticJsonDocument<128> doc;
  doc["device_id"] = DEVICE_ID;
  doc["provisioned"] = provisioned;
  doc["wifi_connected"] = wifiConnected;

  String out;
  serializeJson(doc, out);
  return out;
}

void updateDeviceInfoCharacteristic() {
  if (!deviceInfoChar) return;

  String payload = makeDeviceInfoJson();
  deviceInfoChar->setValue(payload.c_str());

  Serial.println("device_info updated:");
  Serial.println(payload);
}

// =======================================================
// STORAGE
// =======================================================

void saveCredentialsOnly(const String& key, const String& ssid, const String& pass) {
  prefs.begin("plantiq", false);
  prefs.putString("device_key", key);
  prefs.putString("wifi_ssid", ssid);
  prefs.putString("wifi_pass", pass);
  prefs.end();

  savedDeviceKey = key;
  savedWifiSsid = ssid;
  savedWifiPass = pass;

  Serial.println("Credentials saved");
}

void setProvisionedFlag(bool value) {
  prefs.begin("plantiq", false);
  prefs.putBool("provisioned", value);
  prefs.end();

  provisioned = value;

  Serial.print("Provisioned flag set to: ");
  Serial.println(provisioned ? "true" : "false");
}

void loadProvisioning() {
  prefs.begin("plantiq", true);
  savedDeviceKey = prefs.getString("device_key", "");
  savedWifiSsid  = prefs.getString("wifi_ssid", "");
  savedWifiPass  = prefs.getString("wifi_pass", "");
  provisioned    = prefs.getBool("provisioned", false);
  prefs.end();

  Serial.println("Loaded provisioning:");
  Serial.print("Device key: ");
  Serial.println(savedDeviceKey);
  Serial.print("SSID: ");
  Serial.println(savedWifiSsid);
  Serial.print("Password: ");
  Serial.println(savedWifiPass);
  Serial.print("Provisioned: ");
  Serial.println(provisioned ? "true" : "false");
}

void clearProvisioning() {
  prefs.begin("plantiq", false);
  prefs.clear();
  prefs.end();

  provisioned = false;
  wifiConnected = false;
  savedDeviceKey = "";
  savedWifiSsid = "";
  savedWifiPass = "";

  Serial.println("Provisioning cleared");
}

// =======================================================
// INTERNET TEST
// =======================================================

bool testInternetConnection() {
  Serial.println("Testing internet connection...");

  HTTPClient http;
  http.begin("http://example.com");

  int httpCode = http.GET();

  if (httpCode > 0) {
    Serial.print("HTTP code: ");
    Serial.println(httpCode);

    if (httpCode == 200) {
      Serial.println("Internet OK");
      http.end();
      return true;
    }
  }

  Serial.println("Internet test failed");
  http.end();
  return false;
}

// =======================================================
// WIFI
// =======================================================

bool connectToWiFi(const String& ssid, const String& pass) {
  if (ssid.length() == 0) {
    Serial.println("WiFi SSID empty");
    wifiConnected = false;
    updateDeviceInfoCharacteristic();
    return false;
  }

  Serial.println("Connecting to WiFi...");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(300);

  WiFi.begin(ssid.c_str(), pass.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  wifiConnected = (WiFi.status() == WL_CONNECTED);

  if (wifiConnected) {
    Serial.println("WiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    bool internetOK = testInternetConnection();
    if (!internetOK) {
      Serial.println("Connected to WiFi but internet test failed");
    }
  } else {
    Serial.println("WiFi connection failed");
  }

  updateDeviceInfoCharacteristic();
  return wifiConnected;
}

// =======================================================
// BLE CONTROL
// =======================================================

void stopBLE() {
  if (bleStopped) return;
  if (advertising) advertising->stop();
  bleStopped = true;
  Serial.println("BLE stopped");
}

void startBLE() {
  if (!advertising) return;
  advertising->start();
  bleStopped = false;
  Serial.println("BLE advertising started");
}

// =======================================================
// STATUS PRINT
// =======================================================

void printState() {
  Serial.println("====== DEVICE STATE ======");
  Serial.print("Provisioned: ");
  Serial.println(provisioned ? "true" : "false");
  Serial.print("WiFi connected: ");
  Serial.println(wifiConnected ? "true" : "false");
  Serial.print("BLE client connected: ");
  Serial.println(bleClientConnected ? "true" : "false");
  Serial.print("BLE stopped: ");
  Serial.println(bleStopped ? "true" : "false");
  Serial.print("Device key: ");
  Serial.println(savedDeviceKey);
  Serial.print("SSID: ");
  Serial.println(savedWifiSsid);
  Serial.print("Password: ");
  Serial.println(savedWifiPass);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("IP: not connected");
  }
  Serial.println("==========================");
}

// =======================================================
// FACTORY RESET
// =======================================================

void factoryReset() {
  Serial.println("Factory reset");

  clearProvisioning();

  WiFi.disconnect(true, true);
  delay(200);

  startBLE();
  updateDeviceInfoCharacteristic();
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
    Serial.println("Encrypted payload too short");
    return false;
  }

  const size_t ivLen = 16;
  const size_t cipherLen = inputLen - ivLen;

  if (cipherLen == 0 || (cipherLen % 16) != 0) {
    Serial.println("Ciphertext length invalid for AES-CBC");
    return false;
  }

  uint8_t iv[16];
  memcpy(iv, input, 16);

  uint8_t* decrypted = (uint8_t*)malloc(cipherLen);
  if (!decrypted) {
    Serial.println("Memory allocation failed");
    return false;
  }

  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);

  int rc = mbedtls_aes_setkey_dec(&ctx, AES_KEY, 128);
  if (rc != 0) {
    Serial.printf("AES setkey failed: %d\n", rc);
    mbedtls_aes_free(&ctx);
    free(decrypted);
    return false;
  }

  rc = mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_DECRYPT, cipherLen, iv, input + ivLen, decrypted);
  mbedtls_aes_free(&ctx);

  if (rc != 0) {
    Serial.printf("AES decrypt failed: %d\n", rc);
    free(decrypted);
    return false;
  }

  size_t plainLen = cipherLen;
  if (!pkcs7Unpad(decrypted, plainLen)) {
    Serial.println("PKCS7 unpad failed");
    free(decrypted);
    return false;
  }

  plaintextOut.reserve(plainLen);
  plaintextOut = "";
  for (size_t i = 0; i < plainLen; i++) {
    plaintextOut += (char)decrypted[i];
  }

  free(decrypted);
  return true;
}

// =======================================================
// BLE CALLBACKS
// =======================================================

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleClientConnected = true;
    Serial.println("BLE connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    bleClientConnected = false;
    Serial.println("BLE disconnected");

    if (!bleStopped) {
      advertising->start();
    }
  }
};

class DeviceInfoCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic* characteristic) override {
    characteristic->setValue(makeDeviceInfoJson().c_str());
    Serial.println("device_info read");
    Serial.println(makeDeviceInfoJson());
  }
};

class ProvisionCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    size_t rawLen = characteristic->getLength();
    uint8_t* rawData = characteristic->getData();

    Serial.print("Provision write received, bytes: ");
    Serial.println((int)rawLen);

    if (rawData == nullptr || rawLen == 0) {
      Serial.println("Invalid payload: empty write");
      return;
    }

    String decryptedJson;
    if (!decryptProvisionPayload(rawData, rawLen, decryptedJson)) {
      Serial.println("Provision payload decryption failed");
      return;
    }

    decryptedJson.trim();

    Serial.println("Decrypted provisioning JSON:");
    Serial.println(decryptedJson);

    if (!decryptedJson.startsWith("{")) {
      Serial.println("Invalid decrypted payload: expected JSON");
      return;
    }

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, decryptedJson);

    if (err) {
      Serial.print("JSON parse error: ");
      Serial.println(err.c_str());
      return;
    }

    if (!doc.containsKey("device_key") ||
        !doc.containsKey("wifi_ssid") ||
        !doc.containsKey("wifi_pass")) {
      Serial.println("Provisioning JSON missing required fields");
      return;
    }

    String key  = doc["device_key"].as<String>();
    String ssid = doc["wifi_ssid"].as<String>();
    String pass = doc["wifi_pass"].as<String>();

    Serial.println("Provision values:");
    Serial.print("Device Key: ");
    Serial.println(key);
    Serial.print("WiFi SSID: ");
    Serial.println(ssid);
    Serial.print("WiFi Pass: ");
    Serial.println(pass);

    if (key.length() == 0 || ssid.length() == 0) {
      Serial.println("Invalid provisioning values");
      return;
    }

    saveCredentialsOnly(key, ssid, pass);
    setProvisionedFlag(false);
    updateDeviceInfoCharacteristic();

    if (connectToWiFi(ssid, pass)) {
      Serial.println("Provisioning success");
      setProvisionedFlag(true);
      updateDeviceInfoCharacteristic();
      stopBLE();
    } else {
      Serial.println("WiFi failed. Credentials saved, but device remains unprovisioned.");
      setProvisionedFlag(false);
      updateDeviceInfoCharacteristic();
    }
  }
};

// =======================================================
// BLE SETUP
// =======================================================

void setupBLE() {
  String name = "PlantIQ-";
  name += DEVICE_ID;

  Serial.print("BLE device: ");
  Serial.println(name);

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
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  provisionChar->setCallbacks(new ProvisionCallbacks());

  service->start();

  advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  bleStopped = false;

  Serial.println("BLE advertising started");
  Serial.println("Type 'p' to print device state");
  Serial.println("Type 'r' to factory reset");
}

// =======================================================
// SERIAL COMMANDS
// =======================================================

void handleSerial() {
  if (!Serial.available()) return;

  char c = Serial.read();

  if (c == 'p' || c == 'P') {
    printState();
  } else if (c == 'r' || c == 'R') {
    factoryReset();
  }
}

// =======================================================
// SETUP
// =======================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  loadProvisioning();
  setupBLE();
  updateDeviceInfoCharacteristic();

  if (savedWifiSsid.length() > 0) {
    Serial.println("Saved credentials found. Attempting WiFi...");

    if (connectToWiFi(savedWifiSsid, savedWifiPass)) {
      setProvisionedFlag(true);
      updateDeviceInfoCharacteristic();
      stopBLE();
    } else {
      setProvisionedFlag(false);
      updateDeviceInfoCharacteristic();
      Serial.println("Boot WiFi reconnect failed. BLE stays available.");
    }
  } else {
    Serial.println("No saved credentials. Waiting for BLE provisioning.");
  }
}

// =======================================================
// LOOP
// =======================================================

void loop() {
  handleSerial();

  if (millis() - lastWifiCheckMs > WIFI_CHECK_INTERVAL_MS) {
    lastWifiCheckMs = millis();

    bool now = (WiFi.status() == WL_CONNECTED);

    if (now != wifiConnected) {
      wifiConnected = now;

      if (!wifiConnected) {
        provisioned = false;
      }

      updateDeviceInfoCharacteristic();
    }
  }
}
