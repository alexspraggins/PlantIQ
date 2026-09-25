#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

static const char* DEVICE_ID = "esp32-01";
static bool provisioned = false;

#define SERVICE_UUID      "12345678-1234-1234-1234-1234567890ab"
#define DEVICE_INFO_UUID  "abcd1234-1234-1234-1234-abcdefabcdef"
#define PROVISION_UUID    "dcba4321-4321-4321-4321-fedcbafedcba"

BLECharacteristic* deviceInfoChar = nullptr;
BLECharacteristic* provisionChar  = nullptr;

String makeDeviceInfoJson() {
  String s = "{";
  s += "\"device_id\":\"";
  s += DEVICE_ID;
  s += "\",\"provisioned\":";
  s += (provisioned ? "true" : "false");
  s += "}";
  return s;
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("BLE client connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    Serial.println("BLE client disconnected");
    BLEDevice::startAdvertising();
  }
};

class DeviceInfoCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic* pCharacteristic) override {
    String payload = makeDeviceInfoJson();
    pCharacteristic->setValue(payload.c_str());

    Serial.println("device_info read");
    Serial.println(payload);
  }
};

class ProvisionCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue();

    Serial.println("onWrite callback fired");
    Serial.print("Raw length: ");
    Serial.println(value.length());

    value.trim();

    Serial.print("Received: ");
    Serial.println(value);

    if (value.length() == 0) {
      Serial.println("Empty write");
      return;
    }

    if (!value.startsWith("{")) {
      Serial.println("Plain text write detected");
      return;
    }

    bool hasDeviceKey = value.indexOf("\"device_key\"") >= 0;
    bool hasWifiSsid  = value.indexOf("\"wifi_ssid\"") >= 0;
    bool hasWifiPass  = value.indexOf("\"wifi_pass\"") >= 0;

    Serial.print("device_key present: ");
    Serial.println(hasDeviceKey ? "YES" : "NO");

    Serial.print("wifi_ssid present: ");
    Serial.println(hasWifiSsid ? "YES" : "NO");

    Serial.print("wifi_pass present: ");
    Serial.println(hasWifiPass ? "YES" : "NO");

    if (hasDeviceKey && hasWifiSsid && hasWifiPass) {
      Serial.println("Provisioning JSON format looks correct.");
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(500);

  String bleName = "PlantIQ-";
  bleName += DEVICE_ID;

  Serial.println("Starting BLE...");
  Serial.print("Device name: ");
  Serial.println(bleName);

  BLEDevice::init(bleName.c_str());

  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  deviceInfoChar = pService->createCharacteristic(
    DEVICE_INFO_UUID,
    BLECharacteristic::PROPERTY_READ
  );
  deviceInfoChar->setCallbacks(new DeviceInfoCallbacks());
  deviceInfoChar->setValue(makeDeviceInfoJson().c_str());

  provisionChar = pService->createCharacteristic(
    PROVISION_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  provisionChar->setCallbacks(new ProvisionCallbacks());
  provisionChar->setValue("");

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("BLE advertising started");
  Serial.println("Look for PlantIQ-esp32-01 in LightBlue");
  Serial.println("READ  UUID: abcd1234-1234-1234-1234-abcdefabcdef");
  Serial.println("WRITE UUID: dcba4321-4321-4321-4321-fedcbafedcba");
}

void loop() {
  delay(1000);
}