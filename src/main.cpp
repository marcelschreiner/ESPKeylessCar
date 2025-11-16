/*
 * ESP32 Dynamic iPhone Keyless System v7 (Open Source Ready)
 * Author: crazyhoesl
 * Date: 27. Juli 2025
 * 
 * Features:
 * - Dynamic IRK learning via BLE pairing
 * - Up to 10 devices supported
 * - EEPROM storage for persistent IRKs
 * - 1-minute pairing window on startup
 * - LED status feedback
 * - Full serial debugging
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEAdvertising.h>
#include "mbedtls/aes.h"
#include "esp_task_wdt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "EEPROM.h"

// ========================================
// CONFIGURATION
// ========================================
#define MAX_DEVICES 10
#define EEPROM_SIZE 512
#define PAIRING_TIMEOUT_MS 30000  // 30 seconds pairing window

// Pin definitions
#define LED_PIN 2
#define KEY_POWER_PIN 23
#define LOCK_BUTTON_PIN 19
#define UNLOCK_BUTTON_PIN 18

// POWER DELAYS
#define POWER_ON_DELAY 500
#define POWER_OFF_DELAY 200

// BLE Service UUIDs for pairing mode - Health/Fitness device
#define DEVICE_INFO_SERVICE_UUID    "180A"  // Device Information Service
#define BATTERY_SERVICE_UUID        "180F"  // Battery Service  
#define HEART_RATE_SERVICE_UUID     "180D"  // Heart Rate Service (makes it look like fitness tracker)
#define CHARACTERISTIC_UUID         "2A37"  // Heart Rate Measurement

// ========================================
// EEPROM STRUCTURE
// ========================================
#define MAGIC_BYTES_ADDR 0
#define DEVICE_COUNT_ADDR 4
#define DEVICES_START_ADDR 8
#define DEVICE_ENTRY_SIZE 32  // 16 bytes IRK + 16 bytes name
#define MAGIC_VALUE 0xDEADBEEF

// ========================================
// GLOBAL VARIABLES
// ========================================

// System state
enum SystemMode {
    MODE_PAIRING,
    MODE_KEYLESS
};
volatile SystemMode currentMode = MODE_PAIRING;

// BLE objects
BLEServer* pServer = NULL;
BLEScan* pBLEScan = NULL;

// Pairing state
unsigned long pairingStartTime = 0;

// ========================================
// LED CONTROL FUNCTIONS
// ========================================

class LEDController {
public:
    LEDController(int pin) : pin(pin) {}

    void init() {
        pinMode(pin, OUTPUT);
        set(false);
        lastBlink = 0;
    }

    void set(bool state) {
        digitalWrite(pin, state);
        ledState = state;
    }

    void blink(unsigned long interval) {
        if (millis() - lastBlink >= interval) {
            set(!ledState);
            lastBlink = millis();
        }
    }

    void blinkPattern(int count, unsigned long onTime, unsigned long offTime) {
        for (int i = 0; i < count; i++) {
            set(true);
            delay(onTime);
            set(false);
            if (i < count - 1) delay(offTime);
        }
    }

private:
    int pin;
    bool ledState = false;
    unsigned long lastBlink = 0;
};

// ========================================
// EEPROM FUNCTIONS
// ========================================

class DeviceStorage {
public:
    void init() {
        EEPROM.begin(EEPROM_SIZE);
    }

    void load() {
        Serial.println("📖 Loading devices from EEPROM...");

        uint32_t magic = EEPROM.readULong(MAGIC_BYTES_ADDR);
        if (magic != MAGIC_VALUE) {
            Serial.println("❌ No valid device data found");
            numKnownDevices = 0;
            return;
        }

        numKnownDevices = EEPROM.readInt(DEVICE_COUNT_ADDR);
        if (numKnownDevices < 0 || numKnownDevices > MAX_DEVICES) {
            Serial.println("❌ Invalid device count in EEPROM");
            numKnownDevices = 0;
            return;
        }

        for (int i = 0; i < numKnownDevices; i++) {
            int addr = DEVICES_START_ADDR + (i * DEVICE_ENTRY_SIZE);

            for (int j = 0; j < 16; j++) {
                knownDevices[i].irk[j] = EEPROM.readByte(addr + j);
            }
            for (int j = 0; j < 16; j++) {
                knownDevices[i].name[j] = EEPROM.readByte(addr + 16 + j);
            }
            knownDevices[i].name[15] = '\0'; // Ensure null termination
        }

        Serial.printf("✅ %d devices loaded:\n", numKnownDevices);
        for (int i = 0; i < numKnownDevices; i++) {
            Serial.printf("  %d: %s - ", i + 1, knownDevices[i].name);

            for (int j = 0; j < 16; j++) {
                if (knownDevices[i].irk[j] < 16) Serial.print("0");
                Serial.print(knownDevices[i].irk[j], HEX);
            }
            Serial.println();
        }

        return;
    }

    bool add(const uint8_t* irk, const char* name) {
        if (numKnownDevices >= MAX_DEVICES) {
            Serial.println("❌ Maximum device limit reached!");
            return false;
        }

        // Check duplicate
        for (int i = 0; i < numKnownDevices; i++) {
            if (memcmp(knownDevices[i].irk, irk, 16) == 0) {
                Serial.printf("⚠️ Device %s already known, skipping...\n", name);
                return false;
            }
        }

        // Add entry
        memcpy(knownDevices[numKnownDevices].irk, irk, 16);
        strncpy(knownDevices[numKnownDevices].name, name, 15);
        knownDevices[numKnownDevices].name[15] = '\0';

        numKnownDevices++;

        Serial.printf("✅ Added device: %s\n", name);
        save();

        return true;
    }

    int count() const {
        return numKnownDevices;
    }

    void getIRK(int index, uint8_t* outIRK) const {
        if (index >= 0 && index < numKnownDevices){
            memcpy(outIRK, knownDevices[index].irk, 16);
        }
    }

    void getName(int index, char* outName) const {
        if (index >= 0 && index < numKnownDevices){
            strncpy(outName, knownDevices[index].name, 16);
        }
    }

private:
    struct Device {
        uint8_t irk[16];
        char name[16];
    };

    Device knownDevices[MAX_DEVICES];
    int numKnownDevices = 0;

    void save() {
        Serial.println("💾 Saving devices to EEPROM...");

        EEPROM.writeULong(MAGIC_BYTES_ADDR, MAGIC_VALUE);
        EEPROM.writeInt(DEVICE_COUNT_ADDR, numKnownDevices);

        for (int i = 0; i < numKnownDevices; i++) {
            int addr = DEVICES_START_ADDR + (i * DEVICE_ENTRY_SIZE);

            for (int j = 0; j < 16; j++) {
                EEPROM.writeByte(addr + j, knownDevices[i].irk[j]);
            }
            for (int j = 0; j < 16; j++) {
                EEPROM.writeByte(addr + 16 + j, knownDevices[i].name[j]);
            }
        }

        EEPROM.commit();
        Serial.printf("✅ %d devices saved to EEPROM!\n", numKnownDevices);
    }
};
DeviceStorage storage;

// ========================================
// CRYPTO FUNCTIONS
// ========================================

void aes128_ecb_fast(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]) {
    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    mbedtls_aes_setkey_enc(&ctx, key, 128);
    mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, in, out);
    mbedtls_aes_free(&ctx);
}

int verifyRPA(const uint8_t* rpaAddress, uint8_t numDevices) {
    if ((rpaAddress[0] & 0xC0) != 0x40) {
        return -1; // Not a valid RPA
    }
    
    uint8_t prand[3] = {rpaAddress[0], rpaAddress[1], rpaAddress[2]};
    uint8_t rpaHash[3] = {rpaAddress[3], rpaAddress[4], rpaAddress[5]};
    
    uint8_t input[16] = {0};
    input[13] = prand[0];
    input[14] = prand[1];
    input[15] = prand[2];
    uint8_t irk[16];
    
    for (int deviceIndex = 0; deviceIndex < numDevices; deviceIndex++) {
        uint8_t aesResult[16];
        storage.getIRK(deviceIndex, irk);
        aes128_ecb_fast(irk, input, aesResult);
        
        uint8_t reversedResult[16];
        for (int i = 0; i < 16; i++) {
            reversedResult[i] = aesResult[15 - i];
        }
        
        uint8_t computedHash[3] = {reversedResult[0], reversedResult[1], reversedResult[2]};
        
        bool matches = (computedHash[0] == rpaHash[2] && 
                       computedHash[1] == rpaHash[1] && 
                       computedHash[2] == rpaHash[0]);
        
        if (matches) {
            return deviceIndex;
        }
    }
    
    return -1;
}

// ========================================
// KEYLESS SYSTEM FUNCTIONS
// ========================================

class LockController {
public:
    LockController(int lockPin, int unlockPin, int keyPowerPin)
        : lockPin(lockPin), unlockPin(unlockPin), keyPowerPin(keyPowerPin) {}

    void init() {
        pinMode(lockPin, INPUT);   // Set   lock Pin to a high impedance state
        pinMode(unlockPin, INPUT); // Set unlock Pin to a high impedance state
        pinMode(keyPowerPin, OUTPUT);
    
        digitalWrite(lockPin, LOW);
        digitalWrite(unlockPin, LOW);
        digitalWrite(keyPowerPin, LOW);

        stateUnlocked = true;
        lock(); // start locked
    }

    void lock() {
        if (!stateUnlocked) {
            //Serial.println("Already locked");
            return;
        }
        digitalWrite(lockPin, LOW);
        pinMode(lockPin, OUTPUT);
        delay(100);
        pinMode(lockPin, INPUT);
        Serial.println("🔒 Lock triggered");
        stateUnlocked = false;
        deactivateKeyPower();
    }

    void unlock() {
        if (stateUnlocked) {
            //Serial.println("Already unlocked");
            return;
        }
        activateKeyPower();
        digitalWrite(unlockPin, LOW);
        pinMode(unlockPin, OUTPUT);
        delay(100);
        pinMode(unlockPin, INPUT);
        Serial.println("🔓 Unlock triggered");
        stateUnlocked = true;
    }

    bool isUnlocked() const {
        return stateUnlocked;
    }

private:
    int lockPin;
    int unlockPin;
    int keyPowerPin;
    bool stateUnlocked = false;

    void activateKeyPower() {
        digitalWrite(keyPowerPin, HIGH);
        Serial.println("🔌 Key power activated");
        delay(POWER_ON_DELAY);
    }

    void deactivateKeyPower() {
        delay(POWER_OFF_DELAY);
        digitalWrite(keyPowerPin, LOW);
        Serial.println("🔌 Key power deactivated");
    }
};


class BLEPresenceMonitor {
public:
    BLEPresenceMonitor() {
        for (int i = 0; i < MAX_DEVICES; i++) {
            lastSeen[i] = 0;
            weakStart[i] = 0;
            present[i] = false;
        }
    }

    // Wird im BLE-Thread aufgerufen
    void updateDevice(int index, int rssi) {
        if (index < 0 || index >= MAX_DEVICES) return;

        unsigned long now = millis();

        Serial.printf("📶 Device %d RSSI: %d dBm\n", index + 1, rssi);

        portENTER_CRITICAL(&mux);

        if (rssi > RSSI_UNLOCK_THRESHOLD) {
            lastSeen[index] = now;
            weakStart[index] = 0;
            present[index] = true;
        }
        else if (rssi <= RSSI_LOCK_THRESHOLD) {
            if (weakStart[index] == 0) weakStart[index] = now;
            if (now - weakStart[index] >= ABSENCE_TIME) present[index] = false;
        }
        else {
            if (present[index]) lastSeen[index] = now;
        }

        portEXIT_CRITICAL(&mux);
    }

    // Wird im loop() aufgerufen
    void refresh() {
        unsigned long now = millis();

        portENTER_CRITICAL(&mux);
        for (int i = 0; i < MAX_DEVICES; i++) {
            if (present[i] && now - lastSeen[i] >= ABSENCE_TIME) {
                present[i] = false;
            }
        }
        portEXIT_CRITICAL(&mux);
    }

    bool anyDevicePresent() const {
        portENTER_CRITICAL(&mux);

        for (int i = 0; i < MAX_DEVICES; i++) {
            if (present[i]) {
                portEXIT_CRITICAL(&mux);
                return true;
            }
        }

        portEXIT_CRITICAL(&mux);
        return false;
    }

private:
    // Schwellwerte
    static constexpr int RSSI_UNLOCK_THRESHOLD = -65;
    static constexpr int RSSI_LOCK_THRESHOLD  = -90;
    static constexpr unsigned long ABSENCE_TIME = 5000; // 5s

    // Daten
    unsigned long lastSeen[MAX_DEVICES];     // Zeitpunkt wann zuletzt stark sichtbar
    unsigned long weakStart[MAX_DEVICES];    // Zeitpunkt wann schwach oder unsichtbar wurde
    bool present[MAX_DEVICES];               // aktueller Präsenzzustand (true/false)

    // Thread-Sicherheit
    mutable portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
};

BLEPresenceMonitor presenceMonitor;

// ========================================
// BLE PAIRING CLASSES
// ========================================

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Serial.println("📱 Device connected for pairing!");
    }

    void onDisconnect(BLEServer* pServer) {
        Serial.println("📱 Device disconnected!");
        delay(500);
        if (currentMode == MODE_PAIRING) {
            pServer->startAdvertising();
            Serial.println("🔄 Restarting advertising...");
        }
    }
};

class MySecurity : public BLESecurityCallbacks {
    uint32_t onPassKeyRequest() {
        Serial.println("📱 iPhone requesting passkey...");
        return 123456;
    }

    void onPassKeyNotify(uint32_t pass_key) {
        Serial.printf("=== PASSKEY NOTIFY: %d ===\n", pass_key);
    }

    bool onConfirmPIN(uint32_t pass_key) {
        Serial.printf("=== CONFIRM PIN: %d ===\n", pass_key);
        return (pass_key == 123456); // Only accept our fixed PIN
    }

    bool onSecurityRequest() {
        Serial.println("=== SECURITY REQUEST ===");
        return true;
    }

    void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
        Serial.println("=======================================");
        Serial.println("=== AUTHENTICATION COMPLETE ===");
        
        if (cmpl.success) {
            Serial.println("🎉 PAIRING SUCCESSFUL!");
            
            esp_bd_addr_t bda;
            memcpy(bda, cmpl.bd_addr, 6);
            
            Serial.print("📱 iPhone Address: ");
            for (int i = 0; i < 6; i++) {
                if (bda[i] < 16) Serial.print("0");
                Serial.print(bda[i], HEX);
                if (i < 5) Serial.print(":");
            }
            Serial.println();
            
            // Extract IRK
            int dev_num = esp_ble_get_bond_device_num();
            if (dev_num > 0) {
                esp_ble_bond_dev_t *bond_device_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
                esp_ble_get_bond_device_list(&dev_num, bond_device_list);
                
                for (int i = 0; i < dev_num; i++) {
                    bool isCurrentDevice = true;
                    for (int j = 0; j < 6; j++) {
                        if (bond_device_list[i].bd_addr[j] != bda[j]) {
                            isCurrentDevice = false;
                            break;
                        }
                    }
                    
                    if (isCurrentDevice) {
                        // Create device name
                        char deviceName[16];
                        snprintf(deviceName, sizeof(deviceName), "Device_%02d", storage.count() + 1);
                        
                        // Fix IRK byte order - ESP32 BLE stack returns IRK in reverse order
                        uint8_t correctedIRK[16];
                        for (int k = 0; k < 16; k++) {
                            correctedIRK[k] = bond_device_list[i].bond_key.pid_key.irk[15 - k];
                        }
                        
                        Serial.println("🔍 Original IRK (as received):");
                        for (int k = 0; k < 16; k++) {
                            if (bond_device_list[i].bond_key.pid_key.irk[k] < 16) Serial.print("0");
                            Serial.print(bond_device_list[i].bond_key.pid_key.irk[k], HEX);
                        }
                        Serial.println();
                        
                        Serial.println("🔄 Corrected IRK (byte-reversed):");
                        for (int k = 0; k < 16; k++) {
                            if (correctedIRK[k] < 16) Serial.print("0");
                            Serial.print(correctedIRK[k], HEX);
                        }
                        Serial.println();
                        
                        // Add device with corrected IRK
                        storage.add(correctedIRK, deviceName);
                        
                        Serial.println("🔑 IRK successfully extracted and saved!");
                        Serial.println("🔄 Restarting ESP32 for clean BLE initialization...");
                        delay(2000);
                        ESP.restart(); // Clean restart for proper BLE mode switching
                        break;
                    }
                }
                free(bond_device_list);
            }
        } else {
            Serial.printf("❌ PAIRING FAILED! Reason: %d\n", cmpl.fail_reason);
        }
        Serial.println("=======================================");
    }
};

// ========================================
// BLE KEYLESS SCAN CALLBACK
// ========================================

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        char deviceName[16];
        uint8_t irk[16];
        
        esp_bd_addr_t* addr = advertisedDevice.getAddress().getNative();
        
        if (((*addr)[0] & 0xC0) == 0x40) { // RPA check
            int matchedDevice = verifyRPA((uint8_t*)(*addr), storage.count());

            // Check if IRK matches a known device
            if (matchedDevice >= 0) {
                int rssi = advertisedDevice.getRSSI();
                storage.getName(matchedDevice, deviceName);

                presenceMonitor.updateDevice(matchedDevice, rssi);
            }
        }
    }
};

// ========================================
// MODE SWITCHING FUNCTIONS
// ========================================

void startPairingMode() {
    currentMode = MODE_PAIRING;
    pairingStartTime = millis();
    
    Serial.println("🔵 Starting BLE pairing mode...");
    
    // Initialize BLE for pairing
    BLEDevice::init("ESPKV7 Tracker");
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    BLEDevice::setSecurityCallbacks(new MySecurity());
    
    // Create server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    // Create Device Information Service
    BLEService *pDeviceInfoService = pServer->createService(DEVICE_INFO_SERVICE_UUID);
    BLECharacteristic *pManufacturerChar = pDeviceInfoService->createCharacteristic(
        "2A29", BLECharacteristic::PROPERTY_READ
    );
    pManufacturerChar->setValue("ESPKV7 Tech");
    
    BLECharacteristic *pModelChar = pDeviceInfoService->createCharacteristic(
        "2A24", BLECharacteristic::PROPERTY_READ
    );
    pModelChar->setValue("KeyTracker Pro");
    
    BLECharacteristic *pFirmwareChar = pDeviceInfoService->createCharacteristic(
        "2A26", BLECharacteristic::PROPERTY_READ
    );
    pFirmwareChar->setValue("v7.0");
    pDeviceInfoService->start();
    
    // Create Battery Service
    BLEService *pBatteryService = pServer->createService(BATTERY_SERVICE_UUID);
    BLECharacteristic *pBatteryChar = pBatteryService->createCharacteristic(
        "2A19", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pBatteryChar->addDescriptor(new BLE2902());
    uint8_t batteryLevel = 85; // Fake 85% battery
    pBatteryChar->setValue(&batteryLevel, 1);
    pBatteryService->start();
    
    // Create Heart Rate Service (main pairing service)
    BLEService *pHeartRateService = pServer->createService(HEART_RATE_SERVICE_UUID);
    BLECharacteristic *pHeartRateChar = pHeartRateService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pHeartRateChar->addDescriptor(new BLE2902());
    uint8_t heartRateData[2] = {0x00, 75}; // Fake 75 BPM
    pHeartRateChar->setValue(heartRateData, 2);
    pHeartRateService->start();
    
    // Configure security
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_KBDISP;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
    
    // Start advertising as fitness tracker
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    
    // Configure for iPhone fitness device compatibility
    pAdvertising->addServiceUUID(HEART_RATE_SERVICE_UUID);  // Heart rate service first
    pAdvertising->addServiceUUID(BATTERY_SERVICE_UUID);     // Battery service
    pAdvertising->addServiceUUID(DEVICE_INFO_SERVICE_UUID); // Device info service
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x20);   // 20ms min interval
    pAdvertising->setMaxPreferred(0x40);   // 40ms max interval
    
    // Set advertisement data with fitness tracker appearance
    BLEAdvertisementData advertisementData;
    advertisementData.setName("ESPKV7 Tracker");
    advertisementData.setAppearance(0x0340); // Generic: Heart Rate Sensor
    advertisementData.setCompleteServices(BLEUUID(HEART_RATE_SERVICE_UUID));
    advertisementData.setFlags(0x06); // General discoverable mode + BR/EDR not supported
    
    pAdvertising->setAdvertisementData(advertisementData);
    
    // Set scan response data with additional services
    BLEAdvertisementData scanResponseData;
    scanResponseData.setName("ESPKV7 Tracker");
    scanResponseData.setPartialServices(BLEUUID(BATTERY_SERVICE_UUID));
    pAdvertising->setScanResponseData(scanResponseData);
    
    BLEDevice::startAdvertising();
    
    Serial.println("🚀 ESPKV7 Tracker advertising started!");
    Serial.println("📱 Go to iPhone Settings > Bluetooth and look for 'ESPKV7 Tracker' as fitness device");
    if (storage.count() > 0) {
        Serial.printf("⏱️ 30s window to add more devices (or automatic keyless mode after timeout)\n");
    } else {
        Serial.printf("⏱️ Pairing window: %d seconds\n", PAIRING_TIMEOUT_MS / 1000);
    }
}

void startKeylessMode() {
    currentMode = MODE_KEYLESS;
    
    Serial.println("🔐 Starting keyless mode...");

    BLEDevice::init("ESP32 Scanner");
    pBLEScan = BLEDevice::getScan();

    // Attach callback, *mit* Duplikaten
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);

    // Active scan gives RSSI + scan response
    pBLEScan->setActiveScan(true);

    // Interval & window: far better performance for iPhone detection
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(80);

    // Start continuous scanning
    Serial.println("▶️ Starting continuous scan...");
    pBLEScan->start(0, nullptr, false);   // 0 = infinite scanning
}

// ========================================
// MAIN SETUP
// ========================================


// LED controller
LEDController led(LED_PIN);

// Lock controller
LockController lockCtrl(LOCK_BUTTON_PIN, UNLOCK_BUTTON_PIN, KEY_POWER_PIN);



void setup() {
    Serial.begin(115200);
    
    Serial.println("=======================================");
    Serial.println("ESP32: BLE Keyless System");
    Serial.println("=======================================");
    
    // Initialize watchdog
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);
    Serial.println("🐕 Watchdog enabled (30s timeout)");

    // Initialize lock controller
    lockCtrl.init();

    // Initialize LED
    led.init();
    
    // Initialize storage
    storage.init();
    storage.load();
    
    if (storage.count() > 0) {
        if (esp_reset_reason() == ESP_RST_SW) {
            Serial.println("🔐 Software reset detected - starting keyless mode directly...");
            startKeylessMode();
        } else {
            Serial.println("🔄 Power-on boot - starting 30s pairing window to add more devices...");
            startPairingMode();
        }
    } else {
        Serial.println("❌ No known devices found -> Starting pairing mode");
        startPairingMode();
    }
}

// ========================================
// MAIN LOOP
// ========================================

void loop() {
    esp_task_wdt_reset();
    
    if (currentMode == MODE_PAIRING) {
        // Check pairing timeout
        if (millis() - pairingStartTime >= PAIRING_TIMEOUT_MS) {
            if (storage.count() > 0) {
                Serial.println("⏰ Pairing timeout - restarting ESP32 for clean keyless mode");
                ESP.restart();
            } else {
                Serial.println("⏰ Pairing timeout - no devices paired, restarting pairing...");
                pairingStartTime = millis(); // Restart pairing window
            }
        }
        led.blink(200);
        
    } 
    else 
    {
        // MODE_KEYLESS
        presenceMonitor.refresh();

        if (presenceMonitor.anyDevicePresent()) {
            lockCtrl.unlock();
        } else {
            lockCtrl.lock();
        }
        led.set(lockCtrl.isUnlocked());
        delay(100);
    }
}
