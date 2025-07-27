# Usage Examples
## ESP32 Dynamic iPhone Keyless System v7

🎯 **Real-world usage scenarios and step-by-step guides**

## 🚗 Basic Car Integration

### Scenario: Simple Car Door Lock Control

**Hardware Setup:**
```
ESP32 Pin 23 → NPN Transistor Base → Car Key Fob Power
ESP32 Pin 19 → NPN Transistor Base → Car Key Fob Lock Button  
ESP32 Pin 18 → NPN Transistor Base → Car Key Fob Unlock Button
ESP32 Pin 2  → LED + 330Ω Resistor → Ground
ESP32 GND   → Transistor Emitters → Car Key Fob Ground
```

**Expected Behavior:**
```
iPhone approaches (15m) → LED turns ON → Car unlocks after 500ms
iPhone leaves (15m)    → LED turns OFF → Car locks after 10s
```

### Example Serial Output:
```
🔵 ESP32 Dynamic Keyless System v7
🔍 Reset reason: 1
✅ Found 1 known devices
  1: Device_01
🔄 Power-on boot - starting 30s pairing window...
📱 Go to iPhone Settings > Bluetooth and look for 'ESPKV7 Tracker'
⏰ Pairing timeout - restarting ESP32 for clean keyless mode

🔍 Reset reason: 3  
🔐 Software reset detected - starting keyless mode directly...
📡 BLE scanner ready - parameters set successfully
✅ Keyless system ready - monitoring for known devices

🔍 Scanning for known devices...
📱 Device_01 detected (RSSI: -45)
🔌 Key power activated
🔓 Welcome! Activating unlock sequence...
📡 Scan completed, found 28 devices
🔓 Unlock triggered

🔍 Scanning for known devices...
📱 Device_01 detected (RSSI: -43)
📡 Scan completed, found 31 devices

🔍 Scanning for known devices...
📡 Scan completed, found 29 devices
📱 Device_01 timeout
📱 All phones gone (device timeout)
🔒 Lock scheduled
🔒 Lock triggered
🔌 Key power deactivated
```

## 📱 iPhone Setup Process

### Step-by-Step Pairing Guide

**Step 1: Fresh ESP32 Boot**
```
Power on ESP32 → Wait for pairing mode announcement
Expected output: "🔄 Starting pairing mode - connect your iPhone now!"
LED: Slow blinking (1 second intervals)
```

**Step 2: iPhone Connection**
```
1. Open iPhone Settings
2. Tap "Bluetooth"  
3. Look for "ESPKV7 Tracker" in "Other Devices"
4. Tap "ESPKV7 Tracker"
5. iPhone shows: "Bluetooth Pairing Request"
6. Enter PIN: 123456
7. Tap "Pair"
```

**Step 3: IRK Extraction**
```
ESP32 Output:
=== DEVICE CONNECTED ===
📱 iPhone connected for pairing!
=== AUTHENTICATION COMPLETE ===
🎉 PAIRING SUCCESSFUL!
📱 iPhone Address: 48:3B:54:B8:F2:01
🔍 Original IRK (as received):
049001E5A5FECF706480FF1AB5FE87F9
🔄 Corrected IRK (byte-reversed):
F987FEB51AFF806470CFFEA5E5019004
✅ Added device: Device_01
💾 Saving devices to EEPROM...
✅ 1 devices saved to EEPROM!
🔑 IRK successfully extracted and saved!
🔄 Restarting ESP32 for clean BLE initialization...
```

**Step 4: Automatic Restart to Keyless Mode**
```
ESP32 restarts automatically and enters keyless mode
iPhone will be detected and proximity control begins immediately
```

## 🏠 Multiple Device Setup

### Scenario: Family Car with 3 iPhones

**Adding Additional Devices:**
```
1. Power cycle ESP32 (reset button or power off/on)
2. System shows existing devices during 30s pairing window:
   ✅ Found 2 known devices
     1: Device_01  
     2: Device_02
   🔄 Starting 30s pairing window to add more devices...

3. Connect third iPhone during this 30-second window
4. System extracts IRK and saves as Device_03
5. After timeout, system restarts into keyless mode with all 3 devices
```

**Expected Multi-Device Operation:**
```
📱 Device_01 detected (RSSI: -52)
📱 Device_02 detected (RSSI: -48)  
🔓 Welcome! Activating unlock sequence...

[Later when all leave]
📱 Device_01 timeout
📱 Device_02 timeout  
📱 All phones gone (device timeout)
🔒 Lock scheduled
🔒 Lock triggered
```

## 🔧 Configuration Examples

### Adjusting Detection Range

**Close Range (5-8 meters):**
```cpp
const int RSSI_THRESHOLD = -60; // Stronger signal required
```

**Long Range (20-25 meters):**
```cpp  
const int RSSI_THRESHOLD = -90; // Weaker signal accepted
```

**Testing RSSI Values:**
```
📱 Device_01 detected (RSSI: -42)  // Very close (1-2m)
📱 Device_01 detected (RSSI: -55)  // Near (5-8m) 
📱 Device_01 detected (RSSI: -70)  // Medium (10-15m)
📱 Device_01 detected (RSSI: -85)  // Far (18-22m)
```

### Timing Adjustments

**Quick Response Setup:**
```cpp
const unsigned long PROXIMITY_TIMEOUT = 5000;  // 5s timeout
const unsigned long UNLOCK_DELAY = 250;        // 250ms unlock delay
```

**Conservative Setup:**
```cpp
const unsigned long PROXIMITY_TIMEOUT = 15000; // 15s timeout  
const unsigned long UNLOCK_DELAY = 1000;       // 1s unlock delay
```

## 🛠️ Hardware Variations

### Option 1: Direct Key Fob Control
```
ESP32 → Transistor → Key Fob Buttons
Pros: Simple, works with any car
Cons: Requires key fob modification
```

**Wiring:**
```
ESP32 Pin 19 → 2N2222 Base (1kΩ resistor)
2N2222 Collector → Key Fob Lock Button  
2N2222 Emitter → Ground
ESP32 Pin 18 → Second 2N2222 → Key Fob Unlock Button
```

### Option 2: CAN Bus Integration
```
ESP32 → CAN Transceiver → Car CAN Bus
Pros: Professional integration
Cons: Requires car-specific CAN knowledge
```

**Additional Components:**
```cpp
#include <CAN.h>
// Send CAN message for lock/unlock
CAN.beginPacket(0x123);  // Car-specific ID
CAN.write(0x01);         // Lock command
CAN.endPacket();
```

### Option 3: Relay Control
```
ESP32 → Relay Module → Car Central Locking
Pros: No key fob modification needed
Cons: Requires access to car wiring
```

## 🐛 Troubleshooting Examples

### Problem: iPhone Not Detected

**Diagnostic Steps:**
```
1. Check serial output for scan results:
   📡 Scan completed, found 0 devices  ← Problem: No devices found
   📡 Scan completed, found 25 devices ← Good: Devices found

2. Verify RSSI threshold:
   📱 Device_01 weak signal (RSSI: -95) ← Below -80 threshold

3. Check IRK storage:
   ✅ 1 devices loaded from EEPROM:
     1: Device_01 - F987FEB51AFF806470CFFEA5E5019004 ← Good: IRK loaded
```

**Solutions:**
```cpp
// Increase detection range
const int RSSI_THRESHOLD = -90; // Was -80

// Increase scan frequency  
const int SCAN_TIME = 5; // Was 3 seconds

// Check iPhone Bluetooth is ON and not in airplane mode
```

### Problem: BLE Scan Error 259

**Error Pattern:**
```
🔍 Scanning for known devices...
[ 37006][E][BLEScan.cpp:402] start(): esp_ble_gap_set_scan_params: err: 259
📡 Scan completed, found 0 devices
```

**Solution (Already Implemented):**
```
This error is resolved by the auto-restart mechanism.
The system automatically restarts after pairing timeout to ensure
clean BLE scanner initialization.
```

### Problem: Endless Restart Loop

**Error Pattern:**
```
⏰ Pairing timeout - restarting ESP32 for clean keyless mode
[Restart]
🔄 Starting 30s pairing window to add more devices...
⏰ Pairing timeout - restarting ESP32 for clean keyless mode  
[Restart]
[Loop continues...]
```

**Solution (Already Implemented):**
```
System now checks reset reason:
🔍 Reset reason: 1 (Power-on) → Pairing mode
🔍 Reset reason: 3 (Software)  → Keyless mode directly
```

## 📊 Performance Examples

### Memory Usage During Operation
```
Advanced Memory Usage:
RAM:   [=         ]  12.1% (used 39,572 bytes from 327,680 bytes)
Flash: [========= ]  87.2% (used 1,142,765 bytes from 1,310,720 bytes)

Available for future features:
- RAM: ~288KB available
- Flash: ~168KB available
```

### Detection Performance Metrics
```
Typical Performance:
- Detection Range: 15-20 meters (RSSI -80)
- Response Time: 2-4 seconds from approach to unlock
- Lock Delay: 10 seconds after last detection  
- Power Usage: ~80mA during scanning
- Scan Rate: Every 3 seconds + 500ms processing
```

### Multi-Device Handling
```
With 3 devices active:
📱 Device_01 detected (RSSI: -45)
📱 Device_02 detected (RSSI: -67)  
📱 Device_03 detected (RSSI: -72)
🔓 Welcome! Activating unlock sequence...

Performance impact: Minimal
Each additional device adds ~1ms IRK verification time
Memory usage: +32 bytes per device (EEPROM)
```

## 🎯 Advanced Use Cases

### Commercial Parking Garage
```cpp
// Extended range for parking approach
const int RSSI_THRESHOLD = -95;          // 25+ meter range
const unsigned long PROXIMITY_TIMEOUT = 30000; // 30s timeout
const int MAX_DEVICES = 50;             // Support more users

// Log vehicle entries
void logVehicleEntry(const char* deviceName) {
    Serial.printf("ENTRY: %s at %lu\n", deviceName, millis());
    // Send to cloud/database
}
```

### Fleet Vehicle Management
```cpp
// Multiple vehicle profiles
struct VehicleConfig {
    char name[16];
    int rssi_threshold;
    unsigned long timeout;
    uint8_t authorized_devices[10][16]; // IRKs
};

VehicleConfig fleet_vehicles[] = {
    {"Truck_01", -70, 15000, {...}},
    {"Van_02", -80, 10000, {...}},
    {"Car_03", -85, 8000, {...}}
};
```

### Home Automation Integration
```cpp
// MQTT integration for smart home
#include <WiFi.h>
#include <PubSubClient.h>

void publishProximityEvent(bool phoneNearby) {
    if (phoneNearby) {
        mqtt.publish("home/garage/presence", "OCCUPIED");
        mqtt.publish("home/lights/garage", "ON");
    } else {
        mqtt.publish("home/garage/presence", "EMPTY");  
        mqtt.publish("home/lights/garage", "OFF");
    }
}
```

---

**🎉 These examples demonstrate the flexibility and robustness of the ESP32 Dynamic iPhone Keyless System for various real-world applications!**