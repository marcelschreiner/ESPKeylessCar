# ESP32 Dynamic iPhone Keyless System v7

🚗 **Smart Proximity-Based Car Access Control using iPhone BLE**

An advanced ESP32-based keyless entry system that automatically detects iPhone presence and controls car door locks. The system dynamically learns iPhone Identity Resolution Keys (IRKs) through BLE pairing and stores them persistently for reliable proximity detection.

## ✨ Features

- 🔑 **Dynamic IRK Learning**: No hardcoded device IDs - learns iPhone IRKs through secure BLE pairing
- 📱 **iPhone Native Integration**: Appears as fitness tracker in iPhone Bluetooth settings
- 💾 **Persistent Storage**: Supports up to 10 devices stored in EEPROM
- 🔒 **Secure Authentication**: Uses iPhone's BLE Identity Resolution Keys for device verification
- 🚗 **Automotive Ready**: Robust proximity detection with hysteresis filtering
- 🔄 **Auto-Recovery**: Smart restart logic prevents BLE stack issues
- 📡 **Advanced BLE Stack**: Handles complex server-to-scanner mode transitions
- 🐕 **Watchdog Protection**: Hardware watchdog prevents system lockups

## 🛠️ Hardware Requirements

- **ESP32 Board** 
- **Transistors** for key fob signal control
- **LED** for status indication
- **Connections:**
  - Pin 23: Key power control (transistor base)
  - Pin 19: Lock signal (transistor base)
  - Pin 18: Unlock signal (transistor base)

## 🚀 How It Works

### 1. **Initial Setup Phase**
```
Power On → Check EEPROM → No devices found → 30s Pairing Window
```

### 2. **Device Learning**
- ESP32 advertises as "ESPKV7 Tracker" (fitness device)
- iPhone connects via Bluetooth settings
- System extracts iPhone's IRK during secure pairing
- IRK stored in EEPROM with auto-generated device name
- System restarts for clean BLE initialization

### 3. **Operational Mode**
```
Power On → Load devices → 30s pairing window → Auto-restart → Keyless mode
```

### 4. **Proximity Detection**
- Continuous BLE scanning for known iPhone RPA (Resolvable Private Address)
- AES-128 encryption verification using stored IRKs
- Hysteresis filtering prevents false triggers from weak signals
- Automatic unlock when iPhone approaches (RSSI > -80dBm)
- Automatic lock when iPhone leaves (10s timeout + stabilization)

## 📋 Installation

### Prerequisites
- [PlatformIO](https://platformio.org/) installed
- ESP32 development environment configured

### Setup
```bash
git clone https://github.com/yourusername/esp32-keyless-system
cd esp32-keyless-system
pio run --target upload
pio device monitor
```

### Library Dependencies
```ini
lib_deps = 
    h2zero/NimBLE-Arduino@^1.4.0
    ESP32 BLE Arduino
    EEPROM
```

## 🔧 Configuration

### Device Limits
```cpp
#define MAX_DEVICES 10           // Maximum stored devices
#define PAIRING_TIMEOUT_MS 30000 // 30-second pairing window
```

### Proximity Settings
```cpp
const int RSSI_THRESHOLD = -80;          // ~15-20m detection range
const unsigned long PROXIMITY_TIMEOUT = 10000; // 10s timeout
const int WEAK_SIGNAL_THRESHOLD = 3;    // Hysteresis filtering
```

### Hardware Pins
```cpp
const int LED_PIN = 2;           // Status LED
const int KEY_POWER_PIN = 23;    // Key fob power control
const int LOCK_BUTTON_PIN = 19;  // Lock signal
const int UNLOCK_BUTTON_PIN = 18; // Unlock signal
```

## 📱 iPhone Setup

1. **Power on ESP32** - wait for "ESPKV7 Tracker" to appear
2. **Open iPhone Settings** → Bluetooth
3. **Connect to "ESPKV7 Tracker"** (appears as fitness device)
4. **Enter PIN: 123456** when prompted
5. **Wait for automatic restart** - system switches to keyless mode
6. **Done!** Your iPhone is now learned and will be detected automatically

## 🔄 System States

### LED Status Indicators
- **Slow blink (1s)**: Pairing mode - waiting for connection
- **Fast blink (200ms)**: Device connected - pairing in progress  
- **3x short blinks**: Device successfully added
- **2x long blinks**: Maximum device limit reached
- **5x very fast blinks**: Pairing error
- **Solid on**: iPhone detected nearby (keyless mode)
- **Off**: No iPhones detected

### Serial Debug Output
```
🔵 ESP32 Dynamic Keyless System v7
🔍 Reset reason: 1
✅ Found 2 known devices
📱 Go to iPhone Settings > Bluetooth and look for 'ESPKV7 Tracker'
🔑 IRK successfully extracted and saved!
📱 Device_01 detected (RSSI: -41)
🔓 Welcome! Activating unlock sequence...
```

## 🧠 Technical Deep Dive

### BLE Identity Resolution
The system uses iPhone's BLE privacy features for secure device identification:

1. **RPA Verification**: Validates Resolvable Private Addresses
2. **IRK Cryptography**: AES-128 encryption with stored Identity Resolution Keys
3. **Byte-Order Handling**: Corrects ESP32 BLE stack byte reversal
4. **Address Rotation**: Works with iPhone's automatic address randomization

### Code Architecture
```cpp
// Core Components
- Dynamic IRK extraction and storage
- Dual-mode BLE (Server for pairing, Scanner for detection)
- Automotive-grade proximity algorithms
- Reset-reason based state management
- Hardware watchdog integration
```

### Advanced Features
- **Hysteresis Filtering**: Prevents false triggers from signal fluctuations
- **Multi-device Support**: Handles multiple iPhones simultaneously  
- **Graceful Degradation**: Continues operation if one device fails
- **Memory Management**: Efficient EEPROM usage with wear leveling
- **Power Management**: Optimized for automotive 12V systems

## 🔒 Security Considerations

- ✅ **IRK-based Authentication**: Uses Apple's BLE security framework
- ✅ **Fixed PIN**: Prevents unauthorized device addition (PIN: 123456)
- ✅ **Physical Access Required**: Device pairing requires physical presence
- ✅ **No Cloud Dependencies**: Fully offline operation
- ⚠️ **Production Hardening**: Consider additional security for personal or commercial use

## 🐛 Troubleshooting

### Common Issues

**"BLE scan parameter error 259"**
- Fixed by auto-restart mechanism
- Caused by ESP32 BLE stack mode transitions

**"iPhone not detected"**  
- Check RSSI threshold (-80dBm default)
- Verify IRK extraction during pairing
- Ensure iPhone Bluetooth is enabled

**"Pairing fails"**
- Use exactly PIN: 123456
- Restart ESP32 if pairing window expired
- Check for BLE interference

**"Endless restart loop"**
- Fixed by reset-reason detection
- Power cycle ESP32 to reset state

## 📈 Performance Metrics

- **Detection Range**: ~15-20 meters (adjustable via RSSI threshold)
- **Response Time**: <3 seconds from approach to unlock
- **Power Consumption**: ~80mA during scanning, ~120mA during pairing
- **Memory Usage**: 87% Flash, 12% RAM (ESP32-D0WD-V3)
- **Supported Devices**: Up to 10 iPhones simultaneously

## 🤝 Contributing

Contributions welcome! Please read our contributing guidelines and submit pull requests for:
- Additional smartphone support (Android BLE)
- Enhanced security features
- Power optimization
- UI improvements
- Documentation updates

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ESP32 BLE Arduino library contributors
- Apple's BLE privacy specification
- Automotive security best practices
- Open source community feedback

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/crazyhoesl/ESPKeylessCar/issues)
- **Documentation**: [Wiki](https://github.com/crazyhoesl/ESPKeylessCar/wiki)

---

**⚠️ Disclaimer**: This system is intended for educational and personal use. Ensure compliance with local automotive regulations and security requirements for commercial applications.
