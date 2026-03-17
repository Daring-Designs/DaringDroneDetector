# Daring Drone Detector

An ESP32-based portable drone signal scanner that detects drones via three methods simultaneously:

- **ELRS LoRa** — CAD sweep on 900MHz + 2.4GHz FHSS channels with packet verification
- **WiFi Remote ID** — Promiscuous mode parsing of DJI proprietary and ASTM F3411 standard beacons
- **BLE OpenDroneID** — Bluetooth LE scan for ASTM OpenDroneID advertisements (UUID 0xFFFA)

Built around a Heltec Vision Master E290 (ESP32-S3 + 2.9" e-ink display) with a Waveshare Core1121-XF dual-band LoRa radio.

## 3D Printed Case

Printable enclosure available on Printables: [TODO: Add Printables link](https://www.printables.com/)

## Hardware / Bill of Materials

| Component | Description |
|-----------|-------------|
| Heltec Vision Master E290 | ESP32-S3 dev board with 2.9" e-ink display |
| Waveshare Core1121-XF | LR1121 dual-band LoRa radio (900MHz + 2.4GHz) |
| HGLRC M100 Mini GPS | UART GPS module |
| Passive piezo buzzer | With transistor driver to 5V |
| 5-way navigation switch | For UI navigation |
| LiPo battery | 3.7V single cell |
| 915MHz antenna | For ELRS LoRa reception |

### Wiring

| GPIO | Function | GPIO | Function |
|------|----------|------|----------|
| 3 | LR1121 CS | 38 | GPS TX |
| 9 | LR1121 SCK | 39 | GPS RX |
| 10 | LR1121 MOSI | 45 | Buzzer |
| 11 | LR1121 MISO | 8 | NAV UP |
| 17 | LR1121 BUSY | 12 | NAV DOWN |
| 40 | LR1121 DIO9 | 13 | NAV LEFT |
| 42 | LR1121 RESET | 14 | NAV RIGHT |
| 7 | VBAT read | 41 | NAV CENTER |

E-ink display pins (1, 2, 4, 5, 6) are managed by the heltec-eink-modules library.

## Software Setup

### 1. Install Arduino IDE

Download and install [Arduino IDE 2.x](https://www.arduino.cc/en/software).

### 2. Add Heltec ESP32 Board Package

1. Open **File > Preferences**
2. Add this URL to **Additional Board Manager URLs**:
   ```
   https://resource.heltec.cn/download/package_heltec_esp32_index.json
   ```
3. Open **Tools > Board > Boards Manager**, search for **Heltec ESP32**, and install it

### 3. Install Required Libraries

Open **Tools > Manage Libraries** and install:

- **RadioLib** — LoRa radio driver (LR1121 support)
- **TinyGPSPlus** — NMEA GPS parsing
- **heltec-eink-modules** — E-ink display driver for Vision Master E290

The following are included with the ESP32 Arduino core (no separate install needed):
- BLEDevice, WiFi, esp_wifi

### 4. Select Board & Upload

1. Open `DaringDroneDetector.ino` in Arduino IDE
2. Select **Tools > Board > Heltec Vision Master E290**
3. Select the correct serial port under **Tools > Port**
4. Click **Upload**

### Using arduino-cli

```bash
arduino-cli compile --fqbn Heltec-esp32:esp32:heltec_vision_master_e290 DaringDroneDetector.ino
arduino-cli upload --fqbn Heltec-esp32:esp32:heltec_vision_master_e290 -p /dev/ttyUSB0 DaringDroneDetector.ino
```

## Serial Commands

Connect at 115200 baud. Available commands:

| Command | Description |
|---------|-------------|
| `status` | Show scanner status and detection counts |
| `dump` | Print all detections in human-readable format |
| `json` | Export all detections as JSON |
| `clear` | Clear the detection buffer |
| `help` | Show available commands |

## How It Works

The firmware uses a non-blocking cooperative multitasking pattern. The main `loop()` calls task functions that each do a small unit of work and return immediately:

1. **ELRS Detection** — Phase 1: CAD sweep across 40 FHSS channels detects LoRa energy. When enough hits accumulate, Phase 2 attempts implicit-header packet receive across all known ELRS 900MHz air rates for confirmation.
2. **WiFi Remote ID** — Promiscuous mode callback inspects management frames for Vendor Specific IEs. Supports DJI proprietary (OUI `26:37:12`) and ASTM standard (OUI `FA:0B:BC`).
3. **BLE OpenDroneID** — 1-second BLE scans look for OpenDroneID service UUID. WiFi promiscuous mode is paused during BLE scans to avoid 2.4GHz radio contention.
4. **Display** — E-ink refreshes every 5 seconds using a partial/full refresh cycle (20 partials then 1 full to prevent ghosting).

## License

This project is licensed under [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/). See [LICENSE](LICENSE) for details.
