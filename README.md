# Daring Drone Detector

A portable ESP32-based drone signal scanner that detects drones via three methods simultaneously:

- **ELRS LoRa** — CAD sweep on 900MHz + 2.4GHz FHSS channels with lock/track/expire slot tracking
- **WiFi Remote ID** — Promiscuous mode parsing of DJI proprietary and ASTM F3411 standard beacons
- **BLE OpenDroneID** — Bluetooth LE scan for ASTM OpenDroneID advertisements (UUID 0xFFFA)

Two hardware builds are available:

| Build | Display | Input | Extras |
|-------|---------|-------|--------|
| [Heltec Vision Master E290](Heltec_e290/README.md) | 2.9" e-ink | 5-way nav switch | GPS, piezo buzzer, sleep/wake |
| [LilyGO T-Display S3 Pro](LillyGo_S3_Pro_LR1121/README.md) | 222×480 IPS touch | Touchscreen | I2S speaker, vibration motor, RTC, battery PMIC |

Both builds use a **Waveshare Core1121-XF** (LR1121 dual-band LoRa radio) for ELRS detection.

## 3D Printed Case

Printable enclosures available on Printables: [TODO: Add Printables link](https://www.printables.com/)

## How It Works

The firmware uses a non-blocking cooperative multitasking pattern. `loop()` calls task functions that each do a small unit of work and return immediately:

1. **ELRS Detection** — Phase 1: CAD sweep across 40 FHSS channels detects LoRa energy. When enough hits accumulate, Phase 2 attempts implicit-header packet receive across all known ELRS air rates for confirmation. Confirmed signals are assigned a tracking slot (up to 8) with lock/track/expire lifecycle.
2. **WiFi Remote ID** — Promiscuous mode callback inspects management frames for Vendor Specific IEs. Supports DJI proprietary (OUI `26:37:12`), ASTM standard (OUI `FA:0B:BC`), French DRI, Parrot, and WiFi NaN. Detected devices are tracked by MAC address.
3. **BLE OpenDroneID** — 1-second BLE scans look for OpenDroneID service UUID. WiFi promiscuous mode is paused during BLE scans to avoid 2.4GHz radio contention. Detected devices are tracked by BLE address.
4. **Detection Log** — Ring buffer of 50 entries with signal type, RSSI, frequency, identifier, GPS coordinates (E290), and timestamp.

## Serial Commands

Both builds expose the same serial interface at 115200 baud:

| Command | Description |
|---------|-------------|
| `status` | Scanner status and detection counts |
| `dump` | All detections in human-readable format |
| `json` | Export all detections as JSON |
| `clear` | Clear the detection buffer |
| `help` | Show available commands |

## Build-Specific Setup

See the README in each build's folder for hardware wiring, library requirements, board selection, and UI details:

- [Heltec_e290/README.md](Heltec_e290/README.md)
- [LillyGo_S3_Pro_LR1121/README.md](LillyGo_S3_Pro_LR1121/README.md)

## License

This project is licensed under [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/). See [LICENSE](LICENSE) for details.
