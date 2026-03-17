# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DaringDroneDetector is an ESP32-based drone signal scanner running on a Heltec Vision Master E290 (ESP32-S3 + 2.9" e-ink display). It detects drones via three methods simultaneously: ELRS LoRa (900MHz + 2.4GHz), WiFi Remote ID, and BLE OpenDroneID. All code lives in a single `DaringDroneDetector.ino` sketch.

## Build & Upload

This is an Arduino IDE project (no platformio.ini). To compile and upload:

- **Board**: Heltec Vision Master E290 (install Heltec ESP32 board package)
- **Required libraries**: RadioLib, TinyGPSPlus, heltec-eink-modules
- **Built-in libraries**: BLEDevice, WiFi, esp_wifi (from ESP32 Arduino core)
- Upload via Arduino IDE or `arduino-cli compile && arduino-cli upload`
- Board package URL: `https://resource.heltec.cn/download/package_heltec_esp32_index.json`
- FQBN: `Heltec-esp32:esp32:heltec_vision_master_e290`

There are no tests or linting configured.

## Architecture

The sketch uses a **non-blocking cooperative multitasking** pattern. `loop()` calls task functions that each do a small unit of work and return immediately:

- `gps_task()` — reads available UART bytes, feeds TinyGPSPlus
- `cad_scan_task()` — ELRS detection: Phase 1 (CAD sweep across 40 channels) triggers Phase 2 (implicit header packet verification across 8 air rates)
- `wifi_scan_task()` — hops WiFi channels (weighted toward ch6 for NaN Remote ID)
- `ble_scan_task()` — non-blocking 1s BLE scan with callback for OpenDroneID (UUID 0xFFFA)
- `nav_task()` — 5-way switch input handling with debounce and long-press detection
- `buzzer_task()` — non-blocking beep pattern state machine
- Display refresh every 5s with partial/full refresh cycle (20 partials then 1 full to prevent ghosting)

## Hardware & Pin Map

**Radio**: Waveshare Core1121-XF (LR1121, dual-band) on FSPI bus. E-ink display uses HSPI (separate bus, no contention).

| GPIO | Function | GPIO | Function |
|------|----------|------|----------|
| 3 | LR1121 CS | 38 | GPS TX |
| 9 | LR1121 SCK | 39 | GPS RX |
| 10 | LR1121 MOSI | 45 | Buzzer |
| 11 | LR1121 MISO | 8 | NAV UP |
| 17 | LR1121 BUSY | 12 | NAV DOWN |
| 40 | LR1121 DIO9 | 13 | NAV LEFT |
| 42 | LR1121 RESET | 14 | NAV RIGHT |
| 7 | VBAT read (reserved) | 41 | NAV CENTER |

E-ink pins (1,2,4,5,6) managed by heltec-eink-modules library.

## Key Technical Details

**ELRS Detection**: Two-phase approach. CAD sweep detects LoRa energy on 40 FHSS channels (903.5-926.9 MHz). When 4+ hits on 3+ channels in a 5s window, verification phase tries implicit header receive across all known ELRS 900MHz air rates (250Hz through 25Hz). Critical settings: `setCRC(false)` (ELRS disables hardware CRC), correct coding rate per mode (CR 4/7 or 4/8), `implicitHeader(payloadLen)`, sync word 0x12.

**WiFi Remote ID**: Promiscuous mode callback parses management frames for Vendor Specific IEs (tag 221). DJI proprietary OUI is `26:37:12`, ASTM standard is `FA:0B:BC`. DJI drones only broadcast Remote ID when armed/flying with GPS lock.

**BLE/WiFi Coexistence**: ESP32 shares the 2.4GHz radio. WiFi promiscuous mode is paused (`esp_wifi_set_promiscuous(false)`) during BLE scans and re-enabled on completion.

**Detection Storage**: Ring buffer of 50 entries, each with signal type, RSSI, frequency, identifier string, GPS coordinates, and timestamp. Serial commands: `dump`/`json`, `status`, `clear`, `help`.

## File Descriptions

- `DaringDroneDetector.ino` — entire application (~1945 lines)
- `splash_bitmap.h` — generated C byte array from `daringDrones.bmp` (296x128 1-bit, palette-inverted for GFX drawBitmap)
- `daringDrones.bmp` — source splash image
