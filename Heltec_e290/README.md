# Heltec Vision Master E290

ESP32-S3 build with a 2.9" e-ink display, GPS, and a 5-way navigation switch. Designed for field use where low power and always-on visibility matter.

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

## Wiring

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

1. Open `Heltec_e290/Heltec_e290.ino` in Arduino IDE
2. Select **Tools > Board > Heltec Vision Master E290**
3. Select the correct serial port under **Tools > Port**
4. Click **Upload**

### Using arduino-cli

```bash
arduino-cli compile --fqbn Heltec-esp32:esp32:heltec_vision_master_e290 Heltec_e290
arduino-cli upload --fqbn Heltec-esp32:esp32:heltec_vision_master_e290 -p /dev/ttyUSB0 Heltec_e290
```

## UI Navigation

The 5-way switch controls the display:

| Input | Action |
|-------|--------|
| LEFT / RIGHT | Cycle display pages |
| UP / DOWN | Scroll detection log (on Detections page) / adjust settings |
| CENTER (short) | Toggle setting (on Settings page) |
| CENTER (hold 1.5s) | Toggle a setting |
| CENTER (hold 7s) | Sleep / wake |

### Display Pages

1. **Summary** — Active ELRS/WiFi/BLE tracking counts, RSSI, GPS fix, detection log count
2. **Detections** — Scrollable log of recent detections with RSSI, SNR, and identifier
3. **GPS** — Current coordinates, satellite count, HDOP
4. **Settings** — Toggle buzzer, scan modes, GPS, and ELRS packet type filters

## Serial Commands

Connect at 115200 baud:

| Command | Description |
|---------|-------------|
| `status` | Scanner status and detection counts |
| `dump` | All detections in human-readable format |
| `json` | Export all detections as JSON |
| `clear` | Clear the detection buffer |
| `help` | Show available commands |
