# LilyGO T-Display S3 Pro

ESP32-S3 build with a 222×480 IPS touchscreen, I2S speaker, vibration motor, and SY6970 battery management. Designed for a more interactive handheld experience with a full-colour Game Boy-inspired UI.

## Hardware / Bill of Materials

| Component | Description |
|-----------|-------------|
| LilyGO T-Display S3 Pro | ESP32-S3 dev board with 222×480 IPS TFT and CST226SE touch |
| Waveshare Core1121-XF | LR1121 dual-band LoRa radio (900MHz + 2.4GHz) |
| ERM vibration motor | Haptic alert on detection (GPIO45) |
| LiPo battery | Managed by onboard SY6970 PMIC |
| 915MHz antenna | For ELRS LoRa reception |

> **Note:** No GPS module on this build — GPIO conflicts with the LCD chip select prevent using the same UART pins as the E290 build, and the touchscreen provides enough interaction without it.

## Wiring

| GPIO | Function | GPIO | Function |
|------|----------|------|----------|
| 5 | LR1121 CS | 48 | LCD backlight (PWM) |
| 3 | LR1121 SCK | 45 | Vibration motor |
| 1 | LR1121 MOSI | 0 | Side button (boot) |
| 4 | LR1121 MISO | 21 | Side button 2 |
| 6 | LR1121 BUSY | 8 | I2C SDA (RTC) |
| 7 | LR1121 DIO9 | 9 | I2C SCL (RTC) |
| 2 | LR1121 RESET | | |

Display, touch (CST226SE at 0x5A), and battery (SY6970) are handled by their respective libraries/drivers.

## Software Setup

### 1. Install Arduino IDE

Download and install [Arduino IDE 2.x](https://www.arduino.cc/en/software).

### 2. Add Espressif ESP32 Board Package

1. Open **File > Preferences**
2. Add this URL to **Additional Board Manager URLs**:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Open **Tools > Board > Boards Manager**, search for **esp32**, install the **Espressif Systems** package

### 3. Install Required Libraries

Open **Tools > Manage Libraries** and install:

- **RadioLib** — LoRa radio driver (LR1121 support)
- **Arduino_GFX_Library** — TFT display driver

The following are included with the ESP32 Arduino core:
- BLEDevice, WiFi, esp_wifi, Wire

### 4. Select Board & Upload

1. Open `LillyGo_S3_Pro_LR1121/LillyGo_S3_Pro_LR1121.ino` in Arduino IDE
2. Select **Tools > Board > ESP32S3 Dev Module**
3. Set **Tools > Partition Scheme > 16M Flash (3MB APP/9.9MB FATFS)**
4. Set **Tools > Flash Size > 16MB**
5. Select the correct serial port under **Tools > Port**
6. Click **Upload**

### Using arduino-cli

```bash
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:PartitionScheme=app3M_fat9M_16MB,FlashSize=16M" \
  LillyGo_S3_Pro_LR1121
arduino-cli upload \
  --fqbn "esp32:esp32:esp32s3:PartitionScheme=app3M_fat9M_16MB,FlashSize=16M" \
  -p /dev/ttyUSB0 LillyGo_S3_Pro_LR1121
```

## UI Navigation

The touchscreen is the primary input. Swipe and tap gestures control the UI:

| Gesture | Action |
|---------|--------|
| Swipe left / right | Cycle display pages |
| Tap a setting | Toggle ON/OFF (on Config page) |
| Side button (GPIO0) | Reserved / boot |

### Display Pages

1. **Summary** — Active ELRS/WiFi/BLE tracking counts, RSSI bars, battery percentage
2. **Log** — Scrollable detection log with signal type, RSSI, and identifier
3. **Config** — Toggle buzzer, vibration, and scan modes; set the RTC clock

## RTC Clock

The board includes a PCF85063 RTC (I2C 0x51). Set the time from the Config page:

1. Swipe to the **Config** page
2. Tap **SET CLOCK**
3. Tap the HOUR, MIN, SEC, DAY, MON, or YEAR field to select it
4. Use the **+** / **−** buttons to adjust
5. Tap **SAVE** to write to the RTC

Time is used to timestamp detections in the log.

## Serial Commands

Connect at 115200 baud:

| Command | Description |
|---------|-------------|
| `status` | Scanner status and detection counts |
| `dump` | All detections in human-readable format |
| `json` | Export all detections as JSON |
| `clear` | Clear the detection buffer |
| `help` | Show available commands |
