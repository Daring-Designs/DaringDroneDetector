// ============================================================================
// Drone Signal Scanner - LilyGO T-Display S3 Pro LR1121
// Scans ELRS 915MHz + 2.4GHz (LR1121 LoRa CAD), WiFi (DJI Remote ID), BLE (OpenDroneID)
// Radio: LR1121 on-board (dual-band 900MHz + 2.4GHz), shared SPI bus
// Display: ST7796 IPS TFT 222x480, Arduino_GFX
// Input: CST226SE touch screen
// Audio: MAX98357A I2S speaker (GPIO4/15/11) for detection alerts
// Battery: SY6970 charger IC via I2C (SDA=5, SCL=6)
// ============================================================================

#include <RadioLib.h>
#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEUtils.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "esp_bt.h"
#include <Wire.h>
#include <driver/i2s.h>
#include "splash_bitmap.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// LR1121 LoRa radio (shared SPI bus with LCD and SD)
#define LORA_SCK    18
#define LORA_MOSI   17
#define LORA_MISO   8
#define LORA_NSS    7    // LR1121 CS
#define LORA_DIO9   40   // LR1121 interrupt
#define LORA_RESET  10   // LR1121 reset
#define LORA_BUSY   46   // LR1121 busy

// ST7796 TFT display (shared SPI bus)
#define LCD_SCLK    18
#define LCD_MOSI    17
#define LCD_MISO    8
#define LCD_CS      39
#define LCD_DC      9
#define LCD_RST     47
#define LCD_BL      48   // Backlight PWM

// Touchscreen CST226SE (I2C)
#define TOUCH_SDA   5
#define TOUCH_SCL   6
#define TOUCH_RST   13
#define TOUCH_INT   21

// I2C bus (shared by touch, RTC, SY6970 charger, IMU)
#define I2C_SDA     5
#define I2C_SCL     6

// I2S speaker (MAX98357A)
#define I2S_BCLK    4
#define I2S_LRCLK   15
#define I2S_DATA    11
#define I2S_SD_MODE 41   // Speaker enable (HIGH = active)

// Power
#define LDO_EN      42   // RT9080 LDO enable — must be HIGH before radio/display init

// Side buttons
#define BTN_NEXT    12   // Right side button — next page
#define BTN_PREV    16   // Left side button  — previous page

// Vibration motor
#define MOTOR_PIN   45   // ERM motor via MOSFET driver

// SY6970 battery charger
#define SY6970_ADDR 0x6A

// PCF85063 RTC (NXP, I2C 0x51)
#define RTC_ADDR    0x51

// ============================================================================
// DISPLAY
// ============================================================================

#define LCD_WIDTH   222
#define LCD_HEIGHT  480

Arduino_DataBus* bus = nullptr;
Arduino_GFX* gfx = nullptr;

// ============================================================================
// CONFIGURATION
// ============================================================================

#define MAX_DETECTIONS      50
#define BLE_SCAN_TIME_SEC   1
#define WIFI_SCAN_DWELL_MS  50

// Battery monitoring
#define BATT_READ_INTERVAL_MS 10000
#define BATT_LOW_PERCENT      10
#define BATT_VOLTAGE_FULL     4.2f
#define BATT_VOLTAGE_EMPTY    3.0f

// ============================================================================
// SETTINGS
// ============================================================================

struct Settings {
  bool speakerEnabled;
  bool vibrateEnabled;
  bool elrsScanEnabled;
  bool wifiScanEnabled;
  bool bleScanEnabled;
  bool elrsFilterRC;
  bool elrsFilterMSP;
  bool elrsFilterSYNC;
  bool elrsFilterTLM;
};

Settings settings = {
  .speakerEnabled   = true,
  .vibrateEnabled   = true,
  .elrsScanEnabled  = true,
  .wifiScanEnabled  = true,
  .bleScanEnabled   = true,
  .elrsFilterRC     = true,
  .elrsFilterMSP    = true,
  .elrsFilterSYNC   = true,
  .elrsFilterTLM    = true,
};

#define NUM_SETTINGS 10

// ============================================================================
// ELRS CHANNEL TABLES & RATES (identical to Heltec version)
// ============================================================================

const float ELRS_FREQ_TABLE[] = {
  903.5, 904.1, 904.7, 905.3, 905.9, 906.5, 907.1, 907.7,
  908.3, 908.9, 909.5, 910.1, 910.7, 911.3, 911.9, 912.5,
  913.1, 913.7, 914.3, 914.9, 915.5, 916.1, 916.7, 917.3,
  917.9, 918.5, 919.1, 919.7, 920.3, 920.9, 921.5, 922.1,
  922.7, 923.3, 923.9, 924.5, 925.1, 925.7, 926.3, 926.9
};
const int ELRS_NUM_CHANNELS = sizeof(ELRS_FREQ_TABLE) / sizeof(ELRS_FREQ_TABLE[0]);

const float ELRS_24_FREQ_TABLE[] = {
  2400.4, 2401.4, 2402.4, 2403.4, 2404.4, 2405.4, 2406.4, 2407.4,
  2408.4, 2409.4, 2410.4, 2411.4, 2412.4, 2413.4, 2414.4, 2415.4,
  2416.4, 2417.4, 2418.4, 2419.4, 2420.4, 2421.4, 2422.4, 2423.4,
  2424.4, 2425.4, 2426.4, 2427.4, 2428.4, 2429.4, 2430.4, 2431.4,
  2432.4, 2433.4, 2434.4, 2435.4, 2436.4, 2437.4, 2438.4, 2439.4,
  2440.4, 2441.4, 2442.4, 2443.4, 2444.4, 2445.4, 2446.4, 2447.4,
  2448.4, 2449.4, 2450.4, 2451.4, 2452.4, 2453.4, 2454.4, 2455.4,
  2456.4, 2457.4, 2458.4, 2459.4, 2460.4, 2461.4, 2462.4, 2463.4,
  2464.4, 2465.4, 2466.4, 2467.4, 2468.4, 2469.4, 2470.4, 2471.4,
  2472.4, 2473.4, 2474.4, 2475.4, 2476.4, 2477.4, 2478.4, 2479.4,
};
#define ELRS_24_NUM_CHANNELS 80

// ============================================================================
// DETECTION TYPES
// ============================================================================

enum SignalType { SIG_ELRS, SIG_WIFI, SIG_BLE };

struct Detection {
  int rssi;
  float snr;
  SignalType type;
  float frequency;
  char identifier[24];
  uint8_t elrsPktType;
  uint8_t hour, minute, second;
  uint8_t day, month;
  uint16_t year;
  bool valid;
};

// ============================================================================
// GLOBALS
// ============================================================================

// Radio — LR1121 on shared SPI (same SPI2 bus as display; gfx->begin() initializes it)
Module* loraMod = nullptr;
LR1121* radio = nullptr;

// BLE
BLEScan* pBLEScan = nullptr;

// Detection ring buffer
Detection detections[MAX_DETECTIONS];
int detectionHead = 0;
int detectionCount = 0;

// Signal counters
volatile int elrsHitCount = 0;
volatile int elrsStrongestRSSI = -999;
volatile float elrsBestSNR = -999;
volatile int wifiStrongestRSSI = -999;
volatile int bleStrongestRSSI = -999;
int totalElrsHits = 0;
int totalWifiHits = 0;
int totalBleHits = 0;

// Battery
float battVoltage = 0.0;
int battPercent = -1;
unsigned long lastBattRead = 0;

// Timing
unsigned long lastDisplayRefresh = 0;
unsigned long lastWifiScan = 0;
bool displayDirty = true;    // set true whenever state changes; display_task redraws and clears

// RTC set mode
bool rtcSetMode = false;
int  rtcSetField = 0;    // 0=hour 1=min 2=sec 3=day 4=month 5=year(2-digit)
uint8_t rtcEdit[6];      // working copy while editing

// UI state
int displayPage = 0;      // 0=summary, 1=detections, 2=settings
int detectionScrollOffset = 0;
#define NUM_PAGES 3

// WiFi
volatile bool wifiScanActive = false;

// ============================================================================
// COLORS (RGB565)
// ============================================================================

// DMG green palette
#define C_DMG1  0x3306   // #306230 — dark green
#define C_DMG2  0x8D61   // #8bac0f — light green
#define C_DMG3  0x9DE1   // #9bbc0f — brightest green

// Neutrals
#define C_BLACK  0x0000  // #000000
#define C_DARK   0x2104  // #222222 — background
#define C_GRAY   0x4228  // #444444 — panels / inactive

// Accent
#define C_ORANGE 0xFAE0  // #ff5e00 — active detection / alert

// Theme aliases
#define C_BG     C_DARK   // Screen background
#define C_FG     C_DMG3   // Primary text
#define C_DIM    C_GRAY   // Borders / inactive
#define C_MED    C_DMG2   // Secondary text

// Signal type colors
#define C_ELRS   C_DMG3   // Bright green — ELRS RC link
#define C_WIFI   C_DMG2   // Mid green — WiFi Remote ID
#define C_BLE    C_DMG2   // Mid green — BLE OpenDroneID

// ============================================================================
// OPENDRONEIDS STRUCTURES
// ============================================================================

#define ODID_SERVICE_UUID   0xFFFA
#define ODID_APP_CODE       0x0D
#define ODID_MSG_BASIC_ID   0x0
#define ODID_MSG_LOCATION   0x1
#define ODID_MSG_SYSTEM     0x4
#define ODID_MSG_OPERATOR   0x5

struct ODIDData {
  char droneId[21];
  double droneLat, droneLng;
  float droneAlt;
  char operatorId[21];
  bool hasBasicId, hasLocation, hasOperatorId;
};

// Speaker tone state
bool speakerActive = false;
unsigned long speakerStartTime = 0;
int speakerBeepsRemaining = 0;
bool speakerToneOn = false;
int speakerToneFreq = 2000;
#define SPEAKER_BEEP_MS  80
#define SPEAKER_GAP_MS   60

// Vibration motor state
unsigned long motorOffTime = 0;
bool motorOn = false;
#define MOTOR_PULSE_MS  120   // Duration of one buzz

// ============================================================================
// ELRS STRUCTS & STATE (identical logic to Heltec, different radio SPI)
// ============================================================================

struct ElrsAirRate {
  float    bw;
  uint8_t  sf;
  uint8_t  cr;
  bool     crLongInterleave;
  uint8_t  payloadLen;
  const char* name;
};

const ElrsAirRate ELRS_900_RATES[] = {
  { 500.0,  5,  8, false,  8,  "250Hz"     },
  { 500.0,  5,  7, false, 13,  "200HzFull" },
  { 500.0,  6,  7, false,  8,  "200Hz"     },
  { 500.0,  6,  8, false, 13,  "100HzFull" },
  { 500.0,  7,  7, false,  8,  "100Hz"     },
  { 500.0,  8,  7, false,  8,  "50Hz"      },
  { 500.0,  9,  7, false,  8,  "25Hz"      },
  { 500.0,  6,  7, false,  8,  "D50"       },
};
#define ELRS_900_NUM_RATES 8

const ElrsAirRate ELRS_24_RATES[] = {
  { 812.5,  5,  6, true,  8,  "2.4 500Hz"     },
  { 812.5,  5,  8, true, 13,  "2.4 333HzFull" },
  { 812.5,  6,  8, true,  8,  "2.4 250Hz"     },
  { 812.5,  7,  8, true,  8,  "2.4 150Hz"     },
  { 812.5,  7,  8, true, 13,  "2.4 100HzFull" },
  { 812.5,  8,  8, true,  8,  "2.4 50Hz"      },
};
#define ELRS_24_NUM_RATES 6

enum ElrsBand { BAND_900, BAND_24 };
ElrsBand elrsCurrentBand = BAND_900;

int cadChannelIndex = 0;
#define CAD_WINDOW_MS           5000
#define CAD_MIN_CHANNELS        3
#define CAD_MIN_HITS            4
#define ELRS_REPORT_COOLDOWN_MS 15000

uint8_t cadChannelHits[80] = {0};
int cadTotalHits = 0;
unsigned long cadWindowStart = 0;

#define MAX_TRACK_SLOTS         8
#define TRACK_TIMEOUT_MS        10000
#define TRACK_LOG_FAST_MS       300
#define TRACK_LOG_SLOW_MS       3000

unsigned long track_log_interval(float rssi) {
  int clamped = constrain((int)rssi, -90, -30);
  return map(clamped, -90, -30, TRACK_LOG_SLOW_MS, TRACK_LOG_FAST_MS);
}

struct TrackSlot {
  bool active;
  const ElrsAirRate* rate;
  ElrsBand band;
  uint8_t channels[80];
  int numChannels;
  int chIdx;
  unsigned long lastRx;
  unsigned long lastLog;
  int rxCount;
  float bestRSSI, bestSNR, smoothedRSSI;
  uint8_t lastPktType;
  uint8_t id;
};

TrackSlot trackSlots[MAX_TRACK_SLOTS];
int activeTrackCount = 0;

// ---- WiFi tracking slots ----
#define MAX_WIFI_TRACKS          8
#define WIFI_TRACK_TIMEOUT_MS    15000
#define WIFI_TRACK_LOG_MS        5000

struct WifiTrack {
  bool     active;
  uint8_t  mac[6];
  char     desc[20];   // "DJI-RID", "ASTM-RID", SSID, etc.
  float    smoothedRSSI;
  int      bestRSSI;
  unsigned long lastSeen;
  unsigned long lastLog;
  int      hitCount;
  uint8_t  id;
};
WifiTrack wifiTracks[MAX_WIFI_TRACKS];
int wifiActiveCount = 0;

// ---- BLE tracking slots ----
#define MAX_BLE_TRACKS           8
#define BLE_TRACK_TIMEOUT_MS     30000
#define BLE_TRACK_LOG_MS         10000

struct BleTrack {
  bool     active;
  char     addr[18];   // "aa:bb:cc:dd:ee:ff"
  char     desc[20];   // drone ID or "ODID"
  float    smoothedRSSI;
  int      bestRSSI;
  unsigned long lastSeen;
  unsigned long lastLog;
  int      hitCount;
  uint8_t  id;
};
BleTrack bleTracks[MAX_BLE_TRACKS];
int bleActiveCount = 0;

int schedNextSlot = 0;
bool verifyPending = false;
int verifyRateIndex = 0;
int verifyRatesTried = 0;
ElrsBand verifyBand = BAND_900;
uint8_t verifyCadHits[80];

const ElrsAirRate* radioCurrentRate = NULL;
ElrsBand radioCurrentBand = BAND_900;
bool radioInCadMode = true;

// Forward declarations
void configure_radio_for_slot(TrackSlot* slot);
TrackSlot* allocate_track_slot();
TrackSlot* find_duplicate_track(ElrsBand band, const ElrsAirRate* rate);
void expire_tracks(unsigned long now);
void clear_all_tracks();
void run_verify_step(unsigned long now);
WifiTrack* find_wifi_track(const uint8_t* mac);
WifiTrack* alloc_wifi_track();
void expire_wifi_tracks();
BleTrack* find_ble_track(const char* addr);
BleTrack* alloc_ble_track();
void expire_ble_tracks();
void run_track_step(TrackSlot* slot, unsigned long now);
void run_cad_step(unsigned long now);

unsigned long elrsLastReportTime = 0;
const char* lastElrsDetectedRate = NULL;
uint8_t lastElrsPktType = 0xFF;

// RTC current time (updated every second by rtc_task)
struct { uint8_t hour, minute, second, day, month; uint16_t year; } rtcNow = {};
bool rtcValid = false;

// Forward declarations
void cad_scan_task();
void wifi_scan_task();
void ble_scan_task();
void battery_task();
void log_hit(SignalType type, int rssi, float freq, const char* id, float snr = 0, uint8_t elrsPktType = 0xFF);
void display_task();
void btn_task();
void speaker_task();
void speaker_beep(int count, int freq);
void motor_task();
void motor_buzz(int ms);
void dump_json();
void rtc_task();
void rtc_set(uint8_t h, uint8_t m, uint8_t s, uint8_t d, uint8_t mo, uint16_t yr);
// ============================================================================
// ELRS HELPERS (identical to Heltec)
// ============================================================================

int elrs_num_channels() {
  return (elrsCurrentBand == BAND_900) ? ELRS_NUM_CHANNELS : ELRS_24_NUM_CHANNELS;
}
const float* elrs_freq_table() {
  return (elrsCurrentBand == BAND_900) ? ELRS_FREQ_TABLE : ELRS_24_FREQ_TABLE;
}
const ElrsAirRate* elrs_rates() {
  return (elrsCurrentBand == BAND_900) ? ELRS_900_RATES : ELRS_24_RATES;
}
int elrs_num_rates() {
  return (elrsCurrentBand == BAND_900) ? ELRS_900_NUM_RATES : ELRS_24_NUM_RATES;
}
const char* elrs_band_name() {
  return (elrsCurrentBand == BAND_900) ? "900" : "2.4G";
}
void elrs_reset_window() {
  cadTotalHits = 0;
  memset(cadChannelHits, 0, sizeof(cadChannelHits));
  cadWindowStart = millis();
}

const char* elrs_pkt_type_name(uint8_t t) {
  switch (t) { case 0: return "RC"; case 1: return "MSP"; case 2: return "SYNC"; case 3: return "TLM"; default: return "?"; }
}
bool elrs_pkt_type_allowed(uint8_t t) {
  switch (t) { case 0: return settings.elrsFilterRC; case 1: return settings.elrsFilterMSP;
               case 2: return settings.elrsFilterSYNC; case 3: return settings.elrsFilterTLM; default: return true; }
}

void elrs_restore_cad_config() {
  int16_t state;
  if (elrsCurrentBand == BAND_900) {
    state = radio->setBandwidth(500.0);
    state = radio->setFrequency(915.0, true);
    radio->setSpreadingFactor(6);
    radio->setCodingRate(7);
  } else {
    state = radio->setBandwidth(812.5, true);
    state = radio->setFrequency(2440.0, true);
    radio->setSpreadingFactor(6);
    radio->setCodingRate(8, true);
  }
  radio->setCRC(false);
  radio->explicitHeader();
  (void)state;
}

void elrs_switch_band() {
  elrsCurrentBand = (elrsCurrentBand == BAND_900) ? BAND_24 : BAND_900;
  cadChannelIndex = 0;
  elrs_restore_cad_config();
}

// ============================================================================
// LOG HIT
// ============================================================================

void log_hit(SignalType type, int rssi, float freq, const char* id, float snr, uint8_t elrsPktType) {
  Detection* d = &detections[detectionHead];
  d->valid = true;
  d->type = type;
  d->rssi = rssi;
  d->snr = snr;
  d->elrsPktType = elrsPktType;
  d->frequency = freq;
  strncpy(d->identifier, id, sizeof(d->identifier) - 1);
  d->identifier[sizeof(d->identifier) - 1] = '\0';
  d->hour = rtcNow.hour; d->minute = rtcNow.minute; d->second = rtcNow.second;
  d->day  = rtcNow.day;  d->month  = rtcNow.month;  d->year   = rtcNow.year;

  detectionHead = (detectionHead + 1) % MAX_DETECTIONS;
  if (detectionCount < MAX_DETECTIONS) detectionCount++;
  displayDirty = true;

  switch (type) {
    case SIG_ELRS:
      elrsHitCount++; totalElrsHits++;
      if (rssi > elrsStrongestRSSI) elrsStrongestRSSI = rssi;
      if (snr > elrsBestSNR) elrsBestSNR = snr;
      break;
    case SIG_WIFI:
      totalWifiHits++;
      if (rssi > wifiStrongestRSSI) wifiStrongestRSSI = rssi;
      break;
    case SIG_BLE:
      totalBleHits++;
      if (rssi > bleStrongestRSSI) bleStrongestRSSI = rssi;
      break;
  }

  Serial.printf("[HIT] type=%s rssi=%d freq=%.1f id=%s\n",
    type == SIG_ELRS ? "ELRS" : type == SIG_WIFI ? "WiFi" : "BLE", rssi, freq, id);

  if (settings.speakerEnabled) {
    int f = map(constrain(rssi, -100, -20), -100, -20, 1000, 4000);
    switch (type) {
      case SIG_ELRS: speaker_beep(1, f); break;
      case SIG_WIFI: speaker_beep(2, f); break;
      case SIG_BLE:  speaker_beep(3, f); break;
    }
  }
  if (settings.vibrateEnabled) motor_buzz(MOTOR_PULSE_MS);
}

// ============================================================================
// MULTI-TRACK HELPERS (identical logic to Heltec)
// ============================================================================

void configure_radio_for_slot(TrackSlot* slot) {
  if (slot->rate == radioCurrentRate && slot->band == radioCurrentBand && !radioInCadMode) return;
  radio->setBandwidth(slot->rate->bw, slot->band == BAND_24);
  radio->setSpreadingFactor(slot->rate->sf);
  radio->setCodingRate(slot->rate->cr, slot->rate->crLongInterleave);
  radio->setSyncWord(0x12);
  radio->implicitHeader(slot->rate->payloadLen);
  radio->setCRC(false);
  radioCurrentRate = slot->rate;
  radioCurrentBand = slot->band;
  radioInCadMode = false;
}

TrackSlot* allocate_track_slot() {
  for (int i = 0; i < MAX_TRACK_SLOTS; i++) {
    if (!trackSlots[i].active) return &trackSlots[i];
  }
  int weakest = 0;
  for (int i = 1; i < MAX_TRACK_SLOTS; i++) {
    if (trackSlots[i].smoothedRSSI < trackSlots[weakest].smoothedRSSI) weakest = i;
  }
  TrackSlot* slot = &trackSlots[weakest];
  char chanStr[32];
  snprintf(chanStr, sizeof(chanStr), "%s%s/EVICT#%d", (slot->band == BAND_900) ? "900 " : "", slot->rate->name, slot->id);
  log_hit(SIG_ELRS, (int)slot->bestRSSI, 0, chanStr, slot->bestSNR, slot->lastPktType);
  slot->active = false;
  activeTrackCount--;
  return slot;
}

TrackSlot* find_duplicate_track(ElrsBand band, const ElrsAirRate* rate) {
  for (int i = 0; i < MAX_TRACK_SLOTS; i++) {
    if (trackSlots[i].active && trackSlots[i].band == band && trackSlots[i].rate == rate) return &trackSlots[i];
  }
  return NULL;
}

void expire_tracks(unsigned long now) {
  for (int i = 0; i < MAX_TRACK_SLOTS; i++) {
    TrackSlot* slot = &trackSlots[i];
    if (!slot->active) continue;
    if (now - slot->lastRx >= TRACK_TIMEOUT_MS) {
      Serial.printf("[ELRS %s] TRACK#%d: lost after %ds, %d pkts\n",
                    (slot->band == BAND_900) ? "900" : "2.4G", slot->id, TRACK_TIMEOUT_MS / 1000, slot->rxCount);
      if (slot->bestRSSI > -999 && now - slot->lastLog > 1000) {
        char chanStr[32];
        snprintf(chanStr, sizeof(chanStr), "%s%s/LOST#%d", (slot->band == BAND_900) ? "900 " : "", slot->rate->name, slot->id);
        log_hit(SIG_ELRS, (int)slot->bestRSSI, 0, chanStr, slot->bestSNR, slot->lastPktType);
      }
      slot->active = false;
      activeTrackCount--;
      displayDirty = true;
    }
  }
}

void clear_all_tracks() {
  for (int i = 0; i < MAX_TRACK_SLOTS; i++) { trackSlots[i].active = false; trackSlots[i].id = i; }
  activeTrackCount = 0;
  verifyPending = false;
  schedNextSlot = 0;
  for (int i = 0; i < MAX_WIFI_TRACKS; i++) { wifiTracks[i].active = false; wifiTracks[i].id = i; }
  wifiActiveCount = 0;
  for (int i = 0; i < MAX_BLE_TRACKS;  i++) { bleTracks[i].active  = false; bleTracks[i].id  = i; }
  bleActiveCount = 0;
}

// ---- WiFi track helpers ----

WifiTrack* find_wifi_track(const uint8_t* mac) {
  for (int i = 0; i < MAX_WIFI_TRACKS; i++)
    if (wifiTracks[i].active && memcmp(wifiTracks[i].mac, mac, 6) == 0) return &wifiTracks[i];
  return NULL;
}

WifiTrack* alloc_wifi_track() {
  for (int i = 0; i < MAX_WIFI_TRACKS; i++) if (!wifiTracks[i].active) return &wifiTracks[i];
  int weakest = 0;
  for (int i = 1; i < MAX_WIFI_TRACKS; i++)
    if (wifiTracks[i].smoothedRSSI < wifiTracks[weakest].smoothedRSSI) weakest = i;
  wifiTracks[weakest].active = false; wifiActiveCount--;
  return &wifiTracks[weakest];
}

void expire_wifi_tracks() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_WIFI_TRACKS; i++) {
    if (!wifiTracks[i].active) continue;
    if (now - wifiTracks[i].lastSeen >= WIFI_TRACK_TIMEOUT_MS) {
      char logId[24]; snprintf(logId, sizeof(logId), "%.12s/LST#%d", wifiTracks[i].desc, i);
      log_hit(SIG_WIFI, wifiTracks[i].bestRSSI, 0, logId);
      wifiTracks[i].active = false; wifiActiveCount--; displayDirty = true;
    }
  }
}

// ---- BLE track helpers ----

BleTrack* find_ble_track(const char* addr) {
  for (int i = 0; i < MAX_BLE_TRACKS; i++)
    if (bleTracks[i].active && strcmp(bleTracks[i].addr, addr) == 0) return &bleTracks[i];
  return NULL;
}

BleTrack* alloc_ble_track() {
  for (int i = 0; i < MAX_BLE_TRACKS; i++) if (!bleTracks[i].active) return &bleTracks[i];
  int weakest = 0;
  for (int i = 1; i < MAX_BLE_TRACKS; i++)
    if (bleTracks[i].smoothedRSSI < bleTracks[weakest].smoothedRSSI) weakest = i;
  bleTracks[weakest].active = false; bleActiveCount--;
  return &bleTracks[weakest];
}

void expire_ble_tracks() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_BLE_TRACKS; i++) {
    if (!bleTracks[i].active) continue;
    if (now - bleTracks[i].lastSeen >= BLE_TRACK_TIMEOUT_MS) {
      char logId[24]; snprintf(logId, sizeof(logId), "%.12s/LST#%d", bleTracks[i].desc, i);
      log_hit(SIG_BLE, bleTracks[i].bestRSSI, 0, logId);
      bleTracks[i].active = false; bleActiveCount--; displayDirty = true;
    }
  }
}

void run_verify_step(unsigned long now) {
  elrsCurrentBand = verifyBand;
  int numRates = elrs_num_rates();
  const ElrsAirRate* rates = elrs_rates();
  int numChannels = elrs_num_channels();
  const float* freqTable = elrs_freq_table();

  if (verifyRatesTried >= numRates) {
    Serial.printf("[ELRS %s] Verify: no packet confirmed\n", elrs_band_name());
    elrs_reset_window(); elrs_switch_band(); verifyPending = false; return;
  }

  const ElrsAirRate* rate = &rates[verifyRateIndex % numRates];
  radio->setBandwidth(rate->bw, verifyBand == BAND_24);
  radio->setSpreadingFactor(rate->sf);
  radio->setCodingRate(rate->cr, rate->crLongInterleave);
  radio->setSyncWord(0x12);
  radio->implicitHeader(rate->payloadLen);
  radio->setCRC(false);
  radioInCadMode = false; radioCurrentRate = rate; radioCurrentBand = verifyBand;

  int hotCh[3] = {-1, -1, -1};
  for (int n = 0; n < 3; n++) {
    int best = -1;
    for (int i = 0; i < numChannels; i++) {
      if (verifyCadHits[i] == 0) continue;
      bool skip = false;
      for (int j = 0; j < n; j++) if (hotCh[j] == i) { skip = true; break; }
      if (skip) continue;
      if (best < 0 || verifyCadHits[i] > verifyCadHits[best]) best = i;
    }
    hotCh[n] = best;
  }

  Serial.printf("[ELRS %s] Verify %s: ", elrs_band_name(), rate->name);
  uint8_t rxBuf[16];
  bool received = false; float rxRssi = 0, rxSnr = 0, rxFreq = 0; uint8_t rxPktType = 0;
  for (int c = 0; c < 3; c++) {
    if (hotCh[c] < 0) continue;
    float freq = freqTable[hotCh[c]];
    radio->setFrequency(freq);
    if (radio->receive(rxBuf, rate->payloadLen, 100) == RADIOLIB_ERR_NONE) {
      rxPktType = (rxBuf[0] >> 6) & 0x03;
      rxRssi = radio->getRSSI(); rxSnr = radio->getSNR(); rxFreq = freq;
      received = true;
      Serial.printf("PACKET RSSI:%.0f SNR:%.1f\n", rxRssi, rxSnr);
      break;
    }
  }

  if (received) {
    lastElrsDetectedRate = rate->name; lastElrsPktType = rxPktType; elrsLastReportTime = now;
    if (!elrs_pkt_type_allowed(rxPktType)) {
      elrs_reset_window(); elrs_switch_band(); verifyPending = false; return;
    }
    int activeCh = 0;
    for (int i = 0; i < numChannels; i++) if (verifyCadHits[i] > 0) activeCh++;

    TrackSlot* existing = find_duplicate_track(verifyBand, rate);
    if (existing) {
      existing->lastRx = now;
      existing->numChannels = 0;
      for (int i = 0; i < numChannels && existing->numChannels < 80; i++)
        if (verifyCadHits[i] > 0) existing->channels[existing->numChannels++] = (uint8_t)i;
    } else {
      TrackSlot* slot = allocate_track_slot();
      slot->active = true; slot->rate = rate; slot->band = verifyBand;
      slot->numChannels = 0;
      for (int i = 0; i < numChannels && slot->numChannels < 80; i++)
        if (verifyCadHits[i] > 0) slot->channels[slot->numChannels++] = (uint8_t)i;
      slot->chIdx = 0; slot->lastRx = now; slot->lastLog = now; slot->rxCount = 1;
      slot->bestRSSI = rxRssi; slot->bestSNR = rxSnr; slot->smoothedRSSI = rxRssi;
      slot->lastPktType = rxPktType; activeTrackCount++;
    }
    TrackSlot* logSlot = existing ? existing : find_duplicate_track(verifyBand, rate);
    char chanStr[32];
    snprintf(chanStr, sizeof(chanStr), "%s%s/%dch#%d", (verifyBand == BAND_900) ? "900 " : "", rate->name, activeCh, logSlot->id);
    log_hit(SIG_ELRS, (int)rxRssi, rxFreq, chanStr, rxSnr, rxPktType);
    elrs_reset_window(); elrs_switch_band(); verifyPending = false; return;
  }
  Serial.println("no packet");
  verifyRateIndex++; verifyRatesTried++;
}

void run_track_step(TrackSlot* slot, unsigned long now) {
  if (slot->numChannels == 0) { slot->active = false; activeTrackCount--; return; }
  configure_radio_for_slot(slot);
  const float* tFreqTable = (slot->band == BAND_900) ? ELRS_FREQ_TABLE : ELRS_24_FREQ_TABLE;
  for (int attempt = 0; attempt < 3; attempt++) {
    int ch = slot->channels[slot->chIdx % slot->numChannels]; slot->chIdx++;
    radio->setFrequency(tFreqTable[ch]);
    uint8_t rxBuf[16];
    if (radio->receive(rxBuf, slot->rate->payloadLen, 20) == RADIOLIB_ERR_NONE) {
      uint8_t pktType = (rxBuf[0] >> 6) & 0x03;
      float rssi = radio->getRSSI(), snr = radio->getSNR();
      slot->lastRx = now; slot->rxCount++; slot->lastPktType = pktType; lastElrsPktType = pktType;
      if (rssi > slot->bestRSSI) slot->bestRSSI = rssi;
      if (snr > slot->bestSNR) slot->bestSNR = snr;
      slot->smoothedRSSI = (slot->smoothedRSSI < -900) ? rssi : (0.3f * rssi + 0.7f * slot->smoothedRSSI);
      if ((int)rssi > elrsStrongestRSSI) elrsStrongestRSSI = (int)rssi;
      if (snr > elrsBestSNR) elrsBestSNR = snr;
      unsigned long logInterval = track_log_interval(slot->smoothedRSSI);
      if (now - slot->lastLog >= logInterval) {
        if (elrs_pkt_type_allowed(pktType)) {
          char chanStr[32];
          snprintf(chanStr, sizeof(chanStr), "%s%s/TRK#%d", (slot->band == BAND_900) ? "900 " : "", slot->rate->name, slot->id);
          log_hit(SIG_ELRS, (int)slot->bestRSSI, tFreqTable[ch], chanStr, slot->bestSNR, pktType);
          elrsLastReportTime = now;
        }
        slot->lastLog = now; slot->bestRSSI = -999; slot->bestSNR = -999;
      }
      break;
    }
  }
}

void run_cad_step(unsigned long now) {
  int numChannels = elrs_num_channels();
  const float* freqTable = elrs_freq_table();
  if (!radioInCadMode || radioCurrentBand != elrsCurrentBand) {
    elrs_restore_cad_config(); radioInCadMode = true; radioCurrentBand = elrsCurrentBand; radioCurrentRate = NULL;
  }
  if (cadWindowStart > 0 && now - cadWindowStart >= CAD_WINDOW_MS) {
    int activeChannels = 0;
    for (int i = 0; i < numChannels; i++) if (cadChannelHits[i] > 0) activeChannels++;
    bool triggered = (activeChannels >= CAD_MIN_CHANNELS && cadTotalHits >= CAD_MIN_HITS);
    if (triggered && now - elrsLastReportTime > ELRS_REPORT_COOLDOWN_MS) {
      verifyPending = true; verifyBand = elrsCurrentBand;
      memcpy(verifyCadHits, cadChannelHits, sizeof(verifyCadHits));
      verifyRateIndex = 0; verifyRatesTried = 0; return;
    }
    elrs_reset_window();
    int tracks900 = 0, tracks24 = 0;
    for (int i = 0; i < MAX_TRACK_SLOTS; i++) {
      if (!trackSlots[i].active) continue;
      if (trackSlots[i].band == BAND_900) tracks900++; else tracks24++;
    }
    if (tracks900 > tracks24) elrsCurrentBand = BAND_24;
    else if (tracks24 > tracks900) elrsCurrentBand = BAND_900;
    else elrsCurrentBand = (elrsCurrentBand == BAND_900) ? BAND_24 : BAND_900;
    cadChannelIndex = 0; elrs_restore_cad_config();
    radioInCadMode = true; radioCurrentBand = elrsCurrentBand; radioCurrentRate = NULL; return;
  }
  if (cadWindowStart == 0) cadWindowStart = now;
  if (cadChannelIndex >= numChannels) { cadChannelIndex = 0; return; }
  if (radio->setFrequency(freqTable[cadChannelIndex]) == RADIOLIB_ERR_NONE) {
    if (radio->scanChannel() == RADIOLIB_LORA_DETECTED) { cadTotalHits++; cadChannelHits[cadChannelIndex]++; }
  }
  cadChannelIndex++;
}

void cad_scan_task() {
  unsigned long now = millis();
  expire_tracks(now);
  if (verifyPending) { run_verify_step(now); return; }
  if (activeTrackCount == 0) { run_cad_step(now); return; }
  int activeSlots[MAX_TRACK_SLOTS], n = 0;
  for (int i = 0; i < MAX_TRACK_SLOTS; i++) if (trackSlots[i].active) activeSlots[n++] = i;
  if (schedNextSlot >= n) { run_cad_step(now); schedNextSlot = 0; }
  else { run_track_step(&trackSlots[activeSlots[schedNextSlot]], now); schedNextSlot++; }
}

// ============================================================================
// WIFI SCAN (identical to Heltec)
// ============================================================================

void wifi_sniffer_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* frame = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;
  int rssi = pkt->rx_ctrl.rssi;
  if (len < 24) return;
  uint8_t frameSubtype = (frame[0] >> 4) & 0x0F;
  if ((frame[0] >> 2 & 0x03) != 0) return;

  // Determine if this is a drone-related frame and build a short description
  char desc[20] = "";
  const uint8_t* src = &frame[10];  // source MAC

  if (frameSubtype == 8 || frameSubtype == 5) {
    // Check SSID for known drone keywords
    int pos = 36;
    if (pos + 2 <= len && frame[pos] == 0) {
      int ssidLen = frame[pos + 1];
      if (ssidLen > 0 && ssidLen <= 32 && pos + 2 + ssidLen <= len) {
        char ssid[33] = {0};
        memcpy(ssid, &frame[pos + 2], ssidLen);
        if (strstr(ssid, "DJI") || strstr(ssid, "drone") || strstr(ssid, "DRONE") ||
            strstr(ssid, "Skydio") || strstr(ssid, "Autel") || strstr(ssid, "FIMI")) {
          snprintf(desc, sizeof(desc), "%.19s", ssid);
        }
      }
    }
    // Check vendor-specific Remote ID IEs
    pos = 36;
    while (pos + 2 < len && desc[0] == '\0') {
      uint8_t tagNum = frame[pos], tagLen = frame[pos + 1];
      if (pos + 2 + tagLen > len) break;
      if (tagNum == 221 && tagLen >= 4) {
        const uint8_t* d = &frame[pos + 2];
        if      (d[0] == 0xFA && d[1] == 0x0B && d[2] == 0xBC) strncpy(desc, "ASTM-RID", sizeof(desc));
        else if (d[0] == 0x26 && d[1] == 0x37 && d[2] == 0x12) strncpy(desc, "DJI-RID",  sizeof(desc));
        else if (d[0] == 0x6A && d[1] == 0x5C && d[2] == 0x35) strncpy(desc, "FR-RID",   sizeof(desc));
        else if (d[0] == 0x50 && d[1] == 0x6F && d[2] == 0x9A && d[3] == 0x13) strncpy(desc, "NaN-RID", sizeof(desc));
      }
      pos += 2 + tagLen;
    }
  }
  if (desc[0] == '\0' && frameSubtype == 13 && len > 30 &&
      frame[4] == 0x51 && frame[5] == 0x6F && frame[6] == 0x9A &&
      frame[7] == 0x01 && frame[8] == 0x00 && frame[9] == 0x00) {
    strncpy(desc, "NaN-RID", sizeof(desc));
  }
  if (desc[0] == '\0') return;  // not a drone signal

  // Find or allocate a tracking slot keyed on source MAC
  unsigned long now = millis();
  WifiTrack* track = find_wifi_track(src);
  if (!track) {
    track = alloc_wifi_track();
    memcpy(track->mac, src, 6);
    strncpy(track->desc, desc, sizeof(track->desc) - 1);
    track->active       = true;
    track->smoothedRSSI = rssi;
    track->bestRSSI     = rssi;
    track->hitCount     = 1;
    track->lastSeen     = now;
    track->lastLog      = now;
    wifiActiveCount++;
    // Log lock event
    char logId[24]; snprintf(logId, sizeof(logId), "%.12s/LCK#%d", desc, (int)(track - wifiTracks));
    log_hit(SIG_WIFI, rssi, 0, logId);
  } else {
    track->lastSeen     = now;
    track->hitCount++;
    track->smoothedRSSI = 0.3f * rssi + 0.7f * track->smoothedRSSI;
    if (rssi > track->bestRSSI) track->bestRSSI = rssi;
    if (now - track->lastLog >= WIFI_TRACK_LOG_MS) {
      char logId[24]; snprintf(logId, sizeof(logId), "%.12s/TRK#%d", track->desc, (int)(track - wifiTracks));
      log_hit(SIG_WIFI, (int)track->smoothedRSSI, 0, logId);
      track->lastLog  = now;
      track->bestRSSI = rssi;
    }
  }
}

const uint8_t WIFI_HOP_SEQ[] = { 1,6, 2,6, 3,6, 4,6, 5,6, 7,6, 8,6, 9,6, 10,6, 11,6, 12,6, 13,6 };
#define WIFI_HOP_LEN (sizeof(WIFI_HOP_SEQ))
int wifiHopIndex = 0;

void wifi_scan_task() {
  expire_wifi_tracks();
  esp_wifi_set_channel(WIFI_HOP_SEQ[wifiHopIndex], WIFI_SECOND_CHAN_NONE);
  wifiHopIndex = (wifiHopIndex + 1) % WIFI_HOP_LEN;
}
void wifi_start() {
  WiFi.mode(WIFI_STA); WiFi.disconnect(); delay(10);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_cb);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  wifiScanActive = true;
  Serial.println("[WiFi] Promiscuous mode started");
}
void wifi_stop() { esp_wifi_set_promiscuous(false); wifiScanActive = false; }

// ============================================================================
// BLE SCAN (identical to Heltec)
// ============================================================================

void parse_odid_basic_id(const uint8_t* p, ODIDData* o) { memcpy(o->droneId, &p[2], 20); o->droneId[20]='\0'; o->hasBasicId=true; }
void parse_odid_location(const uint8_t* p, ODIDData* o) {
  int32_t rawLat, rawLon; memcpy(&rawLat, &p[5], 4); memcpy(&rawLon, &p[9], 4);
  o->droneLat = (double)rawLat/1e7; o->droneLng = (double)rawLon/1e7;
  uint16_t rawAlt; memcpy(&rawAlt, &p[15], 2); o->droneAlt = (float)rawAlt*0.5f-1000.0f; o->hasLocation=true;
}
void parse_odid_operator_id(const uint8_t* p, ODIDData* o) { memcpy(o->operatorId, &p[2], 20); o->operatorId[20]='\0'; o->hasOperatorId=true; }

volatile int bleAdvCount = 0;
bool bleScanRunning = false;
unsigned long bleScanStartTime = 0;

class ODIDAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    bleAdvCount++;
    int rssi = advertisedDevice.getRSSI();
    String addrStr = advertisedDevice.getAddress().toString();
    const char* addr = addrStr.c_str();

    uint8_t* payload = advertisedDevice.getPayload();
    size_t payloadLen = advertisedDevice.getPayloadLength();
    size_t pos = 0;
    while (pos + 1 < payloadLen) {
      uint8_t adLen = payload[pos];
      if (adLen == 0 || pos + 1 + adLen > payloadLen) break;
      if (payload[pos+1] == 0x16 && adLen >= 6) {
        uint16_t uuid = payload[pos+2] | (payload[pos+3] << 8);
        if (uuid == ODID_SERVICE_UUID && payload[pos+4] == ODID_APP_CODE && adLen >= 29) {
          const uint8_t* msg = &payload[pos+6];
          uint8_t msgType = (msg[0] >> 4) & 0x0F;

          // Extract a short description from the ODID message
          char desc[20] = "ODID";
          ODIDData odid = {};
          if (msgType == ODID_MSG_BASIC_ID) {
            parse_odid_basic_id(msg, &odid);
            snprintf(desc, sizeof(desc), "%.19s", odid.droneId[0] ? odid.droneId : "ODID");
          }

          // Find or allocate a BLE tracking slot keyed on device address
          unsigned long now = millis();
          BleTrack* track = find_ble_track(addr);
          if (!track) {
            track = alloc_ble_track();
            strncpy(track->addr, addr, sizeof(track->addr) - 1);
            strncpy(track->desc, desc, sizeof(track->desc) - 1);
            track->active       = true;
            track->smoothedRSSI = rssi;
            track->bestRSSI     = rssi;
            track->hitCount     = 1;
            track->lastSeen     = now;
            track->lastLog      = now;
            bleActiveCount++;
            char logId[24]; snprintf(logId, sizeof(logId), "%.12s/LCK#%d", desc, (int)(track - bleTracks));
            log_hit(SIG_BLE, rssi, 0, logId);
          } else {
            track->lastSeen     = now;
            track->hitCount++;
            track->smoothedRSSI = 0.3f * rssi + 0.7f * track->smoothedRSSI;
            if (rssi > track->bestRSSI) track->bestRSSI = rssi;
            if (now - track->lastLog >= BLE_TRACK_LOG_MS) {
              char logId[24]; snprintf(logId, sizeof(logId), "%.12s/TRK#%d", track->desc, (int)(track - bleTracks));
              log_hit(SIG_BLE, (int)track->smoothedRSSI, 0, logId);
              track->lastLog  = now;
              track->bestRSSI = rssi;
            }
          }
          break;  // one tracking update per advertisement
        }
      }
      pos += 1 + adLen;
    }
  }
};

void bleScanComplete(BLEScanResults r) {
  expire_ble_tracks();
  bleScanRunning = false;
  if (settings.wifiScanEnabled) esp_wifi_set_promiscuous(true);
}

void ble_scan_task() {
  for (int i = 0; i < MAX_TRACK_SLOTS; i++) {
    if (trackSlots[i].active && trackSlots[i].smoothedRSSI > -60) return;
  }
  if (bleScanRunning) {
    if (millis() - bleScanStartTime > (BLE_SCAN_TIME_SEC * 2000 + 1000)) {
      pBLEScan->stop(); bleScanRunning = false; pBLEScan->clearResults();
      if (settings.wifiScanEnabled) esp_wifi_set_promiscuous(true);
    }
    return;
  }
  if (settings.wifiScanEnabled) esp_wifi_set_promiscuous(false);
  bleAdvCount = 0; pBLEScan->clearResults();
  pBLEScan->start(BLE_SCAN_TIME_SEC, bleScanComplete, false);
  bleScanRunning = true; bleScanStartTime = millis();
}

void ble_start() {
  BLEDevice::init("DroneScanner");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ODIDAdvertisedDeviceCallbacks(), true);
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  Serial.println("[BLE] Scanner initialized");
}

// ============================================================================
// SPEAKER (I2S MAX98357A)
// ============================================================================

void i2s_init() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num  = I2S_LRCLK,
    .data_out_num = I2S_DATA,
    .data_in_num = I2S_PIN_NO_CHANGE,
  };
  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  digitalWrite(I2S_SD_MODE, HIGH);
}

void i2s_play_tone(int freq_hz, int duration_ms) {
  int sample_rate = 16000;
  int samples = sample_rate * duration_ms / 1000;
  int16_t* buf = (int16_t*)malloc(samples * sizeof(int16_t));
  if (!buf) return;
  for (int i = 0; i < samples; i++) {
    buf[i] = (int16_t)(8000 * sin(2.0 * M_PI * freq_hz * i / sample_rate));
  }
  size_t written;
  i2s_write(I2S_NUM_0, buf, samples * sizeof(int16_t), &written, portMAX_DELAY);
  free(buf);
}

void speaker_beep(int count, int freq) {
  if (speakerBeepsRemaining > 0) return;
  speakerToneFreq = freq;
  speakerBeepsRemaining = count;
  speakerToneOn = true;
  speakerStartTime = millis();
  i2s_play_tone(freq, SPEAKER_BEEP_MS);
}

void speaker_task() {
  if (speakerBeepsRemaining <= 0) return;
  unsigned long elapsed = millis() - speakerStartTime;
  if (speakerToneOn && elapsed >= SPEAKER_BEEP_MS) {
    speakerToneOn = false;
    speakerBeepsRemaining--;
    speakerStartTime = millis();
  } else if (!speakerToneOn && speakerBeepsRemaining > 0 && elapsed >= SPEAKER_GAP_MS) {
    speakerToneOn = true;
    speakerStartTime = millis();
    i2s_play_tone(speakerToneFreq, SPEAKER_BEEP_MS);
  }
}

// ============================================================================
// VIBRATION MOTOR
// ============================================================================

void motor_buzz(int ms) {
  digitalWrite(MOTOR_PIN, HIGH);
  motorOn = true;
  motorOffTime = millis() + ms;
}

void motor_task() {
  if (motorOn && millis() >= motorOffTime) {
    digitalWrite(MOTOR_PIN, LOW);
    motorOn = false;
  }
}

// ============================================================================
// RTC (PCF85063 at 0x51)
// Register map: 0x04=sec, 0x05=min, 0x06=hr, 0x07=day, 0x08=wday, 0x09=mon, 0x0A=year
// ============================================================================

void rtc_task() {
  static unsigned long lastRtcRead = 0;
  if (millis() - lastRtcRead < 1000) return;
  lastRtcRead = millis();

  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x04);  // PCF85063 seconds register
  if (Wire.endTransmission(false) != 0) return;
  Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)7);
  if (Wire.available() < 7) return;

  uint8_t d[7];
  for (int i = 0; i < 7; i++) d[i] = Wire.read();

  // BCD decode — mask out status/unused bits per PCF85063 datasheet
  rtcNow.second = ((d[0] & 0x70) >> 4) * 10 + (d[0] & 0x0F);  // reg 0x04, bit7=OS flag
  rtcNow.minute = ((d[1] & 0x70) >> 4) * 10 + (d[1] & 0x0F);  // reg 0x05
  rtcNow.hour   = ((d[2] & 0x30) >> 4) * 10 + (d[2] & 0x0F);  // reg 0x06, 24h mode
  rtcNow.day    = ((d[3] & 0x30) >> 4) * 10 + (d[3] & 0x0F);  // reg 0x07
  rtcNow.month  = ((d[5] & 0x10) >> 4) * 10 + (d[5] & 0x0F);  // reg 0x09
  rtcNow.year   = 2000 + ((d[6] >> 4) * 10 + (d[6] & 0x0F));  // reg 0x0A, BCD 0-99
  rtcValid = (rtcNow.year >= 2020 && rtcNow.month >= 1 && rtcNow.day >= 1);
}

void rtc_set(uint8_t h, uint8_t m, uint8_t s, uint8_t d, uint8_t mo, uint16_t yr) {
  // Stop clock (STOP bit = reg 0x00 bit 5)
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x00);
  Wire.write(0x20);
  Wire.endTransmission();

  // Write sec/min/hr/day/weekday/month/year in one burst starting at reg 0x04
  uint8_t y2 = (uint8_t)(yr % 100);
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x04);  // PCF85063 seconds register
  Wire.write(((s  / 10) << 4) | (s  % 10));  // OS bit 7 = 0 (oscillator valid)
  Wire.write(((m  / 10) << 4) | (m  % 10));
  Wire.write(((h  / 10) << 4) | (h  % 10));
  Wire.write(((d  / 10) << 4) | (d  % 10));
  Wire.write(0x00);                            // weekday — don't care
  Wire.write(((mo / 10) << 4) | (mo % 10));
  Wire.write(((y2 / 10) << 4) | (y2 % 10));   // BCD year 0-99
  uint8_t r1 = Wire.endTransmission();

  // Restart clock
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x00);
  Wire.write(0x00);
  uint8_t r2 = Wire.endTransmission();

  Serial.printf("[RTC] set %02d:%02d:%02d %04d-%02d-%02d  I2C:%d/%d\n",
                h, m, s, yr, mo, d, r1, r2);

  // Readback to verify
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x04);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)7);
  if (Wire.available() >= 7) {
    uint8_t rb[7];
    for (int i = 0; i < 7; i++) rb[i] = Wire.read();
    Serial.printf("[RTC] readback sec=%02X min=%02X hr=%02X day=%02X wday=%02X mon=%02X yr=%02X\n",
                  rb[0], rb[1], rb[2], rb[3], rb[4], rb[5], rb[6]);
  }

  rtcNow.hour = h; rtcNow.minute = m; rtcNow.second = s;
  rtcNow.day  = d; rtcNow.month  = mo; rtcNow.year   = yr;
  rtcValid = true;
}

// ============================================================================
// BATTERY (SY6970 via I2C)
// ============================================================================

float battery_read_voltage() {
  // SY6970 REG0E: Battery voltage = ((val & 0x7F) * 20 + 2304) mV
  Wire.beginTransmission(SY6970_ADDR);
  Wire.write(0x0E);
  Wire.endTransmission(false);
  Wire.requestFrom(SY6970_ADDR, 1);
  if (Wire.available()) {
    uint8_t val = Wire.read();
    float mV = ((val & 0x7F) * 20.0f) + 2304.0f;
    return mV / 1000.0f;
  }
  return 3.7f;  // Fallback if I2C fails
}

int battery_voltage_to_percent(float v) {
  if (v >= BATT_VOLTAGE_FULL) return 100;
  if (v <= BATT_VOLTAGE_EMPTY) return 0;
  return (int)((v - BATT_VOLTAGE_EMPTY) / (BATT_VOLTAGE_FULL - BATT_VOLTAGE_EMPTY) * 100.0f);
}

void battery_task() {
  unsigned long now = millis();
  if (now - lastBattRead < BATT_READ_INTERVAL_MS && battPercent >= 0) return;
  lastBattRead = now;
  battVoltage = battery_read_voltage();
  int newPercent = battery_voltage_to_percent(battVoltage);
  if (newPercent != battPercent) { battPercent = newPercent; displayDirty = true; }
  Serial.printf("[BATT] %.2fV  %d%%\n", battVoltage, battPercent);
  if (battPercent <= BATT_LOW_PERCENT && battPercent >= 0) {
    Serial.println("[BATT] LOW BATTERY");
  }
}

// ============================================================================
// DISPLAY (TFT color, 222x480 portrait — DMG Game Boy aesthetic)
// ============================================================================

// UI layout constants for 222x480 portrait
#define UI_W          222
#define UI_H          480
#define UI_TITLE_H    26    // size-2 text (16px) + 5px padding each side
#define UI_FOOTER_H   20
#define UI_CONTENT_X  8
#define UI_CONTENT_Y  (UI_TITLE_H + 4)
#define UI_CONTENT_H  (UI_H - UI_TITLE_H - UI_FOOTER_H - 6)
#define UI_LH         20    // line height for size-2 content rows

char _buf[64];

// Draw a GB-style pixel border box
void gb_box(int x, int y, int w, int h, uint16_t col) {
  gfx->drawRect(x, y, w, h, col);
  gfx->drawRect(x + 1, y + 1, w - 2, h - 2, col);  // double-pixel border
}

// Draw a signal strength bar: [####....] style with pixel blocks
void gb_bar(int x, int y, int w, int h, int rssi, uint16_t fg, uint16_t empty) {
  const int SEG = 16;
  int filled = map(constrain(rssi, -100, -20), -100, -20, 0, SEG);
  int segW = w / SEG;
  for (int i = 0; i < SEG; i++) {
    uint16_t c = (i < filled) ? fg : empty;
    gfx->fillRect(x + i * segW, y, segW - 1, h, c);
  }
}

void gfx_text(int x, int y, const char* s, uint16_t fg = C_FG, uint8_t size = 1) {
  gfx->setTextColor(fg);
  gfx->setTextSize(size);
  gfx->setCursor(x, y);
  gfx->print(s);
}

void ui_title(const char* title, const char* right) {
  gfx->fillRect(0, 0, UI_W, UI_TITLE_H, C_DMG1);
  gfx->drawFastHLine(0, UI_TITLE_H - 2, UI_W, C_DMG2);
  gfx->drawFastHLine(0, UI_TITLE_H - 1, UI_W, C_DIM);
  gfx->setTextColor(C_FG); gfx->setTextSize(2);
  gfx->setCursor(UI_CONTENT_X, 5); gfx->print(title);
  if (right) {
    int tw = strlen(right) * 6;   // right label stays size-1 to avoid overlap
    gfx->setTextColor(C_MED); gfx->setTextSize(1);
    gfx->setCursor(UI_W - tw - 4, 9); gfx->print(right);
  }
}

void ui_footer(const char* s) {
  int fy = UI_H - UI_FOOTER_H;
  gfx->fillRect(0, fy, UI_W, UI_FOOTER_H, C_DMG1);
  gfx->drawFastHLine(0, fy, UI_W, C_DIM);
  gfx->drawFastHLine(0, fy + 1, UI_W, C_DMG2);
  gfx->setTextColor(C_MED); gfx->setTextSize(1);
  int tw = strlen(s) * 6;
  gfx->setCursor((UI_W - tw) / 2, fy + 6); gfx->print(s);
}

// Top-down drone icon: arms + rotor rings + body dot
// size=0: small (14px span, arm=5), size=1: large (22px span, arm=9)
void draw_drone_icon(int cx, int cy, uint16_t color, uint8_t size = 0) {
  int arm    = size ? 9 : 5;
  int rotor  = size ? 3 : 2;
  // diagonal arms
  gfx->drawLine(cx, cy, cx - arm, cy - arm, color);
  gfx->drawLine(cx, cy, cx + arm, cy - arm, color);
  gfx->drawLine(cx, cy, cx - arm, cy + arm, color);
  gfx->drawLine(cx, cy, cx + arm, cy + arm, color);
  // rotor rings at each corner
  gfx->drawCircle(cx - arm, cy - arm, rotor, color);
  gfx->drawCircle(cx + arm, cy - arm, rotor, color);
  gfx->drawCircle(cx - arm, cy + arm, rotor, color);
  gfx->drawCircle(cx + arm, cy + arm, rotor, color);
  // center body
  gfx->fillRect(cx - 1, cy - 1, 3, 3, color);
}

// Signal panel: bordered box with label, bar, count, and detail line
void gb_signal_panel(int x, int y, int w, const char* label, uint16_t labelColor,
                     int rssi, int hits, const char* detail) {
  int ph = 64;
  gb_box(x, y, w, ph, C_DIM);
  gfx->fillRect(x + 2, y + 2, w - 4, ph - 4, C_BG);  // clear interior (no ghost pixels)
  // label tag [XXX] — size 2
  gfx->fillRect(x + 3, y + 4, strlen(label) * 12 + 8, 20, C_DIM);
  gfx->setTextColor(labelColor); gfx->setTextSize(2);
  gfx->setCursor(x + 6, y + 5); gfx->print(label);
  // hit count on right — size 2
  snprintf(_buf, sizeof(_buf), "%d", hits);
  int numW = strlen(_buf) * 12;
  gfx->setTextColor(C_MED); gfx->setTextSize(2);
  gfx->setCursor(x + w - numW - 5, y + 5); gfx->print(_buf);
  // signal bar — taller
  int bx = x + 3, by = y + 28, bw = w - 6;
  if (rssi > -999) {
    gb_bar(bx, by, bw, 10, rssi, labelColor, C_DMG1);
    snprintf(_buf, sizeof(_buf), "%ddB", rssi);
    gfx->setTextColor(C_FG); gfx->setTextSize(2);
    gfx->setCursor(x + 3, y + 42); gfx->print(_buf);
  } else {
    gfx->fillRect(bx, by, bw, 10, C_DMG1);
    gfx->setTextColor(C_DIM); gfx->setTextSize(2);
    gfx->setCursor(x + 3, y + 42); gfx->print("--");
  }
  // detail text (rate/type) — size 1, right side of rssi row
  if (detail && detail[0]) {
    int dw = strlen(detail) * 6;
    gfx->setTextColor(C_MED); gfx->setTextSize(1);
    gfx->setCursor(x + w - dw - 5, y + 48); gfx->print(detail);
  }
}

// ============================================================================
// PAGE 0: SUMMARY
// ============================================================================

void display_page_summary() {
  // Title bar
  char pgBuf[12];
  if (battPercent >= 0)
    snprintf(pgBuf, sizeof(pgBuf), "BAT%d%%", battPercent);
  else
    snprintf(pgBuf, sizeof(pgBuf), "BAT--");
  ui_title("DRONE SCAN", pgBuf);

  int y = UI_CONTENT_Y + 4;
  int pw = UI_W - 12;  // panel width = 210

  // ELRS panel (gb_signal_panel self-clears interior)
  char elrsDetail[20] = "";
  if (lastElrsDetectedRate != NULL) {
    if (activeTrackCount > 0)
      snprintf(elrsDetail, sizeof(elrsDetail), "%s TRK%d", lastElrsDetectedRate, activeTrackCount);
    else
      snprintf(elrsDetail, sizeof(elrsDetail), "%s", lastElrsDetectedRate);
  } else if (activeTrackCount > 0) {
    snprintf(elrsDetail, sizeof(elrsDetail), "TRK:%d", activeTrackCount);
  }
  gb_signal_panel(6, y, pw, "ELRS", C_ELRS, elrsStrongestRSSI, totalElrsHits, elrsDetail);
  y += 72;  // ph=64 + 8px gap

  // WiFi panel
  char wifiDetail[20] = "";
  if (wifiActiveCount > 0) snprintf(wifiDetail, sizeof(wifiDetail), "TRK:%d", wifiActiveCount);
  gb_signal_panel(6, y, pw, "WIFI", C_WIFI, wifiStrongestRSSI, totalWifiHits, wifiDetail);
  y += 72;

  // BLE panel
  char bleDetail[20] = "";
  if (bleActiveCount > 0) snprintf(bleDetail, sizeof(bleDetail), "TRK:%d", bleActiveCount);
  gb_signal_panel(6, y, pw, " BLE", C_BLE, bleStrongestRSSI, totalBleHits, bleDetail);
  y += 72;

  // Drone count cards — show currently active tracked signals for all three radios
  const int DC_H   = 52;
  const int DC_GAP = 3;
  const int DC_W   = (pw - DC_GAP * 2) / 3;  // ~68px each
  int    dCounts[3] = { activeTrackCount, wifiActiveCount, bleActiveCount };
  uint16_t dCols[3] = { C_ELRS, C_WIFI, C_BLE };
  const char* dLbls[3] = { "ELRS", "WIFI", "BLE" };

  for (int i = 0; i < 3; i++) {
    int cx  = 6 + i * (DC_W + DC_GAP);
    bool active = dCounts[i] > 0;
    uint16_t col = active ? dCols[i] : C_DIM;

    gfx->fillRect(cx + 2, y + 2, DC_W - 4, DC_H - 4, C_BG);
    gb_box(cx, y, DC_W, DC_H, col);

    int icx = cx + DC_W / 2;
    draw_drone_icon(icx, y + 11, col, 0);

    snprintf(_buf, sizeof(_buf), "%d", dCounts[i]);
    int nw = (int)strlen(_buf) * 12;
    gfx->setTextColor(col); gfx->setTextSize(2);
    gfx->setCursor(icx - nw / 2, y + 22);
    gfx->print(_buf);

    int lw = (int)strlen(dLbls[i]) * 6;
    gfx->setTextColor(col); gfx->setTextSize(1);
    gfx->setCursor(icx - lw / 2, y + 41);
    gfx->print(dLbls[i]);
  }

  if (rtcValid) {
    snprintf(_buf, sizeof(_buf), "%02d:%02d", rtcNow.hour, rtcNow.minute);
  } else {
    snprintf(_buf, sizeof(_buf), "< BTN TO NAVIGATE >");
  }
  ui_footer(_buf);
}

// ============================================================================
// PAGE 1: DETECTIONS
// ============================================================================

void display_page_detections() {
  ui_title("LOG", "2/3");

  const int CARD_H   = 50;
  const int CARD_GAP = 4;
  const int CARD_X   = 4;
  const int CARD_W   = UI_W - CARD_X * 2;  // 214px
  const int ACCENT_W = 5;
  const int TEXT_X   = CARD_X + ACCENT_W + 7;

  int y = UI_CONTENT_Y + 4;
  int listH = UI_H - UI_FOOTER_H - y;
  gfx->fillRect(0, y, UI_W, listH, C_BG);

  int maxShow = listH / (CARD_H + CARD_GAP);
  int maxScroll = detectionCount > maxShow ? detectionCount - maxShow : 0;
  if (detectionScrollOffset > maxScroll) detectionScrollOffset = maxScroll;
  if (detectionScrollOffset < 0) detectionScrollOffset = 0;

  int shown = 0, skipped = 0;
  for (int i = 0; i < detectionCount && shown < maxShow; i++) {
    int idx = (detectionHead - 1 - i + MAX_DETECTIONS) % MAX_DETECTIONS;
    Detection* d = &detections[idx];
    if (!d->valid) continue;
    if (skipped < detectionScrollOffset) { skipped++; continue; }

    uint16_t typeColor = (d->type == SIG_ELRS) ? C_ELRS : (d->type == SIG_WIFI) ? C_WIFI : C_BLE;
    const char* typeTag = (d->type == SIG_ELRS) ? "ELRS" : (d->type == SIG_WIFI) ? "WIFI" : " BLE";

    // Card background + left accent bar
    gfx->fillRoundRect(CARD_X, y, CARD_W, CARD_H, 4, C_GRAY);
    gfx->fillRect(CARD_X, y, ACCENT_W, CARD_H, typeColor);

    // Line 1: type tag + RSSI (left), SNR (right)
    gfx->setTextColor(typeColor); gfx->setTextSize(2);
    gfx->setCursor(TEXT_X, y + 8);
    gfx->print(typeTag);

    snprintf(_buf, sizeof(_buf), " %ddB", d->rssi);
    gfx->setTextColor(C_FG); gfx->setTextSize(2);
    gfx->setCursor(TEXT_X + 52, y + 8);
    gfx->print(_buf);

    if (d->type == SIG_ELRS && d->snr != 0) {
      snprintf(_buf, sizeof(_buf), "%.0fSNR", d->snr);
      int snrW = strlen(_buf) * 12;
      gfx->setTextColor(C_MED); gfx->setTextSize(2);
      gfx->setCursor(CARD_X + CARD_W - snrW - 6, y + 8);
      gfx->print(_buf);
    }

    // Line 2: timestamp + identifier (size 1, bright)
    char lineBuf[40];
    if (d->year >= 2020)
      snprintf(lineBuf, sizeof(lineBuf), "%02d:%02d  %s", d->hour, d->minute, d->identifier);
    else
      snprintf(lineBuf, sizeof(lineBuf), "%s", d->identifier);
    int maxChars = (CARD_W - ACCENT_W - 14) / 6;
    if ((int)strlen(lineBuf) > maxChars) { lineBuf[maxChars-2]='.'; lineBuf[maxChars-1]='.'; lineBuf[maxChars]='\0'; }
    gfx->setTextColor(C_FG); gfx->setTextSize(1);
    gfx->setCursor(TEXT_X, y + 32);
    gfx->print(lineBuf);

    y += CARD_H + CARD_GAP;
    shown++;
  }

  if (shown == 0 && detectionCount == 0) {
    gfx->setTextColor(C_DIM); gfx->setTextSize(2);
    gfx->setCursor((UI_W - 14 * 12) / 2, UI_H / 2 - 8);
    gfx->print("NO SIGNALS");
  }

  if (detectionScrollOffset > 0) {
    gfx->setTextColor(C_MED); gfx->setTextSize(2);
    gfx->setCursor(UI_W / 2 - 6, UI_CONTENT_Y + 4); gfx->print("^");
  }
  snprintf(_buf, sizeof(_buf), "%d/%d", detectionCount, MAX_DETECTIONS);
  ui_footer(_buf);
}

// ============================================================================
// PAGE 2: SETTINGS
// ============================================================================

void display_page_settings() {
  ui_title("CONFIG", "3/3");

  const char* labels[NUM_SETTINGS] = {
    "SPEAKER", "VIBRATE", "ELRS SCAN", "WIFI SCAN", "BLE SCAN",
    "ELRS:RC", "ELRS:MSP", "ELRS:SYNC", "ELRS:TLM", "SET RTC"
  };
  bool values[NUM_SETTINGS] = {
    settings.speakerEnabled, settings.vibrateEnabled,
    settings.elrsScanEnabled, settings.wifiScanEnabled, settings.bleScanEnabled,
    settings.elrsFilterRC, settings.elrsFilterMSP,
    settings.elrsFilterSYNC, settings.elrsFilterTLM,
    rtcValid  // used only for styling the RTC row
  };

  int contentH = UI_H - UI_TITLE_H - UI_FOOTER_H;
  int rowH = contentH / NUM_SETTINGS;
  int y = UI_TITLE_H;

  for (int i = 0; i < NUM_SETTINGS; i++) {
    bool on = values[i];
    uint16_t rowBg = (i % 2 == 0) ? C_DMG1 : C_BG;

    gfx->fillRect(0, y, UI_W, rowH, rowBg);
    gfx->drawFastHLine(0, y + rowH - 1, UI_W, C_DIM);

    int textY = y + (rowH - 16) / 2;
    gfx->setTextColor(C_FG); gfx->setTextSize(2);
    gfx->setCursor(UI_CONTENT_X, textY);
    gfx->print(labels[i]);

    if (i == 9) {
      // RTC action row — show current time or "NOT SET" instead of ON/OFF
      int pillW = 90, pillH = 22;
      int px = UI_W - pillW - 6;
      int py = y + (rowH - pillH) / 2;
      gfx->fillRoundRect(px, py, pillW, pillH, 6, C_BG);
      gfx->drawRoundRect(px, py, pillW, pillH, 6, rtcValid ? C_DMG2 : C_ORANGE);
      if (rtcValid) {
        snprintf(_buf, sizeof(_buf), "%02d:%02d", rtcNow.hour, rtcNow.minute);
        gfx->setTextColor(C_FG); gfx->setTextSize(2);
        gfx->setCursor(px + (pillW - 5 * 12) / 2, py + (pillH - 16) / 2);
      } else {
        snprintf(_buf, sizeof(_buf), "NOT SET");
        gfx->setTextColor(C_ORANGE); gfx->setTextSize(1);
        gfx->setCursor(px + (pillW - 7 * 6) / 2, py + (pillH - 8) / 2);
      }
      gfx->print(_buf);
    } else {
      uint16_t valBg = on ? C_DMG2 : C_GRAY;
      int pillW = 46, pillH = 22;
      int px = UI_W - pillW - 6;
      int py = y + (rowH - pillH) / 2;
      gfx->fillRoundRect(px, py, pillW, pillH, 6, valBg);
      gfx->drawRoundRect(px, py, pillW, pillH, 6, C_FG);
      gfx->setTextColor(C_BG); gfx->setTextSize(2);
      int labelW = on ? 2 * 12 : 3 * 12;
      gfx->setCursor(px + (pillW - labelW) / 2, py + (pillH - 16) / 2);
      gfx->print(on ? "ON" : "OFF");
    }

    y += rowH;
  }

  ui_footer("TAP ROW TO TOGGLE");
}

// ============================================================================
// PAGE 3: RTC SET  (3x2 grid: row 0 = time, row 1 = date, button strip below)
// Fields: 0=HOUR 1=MIN 2=SEC  /  3=DAY 4=MON 5=YEAR
// ============================================================================
// Layout constants (computed at draw time from UI_H/UI_TITLE_H/UI_FOOTER_H):
//   contentH = 434, btnH = 54, fieldAreaH = 380, rowH = 190, colW = 74
#define RTC_COLS    3
#define RTC_FIELD_ROWS 2

void display_page_rtc_set() {
  ui_title("SET CLOCK", rtcValid ? "VALID" : "NOT SET");

  const char* labels[6] = { "HOUR", "MIN", "SEC", "DAY", "MON", "YEAR" };

  int contentH  = UI_H - UI_TITLE_H - UI_FOOTER_H;
  int btnH      = 54;
  int fieldAreaH = contentH - btnH;
  int rowH      = fieldAreaH / RTC_FIELD_ROWS;  // ~190
  int colW      = UI_W / RTC_COLS;              // 74

  // Draw 6 field cells
  for (int i = 0; i < 6; i++) {
    int row = i / RTC_COLS;
    int col = i % RTC_COLS;
    bool sel = (i == rtcSetField);

    int cx = col * colW;
    int cy = UI_TITLE_H + row * rowH;

    uint16_t bg = sel ? C_DMG1 : C_BG;
    gfx->fillRect(cx, cy, colW, rowH, bg);

    // Grid lines
    if (col > 0) gfx->drawFastVLine(cx, cy, rowH, C_DIM);
    gfx->drawFastHLine(cx, cy + rowH - 1, colW, C_DIM);

    // Selection highlight border
    if (sel) {
      gfx->drawRect(cx + 1, cy + 1, colW - 2, rowH - 2, C_ORANGE);
      gfx->drawRect(cx + 2, cy + 2, colW - 4, rowH - 4, C_ORANGE);
    }

    // Label (size 1, centered near top)
    int lw = strlen(labels[i]) * 6;
    gfx->setTextColor(sel ? C_ORANGE : C_DIM);
    gfx->setTextSize(1);
    gfx->setCursor(cx + (colW - lw) / 2, cy + 8);
    gfx->print(labels[i]);

    // Value (size 3 for 2-digit, size 2 for 4-digit year, centered)
    if (i == 5) {
      snprintf(_buf, sizeof(_buf), "%d", 2000 + (int)rtcEdit[5]);
      int vw = 4 * 12;
      gfx->setTextColor(sel ? C_ORANGE : C_FG);
      gfx->setTextSize(2);
      gfx->setCursor(cx + (colW - vw) / 2, cy + rowH / 2 - 8);
    } else {
      snprintf(_buf, sizeof(_buf), "%02d", rtcEdit[i]);
      int vw = 2 * 18;
      gfx->setTextColor(sel ? C_ORANGE : C_FG);
      gfx->setTextSize(3);
      gfx->setCursor(cx + (colW - vw) / 2, cy + rowH / 2 - 12);
    }
    gfx->print(_buf);
  }

  // Button strip
  int btnY = UI_TITLE_H + fieldAreaH;
  int halfW = UI_W / 2;

  // SAVE (left half)
  gfx->fillRect(0, btnY, halfW - 1, btnH, C_DMG1);
  gfx->drawRect(2, btnY + 2, halfW - 5, btnH - 4, C_DMG2);
  gfx->setTextColor(C_FG); gfx->setTextSize(2);
  gfx->setCursor((halfW - 4 * 12) / 2, btnY + (btnH - 16) / 2);
  gfx->print("SAVE");

  // CANCEL (right half)
  gfx->fillRect(halfW + 1, btnY, halfW - 1, btnH, C_BG);
  gfx->drawRect(halfW + 3, btnY + 2, halfW - 5, btnH - 4, C_DIM);
  gfx->setTextColor(C_DIM); gfx->setTextSize(2);
  gfx->setCursor(halfW + (halfW - 6 * 12) / 2, btnY + (btnH - 16) / 2);
  gfx->print("CANCEL");

  ui_footer("TAP FIELD  BTN+ BTN-");
}

// ============================================================================
// DISPLAY TASK
// ============================================================================

void display_task() {
  if (!displayDirty) return;
  if (millis() - lastDisplayRefresh < 200) return;  // min 200ms between redraws
  displayDirty = false;
  lastDisplayRefresh = millis();

  static int8_t lastPage = -1;
  static bool lastRtcSetMode = false;
  if (displayPage != lastPage || rtcSetMode != lastRtcSetMode) {
    gfx->fillScreen(C_BG);
    gb_box(0, 0, UI_W, UI_H, C_DIM);
    lastPage = displayPage;
    lastRtcSetMode = rtcSetMode;
  }

  if (rtcSetMode) { display_page_rtc_set(); return; }

  switch (displayPage) {
    case 0: display_page_summary(); break;
    case 1: display_page_detections(); break;
    case 2: display_page_settings(); break;
  }
}

// ============================================================================
// TOUCH INPUT — CST226SE via I2C
// ============================================================================

// Simple I2C read of touch point
bool touch_get_point(int16_t* x, int16_t* y) {
  Wire.beginTransmission(0x5A);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(0x5A, 6);
  if (Wire.available() < 6) return false;
  uint8_t data[6];
  for (int i = 0; i < 6; i++) data[i] = Wire.read();
  // data[5] = reg 0x05 = finger count (low nibble); 0 = no touch
  if ((data[5] & 0x0F) == 0) return false;
  // data[1]=X_H, data[2]=Y_H, data[3]=[X_L nibble | Y_L nibble]
  // 12-bit X native spans 0-221 (matches display width 222 — no scaling needed)
  // 12-bit Y native spans 0-479 (matches display height 480 — no scaling needed)
  uint16_t xn = ((uint16_t)data[1] << 4) | (data[3] >> 4);
  uint16_t yn = ((uint16_t)data[2] << 4) | (data[3] & 0x0F);
  *x = (int16_t)xn;
  *y = (int16_t)yn;
  if (*x < 0) *x = 0; else if (*x > 221) *x = 221;
  if (*y < 0) *y = 0; else if (*y > 479) *y = 479;
  return true;
}

// Swipe detection state
int16_t touchStartX = 0, touchStartY = 0;
bool touchActive = false;
unsigned long touchStartTime = 0;
#define SWIPE_THRESHOLD 60   // Pixels needed to register a swipe
#define TAP_MAX_MOVE    25   // Max movement to count as a tap
#define TAP_MAX_MS      400  // Max duration to count as a tap

void toggle_setting(int index) {
  motor_buzz(40);  // short confirmation buzz on any settings change
  switch (index) {
    case 0: settings.speakerEnabled = !settings.speakerEnabled; break;
    case 1: settings.vibrateEnabled = !settings.vibrateEnabled; break;
    case 2: settings.elrsScanEnabled = !settings.elrsScanEnabled; break;
    case 3:
      settings.wifiScanEnabled = !settings.wifiScanEnabled;
      if (settings.wifiScanEnabled && !wifiScanActive) wifi_start();
      if (!settings.wifiScanEnabled && wifiScanActive) wifi_stop();
      break;
    case 4:
      settings.bleScanEnabled = !settings.bleScanEnabled;
      if (!settings.bleScanEnabled && pBLEScan && bleScanRunning) { pBLEScan->stop(); bleScanRunning = false; }
      break;
    case 5: settings.elrsFilterRC = !settings.elrsFilterRC; break;
    case 6: settings.elrsFilterMSP = !settings.elrsFilterMSP; break;
    case 7: settings.elrsFilterSYNC = !settings.elrsFilterSYNC; break;
    case 8: settings.elrsFilterTLM = !settings.elrsFilterTLM; break;
    case 9:
      // Enter RTC set mode — pre-populate with current time (or safe defaults)
      // Pre-populate with current RTC time if valid, else default to 2026-03-22 12:00:00
      // rtcEdit[5] = 2-digit year (e.g. 26 = 2026)
      rtcEdit[0] = rtcValid ? rtcNow.hour   : 12;
      rtcEdit[1] = rtcValid ? rtcNow.minute : 0;
      rtcEdit[2] = rtcValid ? rtcNow.second : 0;
      rtcEdit[3] = (rtcValid && rtcNow.day)   ? rtcNow.day   : 22;
      rtcEdit[4] = (rtcValid && rtcNow.month) ? rtcNow.month : 3;
      rtcEdit[5] = (rtcValid && rtcNow.year >= 2000) ? (uint8_t)(rtcNow.year % 100) : 26;
      rtcSetField = 0;
      rtcSetMode = true;
      break;
  }
}

void nav_task() {
  int16_t tx, ty;
  bool touching = touch_get_point(&tx, &ty);

  // Track last known position so release coords are valid
  static int16_t lastTx = 0, lastTy = 0;
  if (touching) { lastTx = tx; lastTy = ty; }

  if (touching && !touchActive) {
    touchActive = true;
    touchStartX = lastTx; touchStartY = lastTy;
    touchStartTime = millis();
  } else if (!touching && touchActive) {
    touchActive = false;
    int16_t dx = lastTx - touchStartX;
    int16_t dy = lastTy - touchStartY;
    unsigned long dt = millis() - touchStartTime;
    int16_t dist = (int16_t)sqrt((float)(dx * dx + dy * dy));

    if (dist < TAP_MAX_MOVE && dt < TAP_MAX_MS) {
      // TAP — guard against double-fire from touch IC glitches
      static unsigned long lastTapMs = 0;
      if (millis() - lastTapMs >= 350) {
        lastTapMs = millis();

        if (rtcSetMode) {
          // 3x2 grid tap detection — row by Y zone, column by X zone
          int contentH  = UI_H - UI_TITLE_H - UI_FOOTER_H;
          int btnH      = 54;
          int fieldAreaH = contentH - btnH;
          int rowH      = fieldAreaH / RTC_FIELD_ROWS;
          int colW      = UI_W / RTC_COLS;
          int btnY      = UI_TITLE_H + fieldAreaH;

          if (lastTy >= btnY) {
            // Button strip
            if (lastTx < UI_W / 2) {
              rtc_set(rtcEdit[0], rtcEdit[1], rtcEdit[2],
                      rtcEdit[3], rtcEdit[4], 2000 + rtcEdit[5]);
              rtcSetMode = false;
              displayPage = 2;
              motor_buzz(80);
            } else {
              rtcSetMode = false;
              displayPage = 2;
            }
            displayDirty = true;
          } else if (lastTy >= UI_TITLE_H) {
            // Field grid
            int row = (lastTy - UI_TITLE_H) / rowH;
            int col = lastTx / colW;
            if (row >= 0 && row < RTC_FIELD_ROWS && col >= 0 && col < RTC_COLS) {
              rtcSetField = row * RTC_COLS + col;
              displayDirty = true;
            }
          }
        } else if (displayPage == 2) {
          // Calculate which settings row was tapped from y position
          int contentH = UI_H - UI_TITLE_H - UI_FOOTER_H;
          int rowH = contentH / NUM_SETTINGS;
          int rowIndex = (lastTy - UI_TITLE_H) / rowH;
          if (rowIndex >= 0 && rowIndex < NUM_SETTINGS) {
            toggle_setting(rowIndex);
            displayDirty = true;
          }
        }
      }
    } else if (abs(dy) > abs(dx) * 2 && abs(dy) > SWIPE_THRESHOLD) {
      // Vertical swipe — only used for detection list scroll
      if (displayPage == 1) {
        if (dy < 0) detectionScrollOffset += 5;
        else         detectionScrollOffset = max(0, detectionScrollOffset - 5);
        displayDirty = true;
      }
    }
  }
}

void btn_task() {
  // Separate raw-input tracking from debounced state to avoid race condition
  static bool nextRaw = false, prevRaw = false;
  static bool nextStable = false, prevStable = false;
  static unsigned long nextTimer = 0, prevTimer = 0;

  bool nr = (digitalRead(BTN_NEXT) == LOW);
  bool pr = (digitalRead(BTN_PREV) == LOW);

  if (nr != nextRaw) { nextRaw = nr; nextTimer = millis(); }
  if (pr != prevRaw) { prevRaw = pr; prevTimer = millis(); }

  const uint8_t rtcMaxV[6] = {23, 59, 59, 31, 12, 99};
  const uint8_t rtcMinV[6] = { 0,  0,  0,  1,  1, 24};  // year min = 2024

  if (millis() - nextTimer >= 50) {
    if (nextRaw && !nextStable) {
      if (rtcSetMode) {
        rtcEdit[rtcSetField]++;
        if (rtcEdit[rtcSetField] > rtcMaxV[rtcSetField])
          rtcEdit[rtcSetField] = rtcMinV[rtcSetField];
        displayDirty = true;
      } else {
        displayPage = (displayPage + 1) % NUM_PAGES;
        displayDirty = true;
      }
    }
    nextStable = nextRaw;
  }
  if (millis() - prevTimer >= 50) {
    if (prevRaw && !prevStable) {
      if (rtcSetMode) {
        if (rtcEdit[rtcSetField] <= rtcMinV[rtcSetField])
          rtcEdit[rtcSetField] = rtcMaxV[rtcSetField];
        else
          rtcEdit[rtcSetField]--;
        displayDirty = true;
      } else {
        displayPage = (displayPage - 1 + NUM_PAGES) % NUM_PAGES;
        displayDirty = true;
      }
    }
    prevStable = prevRaw;
  }
}

// ============================================================================
// SERIAL COMMANDS
// ============================================================================

void dump_json() {
  Serial.println("{\"detections\":[");
  bool first = true;
  for (int i = 0; i < detectionCount; i++) {
    int idx = (detectionHead - detectionCount + i + MAX_DETECTIONS) % MAX_DETECTIONS;
    Detection* d = &detections[idx];
    if (!d->valid) continue;
    if (!first) Serial.print(","); first = false;
    const char* ts = d->type == SIG_ELRS ? "ELRS" : d->type == SIG_WIFI ? "WiFi" : "BLE";
    Serial.printf("{\"type\":\"%s\",\"rssi\":%d,\"snr\":%.1f,\"freq\":%.1f,\"id\":\"%s\","
      "\"pktType\":\"%s\",\"time\":\"%04d-%02d-%02dT%02d:%02d:%02dZ\"}",
      ts, d->rssi, d->snr, d->frequency, d->identifier,
      (d->type == SIG_ELRS && d->elrsPktType <= 3) ? elrs_pkt_type_name(d->elrsPktType) : "",
      d->year, d->month, d->day, d->hour, d->minute, d->second);
    Serial.println();
  }
  Serial.println("]}");
}

void serial_task() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n'); cmd.trim();
  if (cmd == "dump" || cmd == "json") {
    dump_json();
  } else if (cmd == "status") {
    Serial.printf("ELRS hits: %d (strongest: %d dBm)\n", totalElrsHits, elrsStrongestRSSI);
    Serial.printf("ELRS tracking: %d/%d slots\n", activeTrackCount, MAX_TRACK_SLOTS);
    for (int i = 0; i < MAX_TRACK_SLOTS; i++) {
      if (!trackSlots[i].active) continue;
      Serial.printf("  Slot#%d: %s %s  RSSI %.0f  pkts %d\n",
        trackSlots[i].id, (trackSlots[i].band == BAND_900) ? "900" : "2.4G",
        trackSlots[i].rate->name, trackSlots[i].smoothedRSSI, trackSlots[i].rxCount);
    }
    Serial.printf("WiFi: %d  BLE: %d\n", totalWifiHits, totalBleHits);
    Serial.printf("Battery: %.2fV %d%%\n", battVoltage, battPercent);
    Serial.printf("Buffer: %d/%d\n", detectionCount, MAX_DETECTIONS);
  } else if (cmd == "clear") {
    detectionCount = detectionHead = 0;
    totalElrsHits = totalWifiHits = totalBleHits = 0;
    elrsHitCount = 0;
    elrsStrongestRSSI = wifiStrongestRSSI = bleStrongestRSSI = -999;
    elrsBestSNR = -999;
    clear_all_tracks();
    displayDirty = true;
    Serial.println("Cleared");
  } else if (cmd == "help") {
    Serial.println("Commands: dump/json, status, clear, help");
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Drone Signal Scanner - LilyGO T-Display S3 Pro ===");

  // Enable LDO (powers radio, display, and peripherals)
  pinMode(LDO_EN, OUTPUT);
  digitalWrite(LDO_EN, HIGH);
  delay(20);

  // I2C (touch + battery charger)
  Wire.begin(I2C_SDA, I2C_SCL);

  // Side buttons — page navigation
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_PREV, INPUT_PULLUP);

  // Vibration motor
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  // I2S speaker
  pinMode(I2S_SD_MODE, OUTPUT);
  i2s_init();

  // --- Display init ---
  bus = new Arduino_HWSPI(LCD_DC, LCD_CS, LCD_SCLK, LCD_MOSI, LCD_MISO);
  gfx = new Arduino_ST7796(bus, LCD_RST, 0, true, LCD_WIDTH, LCD_HEIGHT, 49, 0, 0, 0);
  gfx->begin();
  gfx->setRotation(0);   // Portrait
  gfx->fillScreen(C_BG);
  gfx->setTextWrap(false);

  // Backlight via LEDC
  ledcAttach(LCD_BL, 2000, 8);
  ledcWrite(LCD_BL, 200);

  // Splash — Daring Drones logo
  gfx->draw16bitRGBBitmap(0, 0, splash_bitmap, SPLASH_W, SPLASH_H);
  delay(2000);

  // Boot log area — GB style
  gfx->fillScreen(C_BG);
  gb_box(0, 0, UI_W, UI_H, C_DIM);
  gfx->fillRect(2, 2, UI_W - 4, 18, C_DMG1);
  gfx->setTextColor(C_FG); gfx->setTextSize(1);
  int tw2 = 11 * 6; gfx->setCursor((UI_W - tw2) / 2, 6); gfx->print("SYSTEM INIT");
  gfx->drawFastHLine(2, 20, UI_W - 4, C_DIM);
  int bootY = 28;
  auto bootLog = [&](const char* label, bool ok) {
    gfx->setTextColor(C_MED); gfx->setCursor(8, bootY); gfx->print("> "); gfx->print(label);
    gfx->setTextColor(ok ? C_FG : C_DMG2);
    gfx->setCursor(UI_W - 34, bootY); gfx->print(ok ? "[OK]" : "[!!]");
    bootY += 14;
  };

  // LoRa LR1121
  // Note: Sharing SPI with LCD — we use the same bus instance
  // LCD_CS is deasserted (HIGH) during radio ops via Module CS management
  loraMod = new Module(LORA_NSS, LORA_DIO9, LORA_RESET, LORA_BUSY, SPI);
  radio = new LR1121(loraMod);
  Serial.print("[LORA] Init LR1121... ");
  int state = radio->begin(915.0, 500.0, 6, 7, 0x12, 10);
  bool loraOk = (state == RADIOLIB_ERR_NONE);
  Serial.printf("%s (err=%d)\n", loraOk ? "OK" : "FAILED", state);
  bootLog("LORA LR1121", loraOk);
  clear_all_tracks();

  // WiFi
  wifi_start();
  bootLog("WIFI PROMISCUOUS", true);

  // BLE
  ble_start();
  bootLog("BLE OPENDRONEID", true);

  // RTC
  rtc_task();
  if (rtcValid)
    Serial.printf("[RTC] %04d-%02d-%02d %02d:%02d:%02d\n",
                  rtcNow.year, rtcNow.month, rtcNow.day,
                  rtcNow.hour, rtcNow.minute, rtcNow.second);
  else
    Serial.println("[RTC] not set / invalid");
  snprintf(_buf, sizeof(_buf), "RTC %02d:%02d %04d-%02d-%02d", rtcNow.hour, rtcNow.minute, rtcNow.year, rtcNow.month, rtcNow.day);
  bootLog(_buf, rtcValid);

  // Battery
  battVoltage = battery_read_voltage();
  battPercent = battery_voltage_to_percent(battVoltage);
  Serial.printf("[BATT] %.2fV  %d%%\n", battVoltage, battPercent);
  snprintf(_buf, sizeof(_buf), "BATTERY %.1fV %d%%", battVoltage, battPercent);
  bootLog(_buf, battPercent > BATT_LOW_PERCENT);

  // Startup beep
  if (settings.speakerEnabled) speaker_beep(2, 1800);

  Serial.println("[SYS] Ready. Commands: dump, status, clear, help");
  delay(1500);

  // Show warning screen if RTC has not been set
  if (!rtcValid) {
    gfx->fillScreen(C_BG);
    gb_box(0, 0, UI_W, UI_H, C_ORANGE);
    gfx->fillRect(2, 2, UI_W - 4, 22, C_ORANGE);
    gfx->setTextColor(C_BG); gfx->setTextSize(2);
    gfx->setCursor((UI_W - 10 * 12) / 2, 4);
    gfx->print("CLOCK!");
    draw_drone_icon(UI_W / 2, 130, C_ORANGE, 1);
    gfx->setTextColor(C_ORANGE); gfx->setTextSize(2);
    gfx->setCursor((UI_W - 11 * 12) / 2, 180);
    gfx->print("RTC NOT SET");
    gfx->setTextColor(C_FG); gfx->setTextSize(1);
    gfx->setCursor(16, 220); gfx->print("Detections won't have");
    gfx->setCursor(16, 234); gfx->print("timestamps until set.");
    gfx->setCursor(16, 260); gfx->print("Set time in:");
    gfx->setTextColor(C_MED); gfx->setTextSize(1);
    gfx->setCursor(16, 274); gfx->print("CONFIG > SET RTC");
    delay(3000);
  }

  displayDirty = true;
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  nav_task();
  btn_task();
  rtc_task();
  serial_task();
  speaker_task();
  motor_task();
  battery_task();

  if (settings.elrsScanEnabled) cad_scan_task();

  if (settings.wifiScanEnabled && millis() - lastWifiScan >= WIFI_SCAN_DWELL_MS) {
    wifi_scan_task();
    lastWifiScan = millis();
  }

  if (settings.bleScanEnabled) ble_scan_task();

  display_task();
}
