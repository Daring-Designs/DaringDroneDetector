// ============================================================================
// Drone Signal Scanner - Heltec Vision Master E290
// Scans ELRS 915MHz + 2.4GHz (LR1121 LoRa CAD), WiFi (DJI Remote ID), BLE (OpenDroneID)
// Radio: Waveshare Core1121-XF (LR1121 dual-band 900MHz + 2.4GHz)
// GPS: HGLRC M100 Mini on UART (TX→GPIO38, RX→GPIO39)
// Buzzer: GPIO45 (passive piezo via transistor to 5V)
// Nav: 5-way switch (UP=8, DOWN=12, LEFT=13, RIGHT=14, CENTER=41)
// ============================================================================

#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <heltec-eink-modules.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEUtils.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include "esp_bt.h"
#include "splash_bitmap.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// LR1121 LoRa radio (dual-band 900MHz + 2.4GHz)
#define LORA_SCK    9
#define LORA_MOSI   10
#define LORA_MISO   11
#define LORA_NSS    3   // LR1121 CS
#define LORA_DIO9   40  // LR1121 interrupt (was DIO1 on SX1262)
#define LORA_RESET  42  // LR1121 reset
#define LORA_BUSY   17  // LR1121 busy

// E-paper display pins are handled by heltec-eink-modules library

// GPS UART
#define GPS_RX      39  // ESP32 RX ← GPS module TX
#define GPS_TX      38  // ESP32 TX → GPS module RX

// Buzzer
#define BUZZER_PIN  45

// Battery ADC
#define VBAT_PIN    7     // ADC1_CH6 - battery voltage via divider

// 5-Way Navigation Switch
#define NAV_UP      13
#define NAV_DOWN    12
#define NAV_LEFT    14
#define NAV_RIGHT   8
#define NAV_CENTER  41
#define LONG_PRESS_MS     7000  // Hold center to sleep/wake
#define MEDIUM_PRESS_MS   1500  // Hold center to toggle settings

// ============================================================================
// CONFIGURATION
// ============================================================================

#define MAX_DETECTIONS      50
#define DISPLAY_REFRESH_MS  5000
// CAD filtering is done via confirmation count, not RSSI
#define BLE_SCAN_TIME_SEC   1     // BLE scan window per cycle
#define WIFI_SCAN_DWELL_MS  50    // Time on each WiFi channel

// Buzzer tone settings
#define BUZZER_FREQ_HZ      2700  // Piezo resonant frequency
#define BUZZER_BEEP_MS      100   // Single beep duration
#define BUZZER_GAP_MS       80    // Gap between multi-beeps

// Battery monitoring
#define BATT_READ_INTERVAL_MS 10000  // Read battery every 10s
#define BATT_LOW_PERCENT      10     // Enter low-battery sleep below this %
#define BATT_VOLTAGE_FULL     4.2    // Fully charged LiPo
#define BATT_VOLTAGE_EMPTY    3.0    // Cutoff voltage
#define BATT_ADC_COEFFICIENT  5.33   // Calibrated against multimeter

// ============================================================================
// SETTINGS (toggled from the Settings display page)
// ============================================================================

struct Settings {
  bool buzzerEnabled;     // Alert buzzer on detection
  bool elrsScanEnabled;   // ELRS 915MHz CAD scanning
  bool wifiScanEnabled;   // WiFi promiscuous scanning
  bool bleScanEnabled;    // BLE OpenDroneID scanning
  bool gpsEnabled;        // GPS parsing
  // ELRS packet type filters (which types trigger detection)
  bool elrsFilterRC;      // RC control packets
  bool elrsFilterMSP;     // MSP data packets
  bool elrsFilterSYNC;    // Sync packets
  bool elrsFilterTLM;     // Telemetry packets
};

// Defaults - everything on
Settings settings = {
  .buzzerEnabled    = true,
  .elrsScanEnabled  = true,
  .wifiScanEnabled  = true,
  .bleScanEnabled   = true,
  .gpsEnabled       = true,
  .elrsFilterRC     = true,
  .elrsFilterMSP    = true,
  .elrsFilterSYNC   = true,
  .elrsFilterTLM    = true,
};

#define NUM_SETTINGS 9       // Number of toggleable settings

// ============================================================================
// ELRS 915MHz FCC CHANNEL TABLE (40 channels, from ExpressLRS FHSS.cpp)
// Range: 903.5 - 926.9 MHz, 600kHz spacing
// ============================================================================

const float ELRS_FREQ_TABLE[] = {
  903.5, 904.1, 904.7, 905.3, 905.9, 906.5, 907.1, 907.7,
  908.3, 908.9, 909.5, 910.1, 910.7, 911.3, 911.9, 912.5,
  913.1, 913.7, 914.3, 914.9, 915.5, 916.1, 916.7, 917.3,
  917.9, 918.5, 919.1, 919.7, 920.3, 920.9, 921.5, 922.1,
  922.7, 923.3, 923.9, 924.5, 925.1, 925.7, 926.3, 926.9
};
const int ELRS_NUM_CHANNELS = sizeof(ELRS_FREQ_TABLE) / sizeof(ELRS_FREQ_TABLE[0]);

// ============================================================================
// DETECTION TYPES AND STRUCTURES
// ============================================================================

enum SignalType {
  SIG_ELRS,
  SIG_WIFI,
  SIG_BLE
};

struct Detection {
  double lat;
  double lng;
  int rssi;
  float snr;             // SNR in dB (ELRS only, 0 for WiFi/BLE)
  SignalType type;
  float frequency;       // MHz for ELRS, channel for WiFi, 0 for BLE
  char identifier[24];   // SSID, MAC, drone ID, or channel info
  uint8_t elrsPktType;   // ELRS packet type: 0=RC, 1=MSP, 2=SYNC, 3=TLM
  uint8_t hour, minute, second;
  uint8_t day, month;
  uint16_t year;
  bool valid;
};

// ============================================================================
// GLOBALS
// ============================================================================

// Radio - LR1121 dual-band LoRa (FSPI bus, e-paper uses HSPI)
SPIClass loraSPI(FSPI);
Module* loraMod = nullptr;
LR1121* radio = nullptr;

// E-paper display (heltec-eink-modules handles all pin/SPI config)
EInkDisplay_VisionMasterE290 display;

// GPS
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

// BLE
BLEScan* pBLEScan = nullptr;

// Detection ring buffer
Detection detections[MAX_DETECTIONS];
int detectionHead = 0;
int detectionCount = 0;

// Scan counters (reset each display cycle for "recent" counts)
volatile int elrsHitCount = 0;
volatile int elrsStrongestRSSI = -999;
volatile float elrsBestSNR = -999;
volatile int wifiDetectedCount = 0;
volatile int wifiStrongestRSSI = -999;
volatile int bleRemoteIDCount = 0;
volatile int bleStrongestRSSI = -999;
int totalElrsHits = 0;
int totalWifiHits = 0;
int totalBleHits = 0;

// Battery state
float battVoltage = 0.0;
int battPercent = -1;  // -1 = not yet read
unsigned long lastBattRead = 0;
bool battLowSleep = false;  // True if sleeping due to low battery

// Timing
unsigned long lastDisplayRefresh = 0;
unsigned long lastCADSweep = 0;
unsigned long lastWifiScan = 0;
unsigned long lastBLEScan = 0;

// Navigation state
bool centerPressed = false;
unsigned long centerDownTime = 0;
bool deviceSleeping = false;
int displayPage = 0;       // 0=summary, 1=detections, 2=GPS, 3=settings
int lastDisplayPage = -1;  // Track page changes for full vs partial refresh
int partialRefreshCount = 0;
#define NUM_PAGES 4
#define MAX_PARTIAL_REFRESHES 20  // Do a full refresh every N partials to prevent ghosting

// Settings page cursor
int settingsCursor = 0;  // Which setting is highlighted

// Detections page scroll offset
int detectionScrollOffset = 0;

// WiFi promiscuous mode
volatile bool wifiScanActive = false;

// ============================================================================
// OpenDroneID structures (must be before function prototypes)
// ============================================================================

#define ODID_SERVICE_UUID   0xFFFA
#define ODID_APP_CODE       0x0D
#define ODID_MSG_BASIC_ID   0x0
#define ODID_MSG_LOCATION   0x1
#define ODID_MSG_SYSTEM     0x4
#define ODID_MSG_OPERATOR   0x5

struct ODIDData {
  char droneId[21];
  double droneLat;
  double droneLng;
  float droneAlt;
  char operatorId[21];
  bool hasBasicId;
  bool hasLocation;
  bool hasOperatorId;
};

// Buzzer state (non-blocking beep pattern)
unsigned long buzzerStartTime = 0;
int buzzerBeepsRemaining = 0;
bool buzzerToneOn = false;
int buzzerCurrentFreq = BUZZER_FREQ_HZ;  // Current tone frequency (modulated by RSSI)

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

// ElrsAirRate must be defined here (before Arduino auto-generated prototypes)
struct ElrsAirRate {
  float    bw;        // Bandwidth in kHz
  uint8_t  sf;        // Spreading factor
  uint8_t  cr;        // Coding rate denominator (7=4/7, 8=4/8)
  bool     crLongInterleave; // true for 2.4GHz long interleaver CR
  uint8_t  payloadLen; // OTA payload bytes (implicit header)
  const char* name;   // Human-readable rate name
};

void gps_task();
void cad_scan_task();
void wifi_scan_task();
void ble_scan_task();
void battery_task();
void log_hit(SignalType type, int rssi, float freq, const char* id, float snr = 0, uint8_t elrsPktType = 0xFF);
void display_task();
void nav_task();
void buzzer_task();
void buzzer_beep(int count, int freq = BUZZER_FREQ_HZ);
void dump_json();
void enter_sleep();
void wake_up();

// ============================================================================
// GPS TASK - Non-blocking NMEA parsing
// ============================================================================

unsigned long lastGpsDebug = 0;
uint32_t gpsCharsProcessed = 0;

void gps_task() {
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    gps.encode(c);
    gpsCharsProcessed++;
  }

  // Print GPS debug info every 5 seconds
  if (millis() - lastGpsDebug >= 5000) {
    lastGpsDebug = millis();
    Serial.printf("[GPS] chars:%lu sentences:%lu failed:%lu sats:%d fix:%s\n",
      gpsCharsProcessed,
      gps.sentencesWithFix(),
      gps.failedChecksum(),
      gps.satellites.isValid() ? (int)gps.satellites.value() : 0,
      gps.location.isValid() ? "yes" : "no");

    // If no data after 30 seconds, periodically send wake commands
    if (gpsCharsProcessed < 5 && millis() > 30000) {
      GPSSerial.println("$PCAS10,0*1C");
      GPSSerial.println("$PMTK101*32");
      GPSSerial.println("");
      Serial.println("[GPS] Sent wake commands");
    }
  }
}

// ============================================================================
// ELRS PACKET TYPE HELPERS
// ============================================================================

const char* elrs_pkt_type_name(uint8_t pktType) {
  switch (pktType) {
    case 0: return "RC";
    case 1: return "MSP";
    case 2: return "SYNC";
    case 3: return "TLM";
    default: return "?";
  }
}

bool elrs_pkt_type_allowed(uint8_t pktType) {
  switch (pktType) {
    case 0: return settings.elrsFilterRC;
    case 1: return settings.elrsFilterMSP;
    case 2: return settings.elrsFilterSYNC;
    case 3: return settings.elrsFilterTLM;
    default: return true;
  }
}

// ============================================================================
// LOG HIT - Store detection in ring buffer
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

  // Attach GPS data
  if (gps.location.isValid()) {
    d->lat = gps.location.lat();
    d->lng = gps.location.lng();
  } else {
    d->lat = 0.0;
    d->lng = 0.0;
  }

  if (gps.time.isValid() && gps.date.isValid()) {
    d->hour = gps.time.hour();
    d->minute = gps.time.minute();
    d->second = gps.time.second();
    d->day = gps.date.day();
    d->month = gps.date.month();
    d->year = gps.date.year();
  } else {
    d->hour = d->minute = d->second = 0;
    d->day = d->month = 0;
    d->year = 0;
  }

  detectionHead = (detectionHead + 1) % MAX_DETECTIONS;
  if (detectionCount < MAX_DETECTIONS) detectionCount++;

  // Update counters
  switch (type) {
    case SIG_ELRS:
      elrsHitCount++;
      totalElrsHits++;
      if (rssi > elrsStrongestRSSI) elrsStrongestRSSI = rssi;
      if (snr > elrsBestSNR) elrsBestSNR = snr;
      break;
    case SIG_WIFI:
      wifiDetectedCount++;
      totalWifiHits++;
      if (rssi > wifiStrongestRSSI) wifiStrongestRSSI = rssi;
      break;
    case SIG_BLE:
      bleRemoteIDCount++;
      totalBleHits++;
      if (rssi > bleStrongestRSSI) bleStrongestRSSI = rssi;
      break;
  }

  // Debug output
  Serial.printf("[HIT] type=%s rssi=%d freq=%.1f id=%s lat=%.6f lng=%.6f\n",
    type == SIG_ELRS ? "ELRS" : type == SIG_WIFI ? "WiFi" : "BLE",
    rssi, freq, id, d->lat, d->lng);

  // Buzzer alert: beep count by type, pitch by RSSI (stronger = higher)
  if (settings.buzzerEnabled) {
    int freq = buzzer_freq_from_rssi(rssi);
    switch (type) {
      case SIG_ELRS: buzzer_beep(1, freq); break;
      case SIG_WIFI: buzzer_beep(2, freq); break;
      case SIG_BLE:  buzzer_beep(3, freq); break;
    }
  }
}

// ============================================================================
// ELRS DETECTION - Dual-band CAD screening + implicit header verification
// ============================================================================
//
// Alternates between 900MHz and 2.4GHz bands each CAD window.
//
// Phase 1: CAD sweep across ELRS FHSS channels.
//          900MHz: 40 channels at BW500, 2.4GHz: 80 channels at BW812.
//          When 4+ hits on 3+ channels detected, trigger Phase 2.
//
// Phase 2: Attempt to receive actual ELRS packets using implicit header mode.
//          Tries all known air rates for the active band.
//          If receive() succeeds -> confirmed ELRS with real RSSI.

// ---- 900MHz ELRS rates (SX127x + LR1121) ----
// All use BW500. OTA4=8 bytes, OTA8=13 bytes.
const ElrsAirRate ELRS_900_RATES[] = {
  //  BW    SF  CR  LI    Len  Name
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

// ---- 2.4GHz ELRS rates (SX1280-compatible LoRa modes) ----
// All use BW812 (812.5 kHz). Long interleaver coding rates.
// FLRC modes (F500, D500, D250) cannot be detected via LoRa CAD.
const ElrsAirRate ELRS_24_RATES[] = {
  //  BW      SF  CR  LI   Len  Name
  { 812.5,  5,  6, true,  8,  "2.4 500Hz"     },  // CR LI 4/6
  { 812.5,  5,  8, true, 13,  "2.4 333HzFull" },  // CR LI 4/8
  { 812.5,  6,  8, true,  8,  "2.4 250Hz"     },  // CR LI 4/8
  { 812.5,  7,  8, true,  8,  "2.4 150Hz"     },  // CR LI 4/8
  { 812.5,  7,  8, true, 13,  "2.4 100HzFull" },  // CR LI 4/8
  { 812.5,  8,  8, true,  8,  "2.4 50Hz"      },  // CR LI 4/8
};
#define ELRS_24_NUM_RATES 6

// ---- 2.4GHz ELRS FHSS channel table (80 channels) ----
// From ExpressLRS FHSS.cpp: 2400.4 to 2479.4 MHz, 1 MHz spacing
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

// ---- Band switching ----
enum ElrsBand { BAND_900, BAND_24 };
ElrsBand elrsCurrentBand = BAND_900;

// ---- Phase 1: CAD sweep config ----
int cadChannelIndex = 0;

#define CAD_WINDOW_MS           5000
#define CAD_MIN_CHANNELS        3
#define CAD_MIN_HITS            4
#define ELRS_REPORT_COOLDOWN_MS 15000

// Max channel count across both bands (80 for 2.4GHz)
uint8_t cadChannelHits[80] = {0};
int cadTotalHits = 0;
unsigned long cadWindowStart = 0;

// ---- Phase 2: Verification state ----
enum ElrsScanPhase { PHASE_CAD, PHASE_VERIFY, PHASE_TRACK };
ElrsScanPhase elrsScanPhase = PHASE_CAD;
int verifyRateIndex = 0;
int verifyRatesTried = 0;

// ---- Phase 3: Tracking state ----
#define TRACK_TIMEOUT_MS        10000  // Release lock after 10s with no packets
#define TRACK_LOG_FAST_MS       300    // Log interval when signal is strong (RSSI >= -30)
#define TRACK_LOG_SLOW_MS       3000   // Log interval when signal is weak (RSSI <= -90)
#define TRACK_CAD_INTERVAL      15     // Do a CAD sweep on alternate band every N track attempts

// Dynamic tracking interval: closer signal = faster updates (and faster beeps)
// RSSI -90 or worse -> 5000ms, RSSI -30 or better -> 1000ms, linear between
unsigned long track_log_interval(float rssi) {
  int clamped = constrain((int)rssi, -90, -30);
  return map(clamped, -90, -30, TRACK_LOG_SLOW_MS, TRACK_LOG_FAST_MS);
}
const ElrsAirRate* trackRate = NULL;   // Locked rate during tracking
ElrsBand trackBand = BAND_900;        // Band we're tracking on
int trackChannels[80];                 // Channel indices with CAD hits at time of lock
int trackNumChannels = 0;             // Number of active tracking channels
int trackChIdx = 0;                   // Current index into trackChannels[]
unsigned long trackLastRx = 0;        // Last successful receive timestamp
unsigned long trackLastLog = 0;       // Last time we logged a detection
int trackRxCount = 0;                 // Packets received during this tracking session
float trackBestRSSI = -999;           // Best RSSI during current log interval
float trackBestSNR = -999;            // Best SNR during current log interval
float trackSmoothedRSSI = -999;       // Smoothed RSSI for interval/frequency calc (not reset)
uint8_t trackLastPktType = 0xFF;      // Last packet type seen while tracking
int trackCadCounter = 0;              // Counter for interleaved CAD sweeps

// ---- Shared state ----
unsigned long elrsLastReportTime = 0;
const char* lastElrsDetectedRate = NULL;
uint8_t lastElrsPktType = 0xFF;  // Last detected ELRS packet type

// Get current band's channel count
int elrs_num_channels() {
  return (elrsCurrentBand == BAND_900) ? ELRS_NUM_CHANNELS : ELRS_24_NUM_CHANNELS;
}

// Get current band's frequency table
const float* elrs_freq_table() {
  return (elrsCurrentBand == BAND_900) ? ELRS_FREQ_TABLE : ELRS_24_FREQ_TABLE;
}

// Get current band's rate table and count
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

// Configure radio for CAD scanning on current band
void elrs_restore_cad_config() {
  int16_t state;
  if (elrsCurrentBand == BAND_900) {
    state = radio->setBandwidth(500.0);
    if (state != RADIOLIB_ERR_NONE) Serial.printf("[ELRS] setBW 500 err=%d\n", state);
    state = radio->setFrequency(915.0, true);  // skip image cal on cross-band switch
    if (state != RADIOLIB_ERR_NONE) Serial.printf("[ELRS] setFreq 915 err=%d\n", state);
    radio->setSpreadingFactor(6);
    radio->setCodingRate(7);
  } else {
    state = radio->setBandwidth(812.5, true);
    if (state != RADIOLIB_ERR_NONE) Serial.printf("[ELRS] setBW 812 err=%d\n", state);
    state = radio->setFrequency(2440.0, true);  // skip image cal on cross-band switch
    if (state != RADIOLIB_ERR_NONE) Serial.printf("[ELRS] setFreq 2440 err=%d\n", state);
    radio->setSpreadingFactor(6);
    radio->setCodingRate(8, true);
  }
  radio->setCRC(false);
  radio->explicitHeader();
}

// Switch to the other band and reset for a new CAD window
void elrs_switch_band() {
  elrsCurrentBand = (elrsCurrentBand == BAND_900) ? BAND_24 : BAND_900;
  cadChannelIndex = 0;
  elrs_restore_cad_config();
}

// Find top 3 channels by hit count (works for either band)
void elrs_top_channels(int* out, int count) {
  int numCh = elrs_num_channels();
  for (int n = 0; n < count; n++) {
    int best = -1;
    for (int i = 0; i < numCh; i++) {
      if (cadChannelHits[i] == 0) continue;
      bool skip = false;
      for (int j = 0; j < n; j++) {
        if (out[j] == i) { skip = true; break; }
      }
      if (skip) continue;
      if (best < 0 || cadChannelHits[i] > cadChannelHits[best]) best = i;
    }
    out[n] = best;
  }
}

void cad_scan_task() {
  unsigned long now = millis();
  int numChannels = elrs_num_channels();
  const float* freqTable = elrs_freq_table();

  // ==== PHASE 2: Implicit header receive verification ====
  if (elrsScanPhase == PHASE_VERIFY) {
    int numRates = elrs_num_rates();
    const ElrsAirRate* rates = elrs_rates();

    if (verifyRatesTried >= numRates) {
      Serial.printf("[ELRS %s] Verify: no ELRS packet confirmed\n", elrs_band_name());
      elrs_reset_window();
      elrs_switch_band();  // Switch bands after verify so 900MHz doesn't starve 2.4GHz
      elrsScanPhase = PHASE_CAD;
      return;
    }

    int rateIdx = verifyRateIndex % numRates;
    const ElrsAirRate* rate = &rates[rateIdx];

    // Configure radio for this exact ELRS mode
    radio->setBandwidth(rate->bw, elrsCurrentBand == BAND_24);
    radio->setSpreadingFactor(rate->sf);
    radio->setCodingRate(rate->cr, rate->crLongInterleave);
    radio->setSyncWord(0x12);
    radio->implicitHeader(rate->payloadLen);
    radio->setCRC(false);

    // Listen on top 3 channels from CAD phase
    int hotCh[3];
    elrs_top_channels(hotCh, 3);

    Serial.printf("[ELRS %s] Verify %s (BW%d/SF%d/%dB): ",
                  elrs_band_name(), rate->name, (int)rate->bw, rate->sf, rate->payloadLen);

    uint8_t rxBuf[16];
    bool received = false;
    float rxRssi = 0;
    float rxSnr = 0;
    float rxFreq = 0;
    uint8_t rxPktType = 0;

    for (int c = 0; c < 3; c++) {
      if (hotCh[c] < 0) continue;
      float freq = freqTable[hotCh[c]];
      radio->setFrequency(freq);

      int state = radio->receive(rxBuf, rate->payloadLen, 100);

      if (state == RADIOLIB_ERR_NONE) {
        // ELRS packet type: top 2 bits of byte 0 (0=RC, 1=MSP, 2=SYNC, 3=TLM)
        rxPktType = (rxBuf[0] >> 6) & 0x03;
        rxRssi = radio->getRSSI();
        rxSnr = radio->getSNR();
        rxFreq = freq;
        received = true;
        Serial.printf("ch%d/%.1f PACKET %dB type=%d RSSI:%.0f SNR:%.1f\n",
                      hotCh[c], freq, rate->payloadLen, rxPktType, rxRssi, rxSnr);
        break;
      } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
        Serial.printf("ch%d err=%d ", hotCh[c], state);
      }
    }

    if (received) {
      // === CONFIRMED ELRS — enter tracking mode ===
      lastElrsDetectedRate = rate->name;
      lastElrsPktType = rxPktType;
      elrsLastReportTime = now;

      int activeChannels = 0;
      for (int i = 0; i < numChannels; i++) {
        if (cadChannelHits[i] > 0) activeChannels++;
      }

      Serial.printf("[ELRS %s] CONFIRMED %s (%s): RSSI %.0f, SNR %.1f, %d ch active -> TRACKING\n",
                    elrs_band_name(), rate->name, elrs_pkt_type_name(rxPktType), rxRssi, rxSnr, activeChannels);

      // Check if this packet type is allowed by filter
      if (!elrs_pkt_type_allowed(rxPktType)) {
        Serial.printf("[ELRS] Packet type %s filtered out, skipping\n", elrs_pkt_type_name(rxPktType));
        elrs_reset_window();
        elrs_switch_band();
        elrsScanPhase = PHASE_CAD;
        return;
      }

      // Log initial detection
      char chanStr[32];
      const char* bandPfx = (elrsCurrentBand == BAND_900) ? "900 " : "";
      snprintf(chanStr, sizeof(chanStr), "%s%s/%dch", bandPfx, rate->name, activeChannels);
      log_hit(SIG_ELRS, (int)rxRssi, rxFreq, chanStr, rxSnr, rxPktType);

      // Set up tracking state
      trackRate = rate;
      trackBand = elrsCurrentBand;
      trackNumChannels = 0;
      for (int i = 0; i < numChannels && trackNumChannels < 80; i++) {
        if (cadChannelHits[i] > 0) {
          trackChannels[trackNumChannels++] = i;
        }
      }
      trackChIdx = 0;
      trackLastRx = now;
      trackLastLog = now;
      trackRxCount = 1;
      trackBestRSSI = rxRssi;
      trackBestSNR = rxSnr;
      trackSmoothedRSSI = rxRssi;
      trackLastPktType = rxPktType;
      trackCadCounter = 0;

      elrsScanPhase = PHASE_TRACK;
      return;
    }

    Serial.println("no packet");
    verifyRateIndex++;
    verifyRatesTried++;
    return;
  }

  // ==== PHASE 3: Track locked signal ====
  if (elrsScanPhase == PHASE_TRACK) {
    // Check timeout — signal lost
    if (now - trackLastRx >= TRACK_TIMEOUT_MS) {
      Serial.printf("[ELRS %s] TRACK: signal lost after %ds, %d pkts total\n",
                    elrs_band_name(), TRACK_TIMEOUT_MS / 1000, trackRxCount);
      // Log final update if we have pending data
      if (trackBestRSSI > -999 && now - trackLastLog > 1000) {
        char chanStr[32];
        const char* bandPfx = (trackBand == BAND_900) ? "900 " : "";
        snprintf(chanStr, sizeof(chanStr), "%s%s/LOST", bandPfx, trackRate->name);
        log_hit(SIG_ELRS, (int)trackBestRSSI, 0, chanStr, trackBestSNR, trackLastPktType);
      }
      elrs_reset_window();
      elrs_switch_band();
      elrsScanPhase = PHASE_CAD;
      return;
    }

    // Periodically do a quick CAD sweep on the OTHER band so we don't miss new signals
    trackCadCounter++;
    if (trackCadCounter >= TRACK_CAD_INTERVAL) {
      trackCadCounter = 0;
      // Quick CAD: switch to other band, scan a few channels, switch back
      ElrsBand otherBand = (trackBand == BAND_900) ? BAND_24 : BAND_900;
      elrsCurrentBand = otherBand;
      elrs_restore_cad_config();

      const float* otherFreqTable = elrs_freq_table();
      int otherNumCh = elrs_num_channels();
      // Scan 8 random channels on the other band
      int cadHits = 0;
      for (int i = 0; i < 8 && i < otherNumCh; i++) {
        int ch = random(0, otherNumCh);
        radio->setFrequency(otherFreqTable[ch]);
        if (radio->scanChannel() == RADIOLIB_LORA_DETECTED) {
          cadHits++;
        }
      }
      if (cadHits >= 2) {
        Serial.printf("[ELRS] TRACK: %d CAD hits on %s band — noted\n", cadHits,
                      otherBand == BAND_900 ? "900" : "2.4G");
      }

      // Switch back to tracking band and restore tracking rate config
      elrsCurrentBand = trackBand;
      radio->setBandwidth(trackRate->bw, trackBand == BAND_24);
      radio->setSpreadingFactor(trackRate->sf);
      radio->setCodingRate(trackRate->cr, trackRate->crLongInterleave);
      radio->setSyncWord(0x12);
      radio->implicitHeader(trackRate->payloadLen);
      radio->setCRC(false);
      return;  // Spent this iteration on CAD, try receive next time
    }

    // Normal tracking: quick receive on next active channel
    if (trackNumChannels == 0) {
      // Shouldn't happen, but bail
      elrs_reset_window();
      elrs_switch_band();
      elrsScanPhase = PHASE_CAD;
      return;
    }

    // Try up to 3 channels per iteration with short timeouts for faster tracking
    const float* tFreqTable = (trackBand == BAND_900) ? ELRS_FREQ_TABLE : ELRS_24_FREQ_TABLE;

    for (int attempt = 0; attempt < 3; attempt++) {
      int ch = trackChannels[trackChIdx % trackNumChannels];
      trackChIdx++;
      float freq = tFreqTable[ch];
      radio->setFrequency(freq);

      uint8_t rxBuf[16];
      int state = radio->receive(rxBuf, trackRate->payloadLen, 20);  // 20ms timeout per channel

      if (state == RADIOLIB_ERR_NONE) {
        uint8_t pktType = (rxBuf[0] >> 6) & 0x03;
        float rssi = radio->getRSSI();
        float snr = radio->getSNR();

        trackLastRx = now;
        trackRxCount++;
        trackLastPktType = pktType;
        lastElrsPktType = pktType;
        if (rssi > trackBestRSSI) trackBestRSSI = rssi;
        if (snr > trackBestSNR) trackBestSNR = snr;
        // Exponential moving average: 30% new, 70% old — smooths jitter
        trackSmoothedRSSI = (trackSmoothedRSSI < -900) ? rssi : (0.3f * rssi + 0.7f * trackSmoothedRSSI);

        // Update global display stats
        if ((int)rssi > elrsStrongestRSSI) elrsStrongestRSSI = (int)rssi;
        if (snr > elrsBestSNR) elrsBestSNR = snr;

        // Periodic log entry while tracking (interval scales with smoothed RSSI)
        unsigned long logInterval = track_log_interval(trackSmoothedRSSI);
        if (now - trackLastLog >= logInterval) {
          if (elrs_pkt_type_allowed(pktType)) {
            char chanStr[32];
            const char* bandPfx = (trackBand == BAND_900) ? "900 " : "";
            snprintf(chanStr, sizeof(chanStr), "%s%s/TRK", bandPfx, trackRate->name);
            log_hit(SIG_ELRS, (int)trackBestRSSI, freq, chanStr, trackBestSNR, pktType);
            elrsLastReportTime = now;
          }
          trackLastLog = now;
          trackBestRSSI = -999;
          trackBestSNR = -999;
          Serial.printf("[ELRS %s] TRACK: %s %d pkts, RSSI %.0f SNR %.1f [%lums]\n",
                        elrs_band_name(), trackRate->name, trackRxCount, rssi, snr, logInterval);
        }
        break;  // Got a packet, done for this iteration
      }
    }
    return;
  }

  // ==== PHASE 1: CAD sweep ====

  // Evaluate window when it expires
  if (cadWindowStart > 0 && now - cadWindowStart >= CAD_WINDOW_MS) {
    int activeChannels = 0;
    for (int i = 0; i < numChannels; i++) {
      if (cadChannelHits[i] > 0) activeChannels++;
    }

    bool triggered = (activeChannels >= CAD_MIN_CHANNELS && cadTotalHits >= CAD_MIN_HITS);

    if (triggered && now - elrsLastReportTime > ELRS_REPORT_COOLDOWN_MS) {
      Serial.printf("[ELRS %s] CAD: %d hits, %d ch -> verifying\n",
                    elrs_band_name(), cadTotalHits, activeChannels);

      elrsScanPhase = PHASE_VERIFY;
      verifyRateIndex = 0;
      verifyRatesTried = 0;
      return;
    }

    Serial.printf("[ELRS %s] CAD: %d hits, %d ch%s\n",
                  elrs_band_name(), cadTotalHits, activeChannels,
                  triggered ? " (cooldown)" : "");

    // Window done — switch bands for next window
    elrs_reset_window();
    elrs_switch_band();
    return;
  }

  if (cadWindowStart == 0) {
    cadWindowStart = now;
  }

  // One channel per loop iteration - wrap at end of FHSS table
  if (cadChannelIndex >= numChannels) {
    cadChannelIndex = 0;
    return;
  }

  float freq = freqTable[cadChannelIndex];

  int state = radio->setFrequency(freq);
  if (state != RADIOLIB_ERR_NONE) {
    cadChannelIndex++;
    return;
  }

  state = radio->scanChannel();

  if (state == RADIOLIB_LORA_DETECTED) {
    cadTotalHits++;
    cadChannelHits[cadChannelIndex]++;
  }

  cadChannelIndex++;
}

// ============================================================================
// WIFI SCAN - Promiscuous mode for DJI Remote ID / WiFi NaN beacons
// ============================================================================

// WiFi promiscuous callback - called from WiFi task context
void wifi_sniffer_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;  // Only management frames

  const wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* frame = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;
  int rssi = pkt->rx_ctrl.rssi;

  if (len < 24) return;

  // Frame control field
  uint8_t frameType = (frame[0] >> 2) & 0x03;    // Type
  uint8_t frameSubtype = (frame[0] >> 4) & 0x0F; // Subtype

  // Check for beacon frames (type=0, subtype=8) and probe responses (type=0, subtype=5)
  // Also check for action frames (type=0, subtype=13) which WiFi NaN uses
  if (frameType != 0) return;

  if (frameSubtype == 8 || frameSubtype == 5) {
    // Beacon or probe response - extract SSID
    // Fixed parameters are 12 bytes after the 24-byte MAC header
    int pos = 36;  // 24 (MAC header) + 12 (fixed params)

    if (pos + 2 > len) return;

    // First tagged parameter should be SSID (tag 0)
    if (frame[pos] == 0) {
      int ssidLen = frame[pos + 1];
      if (ssidLen > 0 && ssidLen <= 32 && pos + 2 + ssidLen <= len) {
        char ssid[33] = {0};
        memcpy(ssid, &frame[pos + 2], ssidLen);

        // Look for DJI-related SSIDs or any drone-like identifiers
        // DJI drones broadcast SSIDs like "DJI-XXXXXXXX" or contain Remote ID
        if (strstr(ssid, "DJI") || strstr(ssid, "drone") || strstr(ssid, "DRONE") ||
            strstr(ssid, "Skydio") || strstr(ssid, "Autel") || strstr(ssid, "FIMI")) {
          // Format MAC address
          char macStr[18];
          snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
            frame[10], frame[11], frame[12], frame[13], frame[14], frame[15]);

          Serial.printf("[WiFi] SSID: %s  MAC: %s  RSSI: %d\n", ssid, macStr, rssi);
          log_hit(SIG_WIFI, rssi, 0, ssid);
        }
      }
    }

    // Also scan for WiFi NaN / Remote ID Vendor Specific elements (tag 221)
    // Walk through all tagged parameters looking for ASTM Remote ID
    pos = 36;
    while (pos + 2 < len) {
      uint8_t tagNum = frame[pos];
      uint8_t tagLen = frame[pos + 1];
      if (pos + 2 + tagLen > len) break;

      if (tagNum == 221 && tagLen >= 4) {
        const uint8_t* data = &frame[pos + 2];
        const char* ridType = NULL;

        // ASTM F3411 / ASD-STAN standard Remote ID
        if (data[0] == 0xFA && data[1] == 0x0B && data[2] == 0xBC)
          ridType = "ASTM-RID";
        // DJI Proprietary DroneID (OUI 26:37:12)
        else if (data[0] == 0x26 && data[1] == 0x37 && data[2] == 0x12)
          ridType = "DJI-RID";
        // French DRI (FRDID)
        else if (data[0] == 0x6A && data[1] == 0x5C && data[2] == 0x35)
          ridType = "FR-RID";
        // Parrot SA
        else if (data[0] == 0x90 && data[1] == 0x3A && data[2] == 0xE6)
          ridType = "Parrot";
        // WiFi Alliance NaN in beacon
        else if (data[0] == 0x50 && data[1] == 0x6F && data[2] == 0x9A && data[3] == 0x13)
          ridType = "NaN-RID";

        if (ridType) {
          char macStr[18];
          snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
            frame[10], frame[11], frame[12], frame[13], frame[14], frame[15]);
          char desc[40];
          snprintf(desc, sizeof(desc), "%s %s", ridType, macStr);
          Serial.printf("[WiFi] %s detected MAC: %s RSSI: %d\n", ridType, macStr, rssi);
          log_hit(SIG_WIFI, rssi, 0, desc);
        }
      }
      pos += 2 + tagLen;
    }
  }

  // WiFi NaN Action frames (subtype 13)
  // NaN Remote ID uses dest MAC 51:6F:9A:01:00:00
  if (frameSubtype == 13 && len > 30) {
    if (frame[4] == 0x51 && frame[5] == 0x6F && frame[6] == 0x9A &&
        frame[7] == 0x01 && frame[8] == 0x00 && frame[9] == 0x00) {
      char macStr[18];
      snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
        frame[10], frame[11], frame[12], frame[13], frame[14], frame[15]);
      Serial.printf("[WiFi] NaN Remote ID action frame MAC: %s RSSI: %d\n", macStr, rssi);
      char desc[40];
      snprintf(desc, sizeof(desc), "NaN-RID %s", macStr);
      log_hit(SIG_WIFI, rssi, 0, desc);
    }
  }
}

// Channel hop sequence: dwell on ch6 (NaN Remote ID) every other hop
const uint8_t WIFI_HOP_SEQ[] = { 1,6, 2,6, 3,6, 4,6, 5,6, 7,6, 8,6, 9,6, 10,6, 11,6, 12,6, 13,6 };
#define WIFI_HOP_LEN (sizeof(WIFI_HOP_SEQ))
int wifiHopIndex = 0;

void wifi_scan_task() {
  esp_wifi_set_channel(WIFI_HOP_SEQ[wifiHopIndex], WIFI_SECOND_CHAN_NONE);
  wifiHopIndex = (wifiHopIndex + 1) % WIFI_HOP_LEN;
}

void wifi_start() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(10);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_cb);

  // Set to channel 1 to start
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  wifiScanActive = true;

  Serial.println("[WiFi] Promiscuous mode started");
}

void wifi_stop() {
  esp_wifi_set_promiscuous(false);
  wifiScanActive = false;
}

// ============================================================================
// BLE SCAN - OpenDroneID packet detection and parsing
// ============================================================================

void parse_odid_basic_id(const uint8_t* payload, ODIDData* odid) {
  // Bytes 2-21: UAS ID (20 bytes, null-terminated ASCII)
  memcpy(odid->droneId, &payload[2], 20);
  odid->droneId[20] = '\0';
  odid->hasBasicId = true;
}

void parse_odid_location(const uint8_t* payload, ODIDData* odid) {
  // Bytes 5-8: Latitude (int32 LE) / 10,000,000
  int32_t rawLat;
  memcpy(&rawLat, &payload[5], 4);
  odid->droneLat = (double)rawLat / 1e7;

  // Bytes 9-12: Longitude (int32 LE) / 10,000,000
  int32_t rawLon;
  memcpy(&rawLon, &payload[9], 4);
  odid->droneLng = (double)rawLon / 1e7;

  // Bytes 15-16: Altitude Geo (uint16 LE) * 0.5 - 1000
  uint16_t rawAlt;
  memcpy(&rawAlt, &payload[15], 2);
  odid->droneAlt = (float)rawAlt * 0.5f - 1000.0f;

  odid->hasLocation = true;
}

void parse_odid_operator_id(const uint8_t* payload, ODIDData* odid) {
  // Bytes 2-21: Operator ID (20 bytes, null-terminated ASCII)
  memcpy(odid->operatorId, &payload[2], 20);
  odid->operatorId[20] = '\0';
  odid->hasOperatorId = true;
}

volatile int bleAdvCount = 0;  // Total BLE advertisements seen per scan

// BLE scan callback class
class ODIDAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    bleAdvCount++;
    int rssi = advertisedDevice.getRSSI();

    // Get raw advertisement payload
    uint8_t* payload = advertisedDevice.getPayload();
    size_t payloadLen = advertisedDevice.getPayloadLength();

    // Walk AD structures looking for Service Data (type 0x16) with UUID 0xFFFA
    size_t pos = 0;
    while (pos + 1 < payloadLen) {
      uint8_t adLen = payload[pos];
      if (adLen == 0 || pos + 1 + adLen > payloadLen) break;

      uint8_t adType = payload[pos + 1];

      // Service Data - 16-bit UUID
      if (adType == 0x16 && adLen >= 6) {
        uint16_t uuid = payload[pos + 2] | (payload[pos + 3] << 8);

        if (uuid == ODID_SERVICE_UUID) {
          // Check application code
          if (payload[pos + 4] == ODID_APP_CODE && adLen >= 29) {
            // pos+5 = counter byte, pos+6..pos+30 = 25-byte ODID message
            const uint8_t* msg = &payload[pos + 6];
            uint8_t msgType = (msg[0] >> 4) & 0x0F;

            ODIDData odid = {};

            switch (msgType) {
              case ODID_MSG_BASIC_ID:
                parse_odid_basic_id(msg, &odid);
                Serial.printf("[BLE] OpenDroneID Basic ID: %s RSSI: %d\n", odid.droneId, rssi);
                log_hit(SIG_BLE, rssi, 0, odid.droneId);
                break;

              case ODID_MSG_LOCATION:
                parse_odid_location(msg, &odid);
                Serial.printf("[BLE] OpenDroneID Location: %.6f, %.6f alt=%.1fm RSSI: %d\n",
                  odid.droneLat, odid.droneLng, odid.droneAlt, rssi);
                {
                  char locStr[24];
                  snprintf(locStr, sizeof(locStr), "%.4f,%.4f", odid.droneLat, odid.droneLng);
                  log_hit(SIG_BLE, rssi, 0, locStr);
                }
                break;

              case ODID_MSG_OPERATOR:
                parse_odid_operator_id(msg, &odid);
                Serial.printf("[BLE] OpenDroneID Operator: %s RSSI: %d\n", odid.operatorId, rssi);
                log_hit(SIG_BLE, rssi, 0, odid.operatorId);
                break;

              default:
                // System, Auth, Self ID, Message Pack - log generically
                {
                  char typeStr[16];
                  snprintf(typeStr, sizeof(typeStr), "ODID-type%d", msgType);
                  log_hit(SIG_BLE, rssi, 0, typeStr);
                }
                break;
            }
          }
        }
      }

      pos += 1 + adLen;
    }
  }
};

bool bleScanRunning = false;
unsigned long bleScanStartTime = 0;

// Callback invoked when BLE scan completes
void bleScanComplete(BLEScanResults scanResults) {
  bleScanRunning = false;
  Serial.printf("[BLE] Scan complete, %d adverts received\n", bleAdvCount);
  // Re-enable WiFi promiscuous after BLE scan
  if (settings.wifiScanEnabled) esp_wifi_set_promiscuous(true);
}

void ble_scan_task() {
  // Skip BLE scans during close ELRS tracking — BLE blocks for 1s and kills tracking rate
  if (elrsScanPhase == PHASE_TRACK && trackSmoothedRSSI > -60) return;

  if (bleScanRunning) {
    // Safety timeout - if scan hasn't completed in 2x expected time, reset
    if (millis() - bleScanStartTime > (BLE_SCAN_TIME_SEC * 2000 + 1000)) {
      pBLEScan->stop();
      bleScanRunning = false;
      pBLEScan->clearResults();
      // Re-enable WiFi promiscuous after BLE scan timeout
      if (settings.wifiScanEnabled) esp_wifi_set_promiscuous(true);
    }
    return;
  }

  // Pause WiFi promiscuous mode during BLE scan to avoid radio contention
  // ESP32 shares 2.4GHz radio between WiFi and BLE
  if (settings.wifiScanEnabled) esp_wifi_set_promiscuous(false);

  // Start a new non-blocking BLE scan with completion callback
  bleAdvCount = 0;
  pBLEScan->clearResults();
  pBLEScan->start(BLE_SCAN_TIME_SEC, bleScanComplete, false);
  bleScanRunning = true;
  bleScanStartTime = millis();
}

void ble_start() {
  BLEDevice::init("DroneScanner");

  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ODIDAdvertisedDeviceCallbacks(), true);
  pBLEScan->setActiveScan(false);   // Passive scan - just listen for advertisements
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  Serial.println("[BLE] Scanner initialized");
}

// ============================================================================
// BATTERY MONITORING
// ============================================================================

float battery_read_voltage() {
  // Enable battery ADC via ADC_CTRL (GPIO46, pin 33 on J2)
  pinMode(46, OUTPUT);
  digitalWrite(46, HIGH);
  delay(10);

  // Read ADC (12-bit) - Heltec factory formula: raw * (1/4096) * 3.3 * coefficient
  uint32_t raw = 0;
  for (int i = 0; i < 8; i++) {
    raw += analogRead(VBAT_PIN);
  }
  raw /= 8;  // Average 8 samples

  digitalWrite(46, LOW);  // Disable ADC to save power

  float vBatt = raw * 0.000244140625 * 3.3 * BATT_ADC_COEFFICIENT;
  Serial.printf("[BATT] ADC raw=%lu -> %.2fV\n", (unsigned long)raw, vBatt);
  return vBatt;
}

int battery_voltage_to_percent(float voltage) {
  if (voltage >= BATT_VOLTAGE_FULL) return 100;
  if (voltage <= BATT_VOLTAGE_EMPTY) return 0;
  // Linear approximation (good enough for LiPo display)
  return (int)((voltage - BATT_VOLTAGE_EMPTY) / (BATT_VOLTAGE_FULL - BATT_VOLTAGE_EMPTY) * 100.0);
}

void enter_low_battery_sleep() {
  Serial.println("[BATT] CRITICAL: Low battery shutdown");
  battLowSleep = true;
  deviceSleeping = true;

  // Stop all radios
  wifi_stop();
  if (pBLEScan && bleScanRunning) {
    pBLEScan->stop();
  }
  bleScanRunning = false;
  radio->sleep();
  ledcDetach(BUZZER_PIN); pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

  // Stop GPS UART and ADC
  GPSSerial.end();
  digitalWrite(46, LOW);   // ADC OFF

  // Show low battery screen
  display.fastmodeOff();
  display.clearMemory();
  display.setRotation(3);
  display.setFont(nullptr);
  display.setTextSize(1);

  // Double border
  display.drawRect(2, 0, 292, 128, BLACK);
  display.drawRect(4, 2, 288, 124, BLACK);

  // Warning icon: large battery outline
  int bx = 98, by = 25, bw = 100, bh = 40;
  display.drawRect(bx, by, bw, bh, BLACK);
  display.drawRect(bx + 1, by + 1, bw - 2, bh - 2, BLACK);
  display.fillRect(bx + bw, by + 12, 6, 16, BLACK);  // Terminal nub
  // Empty battery — just the outline

  // Text
  display.setTextSize(2);
  display.setTextColor(BLACK);
  int tw = 11 * 12;  // "BATTERY LOW" = 11 chars at size 2
  display.setCursor((296 - tw) / 2, by + bh + 10);
  display.print("BATTERY LOW");

  display.setTextSize(1);
  char battBuf[20];
  snprintf(battBuf, sizeof(battBuf), "%.2fV  %d%%", battVoltage, battPercent);
  int sw = strlen(battBuf) * 6;
  display.setCursor((296 - sw) / 2, by + bh + 32);
  display.print(battBuf);

  display.setCursor((296 - 17 * 6) / 2, 112);
  display.print("CHARGE TO RESUME");

  display.update();

  // Enter ESP32 deep sleep — wakes every 30s to check if charging
  Serial.println("[BATT] Entering deep sleep (30s timer wakeup)");
  Serial.flush();
  esp_sleep_enable_timer_wakeup(30 * 1000000ULL);  // 30 seconds in microseconds
  esp_deep_sleep_start();
}

void battery_task() {
  unsigned long now = millis();
  if (now - lastBattRead < BATT_READ_INTERVAL_MS && battPercent >= 0) return;
  lastBattRead = now;

  battVoltage = battery_read_voltage();
  battPercent = battery_voltage_to_percent(battVoltage);

  Serial.printf("[BATT] %.2fV  %d%%\n", battVoltage, battPercent);

  if (battPercent <= BATT_LOW_PERCENT) {
    enter_low_battery_sleep();
  }
}

// ============================================================================
// BUZZER - Non-blocking beep patterns
// ============================================================================

// Map RSSI to buzzer frequency: stronger signal = higher pitch
// RSSI -100 -> 1500Hz, RSSI -20 -> 4000Hz
int buzzer_freq_from_rssi(int rssi) {
  int clamped = constrain(rssi, -100, -20);
  return map(clamped, -100, -20, 1500, 4000);
}

void buzzer_beep(int count, int freq) {
  // Start a beep pattern (non-blocking, managed by buzzer_task)
  if (buzzerBeepsRemaining > 0) return;  // Already beeping
  buzzerCurrentFreq = freq;
  buzzerBeepsRemaining = count;
  buzzerToneOn = true;
  buzzerStartTime = millis();
  ledcAttach(BUZZER_PIN, buzzerCurrentFreq, 8); ledcWriteTone(BUZZER_PIN, buzzerCurrentFreq);
}

void buzzer_task() {
  if (buzzerBeepsRemaining <= 0) return;

  unsigned long elapsed = millis() - buzzerStartTime;

  if (buzzerToneOn && elapsed >= BUZZER_BEEP_MS) {
    // End of tone
    ledcDetach(BUZZER_PIN); pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
    buzzerToneOn = false;
    buzzerBeepsRemaining--;
    buzzerStartTime = millis();
  }
  else if (!buzzerToneOn && buzzerBeepsRemaining > 0 && elapsed >= BUZZER_GAP_MS) {
    // Start next beep after gap
    buzzerToneOn = true;
    buzzerStartTime = millis();
    ledcAttach(BUZZER_PIN, buzzerCurrentFreq, 8); ledcWriteTone(BUZZER_PIN, buzzerCurrentFreq);
  }
}

// ============================================================================
// DISPLAY TASK - E-paper refresh with current status
// ============================================================================

// Helper: print a line at position using snprintf (avoids printf on display)
char _lineBuf[48];

// ============================================================================
// SHARED UI FRAMEWORK - Retro terminal style for all pages
// ============================================================================

#define UI_FRAME_X      2
#define UI_FRAME_Y      0
#define UI_FRAME_W      292
#define UI_FRAME_H      128
#define UI_TITLE_H      14
#define UI_CONTENT_X    8
#define UI_CONTENT_Y    22
#define UI_FOOTER_Y     117
#define UI_RIGHT_COL    200

// Draw the shared page frame: double border + inverted title bar + page indicator
void ui_draw_frame(const char* title, const char* rightLabel) {
  display.setRotation(3);
  display.setFont(nullptr);
  display.setTextSize(1);
  display.setTextColor(BLACK);

  // Double border
  display.drawRect(UI_FRAME_X, UI_FRAME_Y, UI_FRAME_W, UI_FRAME_H, BLACK);
  display.drawRect(UI_FRAME_X + 2, UI_FRAME_Y + 2, UI_FRAME_W - 4, UI_FRAME_H - 4, BLACK);

  // Inverted title bar
  display.fillRect(UI_FRAME_X + 3, UI_FRAME_Y + 3, UI_FRAME_W - 6, UI_TITLE_H, BLACK);
  display.setTextColor(WHITE);
  display.setCursor(8, UI_FRAME_Y + 7);
  display.print("< ");
  display.print(title);
  display.print(" >");

  // Right side: battery icon + voltage + page label
  // Build the text portion first to position everything
  char battText[16];
  char titleRight[24];
  if (battPercent >= 0) {
    snprintf(battText, sizeof(battText), "%.1fV", battVoltage);
  } else {
    battText[0] = '\0';
  }

  // Compose right label: "3.5V 1/4"
  if (battText[0] && rightLabel) {
    snprintf(titleRight, sizeof(titleRight), "%s %s", battText, rightLabel);
  } else if (battText[0]) {
    snprintf(titleRight, sizeof(titleRight), "%s", battText);
  } else if (rightLabel) {
    snprintf(titleRight, sizeof(titleRight), "%s", rightLabel);
  } else {
    titleRight[0] = '\0';
  }

  if (battPercent >= 0) {
    // Draw battery icon (12x8 pixels) to the left of text
    int textW = strlen(titleRight) * 6;
    int battIconX = UI_FRAME_W - 6 - textW - 16;  // 16px = icon width + gap
    int battIconY = UI_FRAME_Y + 5;

    // Battery outline (white on black title bar)
    display.drawRect(battIconX, battIconY, 11, 7, WHITE);
    display.fillRect(battIconX + 11, battIconY + 2, 2, 3, WHITE);  // Terminal nub

    // Fill level (0-9px wide based on percentage)
    int fillW = (battPercent * 9) / 100;
    if (fillW < 1 && battPercent > 0) fillW = 1;
    if (fillW > 0) {
      display.fillRect(battIconX + 1, battIconY + 1, fillW, 5, WHITE);
    }
  }

  if (titleRight[0]) {
    int rLen = strlen(titleRight);
    display.setCursor(UI_FRAME_W - 6 - rLen * 6, UI_FRAME_Y + 7);
    display.print(titleRight);
  }

  // Dotted separator under title
  display.setTextColor(BLACK);
  for (int x = UI_FRAME_X + 4; x < UI_FRAME_X + UI_FRAME_W - 4; x += 2) {
    display.drawPixel(x, UI_FRAME_Y + UI_TITLE_H + 4, BLACK);
  }
}

// Draw inverted footer bar with text
void ui_draw_footer(const char* text) {
  display.fillRect(UI_FRAME_X + 3, UI_FOOTER_Y, UI_FRAME_W - 6, 9, BLACK);
  display.setTextColor(WHITE);
  display.setFont(nullptr);
  display.setTextSize(1);
  // Center the text
  int tLen = strlen(text);
  int cx = (UI_FRAME_W - tLen * 6) / 2;
  display.setCursor(cx, UI_FOOTER_Y + 1);
  display.print(text);
  display.setTextColor(BLACK);
}

// Draw a labeled row: "LABEL......value" with dot leader
// Value is right-aligned against the frame edge so it never overflows
void ui_draw_row(int x, int y, const char* label, const char* value) {
  display.setFont(nullptr);
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(x, y);
  display.print(label);

  int labelEnd = x + strlen(label) * 6;
  int valueW = strlen(value) * 6;
  int valueStart = UI_FRAME_W - 8 - valueW;  // Right-align within frame
  if (valueStart < labelEnd + 12) valueStart = labelEnd + 12;  // Don't overlap label

  // Dot leader
  for (int dx = labelEnd + 4; dx < valueStart - 4; dx += 4) {
    display.drawPixel(dx, y + 3, BLACK);
  }

  display.setCursor(valueStart, y);
  display.print(value);
}

// Draw an inverted badge (white on black pill)
void ui_draw_badge(int x, int y, const char* text) {
  int w = strlen(text) * 6 + 4;
  display.fillRoundRect(x, y - 1, w, 10, 2, BLACK);
  display.setTextColor(WHITE);
  display.setCursor(x + 2, y);
  display.print(text);
  display.setTextColor(BLACK);
}

// ============================================================================
// PAGE 0: SUMMARY - Main dashboard
// ============================================================================

void display_page_summary() {
  char pgBuf[8];
  snprintf(pgBuf, sizeof(pgBuf), "1/%d", NUM_PAGES);
  ui_draw_frame("DRONE SCANNER", pgBuf);

  int y = UI_CONTENT_Y;
  int lh = 11;

  // GPS coordinates row
  if (gps.location.isValid()) {
    snprintf(_lineBuf, sizeof(_lineBuf), "%.5f,%.5f", gps.location.lat(), gps.location.lng());
  } else {
    snprintf(_lineBuf, sizeof(_lineBuf), "searching...");
  }
  ui_draw_row(UI_CONTENT_X, y, "GPS", _lineBuf);
  y += lh;

  // Sats and HDOP side by side with dot leaders
  display.setFont(nullptr);
  display.setTextSize(1);
  display.setTextColor(BLACK);

  int satsVal = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
  display.setCursor(UI_CONTENT_X, y);
  snprintf(_lineBuf, sizeof(_lineBuf), "SATS %d", satsVal);
  display.print(_lineBuf);
  int satsEnd = UI_CONTENT_X + strlen(_lineBuf) * 6;

  // HDOP right-aligned
  snprintf(_lineBuf, sizeof(_lineBuf), "HDOP %.1f", gps.hdop.isValid() ? gps.hdop.hdop() : 99.9);
  int hdopX = UI_FRAME_W - 8 - strlen(_lineBuf) * 6;

  // Dot leader between sats value and HDOP
  for (int dx = satsEnd + 4; dx < hdopX - 4; dx += 4)
    display.drawPixel(dx, y + 3, BLACK);

  display.setCursor(hdopX, y);
  display.print(_lineBuf);
  y += lh + 3;

  // Horizontal separator
  for (int x = UI_CONTENT_X; x < UI_FRAME_W - 8; x += 2)
    display.drawPixel(x, y, BLACK);
  y += 5;

  // Detection counts with badges and RSSI
  display.setCursor(UI_CONTENT_X, y);
  display.print("ELRS");
  snprintf(_lineBuf, sizeof(_lineBuf), "%d", totalElrsHits);
  ui_draw_badge(UI_CONTENT_X + 36, y, _lineBuf);
  if (lastElrsDetectedRate != NULL) {
    display.setCursor(UI_CONTENT_X + 36 + 30, y);
    display.setTextColor(BLACK);
    display.print(lastElrsDetectedRate);
    if (lastElrsPktType <= 3) {
      display.print("/");
      display.print(elrs_pkt_type_name(lastElrsPktType));
    }
  }
  if (elrsStrongestRSSI > -999) {
    if (elrsBestSNR > -999) {
      snprintf(_lineBuf, sizeof(_lineBuf), "%ddB/%.1fSNR", elrsStrongestRSSI, elrsBestSNR);
    } else {
      snprintf(_lineBuf, sizeof(_lineBuf), "%ddBm", elrsStrongestRSSI);
    }
    display.setCursor(UI_FRAME_W - 8 - strlen(_lineBuf) * 6, y);
    display.print(_lineBuf);
  }
  y += lh + 2;

  display.setCursor(UI_CONTENT_X, y);
  display.print("WiFi");
  snprintf(_lineBuf, sizeof(_lineBuf), "%d", totalWifiHits);
  ui_draw_badge(UI_CONTENT_X + 36, y, _lineBuf);
  if (wifiStrongestRSSI > -999) {
    snprintf(_lineBuf, sizeof(_lineBuf), "%ddBm", wifiStrongestRSSI);
    display.setCursor(UI_FRAME_W - 8 - strlen(_lineBuf) * 6, y);
    display.print(_lineBuf);
  }
  y += lh + 2;

  display.setCursor(UI_CONTENT_X, y);
  display.setTextColor(BLACK);
  display.print("BLE");
  snprintf(_lineBuf, sizeof(_lineBuf), "%d", totalBleHits);
  ui_draw_badge(UI_CONTENT_X + 36, y, _lineBuf);
  if (bleStrongestRSSI > -999) {
    snprintf(_lineBuf, sizeof(_lineBuf), "%ddBm", bleStrongestRSSI);
    display.setCursor(UI_FRAME_W - 8 - strlen(_lineBuf) * 6, y);
    display.print(_lineBuf);
  }
  y += lh + 4;

  // Horizontal separator
  for (int x = UI_CONTENT_X; x < UI_FRAME_W - 8; x += 2)
    display.drawPixel(x, y, BLACK);
  y += 5;

  // Timestamp
  display.setCursor(UI_CONTENT_X, y);
  display.setTextColor(BLACK);
  if (gps.time.isValid() && gps.date.isValid()) {
    snprintf(_lineBuf, sizeof(_lineBuf), "%04d-%02d-%02d %02d:%02d:%02dZ",
      gps.date.year(), gps.date.month(), gps.date.day(),
      gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    snprintf(_lineBuf, sizeof(_lineBuf), "NO GPS TIME");
  }
  display.print(_lineBuf);

  // Log count right-aligned
  snprintf(_lineBuf, sizeof(_lineBuf), "LOG %d/%d", detectionCount, MAX_DETECTIONS);
  display.setCursor(UI_RIGHT_COL, y);
  display.print(_lineBuf);

  // Footer
  ui_draw_footer("L/R:PAGES  HOLD 7s:SLEEP");
}

// ============================================================================
// PAGE 1: DETECTIONS - Recent signal log
// ============================================================================

void display_page_detections() {
  char pgBuf[8];
  snprintf(pgBuf, sizeof(pgBuf), "2/%d", NUM_PAGES);
  ui_draw_frame("DETECTIONS", pgBuf);

  int y = UI_CONTENT_Y;
  int lh = 11;

  // Column headers
  display.setFont(nullptr);
  display.setTextSize(1);
  display.setTextColor(BLACK);

  // Column positions
  int colType = UI_CONTENT_X;
  int colRssi = UI_CONTENT_X + 28;
  int colSnr  = UI_CONTENT_X + 58;
  int colInfo = UI_CONTENT_X + 94;

  // Header row with inverted background
  display.fillRect(UI_CONTENT_X - 2, y - 1, UI_FRAME_W - 16, 10, BLACK);
  display.setTextColor(WHITE);
  display.setCursor(colType, y);
  display.print("TYP");
  display.setCursor(colRssi, y);
  display.print("RSSI");
  display.setCursor(colSnr, y);
  display.print("SNR");
  display.setCursor(colInfo, y);
  display.print("INFO");
  display.setTextColor(BLACK);
  y += lh + 2;

  // Clamp scroll offset
  int maxShow = 7;
  int maxScroll = detectionCount > maxShow ? detectionCount - maxShow : 0;
  if (detectionScrollOffset > maxScroll) detectionScrollOffset = maxScroll;
  if (detectionScrollOffset < 0) detectionScrollOffset = 0;

  // Detection list (with scroll)
  int shown = 0;
  int skipped = 0;
  for (int i = 0; i < detectionCount && shown < maxShow; i++) {
    int idx = (detectionHead - 1 - i + MAX_DETECTIONS) % MAX_DETECTIONS;
    Detection* d = &detections[idx];
    if (!d->valid) continue;

    if (skipped < detectionScrollOffset) { skipped++; continue; }

    const char* typeTag;
    if (d->type == SIG_ELRS) typeTag = "ELR";
    else if (d->type == SIG_WIFI) typeTag = "WFI";
    else typeTag = "BLE";

    // Type badge
    ui_draw_badge(colType, y, typeTag);

    // RSSI column
    snprintf(_lineBuf, sizeof(_lineBuf), "%4d", d->rssi);
    display.setCursor(colRssi, y);
    display.print(_lineBuf);

    // SNR column (ELRS only)
    if (d->type == SIG_ELRS && d->snr != 0) {
      snprintf(_lineBuf, sizeof(_lineBuf), "%5.1f", d->snr);
    } else {
      snprintf(_lineBuf, sizeof(_lineBuf), "  --");
    }
    display.setCursor(colSnr, y);
    display.print(_lineBuf);

    // Info column (prepend packet type for ELRS)
    display.setCursor(colInfo, y);
    if (d->type == SIG_ELRS && d->elrsPktType <= 3) {
      display.print(elrs_pkt_type_name(d->elrsPktType));
      display.print(" ");
    }
    display.print(d->identifier);

    y += lh;
    shown++;
  }

  if (shown == 0 && detectionCount == 0) {
    display.setCursor(UI_CONTENT_X + 60, UI_CONTENT_Y + 35);
    display.print("AWAITING SIGNALS...");
  }

  // Show scroll indicators and count
  int first = detectionScrollOffset + 1;
  int last = detectionScrollOffset + shown;
  snprintf(_lineBuf, sizeof(_lineBuf), "%s %d-%d OF %d %s",
    detectionScrollOffset > 0 ? "^" : " ",
    first, last, detectionCount,
    detectionScrollOffset < maxScroll ? "v" : " ");
  ui_draw_footer(_lineBuf);
}

// ============================================================================
// PAGE 2: GPS - Satellite details
// ============================================================================

void display_page_gps() {
  char pgBuf[8];
  snprintf(pgBuf, sizeof(pgBuf), "3/%d", NUM_PAGES);
  ui_draw_frame("GPS TELEMETRY", pgBuf);

  int y = UI_CONTENT_Y;
  int lh = 12;

  // Lat/Lng
  if (gps.location.isValid()) {
    snprintf(_lineBuf, sizeof(_lineBuf), "%.7f", gps.location.lat());
  } else {
    snprintf(_lineBuf, sizeof(_lineBuf), "---");
  }
  ui_draw_row(UI_CONTENT_X, y, "LAT", _lineBuf);
  y += lh;

  if (gps.location.isValid()) {
    snprintf(_lineBuf, sizeof(_lineBuf), "%.7f", gps.location.lng());
  } else {
    snprintf(_lineBuf, sizeof(_lineBuf), "---");
  }
  ui_draw_row(UI_CONTENT_X, y, "LNG", _lineBuf);
  y += lh;

  // Alt
  if (gps.altitude.isValid()) {
    snprintf(_lineBuf, sizeof(_lineBuf), "%.1f m", gps.altitude.meters());
  } else {
    snprintf(_lineBuf, sizeof(_lineBuf), "---");
  }
  ui_draw_row(UI_CONTENT_X, y, "ALT", _lineBuf);
  y += lh + 3;

  // Separator
  for (int x = UI_CONTENT_X; x < UI_FRAME_W - 8; x += 2)
    display.drawPixel(x, y, BLACK);
  y += 5;

  // Sats with badge
  int sats = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
  snprintf(_lineBuf, sizeof(_lineBuf), "%d", sats);
  display.setCursor(UI_CONTENT_X, y);
  display.print("SATS");
  ui_draw_badge(UI_CONTENT_X + 36, y, _lineBuf);

  // HDOP inline
  snprintf(_lineBuf, sizeof(_lineBuf), "HDOP %.1f", gps.hdop.isValid() ? gps.hdop.hdop() : 99.9);
  display.setCursor(UI_CONTENT_X + 80, y);
  display.setTextColor(BLACK);
  display.print(_lineBuf);
  y += lh + 3;

  // Speed
  if (gps.speed.isValid()) {
    snprintf(_lineBuf, sizeof(_lineBuf), "%.1f km/h", gps.speed.kmph());
  } else {
    snprintf(_lineBuf, sizeof(_lineBuf), "---");
  }
  ui_draw_row(UI_CONTENT_X, y, "SPD", _lineBuf);
  y += lh;

  // Heading
  if (gps.course.isValid()) {
    snprintf(_lineBuf, sizeof(_lineBuf), "%.1f deg", gps.course.deg());
  } else {
    snprintf(_lineBuf, sizeof(_lineBuf), "---");
  }
  ui_draw_row(UI_CONTENT_X, y, "HDG", _lineBuf);

  // Fix quality footer
  const char* fixStatus = gps.location.isValid() ? "3D FIX ACQUIRED" : "NO FIX";
  ui_draw_footer(fixStatus);
}

// ============================================================================
// PAGE 3: SETTINGS - Toggle configuration
// ============================================================================

void display_page_settings() {
  char pgBuf[8];
  snprintf(pgBuf, sizeof(pgBuf), "4/%d", NUM_PAGES);
  ui_draw_frame("SETTINGS", pgBuf);

  int y = UI_CONTENT_Y;
  int lh = 10;

  const char* labels[NUM_SETTINGS] = {
    "BUZZER", "ELRS SCAN", "WIFI SCAN", "BLE SCAN", "GPS",
    "ELRS: RC", "ELRS: MSP", "ELRS: SYNC", "ELRS: TLM"
  };
  bool values[NUM_SETTINGS] = {
    settings.buzzerEnabled, settings.elrsScanEnabled,
    settings.wifiScanEnabled, settings.bleScanEnabled,
    settings.gpsEnabled,
    settings.elrsFilterRC, settings.elrsFilterMSP,
    settings.elrsFilterSYNC, settings.elrsFilterTLM
  };

  display.setFont(nullptr);
  display.setTextSize(1);

  for (int i = 0; i < NUM_SETTINGS; i++) {
    bool selected = (settingsCursor == i);

    if (selected) {
      // Highlight bar for selected row
      display.fillRect(UI_CONTENT_X - 2, y - 1, UI_FRAME_W - 16, 11, BLACK);
      display.setTextColor(WHITE);
    } else {
      display.setTextColor(BLACK);
    }

    // Chevron
    display.setCursor(UI_CONTENT_X, y);
    display.print(selected ? "> " : "  ");
    display.print(labels[i]);

    // ON/OFF badge on the right
    const char* state = values[i] ? "ON" : "OFF";
    int bx = UI_RIGHT_COL + 40;
    if (selected) {
      // Already inverted row, draw white badge outline
      display.drawRoundRect(bx, y - 1, strlen(state) * 6 + 4, 10, 2, WHITE);
      display.setCursor(bx + 2, y);
      display.print(state);
    } else {
      if (values[i]) {
        ui_draw_badge(bx, y, state);
      } else {
        display.drawRoundRect(bx, y - 1, strlen(state) * 6 + 4, 10, 2, BLACK);
        display.setCursor(bx + 2, y);
        display.print(state);
      }
    }

    display.setTextColor(BLACK);
    y += lh;
  }

  ui_draw_footer("UP/DN:SELECT  TAP:TOGGLE");
}

void display_task() {
  if (deviceSleeping) return;
  if (millis() - lastDisplayRefresh < DISPLAY_REFRESH_MS) return;
  lastDisplayRefresh = millis();

  // Full refresh on page change or after MAX_PARTIAL_REFRESHES to clear ghosting
  bool needsFullRefresh = (partialRefreshCount >= MAX_PARTIAL_REFRESHES);

  if (needsFullRefresh) {
    display.fastmodeOff();
    partialRefreshCount = 0;
  } else {
    display.fastmodeOn();
    partialRefreshCount++;
  }

  display.clearMemory();
  switch (displayPage) {
    case 0: display_page_summary(); break;
    case 1: display_page_detections(); break;
    case 2: display_page_gps(); break;
    case 3: display_page_settings(); break;
  }
  display.update();

  lastDisplayPage = displayPage;
  Serial.printf("[DISP] %s refresh page %d at %lu ms\n",
    needsFullRefresh ? "Full" : "Partial", displayPage, millis());
}

// ============================================================================
// BUTTON TASK - Short press cycles pages, long press (7s) toggles sleep
// ============================================================================

// Toggle a setting by index and apply side effects (start/stop scanners)
void toggle_setting(int index) {
  switch (index) {
    case 0:
      settings.buzzerEnabled = !settings.buzzerEnabled;
      if (!settings.buzzerEnabled) { ledcWriteTone(BUZZER_PIN, 0); buzzerBeepsRemaining = 0; }
      Serial.printf("[SET] Buzzer: %s\n", settings.buzzerEnabled ? "ON" : "OFF");
      break;
    case 1:
      settings.elrsScanEnabled = !settings.elrsScanEnabled;
      Serial.printf("[SET] ELRS Scan: %s\n", settings.elrsScanEnabled ? "ON" : "OFF");
      break;
    case 2:
      settings.wifiScanEnabled = !settings.wifiScanEnabled;
      if (settings.wifiScanEnabled && !wifiScanActive) wifi_start();
      if (!settings.wifiScanEnabled && wifiScanActive) wifi_stop();
      Serial.printf("[SET] WiFi Scan: %s\n", settings.wifiScanEnabled ? "ON" : "OFF");
      break;
    case 3:
      settings.bleScanEnabled = !settings.bleScanEnabled;
      if (!settings.bleScanEnabled && pBLEScan && bleScanRunning) {
        pBLEScan->stop();
        bleScanRunning = false;
      }
      Serial.printf("[SET] BLE Scan: %s\n", settings.bleScanEnabled ? "ON" : "OFF");
      break;
    case 4:
      settings.gpsEnabled = !settings.gpsEnabled;
      Serial.printf("[SET] GPS: %s\n", settings.gpsEnabled ? "ON" : "OFF");
      break;
    case 5:
      settings.elrsFilterRC = !settings.elrsFilterRC;
      Serial.printf("[SET] ELRS RC Filter: %s\n", settings.elrsFilterRC ? "ON" : "OFF");
      break;
    case 6:
      settings.elrsFilterMSP = !settings.elrsFilterMSP;
      Serial.printf("[SET] ELRS MSP Filter: %s\n", settings.elrsFilterMSP ? "ON" : "OFF");
      break;
    case 7:
      settings.elrsFilterSYNC = !settings.elrsFilterSYNC;
      Serial.printf("[SET] ELRS SYNC Filter: %s\n", settings.elrsFilterSYNC ? "ON" : "OFF");
      break;
    case 8:
      settings.elrsFilterTLM = !settings.elrsFilterTLM;
      Serial.printf("[SET] ELRS TLM Filter: %s\n", settings.elrsFilterTLM ? "ON" : "OFF");
      break;
  }

  // Confirmation beep if buzzer is on
  // No beep on settings change — buzzer reserved for detections
}

void nav_task() {
  // --- Center button: hold detection for sleep/wake ---
  bool centerState = (digitalRead(NAV_CENTER) == LOW);

  if (centerState && !centerPressed) {
    centerPressed = true;
    centerDownTime = millis();
  }
  else if (!centerState && centerPressed) {
    unsigned long holdTime = millis() - centerDownTime;
    centerPressed = false;

    if (holdTime >= LONG_PRESS_MS) {
      // === LONG HOLD (7s) === sleep/wake
      if (deviceSleeping) {
        wake_up();
      } else {
        enter_sleep();
      }
    } else if (holdTime > 50) {
      // === CENTER TAP ===
      if (deviceSleeping) return;

      if (displayPage == 3) {
        // On settings page: toggle the selected setting
        toggle_setting(settingsCursor);
        lastDisplayRefresh = 0;
      }
    }
  }

  if (deviceSleeping) return;

  // --- Directional buttons (with debounce) ---
  static unsigned long lastNavTime = 0;
  if (millis() - lastNavTime < 200) return;  // 200ms debounce

  if (digitalRead(NAV_RIGHT) == LOW) {
    // Right = next page
    displayPage = (displayPage + 1) % NUM_PAGES;
    lastDisplayRefresh = 0;
    Serial.printf("[NAV] Page -> %d\n", displayPage);
    lastNavTime = millis();
  }
  else if (digitalRead(NAV_LEFT) == LOW) {
    // Left = previous page
    displayPage = (displayPage - 1 + NUM_PAGES) % NUM_PAGES;
    lastDisplayRefresh = 0;
    Serial.printf("[NAV] Page -> %d\n", displayPage);
    lastNavTime = millis();
  }
  else if (digitalRead(NAV_DOWN) == LOW) {
    if (displayPage == 1) {
      // Detections page: scroll down
      detectionScrollOffset += 7;  // Scroll by page
      lastDisplayRefresh = 0;
    } else if (displayPage == 3) {
      // Settings page: cursor down
      settingsCursor = (settingsCursor + 1) % NUM_SETTINGS;
      lastDisplayRefresh = 0;
    }
    lastNavTime = millis();
  }
  else if (digitalRead(NAV_UP) == LOW) {
    if (displayPage == 1) {
      // Detections page: scroll up
      detectionScrollOffset = (detectionScrollOffset >= 7) ? detectionScrollOffset - 7 : 0;
      lastDisplayRefresh = 0;
    } else if (displayPage == 3) {
      // Settings page: cursor up
      settingsCursor = (settingsCursor - 1 + NUM_SETTINGS) % NUM_SETTINGS;
      lastDisplayRefresh = 0;
    }
    lastNavTime = millis();
  }
}

// ============================================================================
// SLEEP / WAKE
// ============================================================================

void enter_sleep() {
  Serial.println("[SYS] Entering sleep mode...");
  deviceSleeping = true;

  // Stop scanning
  wifi_stop();
  if (pBLEScan && bleScanRunning) {
    pBLEScan->stop();
  }
  bleScanRunning = false;

  // Put radio to sleep
  radio->sleep();

  // Stop GPS UART
  GPSSerial.end();

  // Disable battery ADC to save power
  digitalWrite(46, LOW);

  // Show sleep screen: splash with text overlaid on padding areas
  display.fastmodeOff();
  display.clearMemory();

  display.drawBitmap(0, 0, splash_bitmap, SPLASH_WIDTH, SPLASH_HEIGHT, BLACK);

  // Overlay text on top/bottom padding of the image
  display.setFont(nullptr);
  display.setTextSize(1);
  display.fillRect(0, 0, 296, 10, WHITE);
  display.setTextColor(BLACK);
  display.setCursor((296 - 14 * 6) / 2, 1);
  display.print("LOW POWER MODE");

  display.fillRect(0, 118, 296, 10, WHITE);
  display.setCursor((296 - 15 * 6) / 2, 119);
  display.print("HOLD 7s TO WAKE");

  display.update();

  Serial.println("[SYS] Sleep mode active");
}

void wake_up() {
  Serial.println("[SYS] Waking up...");
  deviceSleeping = false;

  // Restart GPS UART
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Wake radio and restore config
  radio->standby();
  elrsScanPhase = PHASE_CAD;
  elrs_restore_cad_config();

  // Re-enable battery ADC
  pinMode(46, OUTPUT);
  digitalWrite(46, HIGH);

  // Restart WiFi scanning
  wifi_start();

  // Force full refresh on wake
  lastDisplayPage = -1;
  partialRefreshCount = 0;
  lastDisplayRefresh = 0;

  Serial.println("[SYS] Wake complete");
}

// ============================================================================
// JSON DUMP - Output all detections as JSON over Serial
// ============================================================================

void dump_json() {
  Serial.println("{\"detections\":[");

  bool first = true;
  for (int i = 0; i < detectionCount; i++) {
    // Walk backward from head to get chronological order
    int idx = (detectionHead - detectionCount + i + MAX_DETECTIONS) % MAX_DETECTIONS;
    Detection* d = &detections[idx];
    if (!d->valid) continue;

    if (!first) Serial.print(",");
    first = false;

    const char* typeStr = d->type == SIG_ELRS ? "ELRS" : d->type == SIG_WIFI ? "WiFi" : "BLE";

    Serial.printf(
      "{\"type\":\"%s\",\"rssi\":%d,\"snr\":%.1f,\"freq\":%.1f,\"id\":\"%s\","
      "\"pktType\":\"%s\","
      "\"lat\":%.7f,\"lng\":%.7f,"
      "\"time\":\"%04d-%02d-%02dT%02d:%02d:%02dZ\"}",
      typeStr, d->rssi, d->snr, d->frequency, d->identifier,
      (d->type == SIG_ELRS && d->elrsPktType <= 3) ? elrs_pkt_type_name(d->elrsPktType) : "",
      d->lat, d->lng,
      d->year, d->month, d->day, d->hour, d->minute, d->second
    );
    Serial.println();
  }

  Serial.println("]}");
}

// ============================================================================
// SERIAL COMMAND HANDLER
// ============================================================================

void serial_task() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "dump" || cmd == "json") {
      dump_json();
    } else if (cmd == "status") {
      Serial.printf("ELRS hits: %d (strongest: %d dBm)\n", totalElrsHits, elrsStrongestRSSI);
      Serial.printf("WiFi detected: %d\n", totalWifiHits);
      Serial.printf("BLE Remote ID: %d\n", totalBleHits);
      Serial.printf("GPS: %s (sats: %d)\n",
        gps.location.isValid() ? "fix" : "no fix",
        gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
      Serial.printf("Buffer: %d/%d detections\n", detectionCount, MAX_DETECTIONS);
      Serial.printf("Sleeping: %s\n", deviceSleeping ? "yes" : "no");
    } else if (cmd == "clear") {
      detectionCount = 0;
      detectionHead = 0;
      totalElrsHits = totalWifiHits = totalBleHits = 0;
      elrsHitCount = wifiDetectedCount = bleRemoteIDCount = 0;
      elrsStrongestRSSI = wifiStrongestRSSI = bleStrongestRSSI = -999;
      elrsBestSNR = -999;
      Serial.println("Detections cleared");
    } else if (cmd == "help") {
      Serial.println("Commands: dump/json, status, clear, help");
    }
  }
}

// ============================================================================
// BOOT LOG - Retro terminal-style startup on e-ink display
// ============================================================================

// Layout constants for 296x128 display with 6x8 default font
#define BOOT_FRAME_X       2
#define BOOT_FRAME_Y       0
#define BOOT_FRAME_W       292
#define BOOT_FRAME_H       128
#define BOOT_TITLE_H       14
#define BOOT_LOG_X         8
#define BOOT_LOG_Y_START   20
#define BOOT_LOG_LINE_H    11
#define BOOT_LOG_MAX_LINES 7
#define BOOT_BAR_X         8
#define BOOT_BAR_Y         108
#define BOOT_BAR_W         228
#define BOOT_BAR_H         10
#define BOOT_BAR_LABEL_X   242
#define BOOT_STATUS_COL    222   // Right-aligned status column

struct BootLogEntry {
  char label[30];
  int8_t status;  // -1=pending, 0=fail, 1=ok
};

BootLogEntry bootLogEntries[BOOT_LOG_MAX_LINES];
int bootLogCount = 0;
int bootProgress = 0;   // 0-100
int bootTotalSteps = 5; // E-paper, GPS, LoRa, WiFi, BLE

void bootLogAdd(const char* label, int8_t status) {
  if (bootLogCount < BOOT_LOG_MAX_LINES) {
    strncpy(bootLogEntries[bootLogCount].label, label, 29);
    bootLogEntries[bootLogCount].label[29] = '\0';
    bootLogEntries[bootLogCount].status = status;
    bootLogCount++;
  }
}

void bootLogSetStatus(int idx, int8_t status) {
  if (idx >= 0 && idx < bootLogCount)
    bootLogEntries[idx].status = status;
}

void bootLogRefresh() {
  display.clearMemory();
  display.setFont(nullptr);
  display.setTextSize(1);

  // --- Double border frame ---
  display.drawRect(BOOT_FRAME_X, BOOT_FRAME_Y, BOOT_FRAME_W, BOOT_FRAME_H, BLACK);
  display.drawRect(BOOT_FRAME_X + 2, BOOT_FRAME_Y + 2, BOOT_FRAME_W - 4, BOOT_FRAME_H - 4, BLACK);

  // --- Title bar (inverted) ---
  display.fillRect(BOOT_FRAME_X + 3, BOOT_FRAME_Y + 3, BOOT_FRAME_W - 6, BOOT_TITLE_H, BLACK);
  display.setTextColor(WHITE);
  display.setCursor(8, BOOT_FRAME_Y + 7);
  display.print("< DRONE SCANNER v1.0 >");
  // Right-align subtitle
  display.setCursor(180, BOOT_FRAME_Y + 7);
  display.print("SYSTEM INIT");

  // --- Decorative scan line under title ---
  for (int x = BOOT_FRAME_X + 4; x < BOOT_FRAME_X + BOOT_FRAME_W - 4; x += 2) {
    display.drawPixel(x, BOOT_FRAME_Y + BOOT_TITLE_H + 4, BLACK);
  }

  // --- Log entries with dot leaders ---
  display.setTextColor(BLACK);
  for (int i = 0; i < bootLogCount; i++) {
    int y = BOOT_LOG_Y_START + i * BOOT_LOG_LINE_H;

    // Chevron prompt
    display.setCursor(BOOT_LOG_X, y);
    display.print("> ");

    // Label
    display.print(bootLogEntries[i].label);

    // Dot leader from end of label to status column
    int labelEnd = BOOT_LOG_X + 12 + strlen(bootLogEntries[i].label) * 6;
    for (int dx = labelEnd + 4; dx < BOOT_STATUS_COL - 4; dx += 4) {
      display.drawPixel(dx, y + 3, BLACK);
    }

    // Status badge
    const char* badge;
    if (bootLogEntries[i].status == 1) badge = "[OK]";
    else if (bootLogEntries[i].status == 0) badge = "[!!]";
    else badge = "[..]";

    // Draw status with inverted background for OK
    if (bootLogEntries[i].status == 1) {
      display.fillRect(BOOT_STATUS_COL - 2, y - 1, 28, 10, BLACK);
      display.setTextColor(WHITE);
      display.setCursor(BOOT_STATUS_COL, y);
      display.print(badge);
      display.setTextColor(BLACK);
    } else {
      display.setCursor(BOOT_STATUS_COL, y);
      display.print(badge);
    }
  }

  // --- Progress bar ---
  // Border
  display.drawRect(BOOT_BAR_X, BOOT_BAR_Y, BOOT_BAR_W, BOOT_BAR_H, BLACK);
  // Inner fill
  int fillW = (BOOT_BAR_W - 4) * bootProgress / 100;
  if (fillW > 0) {
    // Striped fill for retro look
    for (int x = 0; x < fillW; x++) {
      if ((x % 4) < 3) {  // 3px filled, 1px gap
        display.drawLine(BOOT_BAR_X + 2 + x, BOOT_BAR_Y + 2,
                         BOOT_BAR_X + 2 + x, BOOT_BAR_Y + BOOT_BAR_H - 3, BLACK);
      }
    }
  }
  // Percentage label
  char pctBuf[8];
  snprintf(pctBuf, sizeof(pctBuf), "%3d%%", bootProgress);
  display.setCursor(BOOT_BAR_LABEL_X, BOOT_BAR_Y + 1);
  display.print(pctBuf);

  // --- Bottom status line (below progress bar) ---
  if (bootProgress >= 100) {
    int bannerY = BOOT_BAR_Y + BOOT_BAR_H + 2;  // 2px below progress bar
    display.fillRect(BOOT_FRAME_X + 3, bannerY, BOOT_FRAME_W - 6, 10, BLACK);
    display.setTextColor(WHITE);
    display.setCursor(70, bannerY + 1);
    display.print(">> ALL SYSTEMS NOMINAL <<");
    display.setTextColor(BLACK);
  }

  display.update();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== Drone Signal Scanner ===");
  Serial.println("Heltec E290 | ELRS + WiFi + BLE");

  // Battery ADC setup
  analogReadResolution(12);
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);  // Full 0-3.3V range
  pinMode(VBAT_PIN, INPUT);

  // Check if waking from low-battery deep sleep
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    float v = battery_read_voltage();
    Serial.printf("[BATT] Timer wakeup: %.2fV\n", v);
    if (v < BATT_VOLTAGE_EMPTY + 0.2) {
      // Still low — go back to deep sleep
      Serial.println("[BATT] Still low, back to sleep");
      Serial.flush();
      esp_sleep_enable_timer_wakeup(30 * 1000000ULL);
      esp_deep_sleep_start();
    }
    Serial.println("[BATT] Voltage recovered — booting normally");
  }

  // 5-Way navigation switch setup
  pinMode(NAV_UP, INPUT_PULLUP);
  pinMode(NAV_DOWN, INPUT_PULLUP);
  pinMode(NAV_LEFT, INPUT_PULLUP);
  pinMode(NAV_RIGHT, INPUT_PULLUP);
  pinMode(NAV_CENTER, INPUT_PULLUP);

  // Buzzer setup (ESP32 LEDC for PWM tone generation)
  bool ledcOk = ledcAttach(BUZZER_PIN, BUZZER_FREQ_HZ, 8);
  Serial.printf("[BUZ] LEDC attach pin %d: %s\n", BUZZER_PIN, ledcOk ? "OK" : "FAILED");
  // Test beep at startup
  ledcAttach(BUZZER_PIN, BUZZER_FREQ_HZ, 8); ledcWriteTone(BUZZER_PIN, BUZZER_FREQ_HZ);
  delay(500);
  ledcDetach(BUZZER_PIN); pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

  // ---- E-Paper Display ----
  // heltec-eink-modules handles Vext, SPI, and pin config automatically
  display.setRotation(3);
  display.setTextWrap(false);

  // Show splash screen (full refresh)
  display.clearMemory();
  display.drawBitmap(0, 0, splash_bitmap, SPLASH_WIDTH, SPLASH_HEIGHT, BLACK);
  display.update();
  delay(2000);  // Show splash for 2 seconds

  Serial.println("[DISP] E-paper initialized");

  // Switch to fast partial refresh for boot log
  display.fastmodeOn();

  // Step 1: E-paper (already done)
  bootLogAdd("E-PAPER DISPLAY", 1);
  bootProgress = 20;
  bootLogRefresh();

  // Step 2: GPS auto-detect
  bootLogAdd("GPS UART PROBE", -1);
  bootLogRefresh();

  bool gpsFound = false;
  {
    const long baudRates[] = {9600, 38400, 57600, 115200};
    const int numBauds = 4;
    const int rxPins[] = {GPS_RX, GPS_TX};
    const int txPins[] = {GPS_TX, GPS_RX};

    for (int p = 0; p < 2 && !gpsFound; p++) {
      for (int b = 0; b < numBauds && !gpsFound; b++) {
        GPSSerial.end();
        GPSSerial.begin(baudRates[b], SERIAL_8N1, rxPins[p], txPins[p]);
        Serial.printf("[GPS] Trying %ld baud, RX=%d TX=%d ... ", baudRates[b], rxPins[p], txPins[p]);

        GPSSerial.println("$PCAS10,0*1C");
        GPSSerial.println("$PMTK101*32");
        GPSSerial.println("");

        unsigned long start = millis();
        int chars = 0;
        bool sawDollar = false;
        while (millis() - start < 2000) {
          while (GPSSerial.available()) {
            char c = GPSSerial.read();
            gps.encode(c);
            chars++;
            if (c == '$') sawDollar = true;
          }
          delay(1);
        }

        if (chars > 10 && sawDollar) {
          Serial.printf("OK (%d chars, NMEA detected)\n", chars);
          gpsFound = true;
        } else if (chars > 10) {
          Serial.printf("data but no NMEA (%d chars)\n", chars);
        } else {
          Serial.printf("no data\n");
        }
      }
    }

    if (!gpsFound) {
      GPSSerial.end();
      GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
      Serial.println("[GPS] No GPS detected, defaulting to 9600 baud RX=38 TX=39");
    }
  }

  bootLogSetStatus(1, gpsFound ? 1 : 0);
  bootProgress = 40;
  bootLogRefresh();

  // Step 3: LoRa LR1121 (dual-band 900MHz + 2.4GHz)
  bootLogAdd("LORA LR1121 DUAL", -1);
  bootLogRefresh();

  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  loraMod = new Module(LORA_NSS, LORA_DIO9, LORA_RESET, LORA_BUSY, loraSPI);
  radio = new LR1121(loraMod);
  Serial.print("[LORA] Initializing LR1121... ");
  int state = radio->begin(
    915.0,              // frequency (MHz) - starts on 900MHz band
    500.0,              // bandwidth (kHz) - BW500 for 900MHz CAD
    6,                  // spreading factor - SF6 for fast CAD phase
    7,                  // coding rate 4/7 - matches most ELRS modes
    0x12,               // sync word - ELRS uses 0x12
    10                  // output power (dBm)
  );

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("OK");
    bootLogSetStatus(2, 1);
  } else {
    Serial.printf("FAILED (error %d)\n", state);
    bootLogSetStatus(2, 0);
  }
  bootProgress = 60;
  bootLogRefresh();

  // Step 4: WiFi
  bootLogAdd("WIFI PROMISCUOUS", -1);
  bootLogRefresh();

  wifi_start();

  bootLogSetStatus(3, 1);
  bootProgress = 80;
  bootLogRefresh();

  // Step 5: BLE
  bootLogAdd("BLE OPENDRONEID", -1);
  bootLogRefresh();

  ble_start();

  bootLogSetStatus(4, 1);
  bootProgress = 100;
  bootLogRefresh();

  // Initial battery read
  battVoltage = battery_read_voltage();
  battPercent = battery_voltage_to_percent(battVoltage);
  Serial.printf("[BATT] %.2fV  %d%%\n", battVoltage, battPercent);

  // Startup beep
  ledcAttach(BUZZER_PIN, BUZZER_FREQ_HZ, 8); ledcWriteTone(BUZZER_PIN, BUZZER_FREQ_HZ);
  delay(150);
  ledcDetach(BUZZER_PIN); pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

  Serial.println("[SYS] All systems initialized");
  Serial.println("[SYS] Commands: dump, status, clear, help");
  Serial.println("===============================\n");

  delay(2000);  // Let user admire the boot screen

  // Return to full refresh mode for normal operation
  display.fastmodeOff();

  // Force first display update after short delay
  lastDisplayRefresh = millis() - DISPLAY_REFRESH_MS + 2000;
}

// ============================================================================
// MAIN LOOP - Non-blocking task scheduler
// ============================================================================

void loop() {
  // Always process GPS data (if enabled)
  if (settings.gpsEnabled) gps_task();

  // Always check navigation and serial
  nav_task();
  serial_task();

  // Buzzer pattern manager (always runs, non-blocking)
  buzzer_task();

  // Battery monitoring (always runs, triggers deep sleep if critical)
  battery_task();

  // Skip scanning tasks if sleeping
  if (deviceSleeping) {
    delay(50);  // Save power while sleeping
    return;
  }

  // ELRS CAD scan - one channel per loop iteration (~1-2ms each)
  if (settings.elrsScanEnabled) cad_scan_task();

  // WiFi channel hop every WIFI_SCAN_DWELL_MS
  if (settings.wifiScanEnabled && millis() - lastWifiScan >= WIFI_SCAN_DWELL_MS) {
    wifi_scan_task();
    lastWifiScan = millis();
  }

  // BLE scan - non-blocking
  if (settings.bleScanEnabled) ble_scan_task();

  // Display refresh every DISPLAY_REFRESH_MS
  display_task();
}
