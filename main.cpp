#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <NimBLEAdvertisedDevice.h>

#include <string.h>
#include <strings.h>   // for strncasecmp
#include <ctype.h>
#include <stdio.h>
#include <stdint.h>

#include "esp_wifi.h"
#include "esp_wifi_types.h"

// ============================================================================
// OUTPUT SELECTION (buzzer vs DNKey eye LEDs)
// ============================================================================

// If no board flag gives us BUZZER_PIN, default to Xiao-style buzzer on GPIO 3
#ifndef BUZZER_PIN
#define BUZZER_PIN 3
#endif

#ifdef USE_DNKEY_EYES

  #ifdef DNKEY_LED_NEOPIXEL
    #include <Adafruit_NeoPixel.h>

    #ifndef NEOPIXEL_DATA_PIN
      #define NEOPIXEL_DATA_PIN 2
    #endif

    #ifndef NEOPIXEL_EYE_COUNT
      #define NEOPIXEL_EYE_COUNT 2
    #endif

    #ifndef NEOPIXEL_EYE_LEFT_IDX
      #define NEOPIXEL_EYE_LEFT_IDX 0
    #endif

    #ifndef NEOPIXEL_EYE_RIGHT_IDX
      #define NEOPIXEL_EYE_RIGHT_IDX 1
    #endif

    #ifndef NEOPIXEL_EYE_BRIGHTNESS
      #define NEOPIXEL_EYE_BRIGHTNESS 40
    #endif

    static Adafruit_NeoPixel g_eyes(NEOPIXEL_EYE_COUNT,
                                    NEOPIXEL_DATA_PIN,
                                    NEO_GRB + NEO_KHZ800);
    static bool g_eyesInit = false;

    static void ensureEyesInit() {
      if (!g_eyesInit) {
        g_eyes.begin();
        g_eyes.clear();
        g_eyes.setBrightness(NEOPIXEL_EYE_BRIGHTNESS);
        g_eyes.show();
        g_eyesInit = true;
      }
    }

    static void setEyeColor(uint8_t r, uint8_t g, uint8_t b) {
      ensureEyesInit();
      for (uint16_t i = 0; i < g_eyes.numPixels(); ++i) {
        g_eyes.setPixelColor(i, g_eyes.Color(r, g, b));
      }
      g_eyes.show();
    }

    static void eyesOff() {
      setEyeColor(0, 0, 0);
    }

    // Boot: double blue flash
    static void eyesBootFlash() {
      setEyeColor(0, 0, 255);
      delay(150);
      eyesOff();
      delay(120);
      setEyeColor(0, 0, 255);
      delay(150);
      eyesOff();
    }

    // Idle heartbeat: short green blink
    static void eyesIdleBlink() {
      setEyeColor(0, 80, 0);   // green
      delay(80);
      eyesOff();
    }

    // New detection: red flash
    static void eyesNewDetectionFlash() {
      setEyeColor(120, 0, 0);  // red
      delay(200);
    }

    // In-range (device still present): solid pink
    static void eyesInRangeSolid() {
      setEyeColor(120, 20, 60);  // pink
    }

  #else
    #warning "USE_DNKEY_EYES is set but DNKEY_LED_NEOPIXEL is not. Falling back to buzzer-only mode."
    #undef USE_DNKEY_EYES
  #endif

#endif  // USE_DNKEY_EYES

// ============================================================================
// CONFIGURATION
// ============================================================================

// We still keep audio constants to reuse for buzzer builds
#define LOW_FREQ           200   // Boot low
#define HIGH_FREQ          800   // Boot high / detection
#define DETECT_FREQ       1000   // Detection beeps
#define HEARTBEAT_FREQ     600   // (unused on DNKey, used on buzzer builds)

#define BOOT_BEEP_DURATION   300
#define DETECT_BEEP_DURATION 150

// WiFi promiscuous
#define MAX_CHANNEL 13
#define CHANNEL_HOP_INTERVAL 500  // ms

// BLE scanning
#define BLE_SCAN_DURATION   1     // seconds
#define BLE_SCAN_INTERVAL   5000  // ms between scans

// Detection state timing
#define DEVICE_TIMEOUT_MS   30000UL  // after 30s of silence, consider out-of-range
#define IDLE_BLINK_MS       3000UL   // idle green blink every 3 seconds

// ============================================================================
// DETECTION PATTERNS
// ============================================================================

static const char* wifi_ssid_patterns[] = {
  "flock", "Flock", "FLOCK",
  "FS Ext Battery",
  "Penguin",
  "Pigvision"
};

static const char* mac_prefixes[] = {
  // FS Ext Battery
  "58:8e:81","cc:cc:cc","ec:1b:bd","90:35:ea","04:0d:84",
  "f0:82:c0","1c:34:f1","38:5b:44","94:34:69","b4:e3:f9",
  // Flock WiFi
  "70:c9:4e","3c:91:80","d8:f3:bc","80:30:49","14:5a:fc",
  "74:4c:a1","08:3a:88","9c:2f:9d","94:08:53","e4:aa:ea"
  // Penguin OUIs are location-specific; leave out for now
};

static const char* device_name_patterns[] = {
  "FS Ext Battery",
  "Penguin",
  "Flock",
  "Pigvision"
};

// ============================================================================
// GLOBAL STATE
// ============================================================================

static uint8_t current_channel = 1;
static unsigned long last_channel_hop = 0;

static bool triggered         = false;   // Have we fired a "new detection" since last idle?
static bool device_in_range   = false;   // Currently think a Flock device is nearby
static unsigned long last_detection_time = 0;
static unsigned long last_idle_blink     = 0;
static unsigned long last_ble_scan       = 0;

static NimBLEScan* pBLEScan = nullptr;

// ============================================================================
// SMALL HELPERS
// ============================================================================

static bool containsIgnoreCase(const char* haystack, const char* needle) {
  if (!haystack || !needle) return false;
  size_t hlen = strlen(haystack);
  size_t nlen = strlen(needle);
  if (nlen == 0 || hlen < nlen) return false;

  for (size_t i = 0; i + nlen <= hlen; ++i) {
    size_t j = 0;
    while (j < nlen) {
      char c1 = tolower((unsigned char)haystack[i + j]);
      char c2 = tolower((unsigned char)needle[j]);
      if (c1 != c2) break;
      ++j;
    }
    if (j == nlen) return true;
  }
  return false;
}

static bool check_mac_prefix(const uint8_t* mac) {
  char mac_str[9];
  snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x", mac[0], mac[1], mac[2]);
  for (size_t i = 0; i < sizeof(mac_prefixes)/sizeof(mac_prefixes[0]); i++) {
    if (strncasecmp(mac_str, mac_prefixes[i], 8) == 0) return true;
  }
  return false;
}

static bool check_ssid_pattern(const char* ssid) {
  if (!ssid) return false;
  for (size_t i = 0; i < sizeof(wifi_ssid_patterns)/sizeof(wifi_ssid_patterns[0]); i++) {
    if (containsIgnoreCase(ssid, wifi_ssid_patterns[i])) return true;
  }
  return false;
}

static bool check_device_name_pattern(const char* name) {
  if (!name) return false;
  for (size_t i = 0; i < sizeof(device_name_patterns)/sizeof(device_name_patterns[0]); i++) {
    if (containsIgnoreCase(name, device_name_patterns[i])) return true;
  }
  return false;
}

// WiFi frame header structs
typedef struct {
  unsigned frame_ctrl:16;
  unsigned duration_id:16;
  uint8_t addr1[6];
  uint8_t addr2[6];
  uint8_t addr3[6];
  unsigned sequence_ctrl:16;
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
} wifi_ieee80211_packet_t;

// ============================================================================
// NOTIFICATION / VISUALS
// ============================================================================

static void beep(int frequency, int duration_ms) {
#ifndef USE_DNKEY_EYES
  tone(BUZZER_PIN, frequency, duration_ms);
  delay(duration_ms);
  noTone(BUZZER_PIN);
#else
  (void)frequency;
  (void)duration_ms;
#endif
}

static void boot_feedback() {
#ifdef USE_DNKEY_EYES
  eyesBootFlash();   // double blue flash
#else
  beep(LOW_FREQ,  BOOT_BEEP_DURATION);
  beep(HIGH_FREQ, BOOT_BEEP_DURATION);
#endif
}

static void new_detection_feedback() {
#ifdef USE_DNKEY_EYES
  eyesNewDetectionFlash();  // red flash
  eyesInRangeSolid();       // then solid pink
#else
  // 3 quick high beeps
  for (int i = 0; i < 3; ++i) {
    beep(DETECT_FREQ, DETECT_BEEP_DURATION);
    delay(60);
  }
#endif
}

static void idle_visual_tick() {
  // Called when idle and it's time for a heartbeat.
#ifdef USE_DNKEY_EYES
  eyesIdleBlink();     // short green blink every 3 seconds
#else
  // On buzzer builds we can either do nothing or add a very soft ping.
  // For now, keep idle quiet.
#endif
}

static void mark_detection_event(const char* source, const char* detail,
                                 const uint8_t* mac, int rssi) {
  // Basic logging
  Serial.print("[DETECTION] ");
  Serial.print(source);
  Serial.print(" ");
  Serial.print(detail);
  Serial.print(" RSSI=");
  Serial.println(rssi);

  last_detection_time = millis();

  if (!triggered) {
    triggered = true;
    device_in_range = true;
    new_detection_feedback();
  } else {
    device_in_range = true;
#ifdef USE_DNKEY_EYES
    eyesInRangeSolid();   // keep pink solid while in range
#endif
  }

  (void)mac;  // reserved if you want to print MACs later
}

// ============================================================================
// WIFI CALLBACK
// ============================================================================

static void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;

  const wifi_promiscuous_pkt_t* ppkt = (wifi_promiscuous_pkt_t*)buff;
  const wifi_ieee80211_packet_t* ipkt = (wifi_ieee80211_packet_t*)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t* hdr = &ipkt->hdr;

  int8_t rssi = ppkt->rx_ctrl.rssi;

  // Parse SSID from tagged parameters
  const uint8_t* payload = (const uint8_t*)ppkt->payload;
  const uint8_t* end = payload + ppkt->rx_ctrl.sig_len;
  const uint8_t* ptr = payload + sizeof(wifi_ieee80211_mac_hdr_t);

  static char ssid_buf[33];
  const char* ssid = nullptr;

  while (ptr + 2 < end) {
    uint8_t tag_number = ptr[0];
    uint8_t tag_length = ptr[1];
    ptr += 2;

    if (tag_number == 0 && tag_length > 0 && tag_length < sizeof(ssid_buf)) {
      memcpy(ssid_buf, ptr, tag_length);
      ssid_buf[tag_length] = '\0';
      ssid = ssid_buf;
      break;
    }

    ptr += tag_length;
  }

  bool matched_ssid = ssid && check_ssid_pattern(ssid);
  bool matched_mac  = check_mac_prefix(hdr->addr2);

  if (matched_ssid || matched_mac) {
    const char* detail = matched_ssid ? (ssid ? ssid : "hidden") : "mac_prefix";
    mark_detection_event("wifi", detail, hdr->addr2, rssi);
  }
}

// ============================================================================
// BLE SCANNING
// ============================================================================

class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) override {
    NimBLEAddress addr = advertisedDevice->getAddress();
    const uint8_t* mac = addr.getNative();
    std::string name = advertisedDevice->getName();
    int8_t rssi = advertisedDevice->getRSSI();

    const char* cname = name.empty() ? nullptr : name.c_str();
    bool matched = false;

    if (cname && check_device_name_pattern(cname)) {
      matched = true;
    } else if (check_mac_prefix(mac)) {
      matched = true;
    }

    if (matched) {
      mark_detection_event("ble", cname ? cname : "BLE_MAC", mac, rssi);
    }
  }
};

// ============================================================================
// CHANNEL HOPPING
// ============================================================================

static void hop_channel() {
  unsigned long now = millis();
  if (now - last_channel_hop >= CHANNEL_HOP_INTERVAL) {
    current_channel++;
    if (current_channel > MAX_CHANNEL) current_channel = 1;
    esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
    last_channel_hop = now;
  }
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

#ifndef USE_DNKEY_EYES
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
#endif

  boot_feedback();

  Serial.println("Flock detector starting…");

  // WiFi promiscuous
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
  esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);

  // BLE scanner
  NimBLEDevice::init("FlockYou");
  NimBLEDevice::setSecurityAuth(true, true, true);

  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(80);

  last_channel_hop = millis();
  last_idle_blink  = millis();
  last_ble_scan    = millis();

  Serial.println("System ready. Hunting for Flock devices…");
}

void loop() {
  unsigned long now = millis();

  hop_channel();

  // Device presence timeout / state machine
  if (device_in_range && (now - last_detection_time >= DEVICE_TIMEOUT_MS)) {
    Serial.println("[INFO] Device out of range, returning to idle.");
    device_in_range = false;
    triggered = false;
#ifdef USE_DNKEY_EYES
    eyesOff();
#endif
  }

  // Idle green heartbeat every 3 seconds when no device in range
  if (!device_in_range && (now - last_idle_blink >= IDLE_BLINK_MS)) {
    idle_visual_tick();
    last_idle_blink = now;
  }

  // BLE scan cadence
  if (!pBLEScan->isScanning() && (now - last_ble_scan >= BLE_SCAN_INTERVAL)) {
    Serial.println("[BLE] scan…");
    pBLEScan->start(BLE_SCAN_DURATION, false);
    last_ble_scan = now;
  }

  // Occasionally clear scan results to free memory
  if (!pBLEScan->isScanning() &&
      (now - last_ble_scan > (BLE_SCAN_DURATION * 1000UL + 5000UL))) {
    pBLEScan->clearResults();
  }

  delay(50);
}
