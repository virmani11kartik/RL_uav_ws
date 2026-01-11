#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>

#ifndef RGB_PIN
#define RGB_PIN 2
#endif
#define NUMPIXELS 1
Adafruit_NeoPixel led(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

static inline void led_set(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

// ---------------- CRSF ----------------
static const uint8_t CRSF_ADDR_FC = 0xC8;
static const uint8_t CRSF_TYPE_RC = 0x16;

HardwareSerial CRSF(1);
constexpr int FC_TX_PIN = 5;
constexpr int FC_RX_PIN = 4;
constexpr int CRSF_BAUDRATE = 420000;

// ---------------- Defaults (us) ----------------
static inline uint16_t def_A()    { return 1500; }
static inline uint16_t def_E()    { return 1500; }
static inline uint16_t def_T()    { return  988; }
static inline uint16_t def_R()    { return 1500; }
static inline uint16_t def_AUX1() { return  900; }
static inline uint16_t def_AUX()  { return 1500; }

constexpr uint32_t RC_TIMEOUT_MS = 500;

// ---------------- us -> CRSF ----------------
static inline uint16_t us_to_crsf_exact(int us) {
  constexpr int US_MIN = 988;
  constexpr int US_MAX = 2012;
  constexpr int CRSF_MIN = 172;
  constexpr int CRSF_MAX = 1811;

  if (us <= US_MIN) return CRSF_MIN;
  if (us >= US_MAX) return CRSF_MAX;

  return (uint16_t)(
    CRSF_MIN +
    ((int64_t)(us - US_MIN) * (CRSF_MAX - CRSF_MIN) + (US_MAX - US_MIN) / 2)
    / (US_MAX - US_MIN)
  );
}

static inline uint16_t clamp_us(int v) {
  if (v < 800)  v = 800;
  if (v > 2200) v = 2200;
  return (uint16_t)v;
}

// ---------------- CRSF CRC8 (poly 0xD5) ----------------
static uint8_t crc8_crsf(const uint8_t* data, size_t len) {
  uint8_t c = 0x00;
  for (size_t i = 0; i < len; i++) {
    c ^= data[i];
    for (int j = 0; j < 8; j++) {
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1);
    }
  }
  return c;
}

// ---------------- Channels in CRSF units ----------------
static uint16_t ch[16];
static uint32_t last_rc_ms = 0;
static int current_rc_us[8];

// ESP-NOW data structure (must match transmitter)
typedef struct {
  int rc_us[8];
  uint32_t timestamp;
} rc_data_t;

static void apply_defaults() {
  current_rc_us[0] = def_A();
  current_rc_us[1] = def_E();
  current_rc_us[2] = def_T();
  current_rc_us[3] = def_R();
  current_rc_us[4] = def_AUX1();
  current_rc_us[5] = def_AUX();
  current_rc_us[6] = def_AUX();
  current_rc_us[7] = def_AUX();
  
  for (int i = 0; i < 8; i++) current_rc_us[i] = clamp_us(current_rc_us[i]);

  for (int i = 0; i < 8; i++) ch[i] = us_to_crsf_exact(current_rc_us[i]);
  for (int i = 8; i < 16; i++) ch[i] = us_to_crsf_exact(1500);
}

static void apply_rc_us(int us8[8]) {
  for (int i = 0; i < 8; i++) {
    us8[i] = clamp_us(us8[i]);
    current_rc_us[i] = us8[i];
  }

  for (int i = 0; i < 8; i++) ch[i] = us_to_crsf_exact(us8[i]);
  for (int i = 8; i < 16; i++) ch[i] = us_to_crsf_exact(1500);

  last_rc_ms = millis();
}

static void send_rc_frame() {
  uint8_t f[26];
  f[0] = CRSF_ADDR_FC;
  f[1] = 24;
  f[2] = CRSF_TYPE_RC;

  // Pack 16x11-bit into 22 bytes payload
  f[3]  = ch[0] & 0xFF;
  f[4]  = (ch[0] >> 8) | (uint8_t)((ch[1] & 0x07) << 3);
  f[5]  = (ch[1] >> 5) | (uint8_t)((ch[2] & 0x3F) << 6);
  f[6]  = ch[2] >> 2;
  f[7]  = (ch[2] >> 10) | (uint8_t)((ch[3] & 0x01FF) << 1);
  f[8]  = (ch[3] >> 7)  | (uint8_t)((ch[4] & 0x0F) << 4);
  f[9]  = (ch[4] >> 4)  | (uint8_t)((ch[5] & 0x01) << 7);
  f[10] = ch[5] >> 1;
  f[11] = (ch[5] >> 9)  | (uint8_t)((ch[6] & 0x3) << 2);
  f[12] = (ch[6] >> 6)  | (uint8_t)((ch[7] & 0x1F) << 5);
  f[13] = ch[7] >> 3;

  f[14] = ch[8] & 0xFF;
  f[15] = (ch[8] >> 8)  | (uint8_t)((ch[9] & 0x07) << 3);
  f[16] = (ch[9] >> 5)  | (uint8_t)((ch[10] & 0x3F) << 6);
  f[17] = ch[10] >> 2;
  f[18] = (ch[10] >> 10) | (uint8_t)((ch[11] & 0x01FF) << 1);
  f[19] = (ch[11] >> 7)  | (uint8_t)((ch[12] & 0x0F) << 4);
  f[20] = (ch[12] >> 4)  | (uint8_t)((ch[13] & 0x01) << 7);
  f[21] = ch[13] >> 1;
  f[22] = (ch[13] >> 9)  | (uint8_t)((ch[14] & 0x3) << 2);
  f[23] = (ch[14] >> 6)  | (uint8_t)((ch[15] & 0x1F) << 5);
  f[24] = ch[15] >> 3;

  f[25] = crc8_crsf(&f[2], 23);
  CRSF.write(f, sizeof(f));
}

// ---------------- LED State Detection ----------------
static void update_led() {
  if (last_rc_ms == 0 || (millis() - last_rc_ms > RC_TIMEOUT_MS)) {
    led_set(255, 0, 0); // RED - no data
    return;
  }

  const bool armed = (current_rc_us[4] >= 1000 && current_rc_us[4] <= 1200);

  constexpr int TOLERANCE = 10;
  const int defaults[8] = {def_A(), def_E(), def_T(), def_R(),
                           def_AUX1(), def_AUX(), def_AUX(), def_AUX()};

  bool is_default_except_aux1 = true;
  for (int i = 0; i < 8; i++) {
    if (i == 4) continue;
    if (abs(current_rc_us[i] - defaults[i]) > TOLERANCE) {
      is_default_except_aux1 = false;
      break;
    }
  }

  if (!armed && is_default_except_aux1) {
    led_set(255, 255, 0); // YELLOW - defaults
  } else if (armed && is_default_except_aux1) {
    led_set(0, 0, 255);   // BLUE - armed
  } else {
    led_set(0, 255, 0);   // GREEN - commanded
  }
}

// ESP-NOW receive callback
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(rc_data_t)) {
    Serial.println("ESP-NOW: Invalid data size");
    return;
  }

  rc_data_t rxData;
  memcpy(&rxData, incomingData, sizeof(rxData));

  Serial.print("ESP-NOW RX from ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" | ");
  for (int i = 0; i < 8; i++) {
    Serial.print(rxData.rc_us[i]);
    Serial.print(" ");
  }
  Serial.println();

  apply_rc_us(rxData.rc_us);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  led.begin();
  led.setBrightness(40);
  led_set(255, 255, 255); // White boot
  delay(200);
  led_set(255, 0, 0); // Red = waiting

  // Set device as a Wi-Fi Station with Long Range
  WiFi.mode(WIFI_STA);
  
  // Enable Long Range mode
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);

  Serial.println("\n\n=== ESP-NOW RECEIVER (Wireless to CRSF) ===");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("**COPY THIS MAC TO TRANSMITTER CODE**");

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    led_set(255, 0, 0);
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  CRSF.begin(CRSF_BAUDRATE, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  delay(100);

  apply_defaults();
  last_rc_ms = 0;

  Serial.print("CRSF on pins TX=");
  Serial.print(FC_TX_PIN);
  Serial.print(" RX=");
  Serial.println(FC_RX_PIN);
  Serial.println("\nLED States:");
  Serial.println("  RED    = No ESP-NOW data");
  Serial.println("  YELLOW = Default values");
  Serial.println("  GREEN  = ROS commanded");
  Serial.println("  BLUE   = Armed");
  Serial.println("\nWaiting for ESP-NOW data...");
}

void loop() {
  // Failsafe to defaults on timeout
  if (last_rc_ms == 0 || (millis() - last_rc_ms > RC_TIMEOUT_MS)) {
    apply_defaults();
  }

  // Send CRSF @ 50 Hz
  static uint32_t tSend = 0;
  uint32_t now = millis();
  if (now - tSend >= 20) {
    tSend = now;
    send_rc_frame();
  }

  update_led();
}