#include <Arduino.h>
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

// IMPORTANT: ESP32-C3 only has Serial1 for hardware UART
// We'll use Serial1 with custom pins for CRSF
HardwareSerial CRSF(1);
constexpr int FC_TX_PIN = 5;          // ESP TX -> FC RX  
constexpr int FC_RX_PIN = 4;          // ESP RX <- FC TX (optional)
constexpr int CRSF_BAUDRATE = 420000;

// ---------------- Defaults (us) ----------------
static inline uint16_t def_A()    { return 1500; }
static inline uint16_t def_E()    { return 1500; }
static inline uint16_t def_T()    { return  885; }
static inline uint16_t def_R()    { return 1500; }
static inline uint16_t def_AUX1() { return  900; }
static inline uint16_t def_AUX()  { return 900; }

constexpr uint32_t RC_TIMEOUT_MS = 250;

// ---------------- us -> CRSF ----------------
static inline uint16_t us_to_crsf_exact(int us)
{
  constexpr int US_MIN = 988;
  constexpr int US_MAX = 2012;
  constexpr int CRSF_MIN = 172;
  constexpr int CRSF_MAX = 1811;

  if (us <= US_MIN) return CRSF_MIN;
  if (us >= US_MAX) return CRSF_MAX;

  return (uint16_t)(
    CRSF_MIN +
    ( (int64_t)(us - US_MIN) * (CRSF_MAX - CRSF_MIN) + (US_MAX - US_MIN) / 2 )
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

static void apply_defaults() {
  int us8[8] = { def_A(), def_E(), def_T(), def_R(), def_AUX1(), def_AUX(), def_AUX(), def_AUX() };
  for (int i=0;i<8;i++) us8[i] = clamp_us(us8[i]);

  ch[0] = us_to_crsf_exact(us8[0]); // A
  ch[1] = us_to_crsf_exact(us8[1]); // E
  ch[2] = us_to_crsf_exact(us8[2]); // T
  ch[3] = us_to_crsf_exact(us8[3]); // R
  ch[4] = us_to_crsf_exact(us8[4]); // AUX1
  ch[5] = us_to_crsf_exact(us8[5]); // AUX2
  ch[6] = us_to_crsf_exact(us8[6]); // AUX3
  ch[7] = us_to_crsf_exact(us8[7]); // AUX4
  for (int i=8;i<16;i++) ch[i] = us_to_crsf_exact(1500);
}

static void apply_rc_us(int us8[8]) {
  for (int i=0;i<8;i++) us8[i] = clamp_us(us8[i]);

  ch[0] = us_to_crsf_exact(us8[0]);
  ch[1] = us_to_crsf_exact(us8[1]);
  ch[2] = us_to_crsf_exact(us8[2]);
  ch[3] = us_to_crsf_exact(us8[3]);
  ch[4] = us_to_crsf_exact(us8[4]);
  ch[5] = us_to_crsf_exact(us8[5]);
  ch[6] = us_to_crsf_exact(us8[6]);
  ch[7] = us_to_crsf_exact(us8[7]);
  for (int i=8;i<16;i++) ch[i] = us_to_crsf_exact(1500);

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
  f[10] =  ch[5] >> 1;
  f[11] = (ch[5] >> 9)  | (uint8_t)((ch[6] & 0x3) << 2);
  f[12] = (ch[6] >> 6)  | (uint8_t)((ch[7] & 0x1F) << 5);
  f[13] =  ch[7] >> 3;

  f[14] =  ch[8] & 0xFF;
  f[15] = (ch[8] >> 8)  | (uint8_t)((ch[9] & 0x07) << 3);
  f[16] = (ch[9] >> 5)  | (uint8_t)((ch[10] & 0x3F) << 6);
  f[17] =  ch[10] >> 2;
  f[18] = (ch[10] >> 10) | (uint8_t)((ch[11] & 0x01FF) << 1);
  f[19] = (ch[11] >> 7)  | (uint8_t)((ch[12] & 0x0F) << 4);
  f[20] = (ch[12] >> 4)  | (uint8_t)((ch[13] & 0x01) << 7);
  f[21] =  ch[13] >> 1;
  f[22] = (ch[13] >> 9)  | (uint8_t)((ch[14] & 0x3) << 2);
  f[23] = (ch[14] >> 6)  | (uint8_t)((ch[15] & 0x1F) << 5);
  f[24] =  ch[15] >> 3;

  f[25] = crc8_crsf(&f[2], 23);
  CRSF.write(f, sizeof(f));
}

// ---------------- USB text RX ----------------
static uint32_t last_debug_ms = 0;

static void poll_usb_text() {
  // Periodic heartbeat to show ESP is alive
  if (millis() - last_debug_ms > 2000) {
    last_debug_ms = millis();
    Serial.print("ALIVE millis=");
    Serial.print(millis());
    Serial.print(" avail=");
    Serial.println(Serial.available());
  }
  
  if (!Serial.available()) return;

  // Show we got data
  int avail = Serial.available();
  Serial.print("DATA avail=");
  Serial.println(avail);

  String line = Serial.readStringUntil('\n');
  line.trim();
  
  Serial.print("LINE len=");
  Serial.print(line.length());
  Serial.print(" [");
  Serial.print(line);
  Serial.println("]");
  
  if (line.length() == 0) {
    Serial.println("EMPTY");
    return;
  }

  // Format: "RC A E T R AUX1 AUX2 AUX3 AUX4"
  unsigned int A, E, T, R, A1, A2, A3, A4;

  int parsed = sscanf(line.c_str(), "RC %u %u %u %u %u %u %u %u",
                      &A, &E, &T, &R, &A1, &A2, &A3, &A4);

  Serial.print("PARSE1=");
  Serial.println(parsed);

  if (parsed == 8) {
    int us8[8] = {(int)A, (int)E, (int)T, (int)R, (int)A1, (int)A2, (int)A3, (int)A4};
    apply_rc_us(us8);
    Serial.println("SUCCESS!");
    return;
  }

  // Backwards compatible: allow just 8 ints without "RC"
  parsed = sscanf(line.c_str(), "%u %u %u %u %u %u %u %u",
                  &A, &E, &T, &R, &A1, &A2, &A3, &A4);

  Serial.print("PARSE2=");
  Serial.println(parsed);

  if (parsed == 8) {
    int us8[8] = {(int)A, (int)E, (int)T, (int)R, (int)A1, (int)A2, (int)A3, (int)A4};
    apply_rc_us(us8);
    Serial.println("SUCCESS2!");
    return;
  }
  
  Serial.println("FAIL");
}

void setup() {
  // USB Serial for ROS communication
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  led.begin();
  led.setBrightness(40);
  led_set(0, 0, 255); // Blue boot
  delay(200);
  led_set(255, 0, 0); // Red = waiting for RC

  // CRITICAL: Begin Serial1 with explicit pins BEFORE configuring it
  // ESP32-C3: Use pins that don't conflict with USB (GPIO20/21 are safe)
  CRSF.begin(CRSF_BAUDRATE, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  
  // Give Serial1 time to initialize
  delay(100);

  apply_defaults();
  last_rc_ms = 0;

  Serial.println("\n\n=== ESP RC BRIDGE READY ===");
  Serial.print("Board: ESP32-C3 | USB Serial: ");
  Serial.println(Serial ? "OK" : "FAIL");
  Serial.print("CRSF Serial1 on pins TX=");
  Serial.print(FC_TX_PIN);
  Serial.print(" RX=");
  Serial.println(FC_RX_PIN);
  Serial.println("Waiting for RC commands...");
  Serial.println("NOTE: LED will turn GREEN when receiving RC data");
  Serial.println();
}

void loop() {
  poll_usb_text();

  // failsafe/defaults if no recent RC command
  if (last_rc_ms == 0 || (millis() - last_rc_ms > RC_TIMEOUT_MS)) {
    apply_defaults();
  }

  // CRSF TX @ 50 Hz
  static uint32_t tSend = 0;
  uint32_t now = millis();
  if (now - tSend >= 20) {
    tSend = now;
    send_rc_frame();
  }

  // LED indicates whether we're receiving RC updates
  if (last_rc_ms != 0 && (millis() - last_rc_ms <= RC_TIMEOUT_MS)) {
    led_set(0, 255, 0); // Green = receiving
  } else {
    led_set(255, 0, 0); // Red = timeout/failsafe
  }
}