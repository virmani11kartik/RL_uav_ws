#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_NeoPixel.h>

#ifndef RGB_PIN
#define RGB_PIN 21
#endif
#define NUM_PIXELS 1
Adafruit_NeoPixel led(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

// CRSF
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS     0x16
constexpr int FC_TX_PIN = 43;
constexpr int FC_RX_PIN = 44;
constexpr int BAUD_RATE = 420000;
HardwareSerial CRSF_FC(1);

// ESP-NOW link timing
static volatile uint32_t lastEspNowRxMs = 0;     // last time we received a packet via ESP-NOW
static volatile uint32_t lastCrsfTxMs   = 0;     // last time we pushed a CRSF frame to FC

// LED timing
static uint32_t lastLedToggleMs = 0;
static bool ledBlink = false;

// RC channels
uint16_t rcChannels[16] = {
  992, 992, 172, 992, 172, 992, 992, 992,
  992, 992, 992, 992, 992, 992, 992, 992
};

uint32_t packetsSent = 0;
uint32_t testStartTime = 0;
int testPhase = 0;

uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t c = 0x00;
  for (size_t i = 0; i < len; ++i) {
    c ^= data[i];
    for (int b = 0; b < 8; ++b)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1);
  }
  return c;
}

void packAndSendCRSF() {
  uint8_t frame[26];
  frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame[1] = 24;
  frame[2] = CRSF_FRAMETYPE_RC_CHANNELS;

  frame[3]  = (uint8_t)((rcChannels[0]) & 0x07FF);
  frame[4]  = (uint8_t)(((rcChannels[0] & 0x07FF) >> 8) | ((rcChannels[1] & 0x07FF) << 3));
  frame[5]  = (uint8_t)(((rcChannels[1] & 0x07FF) >> 5) | ((rcChannels[2] & 0x07FF) << 6));
  frame[6]  = (uint8_t)(((rcChannels[2] & 0x07FF) >> 2));
  frame[7]  = (uint8_t)(((rcChannels[2] & 0x07FF) >> 10) | ((rcChannels[3] & 0x07FF) << 1));
  frame[8]  = (uint8_t)(((rcChannels[3] & 0x07FF) >> 7)  | ((rcChannels[4] & 0x07FF) << 4));
  frame[9]  = (uint8_t)(((rcChannels[4] & 0x07FF) >> 4)  | ((rcChannels[5] & 0x07FF) << 7));
  frame[10] = (uint8_t)(((rcChannels[5] & 0x07FF) >> 1));
  frame[11] = (uint8_t)(((rcChannels[5] & 0x07FF) >> 9)  | ((rcChannels[6] & 0x07FF) << 2));
  frame[12] = (uint8_t)(((rcChannels[6] & 0x07FF) >> 6)  | ((rcChannels[7] & 0x07FF) << 5));
  frame[13] = (uint8_t)(((rcChannels[7] & 0x07FF) >> 3));

  frame[14] = (uint8_t)((rcChannels[8]) & 0x07FF);
  frame[15] = (uint8_t)(((rcChannels[8] & 0x07FF) >> 8)  | ((rcChannels[9] & 0x07FF) << 3));
  frame[16] = (uint8_t)(((rcChannels[9] & 0x07FF) >> 5)  | ((rcChannels[10] & 0x07FF) << 6));
  frame[17] = (uint8_t)(((rcChannels[10] & 0x07FF) >> 2));
  frame[18] = (uint8_t)(((rcChannels[10] & 0x07FF) >> 10) | ((rcChannels[11] & 0x07FF) << 1));
  frame[19] = (uint8_t)(((rcChannels[11] & 0x07FF) >> 7)  | ((rcChannels[12] & 0x07FF) << 4));
  frame[20] = (uint8_t)(((rcChannels[12] & 0x07FF) >> 4)  | ((rcChannels[13] & 0x07FF) << 7));
  frame[21] = (uint8_t)(((rcChannels[13] & 0x07FF) >> 1));
  frame[22] = (uint8_t)(((rcChannels[13] & 0x07FF) >> 9)  | ((rcChannels[14] & 0x07FF) << 2));
  frame[23] = (uint8_t)(((rcChannels[14] & 0x07FF) >> 6)  | ((rcChannels[15] & 0x07FF) << 5));
  frame[24] = (uint8_t)(((rcChannels[15] & 0x07FF) >> 3));

  frame[25] = crc8(&frame[2], 23);

  CRSF_FC.write(frame, 26);
  packetsSent++;
  lastCrsfTxMs = millis();
}

// =============== ESP-NOW ===============
typedef struct __attribute__((packed)) {
  uint16_t ch[16];     // 172..1811
  uint32_t seq;        // optional
} RcNowMsg;

void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len >= (int)sizeof(RcNowMsg)) {
    const RcNowMsg* m = reinterpret_cast<const RcNowMsg*>(data);
    for (int i = 0; i < 16; ++i) {
      uint16_t v = m->ch[i];
      if (v < 172)  v = 172;
      if (v > 1811) v = 1811;
      rcChannels[i] = v;
    }
    lastEspNowRxMs = millis();
  }
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);
  Serial.println("ESP-NOW ready (receiver). Add your sender as a peer on the sender side.");
}

// =============== TEST PATTERN (fallback) ===============
void updateTestCommands() {
  uint32_t elapsed = millis() - testStartTime;
  testPhase = (elapsed / 3000) % 5;
  switch (testPhase) {
    case 0: rcChannels[0]=992; rcChannels[1]=992; rcChannels[2]=172; rcChannels[3]=992; rcChannels[4]=172; break;
    case 1: rcChannels[0]=1400; rcChannels[1]=992; rcChannels[2]=172; rcChannels[3]=992; rcChannels[4]=172; break;
    case 2: rcChannels[0]=992; rcChannels[1]=1400; rcChannels[2]=172; rcChannels[3]=992; rcChannels[4]=172; break;
    case 3: rcChannels[0]=992; rcChannels[1]=992; rcChannels[2]=172; rcChannels[3]=1400; rcChannels[4]=172; break;
    case 4: rcChannels[0]=992; rcChannels[1]=992; rcChannels[2]=172; rcChannels[3]=992; rcChannels[4]=1811; break;
  }
}

// =============== LED STATUS ===============
inline void setLED(uint8_t r,uint8_t g,uint8_t b){
  led.setPixelColor(0, led.Color(r,g,b));
  led.show();
}

void updateLED() {
  const uint32_t now = millis();
  const bool linkAlive = (now - lastEspNowRxMs) < 1000;     // ESP-NOW seen in last 1s
  const bool txActive  = (now - lastCrsfTxMs)   < 200;      // pushing CRSF recently

  if (now - lastLedToggleMs >= 250) { lastLedToggleMs = now; ledBlink = !ledBlink; }

  if (!linkAlive) {
    // RED blinking = ESP-NOW disconnected
    setLED(ledBlink ? 255 : 16, 0, 0);
    return;
  }

  if (txActive) {
    // PURPLE blinking while transmitting RC
    setLED(ledBlink ? 128 : 16, 0, ledBlink ? 128 : 16);
  } else {
    // Solid GREEN when connected but idle
    setLED(0, 180, 0);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  led.begin();
  led.setBrightness(40);
  setLED(0,0,0);

  CRSF_FC.begin(BAUD_RATE, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);

  initEspNow();

  testStartTime = millis();
}

void loop() {
  const uint32_t now = millis();

  // If no ESP-NOW data yet, keep the test pattern updating every 3s (safe throttle low)
  static uint32_t lastTestUpdate = 0;
  if ((now - lastEspNowRxMs) > 1000) {
    if (now - lastTestUpdate >= 3000) {
      lastTestUpdate = now;
      updateTestCommands();
    }
  }

  // Send CRSF at ~50Hz
  static uint32_t lastSend = 0;
  if (now - lastSend >= 20) {
    lastSend = now;
    packAndSendCRSF();
  }

  updateLED();

  // Optional: print status once per second
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    Serial.printf("CRSF tx=%lu, link=%s, txActive=%s\n",
      packetsSent,
      ((now - lastEspNowRxMs) < 1000 ? "YES" : "NO"),
      ((now - lastCrsfTxMs) < 200 ? "YES" : "NO"));
    Serial.println(WiFi.macAddress());
  }
}
