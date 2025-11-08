#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#ifndef RGB_PIN
#define RGB_PIN  21  
#endif

#define NUM_PIXELS 1

Adafruit_NeoPixel led(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

constexpr int RX_PIN = 44;
constexpr int TX_PIN = 43;
constexpr int BAUD_RATE = 420000;

HardwareSerial CRSF(1);

uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t c = 0x00;
  for (size_t i = 0; i < len; ++i) {
    c ^= data[i];
    for (int b = 0; b < 8; ++b)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ /* poly */ 0xD5) : (uint8_t)(c << 1);
  }
  return c;
}

void setup() {
  Serial.begin(115200):
  CRSF.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  led.begin();
  led.clear();
  led.setBrightness(32);     
  led.setPixelColor(0, led.Color(255, 0, 0));
  led.show();
  CRSF.setRxBufferSize(4096);
}

void loop() {
  static uint32_t t0 = millis();
  if (millis() - t0 > 500) {
    t0 = millis();
    static bool on = false;
    on = !on;
    led.setPixelColor(0, on ? led.Color(0, 255, 0) : 0); 
    led.show();
  }

  static enum {S_ADDR, S_LEN, S_BODY} state = S_ADDR;
  static uint8_t addr,len, buf[64];
  static int idx;

  while(CRSF.available()) {
    uint8_t b = CRSF.read();
    switch(state) {
      case S_ADDR:
        addr = b;
        state = S_LEN;
        break;
      case S_LEN:
        len = b;
        idx = 0;
        if (len == 0 || len > sizeof(buf)) { state = S_ADDR; break; }
        idx = 0; state = S_BODY; break;
      case S_BODY:
        buf[idx++] = b;
        if (idx == len) {
          uint8_t crc = buf[len - 1];
          if (crc8(buf, len - 1) == crc) {
            led.setPixelColor(0, led.Color(0, 0, 255)); // Blue blink = packet received OK
          } else {
            led.setPixelColor(0, led.Color(255, 0, 0)); // Red blink = CRC error
          }
          led.show();
          state = S_ADDR;
        }
        break;
    }
  }
}
