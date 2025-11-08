#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#ifndef RGB_PIN
#define RGB_PIN  21  
#endif

#define NUM_PIXELS 1

Adafruit_NeoPixel led(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  led.begin();
  led.clear();
  led.setBrightness(32);     
  led.setPixelColor(0, led.Color(255, 0, 0));
  led.show();
}

void loop() {
  // simple blink
  static uint32_t t0 = millis();
  if (millis() - t0 > 500) {
    t0 = millis();
    static bool on = false;
    on = !on;
    led.setPixelColor(0, on ? led.Color(0, 255, 0) : 0); 
    led.show();
  }
}
