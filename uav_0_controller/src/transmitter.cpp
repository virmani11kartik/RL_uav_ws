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

// Set receiver MAC address (must match receiver's custom MAC)
uint8_t receiverMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01};

// Data structure to send (matches receiver)
typedef struct {
  int rc_us[8];  // A, E, T, R, AUX1, AUX2, AUX3, AUX4
  uint32_t timestamp;
} rc_data_t;

rc_data_t txData;
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    led_set(0, 255, 0); // Green flash on success
  } else {
    led_set(255, 128, 0); // Orange on fail
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  led.begin();
  led.setBrightness(40);
  led_set(0, 0, 255); // Blue boot
  delay(200);

  // Set device as a Wi-Fi Station with Long Range mode
  WiFi.mode(WIFI_STA);
  
  // Enable Long Range mode (if supported)
  #ifdef WIFI_PROTOCOL_LR
  esp_err_t result = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
  if (result == ESP_OK) {
    Serial.println("Long Range mode enabled");
  } else {
    Serial.println("Long Range mode not available, using standard mode");
  }
  #else
  Serial.println("Long Range mode not supported on this platform");
  #endif
  
  Serial.println("\n\n=== ESP-NOW TRANSMITTER (USB to Wireless) ===");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    led_set(255, 0, 0);
    return;
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0; // Use current channel
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    led_set(255, 0, 0);
    return;
  }

  // Initialize with defaults
  txData.rc_us[0] = 1500; // A
  txData.rc_us[1] = 1500; // E
  txData.rc_us[2] = 988;  // T
  txData.rc_us[3] = 1500; // R
  txData.rc_us[4] = 900;  // AUX1
  txData.rc_us[5] = 1500; // AUX2
  txData.rc_us[6] = 1500; // AUX3
  txData.rc_us[7] = 1500; // AUX4

  led_set(255, 255, 0); // Yellow = ready
  Serial.println("Ready to receive RC commands via USB...");
  Serial.println("Format: RC A E T R AUX1 AUX2 AUX3 AUX4");
  Serial.println();
}

void loop() {
  static uint32_t last_debug_ms = 0;
  
  // Periodic heartbeat
  if (millis() - last_debug_ms > 2000) {
    last_debug_ms = millis();
    Serial.print("TX ALIVE | millis=");
    Serial.print(millis());
    Serial.print(" | USB avail=");
    Serial.println(Serial.available());
  }

  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  
  if (line.length() == 0) return;

  Serial.print("RX: [");
  Serial.print(line);
  Serial.println("]");

  // Format: "RC A E T R AUX1 AUX2 AUX3 AUX4"
  unsigned int A, E, T, R, A1, A2, A3, A4;

  int parsed = sscanf(line.c_str(), "RC %u %u %u %u %u %u %u %u",
                      &A, &E, &T, &R, &A1, &A2, &A3, &A4);

  if (parsed != 8) {
    // Try without "RC" prefix
    parsed = sscanf(line.c_str(), "%u %u %u %u %u %u %u %u",
                    &A, &E, &T, &R, &A1, &A2, &A3, &A4);
  }

  if (parsed == 8) {
    txData.rc_us[0] = constrain(A, 800, 2200);
    txData.rc_us[1] = constrain(E, 800, 2200);
    txData.rc_us[2] = constrain(T, 800, 2200);
    txData.rc_us[3] = constrain(R, 800, 2200);
    txData.rc_us[4] = constrain(A1, 800, 2200);
    txData.rc_us[5] = constrain(A2, 800, 2200);
    txData.rc_us[6] = constrain(A3, 800, 2200);
    txData.rc_us[7] = constrain(A4, 800, 2200);
    txData.timestamp = millis();

    // Send data via ESP-NOW
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&txData, sizeof(txData));

    if (result == ESP_OK) {
      Serial.println("Sent via ESP-NOW!");
    } else {
      Serial.println("ESP-NOW send error");
      led_set(255, 0, 0);
    }
  } else {
    Serial.println("Parse failed");
  }
}