// Custom TX/RX Main
// Use BUILD_TX or BUILD_RX build flag to select mode

#include <Arduino.h>
#include "protocol.h"

// Build mode check
#if !defined(BUILD_TX) && !defined(BUILD_RX)
  #error "Please define either BUILD_TX or BUILD_RX in platformio.ini"
#endif

#ifdef BUILD_TX
// ============================================================
// TRANSMITTER (TX) CODE
// ============================================================

// Broadcast to all receivers (no pairing needed!)
uint8_t receiverMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Bench test mode - cycles through all axes
#define BENCH_TEST_MODE true  // Set to false to use static values

// RC channel values (CRSF format: 172-1811, center: 992)
uint16_t rcChannels[16] = {
  992, 992, 172, 992, 992, 992, 992, 992,  // Channels 1-8
  992, 992, 992, 992, 992, 992, 992, 992   // Channels 9-16
};

// Arming logic
bool isArmed = false;
unsigned long armingStartTime = 0;
const unsigned long ARMING_DELAY = 3000;  // 3 seconds to arm

// Bench test variables
float benchTestPhase = 0.0;
const float BENCH_TEST_SPEED = 0.5;  // Radians per second

// Timing
unsigned long lastRcSendMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastStatsMs = 0;
unsigned long lastTxErrorMs = 0;

const unsigned long RC_SEND_INTERVAL = 20;      // 50 Hz (20ms) - ELRS-like speed
const unsigned long HEARTBEAT_INTERVAL = 500;   // 2 Hz (500ms)
const unsigned long STATS_INTERVAL = 1000;      // 1 Hz (every second)
const unsigned long ERROR_PRINT_INTERVAL = 1000; // Max 1 error per second

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=================================");
  Serial.println("Custom Protocol Transmitter (TX)");
  Serial.println("=================================\n");
  
  // Initialize protocol (TX mode, broadcast)
  if (!CustomProtocol_Init(true, receiverMac)) {
    Serial.println("[TX] Protocol init failed!");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("[TX] Ready to transmit!");
  Serial.println("[TX] Broadcasting to all receivers");
  Serial.println("[TX] RC commands: 50 Hz | Heartbeat: 2 Hz | Stats: 1 Hz");
  
  if (BENCH_TEST_MODE) {
    Serial.println("\n⚠️  BENCH TEST MODE ACTIVE");
    Serial.println("🔒 DISARMED - Throttle at minimum for 3 seconds");
    Serial.println("🔓 Will auto-arm after 3 seconds, then throttle cycles");
    armingStartTime = millis();
  }
  
  Serial.println();
}

void loop() {
  unsigned long now = millis();
  
  // Update protocol (handles timeouts, etc.)
  CustomProtocol_Update();
  
  // Arming logic - arm after 3 seconds
  if (!isArmed && (now - armingStartTime >= ARMING_DELAY)) {
    isArmed = true;
    Serial.println("\n🔓 ARMED - Bench test active!\n");
  }
  
  // Send RC commands at 50 Hz
  if (now - lastRcSendMs >= RC_SEND_INTERVAL) {
    lastRcSendMs = now;
    
    if (BENCH_TEST_MODE) {
      // Bench test mode - cycle through all axes for testing
      float dt = RC_SEND_INTERVAL / 1000.0;  // Time delta in seconds
      benchTestPhase += BENCH_TEST_SPEED * dt;
      if (benchTestPhase > 6.28318) benchTestPhase -= 6.28318;  // Wrap at 2*PI
      
      // Calculate smooth sine waves for each axis
      float rollValue = sin(benchTestPhase);                    // Roll oscillates
      float pitchValue = sin(benchTestPhase + 1.57);            // Pitch 90° out of phase
      float yawValue = sin(benchTestPhase * 0.5);               // Yaw slower
      float throttleValue = (sin(benchTestPhase * 0.3) + 1.0) / 2.0;  // Throttle 0-1, slower
      
      // Convert -1..1 to CRSF range (172-1811, center 992)
      rcChannels[0] = (uint16_t)(rollValue * 400.0 + 992.0);     // Roll: ±400 from center
      rcChannels[1] = (uint16_t)(pitchValue * 400.0 + 992.0);    // Pitch: ±400 from center
      
      // SAFETY: Throttle stays at minimum until armed!
      if (isArmed) {
        rcChannels[2] = (uint16_t)(throttleValue * 800.0 + 172.0); // Throttle: 172-972 (half range for safety)
      } else {
        rcChannels[2] = 172;  // THROTTLE MINIMUM - SAFE TO ARM
      }
      
      rcChannels[3] = (uint16_t)(yawValue * 300.0 + 992.0);      // Yaw: ±300 from center
      
      // Aux channels - some switches cycling
      rcChannels[4] = (benchTestPhase < 3.14) ? 172 : 1811;  // AUX1: Switch
      rcChannels[5] = 992;                                    // AUX2: Center
      
    } else {
      // Manual mode - static values (for real joystick input)
      rcChannels[0] = 992;  // Roll center
      rcChannels[1] = 992;  // Pitch center
      rcChannels[2] = 172;  // Throttle minimum
      rcChannels[3] = 992;  // Yaw center
    }
    
    // Send RC command (errors are rate-limited to prevent spam)
    if (!CustomProtocol_SendRcCommand(rcChannels)) {
      // Rate-limit error messages
      if (now - lastTxErrorMs >= ERROR_PRINT_INTERVAL) {
        Serial.println("[TX] Failed to send RC command (rate-limited errors)");
        lastTxErrorMs = now;
      }
    }
  }
  
  // Send heartbeat at 2 Hz
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL) {
    lastHeartbeatMs = now;
    
    if (CustomProtocol_SendHeartbeat()) {
      // Serial.println("[TX] Heartbeat sent");
    }
  }
  
  // Print statistics every second
  if (now - lastStatsMs >= STATS_INTERVAL) {
    lastStatsMs = now;
    
    ProtocolStats stats;
    CustomProtocol_GetStats(&stats);
    
    Serial.println("\n--- TX Statistics ---");
    Serial.printf("Packets Sent: %lu | Received: %lu | Loss: %.1f%%\n", 
                  stats.packetsSent, stats.packetsReceived, stats.packetLossPercent);
    Serial.printf("Link: %s | Last RX: %lu ms ago\n", 
                  stats.linkActive ? "ACTIVE" : "DOWN", stats.lastPacketMs);
    
    // Display telemetry if received
    if (stats.packetsReceived > 0) {
      TelemetryPayload telemetry;
      CustomProtocol_GetTelemetry(&telemetry);
      
      // Show voltage/current or N/A if not available
      if (telemetry.voltage > 0.1) {
        Serial.printf("Battery: %.2fV ", telemetry.voltage);
      } else {
        Serial.print("Battery: N/A ");
      }
      
      if (telemetry.current > 0.1) {
        Serial.printf("%.2fA | ", telemetry.current);
      } else {
        Serial.print("N/A | ");
      }
      
      Serial.printf("RSSI: %ddBm | LQ: %d%%\n", 
                    telemetry.rssi, telemetry.linkQuality);
    }
    
    Serial.println();
  }
  
  // Small delay to prevent watchdog issues
  delay(1);
}

#endif // BUILD_TX

#ifdef BUILD_RX
// ============================================================
// RECEIVER (RX) CODE
// ============================================================

#include "crsf_bridge.h"

// CRSF Bridge instance
CRSFBridge crsfBridge;

// Pin definitions for CRSF output
// Adjust these for your ESP32-C3 board
#define CRSF_TX_PIN 21  // ESP32 TX -> FC RX
#define CRSF_RX_PIN 20  // ESP32 RX -> FC TX (optional for telemetry)

// Timing
unsigned long lastTelemetrySendMs = 0;
unsigned long lastStatsMs = 0;
unsigned long lastRxErrorMs = 0;
unsigned long lastCrsfUpdateMs = 0;

const unsigned long TELEMETRY_SEND_INTERVAL = 100;  // 10 Hz (100ms)
const unsigned long STATS_INTERVAL = 1000;          // 1 Hz (every second, same as TX)
const unsigned long ERROR_PRINT_INTERVAL = 1000;    // Max 1 error per second

// Telemetry values (will be read from FC or sensors)
// For now using simulated values - replace with actual sensor/FC readings
float batteryVoltage = 0.0;  // Will show N/A if 0
float batteryCurrent = 0.0;  // Will show N/A if 0
int16_t rssi = -45;           // Simulated RSSI
uint8_t linkQuality = 100;    // Calculated from packet stats

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=================================");
  Serial.println("Custom Protocol Receiver (RX)");
  Serial.println("=================================\n");
  
  // Initialize protocol (RX mode, will auto-pair with any TX)
  if (!CustomProtocol_Init(false, nullptr)) {
    Serial.println("[RX] Protocol init failed!");
    while (1) {
      delay(1000);
    }
  }
  
  // Initialize CRSF bridge to output to FC
  if (crsfBridge.init(&Serial1, CRSF_TX_PIN, CRSF_RX_PIN)) {
    Serial.println("[RX] CRSF bridge initialized");
    Serial.printf("[RX] Outputting CRSF to FC on pin %d\n", CRSF_TX_PIN);
  } else {
    Serial.println("[RX] WARNING: CRSF bridge init failed!");
  }
  
  Serial.println("[RX] Ready to receive!");
  Serial.println("[RX] Listening for any transmitter (broadcast mode)");
  Serial.println("[RX] Telemetry: 10 Hz | Stats: 1 Hz | CRSF: 50 Hz");
  Serial.println();
}

void loop() {
  unsigned long now = millis();
  
  // Update protocol (handles timeouts, parsing, etc.)
  CustomProtocol_Update();
  
  // Update CRSF bridge
  crsfBridge.update();
  
  // Send RC channels to FC via CRSF at 50 Hz (match reference code)
  if (now - lastCrsfUpdateMs >= 20) {  // 50 Hz like the reference
    lastCrsfUpdateMs = now;
    
    if (crsfBridge.isActive()) {
      if (CustomProtocol_IsLinkActive()) {
        // Get RC channels from protocol
        uint16_t channels[16];
        CustomProtocol_GetRcChannels(channels);
        
        // Send to flight controller via CRSF
        crsfBridge.sendRcChannels(channels);
      } else {
        // Send failsafe values via CRSF
        uint16_t failsafeChannels[16] = {
          992, 992, 172, 992, 172, 172, 172, 172,  // Safe values
          172, 172, 172, 172, 172, 172, 172, 172
        };
        crsfBridge.sendRcChannels(failsafeChannels);
      }
    }
  }
  
  // Send telemetry at 10 Hz (only if link is active)
  if (CustomProtocol_IsLinkActive() && (now - lastTelemetrySendMs >= TELEMETRY_SEND_INTERVAL)) {
    lastTelemetrySendMs = now;
    
    // TODO: Read real telemetry from flight controller or sensors
    // For now, sending placeholder values (0.0 will display as N/A on TX)
    batteryVoltage = 0.0;  // TODO: Read from ADC or FC
    batteryCurrent = 0.0;  // TODO: Read from current sensor or FC
    
    // Calculate link quality based on packet stats
    ProtocolStats stats;
    CustomProtocol_GetStats(&stats);
    linkQuality = (stats.linkActive) ? 100 : 0;
    
    // Send telemetry (errors are rate-limited to prevent spam)
    if (!CustomProtocol_SendTelemetry(batteryVoltage, batteryCurrent, rssi, linkQuality)) {
      // Rate-limit error messages
      if (now - lastRxErrorMs >= ERROR_PRINT_INTERVAL) {
        Serial.println("[RX] Failed to send telemetry (rate-limited errors)");
        lastRxErrorMs = now;
      }
    }
  }
  
  // Print statistics every second
  if (now - lastStatsMs >= STATS_INTERVAL) {
    lastStatsMs = now;
    
    ProtocolStats stats;
    CustomProtocol_GetStats(&stats);
    
    Serial.println("\n--- RX Statistics ---");
    Serial.printf("Packets RX: %lu | TX: %lu | Loss: %.1f%% | CRC Err: %lu\n", 
                  stats.packetsReceived, stats.packetsSent, 
                  stats.packetLossPercent, stats.crcErrors);
    Serial.printf("Link: %s | Last RX: %lu ms ago\n", 
                  stats.linkActive ? "ACTIVE" : "FAILSAFE", stats.lastPacketMs);
    Serial.printf("CRSF: %lu frames sent to FC\n", crsfBridge.getFramesSent());
    
    // Display RC channels if link is active
    if (stats.linkActive) {
      uint16_t channels[16];
      CustomProtocol_GetRcChannels(channels);
      
      Serial.printf("RC: R:%d P:%d T:%d Y:%d | AUX1:%d AUX2:%d\n",
                    channels[0], channels[1], channels[2], channels[3],
                    channels[4], channels[5]);
    } else {
      Serial.println("⚠️  FAILSAFE MODE - No signal");
    }
    
    Serial.println();
  }
  
  // Small delay to prevent watchdog issues
  delay(1);
}

#endif // BUILD_RX
