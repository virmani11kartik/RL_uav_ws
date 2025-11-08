// CRSF Bridge Implementation
// Protocol: https://github.com/crsf-wg/crsf/wiki

#include "crsf_bridge.h"

static HardwareSerial* crsfSerial = nullptr;

static uint16_t rcChannels[16] = {
  992, 992, 172, 992, 172, 992, 992, 992,
  992, 992, 992, 992, 992, 992, 992, 992
};

static uint32_t lastFrameSentMs = 0;
static uint32_t updateIntervalMs = CRSF_UPDATE_INTERVAL_MS;
static CrsfBridgeStats stats = {0};

uint8_t CrsfBridge_CRC8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

void CrsfBridge_PackChannels(uint16_t* channels, uint8_t* output) {
  // 将16个11-bit通道打包成22字节
  output[0]  = (uint8_t)((channels[0]) & 0x07FF);
  output[1]  = (uint8_t)(((channels[0] & 0x07FF) >> 8) | ((channels[1] & 0x07FF) << 3));
  output[2]  = (uint8_t)(((channels[1] & 0x07FF) >> 5) | ((channels[2] & 0x07FF) << 6));
  output[3]  = (uint8_t)(((channels[2] & 0x07FF) >> 2));
  output[4]  = (uint8_t)(((channels[2] & 0x07FF) >> 10) | ((channels[3] & 0x07FF) << 1));
  output[5]  = (uint8_t)(((channels[3] & 0x07FF) >> 7)  | ((channels[4] & 0x07FF) << 4));
  output[6]  = (uint8_t)(((channels[4] & 0x07FF) >> 4)  | ((channels[5] & 0x07FF) << 7));
  output[7]  = (uint8_t)(((channels[5] & 0x07FF) >> 1));
  output[8]  = (uint8_t)(((channels[5] & 0x07FF) >> 9)  | ((channels[6] & 0x07FF) << 2));
  output[9]  = (uint8_t)(((channels[6] & 0x07FF) >> 6)  | ((channels[7] & 0x07FF) << 5));
  output[10] = (uint8_t)(((channels[7] & 0x07FF) >> 3));
  
  output[11] = (uint8_t)((channels[8]) & 0x07FF);
  output[12] = (uint8_t)(((channels[8] & 0x07FF) >> 8)  | ((channels[9] & 0x07FF) << 3));
  output[13] = (uint8_t)(((channels[9] & 0x07FF) >> 5)  | ((channels[10] & 0x07FF) << 6));
  output[14] = (uint8_t)(((channels[10] & 0x07FF) >> 2));
  output[15] = (uint8_t)(((channels[10] & 0x07FF) >> 10) | ((channels[11] & 0x07FF) << 1));
  output[16] = (uint8_t)(((channels[11] & 0x07FF) >> 7)  | ((channels[12] & 0x07FF) << 4));
  output[17] = (uint8_t)(((channels[12] & 0x07FF) >> 4)  | ((channels[13] & 0x07FF) << 7));
  output[18] = (uint8_t)(((channels[13] & 0x07FF) >> 1));
  output[19] = (uint8_t)(((channels[13] & 0x07FF) >> 9)  | ((channels[14] & 0x07FF) << 2));
  output[20] = (uint8_t)(((channels[14] & 0x07FF) >> 6)  | ((channels[15] & 0x07FF) << 5));
  output[21] = (uint8_t)(((channels[15] & 0x07FF) >> 3));
}

bool CrsfBridge_Init(int txPin, int rxPin, int serialPort) {
  Serial.printf("[CRSF] Initializing on UART%d (TX=%d, RX=%d)...\n", 
                serialPort, txPin, rxPin);
  
  if (serialPort == 1) {
    crsfSerial = &Serial1;
  } else if (serialPort == 2) {
    crsfSerial = &Serial2;
  } else {
    Serial.println("[CRSF] Invalid serial port");
    return false;
  }
  
  crsfSerial->begin(CRSF_BAUDRATE, CRSF_SERIAL_CONFIG, rxPin, txPin);
  
  stats.framesSent = 0;
  stats.framesSkipped = 0;
  stats.lastFrameMs = 0;
  stats.fcConnected = false;
  
  Serial.printf("[CRSF] Initialized at %d baud, %d Hz\n", 
                CRSF_BAUDRATE, 1000 / updateIntervalMs);
  return true;
}

void CrsfBridge_SetChannels(uint16_t* channels) {
  for (int i = 0; i < 16; i++) {
    uint16_t val = channels[i];
    if (val < 172) val = 172;
    if (val > 1811) val = 1811;
    rcChannels[i] = val;
  }
}

void CrsfBridge_SendFrame() {
  if (crsfSerial == nullptr) {
    return;
  }
  
  uint8_t frame[CRSF_FRAME_SIZE_RC_CHANNELS];
  
  frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame[1] = 24;
  frame[2] = CRSF_FRAMETYPE_RC_CHANNELS;
  
  CrsfBridge_PackChannels(rcChannels, &frame[3]);
  
  frame[25] = CrsfBridge_CRC8(&frame[2], 23);
  
  size_t written = crsfSerial->write(frame, CRSF_FRAME_SIZE_RC_CHANNELS);
  
  if (written == CRSF_FRAME_SIZE_RC_CHANNELS) {
    stats.framesSent++;
    stats.lastFrameMs = millis();
    stats.fcConnected = true;
  } else {
    stats.framesSkipped++;
  }
}

void CrsfBridge_Update() {
  uint32_t now = millis();
  
  if (now - lastFrameSentMs >= updateIntervalMs) {
    lastFrameSentMs = now;
    CrsfBridge_SendFrame();
  }
}

void CrsfBridge_GetStats(CrsfBridgeStats* stats_out) {
  memcpy(stats_out, &stats, sizeof(CrsfBridgeStats));
}

void CrsfBridge_SetUpdateRate(uint32_t hz) {
  if (hz > 0 && hz <= 500) {
    updateIntervalMs = 1000 / hz;
    Serial.printf("[CRSF] Update rate set to %d Hz (%d ms)\n", hz, updateIntervalMs);
  }
}
