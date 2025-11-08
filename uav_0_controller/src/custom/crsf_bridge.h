// CRSF Bridge to Betaflight
// CRSF桥接到Betaflight
// Protocol: https://github.com/crsf-wg/crsf/wiki

#ifndef CRSF_BRIDGE_H
#define CRSF_BRIDGE_H

#include <Arduino.h>

#define CRSF_ADDRESS_FLIGHT_CONTROLLER  0xC8
#define CRSF_ADDRESS_RADIO_TRANSMITTER  0xEA
#define CRSF_FRAMETYPE_RC_CHANNELS      0x16
#define CRSF_FRAMETYPE_LINK_STATISTICS  0x14
#define CRSF_FRAME_SIZE_RC_CHANNELS     26

#define CRSF_UPDATE_RATE_HZ             50
#define CRSF_UPDATE_INTERVAL_MS         (1000 / CRSF_UPDATE_RATE_HZ)

#define CRSF_BAUDRATE                   420000
#define CRSF_SERIAL_CONFIG              SERIAL_8N1

typedef struct {
  uint32_t framesSent;
  uint32_t framesSkipped;
  uint32_t lastFrameMs;
  bool fcConnected;
} CrsfBridgeStats;

bool CrsfBridge_Init(int txPin, int rxPin, int serialPort = 1);
void CrsfBridge_Update();
void CrsfBridge_SetChannels(uint16_t* channels);  // 16 channels, CRSF format: 172-1811
void CrsfBridge_SendFrame();
void CrsfBridge_GetStats(CrsfBridgeStats* stats);
uint8_t CrsfBridge_CRC8(const uint8_t* data, size_t len);
void CrsfBridge_PackChannels(uint16_t* channels, uint8_t* output);
void CrsfBridge_SetUpdateRate(uint32_t hz);  // 1-500 Hz

#endif // CRSF_BRIDGE_H
