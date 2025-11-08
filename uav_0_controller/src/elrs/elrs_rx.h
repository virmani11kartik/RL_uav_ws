// ELRS RX (Receiver) for ESP32 + SX1280
// Reference: https://github.com/ExpressLRS/ExpressLRS
// Protocol: https://github.com/ExpressLRS/ExpressLRS/wiki/Protocol

#ifndef ELRS_RX_H
#define ELRS_RX_H

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

typedef struct {
  uint32_t packetsReceived;
  uint32_t packetErrors;
  int16_t rssi;
  int8_t snr;
  uint8_t linkQuality;
  uint32_t lastPacketMs;
} ELRS_RX_Stats;

typedef void (*ELRS_RX_Callback)(uint16_t* channels);

bool ELRS_RX_Init();
void ELRS_RX_Update();
void ELRS_RX_GetChannels(uint16_t* channels);  // 16 channels, CRSF format: 172-1811
bool ELRS_RX_IsLinkUp();
void ELRS_RX_GetStats(ELRS_RX_Stats* stats);
void ELRS_RX_SetCallback(ELRS_RX_Callback callback);

#endif // ELRS_RX_H

