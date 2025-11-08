// ELRS TX (Transmitter) for ESP32 + SX1280
// Reference: https://github.com/ExpressLRS/ExpressLRS
// Protocol: https://github.com/ExpressLRS/ExpressLRS/wiki/Protocol

#ifndef ELRS_TX_H
#define ELRS_TX_H

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

bool ELRS_TX_Init();
void ELRS_TX_SetChannels(uint16_t* channels);  // 16 channels, CRSF format: 172-1811
void ELRS_TX_SendPacket();
void ELRS_TX_Update();
void ELRS_TX_GetStats(uint32_t* packets, uint32_t* rate);
void ELRS_TX_SetPacketRate(uint32_t hz);  // 50-500 Hz

#endif // ELRS_TX_H

