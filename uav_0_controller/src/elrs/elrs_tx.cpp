// ELRS TX Implementation
// Reference: https://github.com/ExpressLRS/ExpressLRS
// SX1280 Datasheet: https://www.semtech.com/products/wireless-rf/24-ghz-transceivers/sx1280

#include "elrs_tx.h"

// SX1280 引脚配置
#define LORA_MISO   13
#define LORA_MOSI   11
#define LORA_SCK    12
#define LORA_NSS    10
#define LORA_RST    14
#define LORA_DIO1   9
#define LORA_BUSY   8

#define ELRS_RC_DATA_PACKET 0x16
#define ELRS_PACKET_SIZE 14

SPIClass spiLora(HSPI);
SX1280 lora(&spiLora, LORA_NSS, LORA_BUSY, LORA_DIO1, LORA_RST);

static uint32_t lastPacketSent = 0;
static uint32_t packetInterval = 4;  // 默认250Hz (2ms=500Hz, 4ms=250Hz, 8ms=125Hz)
static uint32_t packetCount = 0;

// RC通道数据 (11-bit CRSF format: 172-1811)
uint16_t txRcChannels[16] = {
  992, 992, 172, 992, 172, 992, 992, 992,
  992, 992, 992, 992, 992, 992, 992, 992
};

bool ELRS_TX_Init() {
  Serial.println("[ELRS TX] Initializing SX1280...");
  
  spiLora.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  
  int state = lora.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ELRS TX] SX1280 init failed, code: %d\n", state);
    return false;
  }
  
  // FLRC模式 (Fast Long Range Communication) - 低延迟RC链路
  state = lora.setFrequency(2440.0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ELRS TX] setFrequency failed, code: %d\n", state);
    return false;
  }
  
  // FLRC参数: BR=1.3Mbps, CR=3/4
  state = lora.setDataRate(RADIOLIB_SX1280_FLRC_BR_1_300_BW_1_2);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ELRS TX] setDataRate failed, code: %d\n", state);
    return false;
  }
  
  state = lora.setCodingRate(RADIOLIB_SX1280_FLRC_CR_3_4);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ELRS TX] setCodingRate failed, code: %d\n", state);
    return false;
  }
  
  state = lora.setOutputPower(10);  // 10 dBm
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ELRS TX] setOutputPower failed, code: %d\n", state);
    return false;
  }
  
  Serial.println("[ELRS TX] SX1280 initialized successfully");
  return true;
}

void ELRS_TX_SetChannels(uint16_t* channels) {
  for (int i = 0; i < 16; i++) {
    uint16_t val = channels[i];
    if (val < 172) val = 172;
    if (val > 1811) val = 1811;
    txRcChannels[i] = val;
  }
}

void ELRS_TX_SendPacket() {
  uint8_t packet[ELRS_PACKET_SIZE];
  
  packet[0] = ELRS_RC_DATA_PACKET;
  
  // 转换11-bit (172-1811) 到 10-bit (0-1023)
  uint16_t ch0 = map(txRcChannels[0], 172, 1811, 0, 1023);
  uint16_t ch1 = map(txRcChannels[1], 172, 1811, 0, 1023);
  uint16_t ch2 = map(txRcChannels[2], 172, 1811, 0, 1023);
  uint16_t ch3 = map(txRcChannels[3], 172, 1811, 0, 1023);
  
  // 打包通道数据 (10-bit each)
  packet[1] = (ch0 & 0xFF);
  packet[2] = ((ch0 >> 8) & 0x03) | ((ch1 & 0x3F) << 2);
  packet[3] = ((ch1 >> 6) & 0x0F) | ((ch2 & 0x0F) << 4);
  packet[4] = ((ch2 >> 4) & 0x3F) | ((ch3 & 0x03) << 6);
  packet[5] = ((ch3 >> 2) & 0xFF);
  
  // 辅助通道 (开关)
  uint8_t auxByte = 0;
  for (int i = 4; i < 12; i++) {
    uint8_t switchPos = (txRcChannels[i] > 992) ? 1 : 0;
    auxByte |= (switchPos << ((i - 4) % 8));
  }
  packet[6] = auxByte;
  
  packet[7] = (packetCount & 0xFF);
  packet[8] = ((packetCount >> 8) & 0xFF);
  
  for (int i = 9; i < ELRS_PACKET_SIZE - 1; i++) {
    packet[i] = 0;
  }
  
  // CRC8校验
  uint8_t crc = 0;
  for (int i = 0; i < ELRS_PACKET_SIZE - 1; i++) {
    crc ^= packet[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
    }
  }
  packet[ELRS_PACKET_SIZE - 1] = crc;
  
  int state = lora.transmit(packet, ELRS_PACKET_SIZE);
  if (state == RADIOLIB_ERR_NONE) {
    packetCount++;
  } else {
    Serial.printf("[ELRS TX] Transmit failed, code: %d\n", state);
  }
}

void ELRS_TX_Update() {
  uint32_t now = millis();
  
  if (now - lastPacketSent >= packetInterval) {
    lastPacketSent = now;
    ELRS_TX_SendPacket();
  }
}

void ELRS_TX_GetStats(uint32_t* packets, uint32_t* rate) {
  *packets = packetCount;
  *rate = 1000 / packetInterval;
}

void ELRS_TX_SetPacketRate(uint32_t hz) {
  if (hz > 0 && hz <= 500) {
    packetInterval = 1000 / hz;
  }
}

