// ELRS RX Implementation
// Reference: https://github.com/ExpressLRS/ExpressLRS
// SX1280 Datasheet: https://www.semtech.com/products/wireless-rf/24-ghz-transceivers/sx1280

#include "elrs_rx.h"

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

static uint32_t lastPacketReceived = 0;
static uint32_t packetsReceived = 0;
static uint32_t packetErrors = 0;
static int16_t lastRSSI = 0;
static int8_t lastSNR = 0;
static bool linkEstablished = false;

// RC通道数据 (11-bit CRSF format: 172-1811)
uint16_t rxRcChannels[16] = {
  992, 992, 172, 992, 172, 992, 992, 992,
  992, 992, 992, 992, 992, 992, 992, 992
};

static ELRS_RX_Callback dataCallback = nullptr;

bool ELRS_RX_Init() {
  Serial.println("[ELRS RX] Initializing SX1280...");
  
  spiLora.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  
  int state = lora.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ELRS RX] SX1280 init failed, code: %d\n", state);
    return false;
  }
  
  // FLRC模式 (Fast Long Range Communication)
  state = lora.setFrequency(2440.0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ELRS RX] setFrequency failed, code: %d\n", state);
    return false;
  }
  
  // FLRC参数 (必须与TX匹配)
  state = lora.setDataRate(RADIOLIB_SX1280_FLRC_BR_1_300_BW_1_2);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ELRS RX] setDataRate failed, code: %d\n", state);
    return false;
  }
  
  state = lora.setCodingRate(RADIOLIB_SX1280_FLRC_CR_3_4);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ELRS RX] setCodingRate failed, code: %d\n", state);
    return false;
  }
  
  state = lora.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ELRS RX] startReceive failed, code: %d\n", state);
    return false;
  }
  
  Serial.println("[ELRS RX] SX1280 initialized successfully, listening...");
  return true;
}

bool ELRS_RX_UnpackPacket(uint8_t* packet, size_t len) {
  if (len != ELRS_PACKET_SIZE) {
    return false;
  }
  
  // CRC校验
  uint8_t crc = 0;
  for (size_t i = 0; i < len - 1; i++) {
    crc ^= packet[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
    }
  }
  
  if (crc != packet[len - 1]) {
    packetErrors++;
    return false;
  }
  
  if (packet[0] != ELRS_RC_DATA_PACKET) {
    return false;
  }
  
  // 解包通道数据 (10-bit)
  uint16_t ch0 = packet[1] | ((packet[2] & 0x03) << 8);
  uint16_t ch1 = ((packet[2] >> 2) & 0x3F) | ((packet[3] & 0x0F) << 6);
  uint16_t ch2 = ((packet[3] >> 4) & 0x0F) | ((packet[4] & 0x3F) << 4);
  uint16_t ch3 = ((packet[4] >> 6) & 0x03) | (packet[5] << 2);
  
  // 转换 10-bit (0-1023) 到 11-bit CRSF (172-1811)
  rxRcChannels[0] = map(ch0, 0, 1023, 172, 1811);
  rxRcChannels[1] = map(ch1, 0, 1023, 172, 1811);
  rxRcChannels[2] = map(ch2, 0, 1023, 172, 1811);
  rxRcChannels[3] = map(ch3, 0, 1023, 172, 1811);
  
  // 辅助通道 (开关)
  uint8_t auxByte = packet[6];
  for (int i = 4; i < 12; i++) {
    uint8_t bit = (auxByte >> ((i - 4) % 8)) & 0x01;
    rxRcChannels[i] = bit ? 1811 : 172;
  }
  
  for (int i = 12; i < 16; i++) {
    rxRcChannels[i] = 992;
  }
  
  return true;
}

void ELRS_RX_Update() {
  int state = lora.readData(nullptr, 0);
  
  if (state == RADIOLIB_ERR_NONE) {
    uint8_t packet[ELRS_PACKET_SIZE];
    int len = lora.getPacketLength();
    
    if (len > 0 && len <= ELRS_PACKET_SIZE) {
      state = lora.readData(packet, len);
      
      if (state == RADIOLIB_ERR_NONE) {
        lastRSSI = lora.getRSSI();
        lastSNR = lora.getSNR();
        
        if (ELRS_RX_UnpackPacket(packet, len)) {
          packetsReceived++;
          lastPacketReceived = millis();
          linkEstablished = true;
          
          if (dataCallback != nullptr) {
            dataCallback(rxRcChannels);
          }
        }
      }
    }
    
    lora.startReceive();
  }
  
  // 链路超时检测 (1秒)
  if (millis() - lastPacketReceived > 1000) {
    linkEstablished = false;
    
    // 失效保护: 油门低，其他居中
    rxRcChannels[0] = 992;
    rxRcChannels[1] = 992;
    rxRcChannels[2] = 172;
    rxRcChannels[3] = 992;
    for (int i = 4; i < 16; i++) {
      rxRcChannels[i] = 172;
    }
  }
}

void ELRS_RX_GetChannels(uint16_t* channels) {
  for (int i = 0; i < 16; i++) {
    channels[i] = rxRcChannels[i];
  }
}

bool ELRS_RX_IsLinkUp() {
  return linkEstablished && (millis() - lastPacketReceived < 1000);
}

void ELRS_RX_GetStats(ELRS_RX_Stats* stats) {
  stats->packetsReceived = packetsReceived;
  stats->packetErrors = packetErrors;
  stats->rssi = lastRSSI;
  stats->snr = lastSNR;
  stats->linkQuality = ELRS_RX_IsLinkUp() ? 100 : 0;
  stats->lastPacketMs = millis() - lastPacketReceived;
}

void ELRS_RX_SetCallback(ELRS_RX_Callback callback) {
  dataCallback = callback;
}

