// LoRa Integration with Custom Protocol + CRSF Bridge
// LoRa集成示例 (使用SX1280)

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "custom/protocol.h"
#include "custom/crsf_bridge.h"

// SX1280 引脚配置
#define LORA_MISO   13
#define LORA_MOSI   11
#define LORA_SCK    12
#define LORA_NSS    10
#define LORA_RST    14
#define LORA_DIO1   9
#define LORA_BUSY   8

SPIClass spiLora(HSPI);
SX1280 lora(&spiLora, LORA_NSS, LORA_BUSY, LORA_DIO1, LORA_RST);

uint8_t rxBuffer[256];

// 初始化LoRa (RX模式)
bool initLoRa() {
  Serial.println("[LoRa] Initializing SX1280...");
  
  spiLora.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  
  int state = lora.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] Init failed, code: %d\n", state);
    return false;
  }
  
  // FLRC模式配置
  state = lora.setFrequency(2440.0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] setFrequency failed, code: %d\n", state);
    return false;
  }
  
  state = lora.setDataRate(RADIOLIB_SX1280_FLRC_BR_1_300_BW_1_2);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] setDataRate failed, code: %d\n", state);
    return false;
  }
  
  state = lora.setCodingRate(RADIOLIB_SX1280_FLRC_CR_3_4);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] setCodingRate failed, code: %d\n", state);
    return false;
  }
  
  // 开始接收
  state = lora.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] startReceive failed, code: %d\n", state);
    return false;
  }
  
  Serial.println("[LoRa] Initialized - listening on 2440 MHz");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("=== LoRa + Custom Protocol + CRSF ===");
  
  // 初始化自定义协议
  if (!CustomProtocol_Init()) {
    Serial.println("ERROR: Failed to init protocol");
    while (1) delay(1000);
  }
  
  // 初始化CRSF桥接
  if (!CrsfBridge_Init(43, 44, 1)) {
    Serial.println("ERROR: Failed to init CRSF bridge");
    while (1) delay(1000);
  }
  
  CrsfBridge_SetUpdateRate(50);
  
  // 初始化LoRa
  if (!initLoRa()) {
    Serial.println("ERROR: Failed to init LoRa");
    while (1) delay(1000);
  }
  
  Serial.println("System ready - waiting for LoRa data...");
}

void loop() {
  // 接收LoRa数据
  int state = lora.readData(nullptr, 0);
  
  if (state == RADIOLIB_ERR_NONE) {
    int len = lora.getPacketLength();
    
    if (len > 0 && len <= sizeof(rxBuffer)) {
      state = lora.readData(rxBuffer, len);
      
      if (state == RADIOLIB_ERR_NONE) {
        // 获取信号质量
        int16_t rssi = lora.getRSSI();
        int8_t snr = lora.getSNR();
        
        // 解析数据包
        if (CustomProtocol_ParsePacket(rxBuffer, len)) {
          Serial.printf("[LoRa] Parsed packet, RSSI=%d, SNR=%d\n", rssi, snr);
        }
      }
    }
    
    // 重新开始接收
    lora.startReceive();
  }
  
  // 更新协议 (检查超时)
  CustomProtocol_Update();
  
  // 获取RC通道
  uint16_t channels[16];
  CustomProtocol_GetRcChannels(channels);
  
  // 发送到Betaflight
  CrsfBridge_SetChannels(channels);
  CrsfBridge_Update();
  
  // 状态打印
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    
    ProtocolStats pStats;
    CustomProtocol_GetStats(&pStats);
    
    CrsfBridgeStats cStats;
    CrsfBridge_GetStats(&cStats);
    
    Serial.println("=== Status ===");
    Serial.printf("Protocol: RX=%lu, Errors=%lu, Link=%s\n",
                  pStats.packetsReceived,
                  pStats.crcErrors + pStats.timeoutErrors,
                  pStats.linkActive ? "UP" : "DOWN");
    
    Serial.printf("CRSF: Frames=%lu, Skipped=%lu\n",
                  cStats.framesSent, cStats.framesSkipped);
    
    Serial.printf("Channels: R=%d P=%d T=%d Y=%d\n",
                  channels[0], channels[1], channels[2], channels[3]);
    
    Serial.println();
  }
}

// ============================================================================
// 地面站发送示例 (TX端)
// ============================================================================

#ifdef LORA_GROUND_STATION

void setupGroundStation() {
  Serial.begin(115200);
  
  Serial.println("[LoRa] Initializing SX1280 TX...");
  
  spiLora.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  
  int state = lora.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] Init failed, code: %d\n", state);
    while (1) delay(1000);
  }
  
  // FLRC模式配置 (必须与RX匹配)
  lora.setFrequency(2440.0);
  lora.setDataRate(RADIOLIB_SX1280_FLRC_BR_1_300_BW_1_2);
  lora.setCodingRate(RADIOLIB_SX1280_FLRC_CR_3_4);
  lora.setOutputPower(10);  // 10 dBm
  
  CustomProtocol_Init();
  
  Serial.println("Ground station ready");
}

void loopGroundStation() {
  static uint32_t lastSend = 0;
  uint32_t now = millis();
  
  if (now - lastSend >= 20) {  // 50Hz
    lastSend = now;
    
    // 构建RC指令
    uint16_t channels[16] = {992, 992, 500, 992, 1811, 992, 992, 992,
                              992, 992, 992, 992, 992, 992, 992, 992};
    
    // 打包数据
    RcCommandPayload payload;
    for (int i = 0; i < 16; i++) {
      payload.channels[i] = channels[i];
    }
    payload.timestamp = millis();
    payload.flags = 0;
    
    // 构建完整数据包
    uint8_t packet[256];
    CustomPacket* pkt = (CustomPacket*)packet;
    pkt->magic = CUSTOM_PROTO_MAGIC;
    pkt->version = CUSTOM_PROTO_VERSION;
    pkt->packetType = PKT_RC_COMMAND;
    pkt->sequence = 0;
    pkt->payloadLength = sizeof(RcCommandPayload);
    memcpy(pkt->payload, &payload, sizeof(payload));
    
    size_t dataLen = 6 + sizeof(payload);
    uint16_t crc = CustomProtocol_CRC16(packet, dataLen);
    packet[dataLen] = crc & 0xFF;
    packet[dataLen + 1] = (crc >> 8) & 0xFF;
    
    // 通过LoRa发送
    int state = lora.transmit(packet, dataLen + 2);
    
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("Packet sent");
    } else {
      Serial.printf("Send failed, code: %d\n", state);
    }
  }
}

#endif

