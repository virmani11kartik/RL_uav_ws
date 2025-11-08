// ESP-NOW Integration with Custom Protocol + CRSF Bridge
// ESP-NOW集成示例

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "custom/protocol.h"
#include "custom/crsf_bridge.h"

// ESP-NOW接收回调
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len > 0) {
    // 直接解析接收到的数据包
    if (CustomProtocol_ParsePacket(data, len)) {
      Serial.printf("[ESP-NOW] Parsed packet from %02X:%02X:%02X:%02X:%02X:%02X\n",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
  }
}

// 初始化ESP-NOW
bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init failed");
    return false;
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  
  Serial.println("[ESP-NOW] Initialized");
  Serial.printf("[ESP-NOW] MAC Address: %s\n", WiFi.macAddress().c_str());
  
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("=== ESP-NOW + Custom Protocol + CRSF ===");
  
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
  
  // 初始化ESP-NOW
  if (!initEspNow()) {
    Serial.println("ERROR: Failed to init ESP-NOW");
    while (1) delay(1000);
  }
  
  Serial.println("System ready - waiting for ESP-NOW data...");
}

void loop() {
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

#ifdef ESPNOW_GROUND_STATION

// 添加对端MAC地址
// CHANGE THIS TO THE MAC ADDRESS OF THE RECEIVER OR BROADCAST TO ALL DEVICES
uint8_t receiverMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

void setupGroundStation() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  
  // 添加对端设备
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
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
    
    // 通过ESP-NOW发送
    esp_err_t result = esp_now_send(receiverMac, packet, dataLen + 2);
    
    if (result == ESP_OK) {
      Serial.println("Packet sent");
    } else {
      Serial.println("Send failed");
    }
  }
}

#endif
