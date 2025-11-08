// WiFi UDP Integration with Custom Protocol + CRSF Bridge
// WiFi UDP集成示例

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "custom/protocol.h"
#include "custom/crsf_bridge.h"

// WiFi配置
const char* ssid = "UAV_CONTROL";        // 修改为实际SSID
const char* password = "12345678";       // 修改为实际密码

// UDP配置
WiFiUDP udp;
const uint16_t localPort = 14550;        // 本地监听端口
const uint16_t remotePort = 14551;       // 远程发送端口

uint8_t rxBuffer[512];

// 初始化WiFi (Station模式)
bool initWiFiStation() {
  Serial.printf("[WiFi] Connecting to %s...\n", ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[WiFi] Connection failed");
    return false;
  }
  
  Serial.println("\n[WiFi] Connected");
  Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
  
  return true;
}

// 初始化WiFi (AP模式)
bool initWiFiAP() {
  Serial.printf("[WiFi] Creating AP: %s\n", ssid);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());
  
  return true;
}

// 初始化UDP
bool initUDP() {
  if (udp.begin(localPort)) {
    Serial.printf("[UDP] Listening on port %d\n", localPort);
    return true;
  }
  
  Serial.println("[UDP] Failed to start");
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("=== WiFi UDP + Custom Protocol + CRSF ===");
  
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
  
  // 初始化WiFi (选择Station或AP模式)
  #ifdef WIFI_AP_MODE
  if (!initWiFiAP()) {
  #else
  if (!initWiFiStation()) {
  #endif
    Serial.println("ERROR: Failed to init WiFi");
    while (1) delay(1000);
  }
  
  // 初始化UDP
  if (!initUDP()) {
    Serial.println("ERROR: Failed to init UDP");
    while (1) delay(1000);
  }
  
  Serial.println("System ready - waiting for UDP data...");
}

void loop() {
  // 接收UDP数据
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(rxBuffer, sizeof(rxBuffer));
    
    if (len > 0) {
      // 解析数据包
      if (CustomProtocol_ParsePacket(rxBuffer, len)) {
        Serial.printf("[UDP] Parsed packet from %s:%d\n",
                      udp.remoteIP().toString().c_str(),
                      udp.remotePort());
      }
    }
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
    Serial.printf("WiFi: %s, IP: %s\n",
                  WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                  WiFi.localIP().toString().c_str());
    
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

#ifdef WIFI_GROUND_STATION

IPAddress receiverIP(192, 168, 4, 1);  // 修改为接收端IP

void setupGroundStation() {
  Serial.begin(115200);
  
  // 连接WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  
  udp.begin(localPort);
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
    
    // 通过UDP发送
    udp.beginPacket(receiverIP, remotePort);
    udp.write(packet, dataLen + 2);
    udp.endPacket();
    
    Serial.println("Packet sent");
  }
}

#endif

