// Custom Wireless Protocol
// 自定义无线通信协议

#ifndef CUSTOM_PROTOCOL_H
#define CUSTOM_PROTOCOL_H

#include <Arduino.h>

#define CUSTOM_PROTO_MAGIC      0xA5
#define CUSTOM_PROTO_VERSION    0x01
#define CUSTOM_PROTO_MAX_PAYLOAD 128

enum CustomPacketType {
  PKT_RC_COMMAND    = 0x10,  // RC通道指令
  PKT_DIRECT_CMD    = 0x11,  // 直接控制指令
  PKT_TELEMETRY_REQ = 0x20,  // 遥测请求
  PKT_TELEMETRY_DATA= 0x21,  // 遥测数据
  PKT_CONFIG        = 0x30,  // 配置
  PKT_ACK           = 0xF0,  // 确认
  PKT_NACK          = 0xF1,  // 否定
  PKT_HEARTBEAT     = 0xFF   // 心跳
};

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t version;
  uint8_t packetType;
  uint8_t sequence;
  uint16_t payloadLength;
  uint8_t payload[CUSTOM_PROTO_MAX_PAYLOAD];
  uint16_t crc16;
} CustomPacket;

typedef struct __attribute__((packed)) {
  uint16_t channels[16];   // CRSF format: 172-1811
  uint32_t timestamp;
  uint8_t flags;
} RcCommandPayload;

typedef struct __attribute__((packed)) {
  float roll;       // -1.0 to 1.0
  float pitch;      // -1.0 to 1.0
  float yaw;        // -1.0 to 1.0
  float throttle;   // 0.0 to 1.0
  uint32_t timestamp;
} DirectCommandPayload;

typedef struct __attribute__((packed)) {
  float voltage;
  float current;
  int16_t rssi;
  uint8_t linkQuality;
  uint32_t timestamp;
} TelemetryPayload;

typedef struct {
  uint32_t packetsReceived;
  uint32_t packetsSent;
  uint32_t crcErrors;
  uint32_t timeoutErrors;
  uint32_t lastPacketMs;
  bool linkActive;
} ProtocolStats;

bool CustomProtocol_Init();
void CustomProtocol_Update();
bool CustomProtocol_ParsePacket(const uint8_t* data, size_t len);
bool CustomProtocol_SendPacket(CustomPacketType type, const uint8_t* payload, uint16_t payloadLen);
bool CustomProtocol_SendRcCommand(uint16_t* channels);
bool CustomProtocol_SendDirectCommand(float roll, float pitch, float yaw, float throttle);
void CustomProtocol_GetRcChannels(uint16_t* channels);
bool CustomProtocol_IsLinkActive();
void CustomProtocol_GetStats(ProtocolStats* stats);
uint16_t CustomProtocol_CRC16(const uint8_t* data, size_t len);

#endif // CUSTOM_PROTOCOL_H
