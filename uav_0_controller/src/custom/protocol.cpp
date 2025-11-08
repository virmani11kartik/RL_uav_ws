// Custom Protocol Implementation
// 自定义协议实现

#include "protocol.h"

static uint8_t sequenceNumber = 0;
static uint32_t lastPacketReceivedMs = 0;
static uint32_t lastPacketSentMs = 0;

// RC通道存储
static uint16_t currentRcChannels[16] = {
  992, 992, 172, 992, 172, 992, 992, 992,
  992, 992, 992, 992, 992, 992, 992, 992
};

static ProtocolStats stats = {0};
static uint8_t rxBuffer[256];
static size_t rxBufferPos = 0;

uint16_t CustomProtocol_CRC16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

bool CustomProtocol_Init() {
  Serial.println("[Custom] Initializing...");
  
  stats.packetsReceived = 0;
  stats.packetsSent = 0;
  stats.crcErrors = 0;
  stats.timeoutErrors = 0;
  stats.lastPacketMs = 0;
  stats.linkActive = false;
  
  rxBufferPos = 0;
  
  Serial.println("[Custom] Initialized");
  return true;
}

bool CustomProtocol_ParsePacket(const uint8_t* data, size_t len) {
  if (len < sizeof(CustomPacket) - CUSTOM_PROTO_MAX_PAYLOAD) {
    return false;
  }
  
  const CustomPacket* pkt = reinterpret_cast<const CustomPacket*>(data);
  
  if (pkt->magic != CUSTOM_PROTO_MAGIC) {
    return false;
  }
  
  if (pkt->version != CUSTOM_PROTO_VERSION) {
    Serial.printf("[Custom] Version mismatch: expected 0x%02X, got 0x%02X\n", 
                  CUSTOM_PROTO_VERSION, pkt->version);
    return false;
  }
  
  size_t expectedLen = 6 + pkt->payloadLength + 2;
  if (len < expectedLen) {
    return false;
  }
  
  // CRC校验
  uint16_t receivedCrc = (uint16_t)data[expectedLen - 2] | ((uint16_t)data[expectedLen - 1] << 8);
  uint16_t calculatedCrc = CustomProtocol_CRC16(data, expectedLen - 2);
  
  if (receivedCrc != calculatedCrc) {
    stats.crcErrors++;
    return false;
  }
  
  // 解析数据包
  switch (pkt->packetType) {
    case PKT_RC_COMMAND: {
      if (pkt->payloadLength >= sizeof(RcCommandPayload)) {
        const RcCommandPayload* rcCmd = reinterpret_cast<const RcCommandPayload*>(pkt->payload);
        
        for (int i = 0; i < 16; i++) {
          uint16_t val = rcCmd->channels[i];
          if (val < 172) val = 172;
          if (val > 1811) val = 1811;
          currentRcChannels[i] = val;
        }
        
        lastPacketReceivedMs = millis();
        stats.packetsReceived++;
        stats.linkActive = true;
      }
      break;
    }
    
    case PKT_DIRECT_CMD: {
      if (pkt->payloadLength >= sizeof(DirectCommandPayload)) {
        const DirectCommandPayload* directCmd = reinterpret_cast<const DirectCommandPayload*>(pkt->payload);
        
        // 转换归一化指令到RC通道
        currentRcChannels[0] = (uint16_t)((directCmd->roll + 1.0f) * 819.5f + 172.0f);
        currentRcChannels[1] = (uint16_t)((directCmd->pitch + 1.0f) * 819.5f + 172.0f);
        currentRcChannels[2] = (uint16_t)(directCmd->throttle * 1639.0f + 172.0f);
        currentRcChannels[3] = (uint16_t)((directCmd->yaw + 1.0f) * 819.5f + 172.0f);
        
        for (int i = 0; i < 4; i++) {
          if (currentRcChannels[i] < 172) currentRcChannels[i] = 172;
          if (currentRcChannels[i] > 1811) currentRcChannels[i] = 1811;
        }
        
        lastPacketReceivedMs = millis();
        stats.packetsReceived++;
        stats.linkActive = true;
      }
      break;
    }
    
    case PKT_HEARTBEAT: {
      lastPacketReceivedMs = millis();
      stats.packetsReceived++;
      stats.linkActive = true;
      break;
    }
    
    default:
      break;
  }
  
  return true;
}

void CustomProtocol_Update() {
  uint32_t now = millis();
  
  // 超时检测 (1秒)
  if (stats.linkActive && (now - lastPacketReceivedMs > 1000)) {
    stats.linkActive = false;
    stats.timeoutErrors++;
    Serial.println("[Custom] Link timeout - failsafe");
    
    // 失效保护
    currentRcChannels[0] = 992;
    currentRcChannels[1] = 992;
    currentRcChannels[2] = 172;  // 油门低
    currentRcChannels[3] = 992;
    for (int i = 4; i < 16; i++) {
      currentRcChannels[i] = 172;
    }
  }
  
  stats.lastPacketMs = now - lastPacketReceivedMs;
  
  // TODO: 添加传输层接收代码
}

bool CustomProtocol_SendPacket(CustomPacketType type, const uint8_t* payload, uint16_t payloadLen) {
  if (payloadLen > CUSTOM_PROTO_MAX_PAYLOAD) {
    return false;
  }
  
  uint8_t txBuffer[sizeof(CustomPacket)];
  CustomPacket* pkt = reinterpret_cast<CustomPacket*>(txBuffer);
  
  pkt->magic = CUSTOM_PROTO_MAGIC;
  pkt->version = CUSTOM_PROTO_VERSION;
  pkt->packetType = type;
  pkt->sequence = sequenceNumber++;
  pkt->payloadLength = payloadLen;
  
  if (payload != nullptr && payloadLen > 0) {
    memcpy(pkt->payload, payload, payloadLen);
  }
  
  size_t dataLen = 6 + payloadLen;
  uint16_t crc = CustomProtocol_CRC16(txBuffer, dataLen);
  txBuffer[dataLen] = crc & 0xFF;
  txBuffer[dataLen + 1] = (crc >> 8) & 0xFF;
  
  // TODO: 通过传输层发送
  
  lastPacketSentMs = millis();
  stats.packetsSent++;
  
  return true;
}

bool CustomProtocol_SendRcCommand(uint16_t* channels) {
  RcCommandPayload payload;
  for (int i = 0; i < 16; i++) {
    payload.channels[i] = channels[i];
  }
  payload.timestamp = millis();
  payload.flags = 0;
  
  return CustomProtocol_SendPacket(PKT_RC_COMMAND, 
                                   reinterpret_cast<uint8_t*>(&payload), 
                                   sizeof(payload));
}

bool CustomProtocol_SendDirectCommand(float roll, float pitch, float yaw, float throttle) {
  DirectCommandPayload payload;
  payload.roll = roll;
  payload.pitch = pitch;
  payload.yaw = yaw;
  payload.throttle = throttle;
  payload.timestamp = millis();
  
  return CustomProtocol_SendPacket(PKT_DIRECT_CMD, 
                                   reinterpret_cast<uint8_t*>(&payload), 
                                   sizeof(payload));
}

void CustomProtocol_GetRcChannels(uint16_t* channels) {
  for (int i = 0; i < 16; i++) {
    channels[i] = currentRcChannels[i];
  }
}

bool CustomProtocol_IsLinkActive() {
  return stats.linkActive;
}

void CustomProtocol_GetStats(ProtocolStats* stats_out) {
  memcpy(stats_out, &stats, sizeof(ProtocolStats));
}
