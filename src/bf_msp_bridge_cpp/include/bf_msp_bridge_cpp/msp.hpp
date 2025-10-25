#pragma once
#include <cstdint>
#include <vector>
#include <array>
#include <cstring>

// MSP IDs (core subset)
enum : uint16_t {
  MSP_API_VERSION     = 1,
  MSP_FC_VARIANT      = 2,
  MSP_FC_VERSION      = 3,
  MSP_STATUS          = 101,
  MSP_ATTITUDE        = 108,
  MSP_IMU             = 102,
  MSP_BATTERY_STATE   = 130,
  MSP_MOTOR           = 104,
  MSP_RC              = 105,
  MSP_SET_RAW_RC      = 200,   // command
};

// MSP v1 framing: $M<  size cmd payload checksum
// We’ll use v1 for simplicity (Betaflight still supports it).
namespace msp {

// Build a request (no payload)
inline std::vector<uint8_t> makeRequest(uint8_t cmd) {
  std::vector<uint8_t> pkt;
  pkt.reserve(6);
  pkt.push_back('$'); pkt.push_back('M'); pkt.push_back('<');
  uint8_t size = 0;
  pkt.push_back(size);
  pkt.push_back(cmd);
  uint8_t cksum = size ^ cmd;
  pkt.push_back(cksum);
  return pkt;
}

// Build a request with payload (e.g., MSP_SET_RAW_RC)
inline std::vector<uint8_t> makeRequest(uint8_t cmd, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> pkt;
  pkt.reserve(6 + payload.size());
  pkt.push_back('$'); pkt.push_back('M'); pkt.push_back('<');
  uint8_t size = static_cast<uint8_t>(payload.size());
  pkt.push_back(size);
  pkt.push_back(cmd);
  uint8_t cksum = size ^ cmd;
  for (auto b : payload) { pkt.push_back(b); cksum ^= b; }
  pkt.push_back(cksum);
  return pkt;
}

// A tiny state machine decoder for MSP v1 responses: $M> size cmd payload cksum
struct Decoder {
  enum { H1, H2, DIR, SIZE, CMD, PAYLOAD, CKSUM } state{H1};
  uint8_t size{0}, cmd{0}, cksum{0};
  std::vector<uint8_t> payload;
  bool hasFrame{false};

  void reset() { state=H1; size=cmd=cksum=0; payload.clear(); hasFrame=false; }

  // Feed one byte. When hasFrame=true, payload/cmd contain a full message.
  void feed(uint8_t b) {
    switch(state) {
      case H1: if (b=='$') state=H2; break;
      case H2: if (b=='M') state=DIR; else state=H1; break;
      case DIR: if (b=='>') state=SIZE; else state=H1; break;
      case SIZE: size=b; cksum=b; payload.clear(); state=CMD; break;
      case CMD: cmd=b; cksum ^= b; if (size==0) state=CKSUM; else state=PAYLOAD; break;
      case PAYLOAD:
        payload.push_back(b); cksum ^= b;
        if (payload.size()==size) state=CKSUM;
        break;
      case CKSUM:
        if (cksum==b) { hasFrame=true; } // valid frame
        else { /* drop */ }
        state=H1; break;
    }
  }
};

inline void push_u16(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back(x & 0xFF);
  v.push_back((x >> 8) & 0xFF);
}

} // namespace msp
