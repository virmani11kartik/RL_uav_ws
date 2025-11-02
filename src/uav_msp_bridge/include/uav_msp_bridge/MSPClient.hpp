#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <mutex>

namespace uav_msp_bridge {

class MSPClient {
public:
  MSPClient(const std::string& device, int baud);
  ~MSPClient();

  bool open();
  void close();
  bool isOpen() const;

  // Thread-safe request: writes a command and reads the matching response payload into 'out'.
  // Returns true if a valid frame (cmd-matched + checksum/CRC ok) was received within timeout_s.
  bool request(uint16_t cmd, const std::vector<uint8_t>& payload,
               std::vector<uint8_t>& out, double timeout_s);

private:
  // ---- MSP wire helpers ----
  enum class Proto { V1, V2 };
  struct Frame {
    Proto proto = Proto::V1;
    uint16_t cmd = 0;
    std::vector<uint8_t> payload;
  };

  // io
  bool writeAll(const uint8_t* data, size_t n);
  ssize_t readSome(uint8_t* buf, size_t n, double timeout_s);
  bool drainInput(double seconds);

  // frame build/parse
  bool writeFrameV1(uint16_t cmd, const std::vector<uint8_t>& payload);
  bool writeFrameV2(uint16_t cmd, const std::vector<uint8_t>& payload); // not used by default
  bool readFrame(Frame& f, double timeout_s);
  bool findHeader(uint8_t& h2, uint8_t& h3, double timeout_s);

  static uint8_t checksumV1(uint8_t size, uint8_t cmd, const uint8_t* p, size_t n);
  static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t start = 0);

  // little-endian helpers
  static inline uint16_t le16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
  static inline uint32_t le32(const uint8_t* p) { return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24)); }

private:
  std::string dev_;
  int baud_;
  int fd_ = -1;
  std::mutex io_mtx_;
};

} // namespace uav_msp_bridge
