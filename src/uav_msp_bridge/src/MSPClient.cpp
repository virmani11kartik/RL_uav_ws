#include "uav_msp_bridge/MSPClient.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <cstring>
#include <chrono>
#include <thread>


using namespace std::chrono_literals;

namespace uav_msp_bridge {

MSPClient::MSPClient(const std::string& device, int baud)
: dev_(device), baud_(baud) {}

MSPClient::~MSPClient() { close(); }

bool MSPClient::open() {
  if (fd_ >= 0) return true;
  fd_ = ::open(dev_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) { ::close(fd_); fd_ = -1; return false; }

  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  speed_t spd = B115200;
  switch (baud_) {
    case 9600: spd = B9600; break;
    case 19200: spd = B19200; break;
    case 38400: spd = B38400; break;
    case 57600: spd = B57600; break;
    case 115200: spd = B115200; break;
    case 230400: spd = B230400; break;
    case 460800: spd = B460800; break;
    case 921600: spd = B921600; break;
    default: spd = B115200; break;
  }

  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) { ::close(fd_); fd_ = -1; return false; }

  // Drain any junk bytes
  drainInput(0.15);
  return true;
}

void MSPClient::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool MSPClient::isOpen() const { return fd_ >= 0; }

bool MSPClient::writeAll(const uint8_t* data, size_t n) {
  size_t off = 0;
  while (off < n) {
    ssize_t w = ::write(fd_, data + off, n - off);
    if (w < 0) {
      if (errno == EAGAIN || errno == EINTR) continue;
      return false;
    }
    off += (size_t)w;
  }
  return true;
}

ssize_t MSPClient::readSome(uint8_t* buf, size_t n, double timeout_s) {
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(fd_, &rfds);

  timeval tv{};
  tv.tv_sec  = (int)timeout_s;
  tv.tv_usec = (int)((timeout_s - (int)timeout_s) * 1e6);

  int ret = select(fd_ + 1, &rfds, nullptr, nullptr, (timeout_s >= 0) ? &tv : nullptr);
  if (ret <= 0) return ret; // 0 timeout; -1 error
  ssize_t r = ::read(fd_, buf, n);
  if (r < 0 && (errno == EAGAIN || errno == EINTR)) return 0;
  return r;
}

bool MSPClient::drainInput(double seconds) {
  auto t0 = std::chrono::steady_clock::now();
  uint8_t tmp[256];
  while (std::chrono::steady_clock::now() - t0 < std::chrono::duration<double>(seconds)) {
    ssize_t r = ::read(fd_, tmp, sizeof(tmp));
    if (r <= 0) {
      // sleep a tiny bit to avoid busy loop
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  tcflush(fd_, TCIFLUSH);
  return true;
}

// ---------- MSP helpers ----------
uint8_t MSPClient::checksumV1(uint8_t size, uint8_t cmd, const uint8_t* p, size_t n) {
  uint8_t c = size ^ cmd;
  for (size_t i = 0; i < n; ++i) c ^= p[i];
  return c;
}

uint16_t MSPClient::crc16_ccitt(const uint8_t* data, size_t len, uint16_t start) {
  uint16_t crc = start;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc <<= 1;
    }
  }
  return crc;
}

bool MSPClient::writeFrameV1(uint16_t cmd16, const std::vector<uint8_t>& payload) {
  // MSPv1 uses 8-bit command IDs; for 16-bit we must use v2. Here we assume <=255.
  const uint8_t cmd = static_cast<uint8_t>(cmd16 & 0xFF);
  const uint8_t size = static_cast<uint8_t>(payload.size());
  std::vector<uint8_t> buf;
  buf.reserve(3 + 2 + payload.size() + 1);
  buf.push_back('$'); buf.push_back('M'); buf.push_back('<');
  buf.push_back(size); buf.push_back(cmd);
  buf.insert(buf.end(), payload.begin(), payload.end());
  buf.push_back(checksumV1(size, cmd, payload.data(), payload.size()));
  return writeAll(buf.data(), buf.size());
}

bool MSPClient::writeFrameV2(uint16_t cmd, const std::vector<uint8_t>& payload) {
  // MSPv2: $ X <  flags(1)  size(2 LE)  cmd(2 LE)  payload  crc(2 LE)
  const uint8_t flags = 0; // no fragmentation
  const uint16_t size = (uint16_t)payload.size();
  std::vector<uint8_t> buf;
  buf.reserve(3 + 1 + 2 + 2 + size + 2);
  buf.push_back('$'); buf.push_back('X'); buf.push_back('<');
  buf.push_back(flags);
  buf.push_back((uint8_t)(size & 0xFF));
  buf.push_back((uint8_t)((size >> 8) & 0xFF));
  buf.push_back((uint8_t)(cmd & 0xFF));
  buf.push_back((uint8_t)((cmd >> 8) & 0xFF));
  buf.insert(buf.end(), payload.begin(), payload.end());

  // CRC over: flags,size(2),cmd(2),payload
  uint16_t crc = crc16_ccitt(&buf[3], 1 + 2 + 2 + size, 0);
  buf.push_back((uint8_t)(crc & 0xFF));
  buf.push_back((uint8_t)((crc >> 8) & 0xFF));
  return writeAll(buf.data(), buf.size());
}

bool MSPClient::findHeader(uint8_t& h2, uint8_t& h3, double timeout_s) {
  auto start = std::chrono::steady_clock::now();
  uint8_t b = 0;
  while (true) {
    if (timeout_s >= 0) {
      auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed > timeout_s) return false;
    }
    ssize_t r = readSome(&b, 1, 0.05);
    if (r <= 0) continue;
    if (b != '$') continue;

    // candidate header
    uint8_t tmp[2];
    ssize_t r2 = readSome(tmp, 2, 0.1);
    if (r2 != 2) continue;
    if ((tmp[0] == 'M' && (tmp[1] == '>' || tmp[1] == '!')) ||
        (tmp[0] == 'X' && (tmp[1] == '>' || tmp[1] == '!'))) {
      h2 = tmp[0]; h3 = tmp[1];
      return true;
    }
    // else slide one byte and continue
  }
}

bool MSPClient::readFrame(Frame& f, double timeout_s) {
  // Re-sync scanner: find $M> (v1) or $X> (v2)
  uint8_t h2=0, h3=0;
  if (!findHeader(h2, h3, timeout_s)) return false;

  if (h2 == 'M') {
    // v1
    uint8_t hdr[2];
    if (readSome(hdr, 2, 0.2) != 2) return false;
    const uint8_t size = hdr[0];
    const uint8_t cmd  = hdr[1];

    std::vector<uint8_t> payload(size);
    if (size > 0) {
      if (readSome(payload.data(), size, 0.5) != (ssize_t)size) return false;
    }
    uint8_t csum = 0;
    if (readSome(&csum, 1, 0.2) != 1) return false;
    if (checksumV1(size, cmd, payload.data(), payload.size()) != csum) {
      // Bad frame -> drop one byte and try again
      return readFrame(f, timeout_s); // recursive re-sync (bounded by timeout)
    }
    f.proto = Proto::V1;
    f.cmd = cmd;
    f.payload = std::move(payload);
    return true;
  } else {
    // v2
    uint8_t hdr[5];
    if (readSome(hdr, 5, 0.3) != 5) return false;
    const uint8_t flags = hdr[0];
    (void)flags;
    const uint16_t size = le16(&hdr[1]);
    const uint16_t cmd  = le16(&hdr[3]);

    std::vector<uint8_t> payload(size);
    if (size > 0) {
      if (readSome(payload.data(), size, 0.7) != (ssize_t)size) return false;
    }
    uint8_t crcbuf[2];
    if (readSome(crcbuf, 2, 0.2) != 2) return false;
    const uint16_t crc_rx = (uint16_t)(crcbuf[0] | (crcbuf[1] << 8));

    // CRC over flags,size(2),cmd(2),payload
    std::vector<uint8_t> to_crc;
    to_crc.reserve(1 + 2 + 2 + size);
    to_crc.push_back(flags);
    to_crc.push_back((uint8_t)(size & 0xFF));
    to_crc.push_back((uint8_t)((size >> 8) & 0xFF));
    to_crc.push_back((uint8_t)(cmd & 0xFF));
    to_crc.push_back((uint8_t)((cmd >> 8) & 0xFF));
    to_crc.insert(to_crc.end(), payload.begin(), payload.end());
    const uint16_t crc_calc = crc16_ccitt(to_crc.data(), to_crc.size(), 0);

    if (crc_calc != crc_rx) {
      // Bad frame -> try again (re-sync)
      return readFrame(f, timeout_s);
    }
    f.proto = Proto::V2;
    f.cmd = cmd;
    f.payload = std::move(payload);
    return true;
  }
}

bool MSPClient::request(uint16_t cmd, const std::vector<uint8_t>& payload,
                        std::vector<uint8_t>& out, double timeout_s) {
  std::lock_guard<std::mutex> lk(io_mtx_);
  if (fd_ < 0) return false;

  drainInput(0.01);

  // For Betaflight MSP API 1.46, most standard requests work in v1.
  // If you need >255 cmd or large payloads, switch to writeFrameV2.
  if (!writeFrameV1(cmd, payload)) return false;

  auto t0 = std::chrono::steady_clock::now();
  while (true) {
    double remaining = timeout_s;
    if (timeout_s >= 0) {
      double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (elapsed >= timeout_s) return false;
      remaining = timeout_s - elapsed;
    }

    Frame f;
    if (!readFrame(f, remaining)) return false;
    // Accept both v1/v2 but require matching command
    if (f.cmd == cmd) {
      out = std::move(f.payload);
      return true;
    }
  }
}

} // namespace uav_msp_bridge
