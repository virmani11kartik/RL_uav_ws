// crsf_tx_node.cpp
//
// ROS 2 node that transmits CRSF RC frames over /dev/ttyUSB0 at 420000 baud.
// Input: /rc_aetr_aux (std_msgs/UInt16MultiArray) with 8 values:
//   [A, E, T, R, AUX1, AUX2, AUX3, AUX4] in microseconds.
//
// Defaults (when no RC received recently):
//   A=1500 E=1500 T=885 R=1500 AUX1=975 AUX2-4=1500; ch9-16 = 1500
//
// Wiring (USB-TTL <-> FC UART):
//   USB-TTL TX -> FC RX (Serial RX pad)
//   USB-TTL GND -> FC GND
//   (optional) USB-TTL RX <- FC TX (not required for TX-only)
//   DO NOT connect VCC.
//
// Betaflight:
//   Ports: enable "Serial RX" on that UART, disable MSP on that UART.
//   Config: Receiver Mode = Serial-based receiver, Provider = CRSF.
//

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <algorithm>
#include <chrono>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

// Linux termios2 for arbitrary baud (420000)
#include <asm/termbits.h>
#include <linux/serial.h>

class CrsfTxNode : public rclcpp::Node {
public:
  CrsfTxNode() : Node("crsf_tx_node") {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB1");
    baud_ = declare_parameter<int>("baud", 420000);
    rate_hz_ = declare_parameter<int>("rate_hz", 50);
    rc_timeout_s_ = declare_parameter<double>("rc_timeout_s", 0.2);

    // Defaults in microseconds (your request)
    defaults_us_[0] = 1500; // A
    defaults_us_[1] = 1500; // E
    defaults_us_[2] =  885; // T
    defaults_us_[3] = 1500; // R
    defaults_us_[4] =  975; // AUX1
    defaults_us_[5] = 1500; // AUX2
    defaults_us_[6] = 1500; // AUX3
    defaults_us_[7] = 1500; // AUX4

    // Initialize channels to defaults (CRSF-scaled)
    setDefaults();

    if (!openSerial(port_, baud_)) {
      RCLCPP_FATAL(get_logger(), "Failed to open %s at %d baud", port_.c_str(), baud_);
      throw std::runtime_error("serial open failed");
    }

    sub_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/rc_aetr_aux", 10,
      std::bind(&CrsfTxNode::rcCallback, this, std::placeholders::_1));

    using namespace std::chrono_literals;
    const int period_ms = std::max(1, 1000 / std::max(1, rate_hz_));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&CrsfTxNode::tick, this));

    last_rc_time_ = now();

    RCLCPP_INFO(get_logger(), "CRSF TX: port=%s baud=%d rate_hz=%d topic=/rc_aetr_aux",
                port_.c_str(), baud_, rate_hz_);
  }

  ~CrsfTxNode() override {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  // CRSF basics
  static constexpr uint8_t CRSF_ADDR_FC = 0xC8;
  static constexpr uint8_t CRSF_TYPE_RC = 0x16;

  // Your exact mapping: us [988..2012] -> CRSF [172..1811]
  static inline uint16_t us_to_crsf_exact(int us) {
    constexpr int US_MIN = 988;
    constexpr int US_MAX = 2012;
    constexpr int CRSF_MIN = 172;
    constexpr int CRSF_MAX = 1811;

    if (us <= US_MIN) return CRSF_MIN;
    if (us >= US_MAX) return CRSF_MAX;

    return static_cast<uint16_t>(
      CRSF_MIN +
      (((int64_t)(us - US_MIN) * (CRSF_MAX - CRSF_MIN) + (US_MAX - US_MIN) / 2) / (US_MAX - US_MIN))
    );
  }

  // CRSF CRC8 (poly 0xD5) over [TYPE..PAYLOAD]
  static uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t c = 0x00;
    for (size_t i = 0; i < len; i++) {
      c ^= data[i];
      for (int j = 0; j < 8; j++) {
        c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1);
      }
    }
    return c;
  }

  void setDefaults() {
    // Fill all 16 channels in CRSF units (11-bit values)
    for (int i = 0; i < 16; i++) ch_[i] = us_to_crsf_exact(1500);

    // AETR + AUX1..4
    ch_[0] = us_to_crsf_exact(defaults_us_[0]); // A
    ch_[1] = us_to_crsf_exact(defaults_us_[1]); // E
    ch_[2] = us_to_crsf_exact(defaults_us_[2]); // T
    ch_[3] = us_to_crsf_exact(defaults_us_[3]); // R
    ch_[4] = us_to_crsf_exact(defaults_us_[4]); // AUX1
    ch_[5] = us_to_crsf_exact(defaults_us_[5]); // AUX2
    ch_[6] = us_to_crsf_exact(defaults_us_[6]); // AUX3
    ch_[7] = us_to_crsf_exact(defaults_us_[7]); // AUX4
  }

  // Build the exact 26-byte RC frame you used on ESP
  std::array<uint8_t, 26> buildRcFrame() const {
    std::array<uint8_t, 26> f{};
    f[0] = CRSF_ADDR_FC;
    f[1] = 24;          // TYPE(1) + PAYLOAD(22) + CRC(1)
    f[2] = CRSF_TYPE_RC;

    // Pack 16x 11-bit channels into 22 bytes (same bit packing as your ESP code)
    const uint16_t* ch = ch_.data();

    f[3]  =  ch[0] & 0xFF;
    f[4]  = (ch[0] >> 8) | (uint8_t)((ch[1] & 0x07) << 3);
    f[5]  = (ch[1] >> 5) | (uint8_t)((ch[2] & 0x3F) << 6);
    f[6]  =  ch[2] >> 2;
    f[7]  = (ch[2] >> 10) | (uint8_t)((ch[3] & 0x01FF) << 1);
    f[8]  = (ch[3] >> 7)  | (uint8_t)((ch[4] & 0x0F) << 4);
    f[9]  = (ch[4] >> 4)  | (uint8_t)((ch[5] & 0x01) << 7);
    f[10] =  ch[5] >> 1;
    f[11] = (ch[5] >> 9)  | (uint8_t)((ch[6] & 0x3) << 2);
    f[12] = (ch[6] >> 6)  | (uint8_t)((ch[7] & 0x1F) << 5);
    f[13] =  ch[7] >> 3;

    f[14] =  ch[8] & 0xFF;
    f[15] = (ch[8] >> 8)  | (uint8_t)((ch[9] & 0x07) << 3);
    f[16] = (ch[9] >> 5)  | (uint8_t)((ch[10] & 0x3F) << 6);
    f[17] =  ch[10] >> 2;
    f[18] = (ch[10] >> 10) | (uint8_t)((ch[11] & 0x01FF) << 1);
    f[19] = (ch[11] >> 7)  | (uint8_t)((ch[12] & 0x0F) << 4);
    f[20] = (ch[12] >> 4)  | (uint8_t)((ch[13] & 0x01) << 7);
    f[21] =  ch[13] >> 1;
    f[22] = (ch[13] >> 9)  | (uint8_t)((ch[14] & 0x3) << 2);
    f[23] = (ch[14] >> 6)  | (uint8_t)((ch[15] & 0x1F) << 5);
    f[24] =  ch[15] >> 3;

    // CRC over [TYPE..PAYLOAD] => f[2]..f[24] (23 bytes)
    f[25] = crc8(&f[2], 23);
    return f;
  }

  void rcCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    if (msg->data.size() < 8) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Expected 8 values [A,E,T,R,AUX1..AUX4], got %zu",
                           msg->data.size());
      return;
    }

    // Convert incoming us -> CRSF units
    ch_[0] = us_to_crsf_exact((int)msg->data[0]); // A
    ch_[1] = us_to_crsf_exact((int)msg->data[1]); // E
    ch_[2] = us_to_crsf_exact((int)msg->data[2]); // T
    ch_[3] = us_to_crsf_exact((int)msg->data[3]); // R
    ch_[4] = us_to_crsf_exact((int)msg->data[4]); // AUX1
    ch_[5] = us_to_crsf_exact((int)msg->data[5]); // AUX2
    ch_[6] = us_to_crsf_exact((int)msg->data[6]); // AUX3
    ch_[7] = us_to_crsf_exact((int)msg->data[7]); // AUX4

    // Keep remaining channels at 1500us
    for (int i = 8; i < 16; i++) ch_[i] = us_to_crsf_exact(1500);

    last_rc_time_ = now();
  }

  void tick() {
    // Failsafe: if no updates, revert to defaults
    const double dt = (now() - last_rc_time_).seconds();
    if (dt > rc_timeout_s_) {
      setDefaults();
    }

    auto frame = buildRcFrame();
    const ssize_t n = ::write(fd_, frame.data(), frame.size());
    if (n != (ssize_t)frame.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "CRSF write short/failed: wrote %zd/%zu (errno=%d %s)",
                           n, frame.size(), errno, strerror(errno));
    }
  }

  // Open serial and set arbitrary baud using termios2 (BOTHER)
  bool openSerial(const std::string& port, int baud) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", port.c_str(), strerror(errno));
      return false;
    }

    termios2 tio{};
    if (ioctl(fd_, TCGETS2, &tio) != 0) {
      RCLCPP_ERROR(get_logger(), "TCGETS2 failed: %s", strerror(errno));
      return false;
    }

    // raw-ish
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;

    // 8N1
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~CRTSCTS;

    tio.c_ispeed = baud;
    tio.c_ospeed = baud;

    // Non-blocking-ish
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;

    if (ioctl(fd_, TCSETS2, &tio) != 0) {
      RCLCPP_ERROR(get_logger(), "TCSETS2 failed: %s", strerror(errno));
      return false;
    }

    return true;
  }

private:
  std::string port_;
  int baud_{420000};
  int rate_hz_{50};
  double rc_timeout_s_{0.2};

  int fd_{-1};

  // Incoming defaults in microseconds
  std::array<uint16_t, 8> defaults_us_{};

  // CRSF channel values (11-bit effective range, stored in uint16)
  std::array<uint16_t, 16> ch_{};

  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_rc_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrsfTxNode>());
  rclcpp::shutdown();
  return 0;
}

// // ibus_tx_node.cpp
// //
// // ROS 2 node that transmits iBUS Serial RX frames over a UART (USB-TTL) on /dev/ttyUSB0.
// // Input topic provides AETR + AUX1-4 in microseconds (or your calibrated units).
// //
// // Topic: /rc_aetr_aux   (std_msgs/UInt16MultiArray)
// // Expects: [A, E, T, R, AUX1, AUX2, AUX3, AUX4]
// //
// // Defaults (as requested):
// //   A=1500 E=1500 R=1500 T=885 AUX1=975 AUX2-4=1500
// //
// // Notes:
// // - Betaflight setup for iBUS Serial RX:
// //   Ports tab: enable "Serial RX" on chosen UART (NOT MSP)
// //   Configuration tab: Receiver Mode = "Serial-based receiver", Provider = "IBUS"
// //   CLI:
// //     set receiver_type = SERIAL
// //     set serialrx_provider = IBUS
// //     save
// //
// // - Wiring (USB-TTL <-> FC UART):
// //   TX -> RX, RX -> TX, GND -> GND, DO NOT connect VCC.
// // - Use a non-inverted UART (avoid SBUS-inverted pads).
// //

// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/u_int16_multi_array.hpp>

// #include <array>
// #include <cstdint>
// #include <cstring>
// #include <string>
// #include <vector>
// #include <algorithm>
// #include <sstream>

// #include <fcntl.h>
// #include <unistd.h>
// #include <termios.h>
// #include <errno.h>

// class IbusTxNode : public rclcpp::Node {
// public:
//   IbusTxNode() : Node("ibus_tx_node") {
//     port_    = declare_parameter<std::string>("port", "/dev/ttyUSB1");
//     rate_hz_ = declare_parameter<int>("rate_hz", 100);
//     baud_    = declare_parameter<int>("baud", 115200);
//     timeout_s_ = declare_parameter<double>("rc_timeout_s", 0.2);

//     // Default channels (14 channels total; we actively use first 8)
//     channels_.fill(1500);

//     // Requested defaults (AETR + AUX1..4)
//     channels_[0] = 1500; // A (roll)
//     channels_[1] = 1500; // E (pitch)
//     channels_[2] =  885; // T (throttle)
//     channels_[3] = 1500; // R (yaw)
//     channels_[4] =  975; // AUX1
//     channels_[5] = 1500; // AUX2
//     channels_[6] = 1500; // AUX3
//     channels_[7] = 1500; // AUX4

//     // Keep other channels centered
//     for (int i = 8; i < 14; i++) channels_[i] = 1500;

//     if (!openSerial()) {
//       RCLCPP_FATAL(get_logger(), "Failed to open serial port %s", port_.c_str());
//       throw std::runtime_error("serial open failed");
//     }

//     sub_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
//       "/rc_aetr_aux", 10,
//       std::bind(&IbusTxNode::rcCallback, this, std::placeholders::_1)
//     );

//     using namespace std::chrono_literals;
//     const int period_ms = std::max(1, 1000 / std::max(1, rate_hz_));
//     timer_ = create_wall_timer(
//       std::chrono::milliseconds(period_ms),
//       std::bind(&IbusTxNode::tick, this)
//     );

//     last_rc_time_ = now();

//     RCLCPP_INFO(get_logger(),
//                 "iBUS TX started: port=%s baud=%d rate_hz=%d timeout=%.3fs topic=/rc_aetr_aux",
//                 port_.c_str(), baud_, rate_hz_, timeout_s_);
//     RCLCPP_INFO(get_logger(),
//                 "Defaults: A=%u E=%u T=%u R=%u AUX1=%u AUX2=%u AUX3=%u AUX4=%u",
//                 channels_[0], channels_[1], channels_[2], channels_[3],
//                 channels_[4], channels_[5], channels_[6], channels_[7]);
//   }

//   ~IbusTxNode() override {
//     if (fd_ >= 0) ::close(fd_);
//   }

// private:
//   bool openSerial() {
//     fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
//     if (fd_ < 0) {
//       RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", port_.c_str(), strerror(errno));
//       return false;
//     }

//     termios tty{};
//     if (tcgetattr(fd_, &tty) != 0) {
//       RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
//       return false;
//     }

//     cfmakeraw(&tty);

//     // 8N1
//     tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
//     tty.c_cflag &= ~PARENB;
//     tty.c_cflag &= ~CSTOPB;
//     tty.c_cflag &= ~CRTSCTS;
//     tty.c_cflag |= (CLOCAL | CREAD);

//     tty.c_iflag &= ~(IXON | IXOFF | IXANY);

//     tty.c_cc[VMIN]  = 0;
//     tty.c_cc[VTIME] = 1;

//     // iBUS is typically 115200 8N1
//     speed_t speed = B115200;
//     if (baud_ != 115200) {
//       RCLCPP_WARN(get_logger(),
//                   "baud=%d requested; this node configures 115200 reliably. Using 115200.", baud_);
//       speed = B115200;
//     }

//     cfsetispeed(&tty, speed);
//     cfsetospeed(&tty, speed);

//     if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
//       RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
//       return false;
//     }

//     return true;
//   }

//   void setDefaultsAetrAux() {
//     channels_[0] = 1500; // A
//     channels_[1] = 1500; // E
//     channels_[2] =  885; // T
//     channels_[3] = 1500; // R
//     channels_[4] =  975; // AUX1
//     channels_[5] = 1500; // AUX2
//     channels_[6] = 1500; // AUX3
//     channels_[7] = 1500; // AUX4
//     for (int i = 8; i < 14; i++) channels_[i] = 1500;
//   }

//   void rcCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
//     if (msg->data.size() < 8) {
//       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
//                            "Expected 8 values [A,E,T,R,AUX1..AUX4], got %zu",
//                            msg->data.size());
//       return;
//     }

//     // Input order: A,E,T,R,AUX1..AUX4 (as requested)
//     channels_[0] = static_cast<uint16_t>(msg->data[0]); // A
//     channels_[1] = static_cast<uint16_t>(msg->data[1]); // E
//     channels_[2] = static_cast<uint16_t>(msg->data[2]); // T
//     channels_[3] = static_cast<uint16_t>(msg->data[3]); // R
//     channels_[4] = static_cast<uint16_t>(msg->data[4]); // AUX1
//     channels_[5] = static_cast<uint16_t>(msg->data[5]); // AUX2
//     channels_[6] = static_cast<uint16_t>(msg->data[6]); // AUX3
//     channels_[7] = static_cast<uint16_t>(msg->data[7]); // AUX4

//     // Keep remaining channels centered
//     for (int i = 8; i < 14; i++) channels_[i] = 1500;

//     last_rc_time_ = now();
//   }

//   // Build iBUS frame: 32 bytes:
//   // [0]=0x20, [1]=0x40, [2..29]=14ch * uint16 LE, [30..31]=checksum LE
//   std::array<uint8_t, 32> buildFrame() const {
//     std::array<uint8_t, 32> frame{};
//     frame[0] = 0x20;
//     frame[1] = 0x40;

//     for (int i = 0; i < 14; i++) {
//       uint16_t v = channels_[i];
//       frame[2 + 2*i]     = static_cast<uint8_t>(v & 0xFF);
//       frame[2 + 2*i + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
//     }

//     uint16_t sum = 0;
//     for (int i = 0; i < 30; i++) sum += frame[i];
//     uint16_t cks = static_cast<uint16_t>(0xFFFF - sum);

//     frame[30] = static_cast<uint8_t>(cks & 0xFF);
//     frame[31] = static_cast<uint8_t>((cks >> 8) & 0xFF);
//     return frame;
//   }

//   void tick() {
//     // If no RC updates recently, fall back to requested defaults
//     const double dt = (now() - last_rc_time_).seconds();
//     if (dt > timeout_s_) {
//       setDefaultsAetrAux();
//     }

//     auto frame = buildFrame();
//     ssize_t n = ::write(fd_, frame.data(), frame.size());
//     if (n != static_cast<ssize_t>(frame.size())) {
//       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
//                            "write() short/failed: wrote %zd/%zu (errno=%d %s)",
//                            n, frame.size(), errno, strerror(errno));
//     }
//   }

// private:
//   std::string port_;
//   int rate_hz_{100};
//   int baud_{115200};
//   double timeout_s_{0.2};

//   int fd_{-1};

//   // 14 channels for iBUS; we use first 8 primarily
//   std::array<uint16_t, 14> channels_{};

//   rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Time last_rc_time_{0, 0, RCL_ROS_TIME};
// };

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   try {
//     rclcpp::spin(std::make_shared<IbusTxNode>());
//   } catch (const std::exception& e) {
//     fprintf(stderr, "Fatal: %s\n", e.what());
//   }
//   rclcpp::shutdown();
//   return 0;
// }
