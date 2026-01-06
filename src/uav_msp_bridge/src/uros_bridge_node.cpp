#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <sstream>
#include <cerrno>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

using namespace std::chrono_literals;

namespace {

speed_t baud_to_speed_t(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return B115200;
  }
}

static inline uint16_t clamp_us(int v) {
  if (v < 800)  v = 800;
  if (v > 2200) v = 2200;
  return static_cast<uint16_t>(v);
}

class SerialPort {
public:
  SerialPort() = default;
  ~SerialPort() { close(); }

  bool open(const std::string& dev, int baud) {
    close();
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      perror("open serial");
      return false;
    }

    struct termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
      perror("tcgetattr");
      close();
      return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;   // 1 stop
    tio.c_cflag &= ~PARENB;   // no parity
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    const speed_t spd = baud_to_speed_t(baud);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      perror("tcsetattr");
      close();
      return false;
    }

    // Clear buffers
    tcflush(fd_, TCIOFLUSH);

    // Many USB CDC devices (including ESP) reboot on open; wait for it to come up
    usleep(1200 * 1000);

    return true;
  }

  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool write_all(const uint8_t* data, size_t size) {
    if (fd_ < 0) return false;
    size_t sent = 0;
    while (sent < size) {
      ssize_t n = ::write(fd_, data + sent, size - sent);
      if (n > 0) {
        sent += static_cast<size_t>(n);
      } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        ::usleep(100);
      } else {
        perror("serial write");
        return false;
      }
    }
    return true;
  }

  // ADDED: Read function for debugging
  int read_available(uint8_t* buffer, size_t max_size) {
    if (fd_ < 0) return -1;
    return ::read(fd_, buffer, max_size);
  }

private:
  int fd_ = -1;
};

} // namespace

class URosBridge : public rclcpp::Node {
public:
  URosBridge() : Node("ros_to_esp_rc") {
    port_    = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_    = declare_parameter<int>("baud", 115200);
    rate_hz_ = declare_parameter<double>("rate_hz", 50.0);
    debug_   = declare_parameter<bool>("debug", false);

    // Defaults: A,E,T,R,AUX1,AUX2,AUX3,AUX4
    set_rc({1500, 1500, 885, 1500, 900, 900, 900, 900});

    if (!serial_.open(port_, baud_)) {
      RCLCPP_FATAL(get_logger(), "Failed to open %s @ %d", port_.c_str(), baud_);
      throw std::runtime_error("serial open failed");
    }
    RCLCPP_INFO(get_logger(), "✓ Serial open: %s @ %d", port_.c_str(), baud_);

    sub_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/rc_aetr_aux", rclcpp::QoS(10),
      std::bind(&URosBridge::rc_cb, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&URosBridge::tx_tick, this));

    // ADDED: Timer to read ESP responses for debugging
    if (debug_) {
      read_timer_ = create_wall_timer(
        100ms,
        std::bind(&URosBridge::read_esp_response, this));
    }

    RCLCPP_INFO(get_logger(), "✓ Bridge ready at %.1f Hz", rate_hz_);
  }

private:
  struct Rc8 { uint16_t v[8]; };
  std::atomic_flag lock_ = ATOMIC_FLAG_INIT;
  Rc8 rc_{ {1500, 1500, 885, 1500, 900, 900, 900, 900} };

  void set_rc(const std::initializer_list<int>& vals) {
    Rc8 tmp{};
    int i = 0;
    for (int x : vals) {
      if (i < 8) tmp.v[i++] = clamp_us(x);
    }
    while (lock_.test_and_set(std::memory_order_acquire)) {}
    rc_ = tmp;
    lock_.clear(std::memory_order_release);
  }

  void rc_cb(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    if (msg->data.size() < 8) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                           "RC message has only %zu values, need 8", msg->data.size());
      return;
    }
    Rc8 tmp{};
    for (int i = 0; i < 8; i++) {
      tmp.v[i] = clamp_us(static_cast<int>(msg->data[i]));
    }
    while (lock_.test_and_set(std::memory_order_acquire)) {}
    rc_ = tmp;
    lock_.clear(std::memory_order_release);

    if (debug_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "RC update: A=%u E=%u T=%u R=%u AUX1=%u",
                           tmp.v[0], tmp.v[1], tmp.v[2], tmp.v[3], tmp.v[4]);
    }
  }

  void tx_tick() {
    Rc8 cur{};
    while (lock_.test_and_set(std::memory_order_acquire)) {}
    cur = rc_;
    lock_.clear(std::memory_order_release);

    // Build: "RC A E T R AUX1 AUX2 AUX3 AUX4\n"
    // FIXED: Using %u to match ESP expectations
    char line[128];
    int n = snprintf(line, sizeof(line),
                     "RC %u %u %u %u %u %u %u %u\n",
                     cur.v[0], cur.v[1], cur.v[2], cur.v[3],
                     cur.v[4], cur.v[5], cur.v[6], cur.v[7]);
    if (n <= 0) return;

    tx_count_++;
    const bool ok = serial_.write_all(reinterpret_cast<const uint8_t*>(line),
                                      static_cast<size_t>(n));
    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial write failed");
    } else if (debug_ && (tx_count_ % 50 == 0)) {
      RCLCPP_INFO(get_logger(), "Sent %lu frames: %s", tx_count_, line);
    }
  }

  // ADDED: Read and log ESP responses
  void read_esp_response() {
    uint8_t buffer[256];
    int n = serial_.read_available(buffer, sizeof(buffer) - 1);
    if (n > 0) {
      buffer[n] = '\0';
      RCLCPP_INFO(get_logger(), "ESP: %s", buffer);
    }
  }

  // Params
  std::string port_;
  int baud_;
  double rate_hz_;
  bool debug_;

  // Serial
  SerialPort serial_;

  // Stats
  uint64_t tx_count_ = 0;

  // ROS
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<URosBridge>());
  } catch (const std::exception& e) {
    fprintf(stderr, "URosBridge exception: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}