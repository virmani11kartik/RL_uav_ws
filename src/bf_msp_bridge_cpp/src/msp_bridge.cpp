#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <chrono>
#include <string>
#include <vector>
#include <array>
#include <mutex>

#include "bf_msp_bridge_cpp/msp.hpp"

using namespace std::chrono_literals;

class BfMspBridge : public rclcpp::Node {
public:
  BfMspBridge() : Node("bf_msp_bridge") {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = this->declare_parameter<int>("baudrate", 115200);
    rate_hz_ = this->declare_parameter<int>("request_rate_hz", 100);

    pub_att_ = this->create_publisher<geometry_msgs::msg::Vector3>("/msp/attitude", 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/msp/imu", 10);
    pub_mot_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("/msp/motors", 10);

    sub_rc_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/msp/rc_raw", 10, std::bind(&BfMspBridge::on_rc, this, std::placeholders::_1));

    openSerial();
    reader_ = std::thread(&BfMspBridge::readerLoop, this);
    timer_  = this->create_wall_timer(std::chrono::milliseconds(1000 / std::max(1, rate_hz_)),
                                      std::bind(&BfMspBridge::tick, this));
  }

  ~BfMspBridge() override {
    running_ = false;
    if (reader_.joinable()) reader_.join();
    if (fd_ >= 0) ::close(fd_);
  }

private:
  void openSerial() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open %s", port_.c_str());
      throw std::runtime_error("serial open failed");
    }
    struct termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);
    speed_t sp = B115200;
    if (baud_ == 1000000) sp = B1000000;
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tcsetattr(fd_, TCSANOW, &tio);
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK); // blocking read
    RCLCPP_INFO(get_logger(), "Opened %s @ %d", port_.c_str(), baud_);
  }

  void send(const std::vector<uint8_t>& bytes) {
    std::lock_guard<std::mutex> lk(tx_m_);
    if (fd_ >= 0) ::write(fd_, bytes.data(), bytes.size());
  }

  void tick() {
    // poll typical telemetry (you can rotate/add more as needed)
    send(msp::makeRequest(MSP_ATTITUDE));
    send(msp::makeRequest(MSP_IMU));
    send(msp::makeRequest(MSP_MOTOR));
    // You can also poll MSP_BATTERY_STATE if you want
  }

  void on_rc(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    // Expect up to 16 channels, 1000-2000
    std::vector<uint8_t> pl;
    pl.reserve(32);
    size_t n = std::min<size_t>(16, msg->data.size());
    for (size_t i = 0; i < n; ++i) {
      uint16_t us = static_cast<uint16_t>(msg->data[i]);
      msp::push_u16(pl, us);
    }
    // If fewer than Betaflight expects, pad to at least 8
    while (n < 8) { msp::push_u16(pl, 1500); ++n; }

    send(msp::makeRequest(MSP_SET_RAW_RC, pl));
  }

  void readerLoop() {
    msp::Decoder dec;
    std::vector<uint8_t> buf(256);
    running_ = true;
    while (running_) {
      ssize_t n = ::read(fd_, buf.data(), buf.size());
      if (n <= 0) { std::this_thread::sleep_for(2ms); continue; }
      for (ssize_t i = 0; i < n; ++i) {
        dec.feed(buf[i]);
        if (dec.hasFrame) {
          handleFrame(dec.cmd, dec.payload);
          dec.hasFrame = false;
        }
      }
    }
  }

  static int16_t rd16le(const uint8_t* p) { return static_cast<int16_t>(p[0] | (p[1]<<8)); }
  static uint16_t ru16le(const uint8_t* p){ return static_cast<uint16_t>(p[0] | (p[1]<<8)); }

  void handleFrame(uint8_t cmd, const std::vector<uint8_t>& pl) {
    auto now = this->get_clock()->now();

    if (cmd == MSP_ATTITUDE && pl.size() >= 6) {
      // int16: roll[deg*10], pitch[deg*10], yaw[deg*10]
      int16_t r = rd16le(&pl[0]);
      int16_t p = rd16le(&pl[2]);
      int16_t y = rd16le(&pl[4]);
      geometry_msgs::msg::Vector3 v;
      v.x = r / 10.0; v.y = p / 10.0; v.z = y / 10.0;
      pub_att_->publish(v);

      // Minimal IMU publish with only orientation (roll/pitch/yaw as Euler → no covariances)
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = now;
      imu.header.frame_id = "base_link";
      // We’ll leave orientation as 0; consumers can use /msp/attitude for angles.
      pub_imu_->publish(imu);
    }
    else if (cmd == MSP_MOTOR && pl.size() >= 8*2) {
      std_msgs::msg::UInt16MultiArray m;
      m.data.resize(8);
      for (int i=0;i<8;i++) m.data[i] = ru16le(&pl[i*2]);
      pub_mot_->publish(m);
    }
    // You can add MSP_IMU raw accel/gyro parsing if desired:
    // MSP_IMU: int16 ax, ay, az, gx, gy, gz
  }

  // members
  std::string port_;
  int baud_{115200}, rate_hz_{100};
  int fd_{-1};
  std::mutex tx_m_;
  std::thread reader_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> running_{false};

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_att_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           pub_imu_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_mot_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_rc_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BfMspBridge>());
  rclcpp::shutdown();
  return 0;
}
