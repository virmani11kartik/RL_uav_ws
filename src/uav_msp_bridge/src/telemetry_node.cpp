#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "uav_msp_bridge/MSPClient.hpp"

#include <algorithm>
#include <sstream>
#include <iomanip>
#include <memory>
#include <thread>

// MSP Command IDs
enum : uint16_t {
  MSP_API_VERSION = 1,
  MSP_FC_VARIANT  = 2,
  MSP_FC_VERSION  = 3,
  MSP_BOARD_INFO  = 4,
  MSP_BUILD_INFO  = 5,
  MSP_NAME        = 10,

  MSP_STATUS      = 101,
  MSP_RAW_IMU     = 102,
  MSP_SERVO       = 103,
  MSP_MOTOR       = 104,
  MSP_RC          = 105,
  MSP_RAW_GPS     = 106,
  MSP_COMP_GPS    = 107,
  MSP_ATTITUDE    = 108,
  MSP_ALTITUDE    = 109,
  MSP_ANALOG      = 110,
  MSP_RC_TUNING   = 111,
  MSP_PID         = 112,

  MSP_SENSOR_ALIGNMENT   = 126,
  MSP_VOLTAGE_METERS     = 128,
  MSP_CURRENT_METERS     = 129,
  MSP_BATTERY_STATE      = 130,
  MSP_TEMPERATURE        = 132,
  MSP_ESC_SENSOR_DATA    = 134,
  MSP_CURRENT_METERS_SUM = 135,

  MSP_ARMING_DISABLE_FLAGS = 150,
  MSP_SENSOR_STATUS        = 151
};

static inline uint16_t le16(const std::vector<uint8_t>& v, size_t i) {
  return (i + 1 < v.size()) ? (uint16_t)(v[i] | (v[i+1] << 8)) : 0;
}
static inline int16_t le16s(const std::vector<uint8_t>& v, size_t i) {
  return (int16_t)le16(v, i);
}
static inline uint32_t le32(const std::vector<uint8_t>& v, size_t i) {
  return (i + 3 < v.size()) ? (uint32_t)(v[i] | (v[i+1] << 8) | (v[i+2] << 16) | (v[i+3] << 24)) : 0;
}
static inline int32_t le32s(const std::vector<uint8_t>& v, size_t i) {
  return (int32_t)le32(v, i);
}

using uav_msp_bridge::MSPClient;

class MSPReaderNode : public rclcpp::Node {
public:
  MSPReaderNode() : Node("msp_reader") {
    port_ = declare_parameter<std::string>("device_path", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 115200);
    debug_ = declare_parameter<bool>("debug", false);

    poll_hz_high_   = declare_parameter<double>("poll_hz_high",   20.0);
    poll_hz_medium_ = declare_parameter<double>("poll_hz_medium", 5.0);
    poll_hz_low_    = declare_parameter<double>("poll_hz_low",    1.0);

    enable_att_   = declare_parameter<bool>("enable_attitude", true);
    enable_imu_   = declare_parameter<bool>("enable_imu", true);
    enable_alt_   = declare_parameter<bool>("enable_altitude", true);
    enable_bat_   = declare_parameter<bool>("enable_battery", true);
    enable_mot_   = declare_parameter<bool>("enable_motors", true);
    enable_srv_   = declare_parameter<bool>("enable_servos", false);
    enable_rc_    = declare_parameter<bool>("enable_rc", true);
    enable_gps_   = declare_parameter<bool>("enable_gps", false);
    enable_status_= declare_parameter<bool>("enable_status", false);
    enable_temp_  = declare_parameter<bool>("enable_temperature", false);
    enable_esc_   = declare_parameter<bool>("enable_esc", false);
    enable_arming_= declare_parameter<bool>("enable_arming_flags", true);

    createPublishers();

    client_ = std::make_unique<MSPClient>(port_, baud_);
    if (!client_->open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", port_.c_str());
      throw std::runtime_error("Failed to open serial");
    }
    RCLCPP_INFO(get_logger(), "Connected to FC on %s @ %d baud", port_.c_str(), baud_);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    fetchFCInfo();

    timer_high_ = create_wall_timer(
      std::chrono::milliseconds((int)(1000.0 / poll_hz_high_)),
      std::bind(&MSPReaderNode::tickHigh, this));
    timer_medium_ = create_wall_timer(
      std::chrono::milliseconds((int)(1000.0 / poll_hz_medium_)),
      std::bind(&MSPReaderNode::tickMedium, this));
    timer_low_ = create_wall_timer(
      std::chrono::milliseconds((int)(1000.0 / poll_hz_low_)),
      std::bind(&MSPReaderNode::tickLow, this));
  }

  ~MSPReaderNode() {
    if (client_) client_->close();
  }

private:
  void createPublishers() {
    fc_info_pub_ = create_publisher<std_msgs::msg::String>("/msp/fc_info", 10);

    att_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/msp/attitude", rclcpp::SensorDataQoS());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/msp/imu_raw", rclcpp::SensorDataQoS());
    rc_pub_  = create_publisher<std_msgs::msg::UInt16MultiArray>("/msp/rc_raw", rclcpp::SensorDataQoS());
    motor_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>("/msp/motors", rclcpp::SensorDataQoS());
    alt_pub_ = create_publisher<std_msgs::msg::Float32>("/msp/altitude", 10);
    vario_pub_ = create_publisher<std_msgs::msg::Float32>("/msp/vario", 10);

    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/msp/status", 10);
    bat_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/msp/battery", 10);
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("/msp/temperature", 10);

    gps_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/msp/gps/fix", 10);
    gps_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/msp/gps/velocity", 10);
    esc_pub_ = create_publisher<std_msgs::msg::String>("/msp/esc_telemetry", 10);
    arming_pub_ = create_publisher<std_msgs::msg::UInt32>("/msp/arming_flags", 10);
    servo_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>("/msp/servos", 10);
  }

  void fetchFCInfo() {
    std::vector<uint8_t> resp;
    std::stringstream info;
    bool got = false;

    auto req = [&](uint16_t cmd, double to)->bool{
      resp.clear();
      return client_->request(cmd, {}, resp, to);
    };

    if (req(MSP_API_VERSION, 1.0) && resp.size() >= 3) {
        uint8_t proto = resp[0], api_major = resp[1], api_minor = resp[2];
        info << "API Version: " << (int)api_major << "." << (int)api_minor<< " (proto " << (int)proto << ")\n";
      got = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    if (req(MSP_FC_VARIANT, 1.0) && !resp.empty()) {
      std::string variant(resp.begin(), resp.end());
      // stop at first NUL; do not erase all NULs
      variant = variant.c_str();
      if (!variant.empty()) { info << "FC Variant: " << variant << "\n"; got = true; }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    if (req(MSP_FC_VERSION, 1.0) && resp.size() >= 3) {
      info << "FC Version: " << (int)resp[0] << "." << (int)resp[1] << "." << (int)resp[2] << "\n";
      got = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    // BOARD_INFO can be structured; log hex safely instead of treating as ASCII
    if (client_->request(MSP_BOARD_INFO, {}, resp, 1.0) && resp.size() >= 9) {
        std::string bid(reinterpret_cast<const char*>(&resp[0]), 4);
        uint8_t hwrev = resp[4];
        auto printable = [](uint8_t b){ return b >= 32 && b < 127; };
        std::string ascii;
        for (auto b : resp) ascii.push_back(printable(b) ? char(b) : ' ');
        info << "Board: " << bid << "  hwrev: " << (int)hwrev << "\n";
        info << "Board (heuristic ascii): " << ascii << "\n";
        got = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    // BUILD_INFO: may be absent; tolerate no response
    if (client_->request(MSP_BUILD_INFO, {}, resp, 0.6)) {
        if (resp.size() >= 19) {
            std::string date(resp.begin(), resp.begin()+11);
            std::string time(resp.begin()+11, resp.begin()+19);
            std::string git = (resp.size() >= 26) ? std::string(resp.begin()+19, resp.begin()+26) : "";
            info << "Build: " << date << " " << time;
            if (!git.empty()) info << " (" << git << ")";
            info << "\n";
        } else {
            info << "BUILD_INFO (" << resp.size() << " bytes)\n";
        }
        got = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    if (client_->request(MSP_NAME, {}, resp, 1.0) && !resp.empty()) {
      std::string name(resp.begin(), resp.end());
      name = name.c_str();
      if (!name.empty()) { info << "Name: " << name << "\n"; got = true; }
    }

    auto msg = std_msgs::msg::String();
    msg.data = got ? info.str() : std::string("ERROR: No FC info\n");
    if (got) RCLCPP_INFO(get_logger(), "Flight Controller Info:\n%s", msg.data.c_str());
    else     RCLCPP_WARN(get_logger(), "Failed to retrieve FC info");
    fc_info_pub_->publish(msg);
  }

  void tickHigh() {
    std::vector<uint8_t> resp;
    const auto stamp = now();

    if (enable_att_ && client_->request(MSP_ATTITUDE, {}, resp, 0.08) && resp.size() >= 6) {
      int16_t r10 = le16s(resp, 0), p10 = le16s(resp, 2), hdg = le16s(resp, 4);
      geometry_msgs::msg::Vector3Stamped m;
      m.header.stamp = stamp; m.header.frame_id = "base_link";
      m.vector.x = r10 / 10.0; m.vector.y = p10 / 10.0; m.vector.z = (double)hdg;
      att_pub_->publish(m);
    }

    if (enable_imu_ && client_->request(MSP_RAW_IMU, {}, resp, 0.08) && resp.size() >= 18) {
      sensor_msgs::msg::Imu m;
      m.header.stamp = stamp; m.header.frame_id = "base_link";
      m.linear_acceleration.x = le16s(resp, 0);
      m.linear_acceleration.y = le16s(resp, 2);
      m.linear_acceleration.z = le16s(resp, 4);
      m.angular_velocity.x = le16s(resp, 6);
      m.angular_velocity.y = le16s(resp, 8);
      m.angular_velocity.z = le16s(resp, 10);
      m.orientation_covariance[0] = -1;
      imu_pub_->publish(m);
    }

    if (enable_rc_ && client_->request(MSP_RC, {}, resp, 0.08) && resp.size() >= 2) {
      std_msgs::msg::UInt16MultiArray m;
      size_t n = resp.size() / 2; m.data.resize(n);
      for (size_t i = 0; i < n; ++i) m.data[i] = le16(resp, 2*i);
      rc_pub_->publish(m);
    }

    if (enable_mot_ && client_->request(MSP_MOTOR, {}, resp, 0.08) && resp.size() >= 2) {
      std_msgs::msg::UInt16MultiArray m;
      size_t n = resp.size() / 2; m.data.resize(n);
      for (size_t i = 0; i < n; ++i) m.data[i] = le16(resp, 2*i);
      motor_pub_->publish(m);
    }

    if (enable_alt_ && client_->request(MSP_ALTITUDE, {}, resp, 0.08) && resp.size() >= 6) {
      int32_t alt_cm = le32s(resp, 0);
      int16_t vario  = le16s(resp, 4);
      std_msgs::msg::Float32 a; a.data = alt_cm / 100.0f; alt_pub_->publish(a);
      std_msgs::msg::Float32 v; v.data = vario  / 100.0f; vario_pub_->publish(v);
    }
  }

  void tickMedium() {
    std::vector<uint8_t> resp;
    const auto stamp = now();

    if (enable_status_ && client_->request(MSP_STATUS, {}, resp, 0.15)) {
      diagnostic_msgs::msg::DiagnosticStatus m;
      m.name = "MSP Status"; m.hardware_id = port_;
      m.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      if (resp.size() >= 2)  m.values.push_back(kv("cycle_time_us", std::to_string(le16(resp,0))));
      if (resp.size() >= 4)  m.values.push_back(kv("i2c_errors",    std::to_string(le16(resp,2))));
      if (resp.size() >= 6)  m.values.push_back(kv("sensors",       std::to_string(le16(resp,4))));
      if (resp.size() >= 10) m.values.push_back(kv("flight_modes",  std::to_string(le32(resp,6))));
      if (resp.size() >= 14) m.values.push_back(kv("cpu_load",      std::to_string(le16(resp,12))));
      status_pub_->publish(m);
    }

    if (enable_bat_ && client_->request(MSP_BATTERY_STATE, {}, resp, 0.15)) {
      sensor_msgs::msg::BatteryState m; m.header.stamp = stamp;
      size_t i = 0;
      if (resp.size() >= 1) { uint8_t cells = resp[i++]; m.cell_voltage.resize(cells); }
      if (resp.size() >= i+2) { uint16_t cap_mAh = le16(resp, i); m.capacity = cap_mAh / 1000.0f; i+=2; }
      if (resp.size() >= i+1) { m.voltage = resp[i++] / 10.0f; }
      if (resp.size() >= i+2) { uint16_t mah = le16(resp,i); m.charge = mah / 1000.0f; i+=2; }
      if (resp.size() >= i+2) { uint16_t cur = le16(resp,i); m.current = cur / 100.0f; i+=2; }
      if (resp.size() >= i+1) { m.power_supply_status = resp[i++]; }
      if (resp.size() >= i+2 && !m.cell_voltage.empty()) {
        m.cell_voltage[0] = le16(resp,i)/100.0f; i+=2;
      }
      m.present = true;
      bat_pub_->publish(m);
    }

    if (enable_temp_ && client_->request(MSP_TEMPERATURE, {}, resp, 0.15) && resp.size() >= 2) {
      sensor_msgs::msg::Temperature m;
      m.header.stamp = stamp;
      m.temperature = le16s(resp,0) / 10.0;
      m.variance = 0.0;
      temp_pub_->publish(m);
    }
  }

  void tickLow() {
    std::vector<uint8_t> resp;
    const auto stamp = now();

    if (enable_gps_ && client_->request(MSP_RAW_GPS, {}, resp, 0.25) && resp.size() >= 16) {
      size_t i = 0;
      uint8_t fix = (resp.size() >= 1) ? resp[i++] : 0;
      if (resp.size() >= i+1) i++; // sats
      int32_t lat = (resp.size() >= i+4) ? (int32_t)le32(resp,i) : 0; i+=4;
      int32_t lon = (resp.size() >= i+4) ? (int32_t)le32(resp,i) : 0; i+=4;
      uint16_t alt = (resp.size() >= i+2) ? le16(resp,i) : 0; i+=2;
      uint16_t spd = (resp.size() >= i+2) ? le16(resp,i) : 0; i+=2;
      uint16_t crs = (resp.size() >= i+2) ? le16(resp,i) : 0;

      sensor_msgs::msg::NavSatFix fx;
      fx.header.stamp = stamp; fx.header.frame_id = "gps";
      fx.latitude = lat / 1e7; fx.longitude = lon / 1e7; fx.altitude = alt;
      fx.status.status = (fix==0) ? sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX
                                  : sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      gps_fix_pub_->publish(fx);

      geometry_msgs::msg::TwistStamped vel;
      vel.header.stamp = stamp; vel.header.frame_id = "gps";
      double speed_ms = spd / 100.0;
      double theta = (crs * M_PI) / 180.0;
      vel.twist.linear.x = speed_ms * cos(theta);
      vel.twist.linear.y = speed_ms * sin(theta);
      gps_vel_pub_->publish(vel);
    }

    if (enable_esc_ && client_->request(MSP_ESC_SENSOR_DATA, {}, resp, 0.25)) {
      std::stringstream escs;
      size_t i = 0; int idx = 0;
      while (i + 3 <= resp.size()) { // minimal: temp(1), rpm(2)
        uint8_t temp = resp[i];
        uint16_t rpm = le16(resp, i+1);
        escs << "ESC" << idx++ << ": temp=" << (int)temp << "C rpm=" << rpm << " | ";
        i += 10; // BF packs more; keep 10-step stride as before
      }
      std_msgs::msg::String m; m.data = escs.str();
      esc_pub_->publish(m);
    }

    if (enable_arming_ && client_->request(MSP_ARMING_DISABLE_FLAGS, {}, resp, 0.25) && resp.size() >= 4) {
      uint32_t flags = le32(resp, 0);
      std_msgs::msg::UInt32 m; m.data = flags; arming_pub_->publish(m);
    }

    if (enable_srv_ && client_->request(MSP_SERVO, {}, resp, 0.25) && resp.size() >= 2) {
      std_msgs::msg::UInt16MultiArray m;
      size_t n = resp.size()/2; m.data.resize(n);
      for (size_t i = 0; i < n; ++i) m.data[i] = le16(resp, 2*i);
      servo_pub_->publish(m);
    }
  }

  diagnostic_msgs::msg::KeyValue kv(const std::string& k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = v; return p;
  }

private:
  // client & timers
  std::unique_ptr<MSPClient> client_;
  rclcpp::TimerBase::SharedPtr timer_high_, timer_medium_, timer_low_;

  // params
  std::string port_;
  int baud_;
  bool debug_;
  double poll_hz_high_, poll_hz_medium_, poll_hz_low_;

  // feature flags
  bool enable_att_, enable_imu_, enable_alt_, enable_bat_;
  bool enable_mot_, enable_srv_, enable_rc_, enable_gps_;
  bool enable_status_, enable_temp_, enable_esc_, enable_arming_;

  // pubs
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr att_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr rc_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr alt_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vario_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr bat_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr gps_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr esc_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr arming_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr servo_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fc_info_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MSPReaderNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("msp_reader"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
