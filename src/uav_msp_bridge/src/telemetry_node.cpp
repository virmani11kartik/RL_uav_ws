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
#include <cstring>
#include <sstream>
#include <iomanip>

using uav_msp_bridge::MSPClient;

// MSP Command IDs
enum : uint8_t {
    MSP_API_VERSION = 1,
    MSP_FC_VARIANT = 2,
    MSP_FC_VERSION = 3,
    MSP_BOARD_INFO = 4,
    MSP_BUILD_INFO = 5,
    MSP_NAME = 10,
    
    MSP_STATUS = 101,
    MSP_RAW_IMU = 102,
    MSP_SERVO = 103,
    MSP_MOTOR = 104,
    MSP_RC = 105,
    MSP_RAW_GPS = 106,
    MSP_COMP_GPS = 107,
    MSP_ATTITUDE = 108,
    MSP_ALTITUDE = 109,
    MSP_ANALOG = 110,
    MSP_RC_TUNING = 111,
    MSP_PID = 112,
    
    MSP_SENSOR_ALIGNMENT = 126,
    MSP_VOLTAGE_METERS = 128,
    MSP_CURRENT_METERS = 129,
    MSP_BATTERY_STATE = 130,
    MSP_TEMPERATURE = 132,
    MSP_ESC_SENSOR_DATA = 134,
    MSP_CURRENT_METERS_SUM = 135,
    
    MSP_ARMING_DISABLE_FLAGS = 150,
    MSP_SENSOR_STATUS = 151
};

class MSPReaderNode : public rclcpp::Node {
public:
    MSPReaderNode() : Node("msp_reader") {
        // Declare parameters
        port_ = declare_parameter<std::string>("device_path", "/dev/ttyACM0");
        baud_ = declare_parameter<int>("baud", 115200);
        debug_ = declare_parameter<bool>("debug", false);
        
        // Poll rates for different priority groups
        poll_hz_high_ = declare_parameter<double>("poll_hz_high", 20.0);
        poll_hz_medium_ = declare_parameter<double>("poll_hz_medium", 5.0);
        poll_hz_low_ = declare_parameter<double>("poll_hz_low", 1.0);
        
        // Enable flags for each data category
        enable_att_ = declare_parameter<bool>("enable_attitude", true);
        enable_imu_ = declare_parameter<bool>("enable_imu", true);
        enable_alt_ = declare_parameter<bool>("enable_altitude", true);
        enable_bat_ = declare_parameter<bool>("enable_battery", true);
        enable_mot_ = declare_parameter<bool>("enable_motors", true);
        enable_srv_ = declare_parameter<bool>("enable_servos", false);
        enable_rc_ = declare_parameter<bool>("enable_rc", true);
        enable_gps_ = declare_parameter<bool>("enable_gps", false);
        enable_status_ = declare_parameter<bool>("enable_status", false);
        enable_temp_ = declare_parameter<bool>("enable_temperature", false);
        enable_esc_ = declare_parameter<bool>("enable_esc", false);
        enable_arming_ = declare_parameter<bool>("enable_arming_flags", true);
        
        // Create publishers
        createPublishers();
        
        // Open serial connection
        client_ = std::make_unique<MSPClient>(port_, baud_);
        if (!client_->open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", port_.c_str());
            throw std::runtime_error("Failed to open serial");
        }
        
        RCLCPP_INFO(get_logger(), "Connected to FC on %s @ %d baud", port_.c_str(), baud_);
        
        // Give FC time to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Fetch FC info once at startup
        fetchFCInfo();
        
        // Create timers for different priority groups
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
        if (client_) {
            client_->close();
        }
    }

private:
    void createPublishers() {
        // FC Info
        fc_info_pub_ = create_publisher<std_msgs::msg::String>("/msp/fc_info", 10);
        
        // High priority
        att_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/msp/attitude", rclcpp::SensorDataQoS());
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "/msp/imu_raw", rclcpp::SensorDataQoS());
        rc_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>(
            "/msp/rc_raw", rclcpp::SensorDataQoS());
        motor_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>(
            "/msp/motors", rclcpp::SensorDataQoS());
        alt_pub_ = create_publisher<std_msgs::msg::Float32>("/msp/altitude", 10);
        vario_pub_ = create_publisher<std_msgs::msg::Float32>("/msp/vario", 10);
        
        // Medium priority
        status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
            "/msp/status", 10);
        bat_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/msp/battery", 10);
        temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("/msp/temperature", 10);
        
        // Low priority
        gps_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/msp/gps/fix", 10);
        gps_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/msp/gps/velocity", 10);
        esc_pub_ = create_publisher<std_msgs::msg::String>("/msp/esc_telemetry", 10);
        arming_pub_ = create_publisher<std_msgs::msg::UInt32>("/msp/arming_flags", 10);
        servo_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>("/msp/servos", 10);
        // sensor_status_pub_ = create_publisher<std_msgs::msg::UInt16>("/msp/sensor_status", 10);
    }
    
    void debugLog(const std::string& msg) {
        if (debug_) {
            RCLCPP_INFO(get_logger(), "[DEBUG] %s", msg.c_str());
        }
    }
    
    void debugLogHex(const std::string& prefix, const std::vector<uint8_t>& data) {
        if (!debug_) return;
        
        std::stringstream ss;
        ss << prefix << " [" << data.size() << " bytes]: ";
        for (auto b : data) {
            ss << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        }
        RCLCPP_INFO(get_logger(), "[DEBUG] %s", ss.str().c_str());
    }
    
    void fetchFCInfo() {
        std::vector<uint8_t> resp;
        std::stringstream info;
        bool got_data = false;
        
        RCLCPP_INFO(get_logger(), "Fetching FC information...");
        
        // API Version
        debugLog("Requesting MSP_API_VERSION (1)");
        if (client_->request(MSP_API_VERSION, {}, resp, 1.0)) {
            debugLogHex("MSP_API_VERSION response", resp);
            if (resp.size() >= 3) {
                info << "API Version: " << (int)resp[0] << "." << (int)resp[1] << "." << (int)resp[2] << "\n";
                got_data = true;
            } else {
                RCLCPP_WARN(get_logger(), "MSP_API_VERSION: insufficient data (got %zu bytes)", resp.size());
            }
        } else {
            RCLCPP_WARN(get_logger(), "MSP_API_VERSION: no response");
        }
        
        // Small delay between requests
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // FC Variant
        debugLog("Requesting MSP_FC_VARIANT (2)");
        if (client_->request(MSP_FC_VARIANT, {}, resp, 1.0)) {
            debugLogHex("MSP_FC_VARIANT response", resp);
            if (!resp.empty()) {
                std::string variant(resp.begin(), resp.end());
                // Remove null terminators and whitespace
                variant.erase(std::remove(variant.begin(), variant.end(), '\0'), variant.end());
                if (!variant.empty()) {
                    info << "FC Variant: " << variant << "\n";
                    got_data = true;
                } else {
                    RCLCPP_WARN(get_logger(), "MSP_FC_VARIANT: empty string");
                }
            }
        } else {
            RCLCPP_WARN(get_logger(), "MSP_FC_VARIANT: no response");
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // FC Version
        debugLog("Requesting MSP_FC_VERSION (3)");
        if (client_->request(MSP_FC_VERSION, {}, resp, 1.0)) {
            debugLogHex("MSP_FC_VERSION response", resp);
            if (resp.size() >= 3) {
                info << "FC Version: " << (int)resp[0] << "." << (int)resp[1] << "." << (int)resp[2] << "\n";
                got_data = true;
            } else {
                RCLCPP_WARN(get_logger(), "MSP_FC_VERSION: insufficient data (got %zu bytes)", resp.size());
            }
        } else {
            RCLCPP_WARN(get_logger(), "MSP_FC_VERSION: no response");
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Board Info
        debugLog("Requesting MSP_BOARD_INFO (4)");
        if (client_->request(MSP_BOARD_INFO, {}, resp, 1.0)) {
            debugLogHex("MSP_BOARD_INFO response", resp);
            if (!resp.empty()) {
                std::string board(resp.begin(), resp.end());
                board.erase(std::remove(board.begin(), board.end(), '\0'), board.end());
                if (!board.empty()) {
                    info << "Board: " << board << "\n";
                    got_data = true;
                }
            }
        } else {
            RCLCPP_WARN(get_logger(), "MSP_BOARD_INFO: no response");
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Build Info
        debugLog("Requesting MSP_BUILD_INFO (5)");
        if (client_->request(MSP_BUILD_INFO, {}, resp, 0.5)) {
            if (resp.size() >= 19) {
                std::string date(resp.begin(), resp.begin() + 11);
                date.erase(std::remove(date.begin(), date.end(), '\0'), date.end());
                
                std::string time(resp.begin() + 11, resp.begin() + 19);
                time.erase(std::remove(time.begin(), time.end(), '\0'), time.end());
                
                std::string git_hash;
                if (resp.size() >= 26) {
                    git_hash = std::string(resp.begin() + 19, resp.begin() + 26);
                    git_hash.erase(std::remove(git_hash.begin(), git_hash.end(), '\0'), git_hash.end());
                }
                
                info << "Build: " << date << " " << time;
                if (!git_hash.empty()) {
                    info << " (" << git_hash << ")";
                }
                info << "\n";
                got_data = true;
            }
        } else {
            RCLCPP_WARN(get_logger(), "MSP_BUILD_INFO: no response");
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Name
        debugLog("Requesting MSP_NAME (10)");
        if (client_->request(MSP_NAME, {}, resp, 1.0)) {
            debugLogHex("MSP_NAME response", resp);
            if (!resp.empty()) {
                std::string name(resp.begin(), resp.end());
                name.erase(std::remove(name.begin(), name.end(), '\0'), name.end());
                if (!name.empty()) {
                    info << "Name: " << name << "\n";
                    got_data = true;
                }
            }
        } else {
            RCLCPP_WARN(get_logger(), "MSP_NAME: no response");
        }
        
        std::string info_str = info.str();
        if (got_data) {
            RCLCPP_INFO(get_logger(), "Flight Controller Info:\n%s", info_str.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to retrieve any FC info! Check MSPClient implementation.");
            info_str = "ERROR: No data received from FC\n";
        }
        
        auto msg = std_msgs::msg::String();
        msg.data = info_str;
        fc_info_pub_->publish(msg);
    }
    
    // High priority polling (20Hz): attitude, IMU, RC, motors, altitude
    void tickHigh() {
        std::vector<uint8_t> resp;
        const auto stamp = now();
        
        // Attitude
        if (enable_att_ && client_->request(MSP_ATTITUDE, {}, resp, 0.05)) {
            if (resp.size() >= 6) {
                int16_t roll_raw = *(int16_t*)&resp[0];
                int16_t pitch_raw = *(int16_t*)&resp[2];
                int16_t yaw_raw = *(int16_t*)&resp[4];
                
                auto msg = geometry_msgs::msg::Vector3Stamped();
                msg.header.stamp = stamp;
                msg.header.frame_id = "base_link";
                msg.vector.x = roll_raw / 10.0;
                msg.vector.y = pitch_raw / 10.0;
                msg.vector.z = (double)yaw_raw;
                att_pub_->publish(msg);
            }
        }
        
        // Raw IMU
        if (enable_imu_ && client_->request(MSP_RAW_IMU, {}, resp, 0.05)) {
            if (resp.size() >= 18) {
                int16_t* vals = (int16_t*)resp.data();
                
                auto msg = sensor_msgs::msg::Imu();
                msg.header.stamp = stamp;
                msg.header.frame_id = "base_link";
                
                // Accelerometer (raw values)
                msg.linear_acceleration.x = vals[0];
                msg.linear_acceleration.y = vals[1];
                msg.linear_acceleration.z = vals[2];
                
                // Gyroscope (raw values)
                msg.angular_velocity.x = vals[3];
                msg.angular_velocity.y = vals[4];
                msg.angular_velocity.z = vals[5];
                
                // No orientation data from raw IMU
                msg.orientation_covariance[0] = -1;
                
                imu_pub_->publish(msg);
            }
        }
        
        // RC Channels
        if (enable_rc_ && client_->request(MSP_RC, {}, resp, 0.05)) {
            auto msg = std_msgs::msg::UInt16MultiArray();
            msg.data.resize(resp.size() / 2);
            for (size_t i = 0; i + 1 < resp.size(); i += 2) {
                msg.data[i / 2] = (uint16_t)(resp[i] | (resp[i + 1] << 8));
            }
            rc_pub_->publish(msg);
        }
        
        // Motors
        if (enable_mot_ && client_->request(MSP_MOTOR, {}, resp, 0.05)) {
            auto msg = std_msgs::msg::UInt16MultiArray();
            msg.data.resize(resp.size() / 2);
            for (size_t i = 0; i + 1 < resp.size(); i += 2) {
                msg.data[i / 2] = (uint16_t)(resp[i] | (resp[i + 1] << 8));
            }
            motor_pub_->publish(msg);
        }
        
        // Altitude
        if (enable_alt_ && client_->request(MSP_ALTITUDE, {}, resp, 0.05)) {
            if (resp.size() >= 6) {
                int32_t alt_cm = *(int32_t*)&resp[0];
                int16_t vario_cms = *(int16_t*)&resp[4];
                
                auto alt_msg = std_msgs::msg::Float32();
                alt_msg.data = alt_cm / 100.0f;
                alt_pub_->publish(alt_msg);
                
                auto vario_msg = std_msgs::msg::Float32();
                vario_msg.data = vario_cms / 100.0f;
                vario_pub_->publish(vario_msg);
            }
        }
    }
    
    // Medium priority polling (5Hz): status, battery, temperature
    void tickMedium() {
        std::vector<uint8_t> resp;
        const auto stamp = now();
        
        // Status
        if (enable_status_ && client_->request(MSP_STATUS, {}, resp, 0.1)) {
            auto msg = diagnostic_msgs::msg::DiagnosticStatus();
            msg.name = "MSP Status";
            msg.hardware_id = port_;
            msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            
            if (resp.size() >= 2) {
                uint16_t cycle_us = *(uint16_t*)&resp[0];
                msg.values.push_back(makeKeyValue("cycle_time_us", std::to_string(cycle_us)));
            }
            if (resp.size() >= 4) {
                uint16_t i2c_err = *(uint16_t*)&resp[2];
                msg.values.push_back(makeKeyValue("i2c_errors", std::to_string(i2c_err)));
            }
            if (resp.size() >= 6) {
                uint16_t sensors = *(uint16_t*)&resp[4];
                msg.values.push_back(makeKeyValue("sensors", std::to_string(sensors)));
            }
            if (resp.size() >= 10) {
                uint32_t modes = *(uint32_t*)&resp[6];
                msg.values.push_back(makeKeyValue("flight_modes", std::to_string(modes)));
            }
            if (resp.size() >= 14) {
                uint16_t cpu_load = *(uint16_t*)&resp[12];
                msg.values.push_back(makeKeyValue("cpu_load", std::to_string(cpu_load)));
            }
            
            status_pub_->publish(msg);
        }
        
        // Battery State
        if (enable_bat_ && client_->request(MSP_BATTERY_STATE, {}, resp, 0.1)) {
            auto msg = sensor_msgs::msg::BatteryState();
            msg.header.stamp = stamp;
            
            size_t i = 0;
            if (resp.size() >= 1) {
                uint8_t cells = resp[i++];
                msg.cell_voltage.resize(cells);
            }
            if (resp.size() >= i + 2) {
                uint16_t capacity = *(uint16_t*)&resp[i];
                msg.capacity = capacity / 1000.0f;
                i += 2;
            }
            if (resp.size() >= i + 1) {
                msg.voltage = resp[i++] / 10.0f;
            }
            if (resp.size() >= i + 2) {
                uint16_t mah_drawn = *(uint16_t*)&resp[i];
                msg.charge = mah_drawn / 1000.0f;
                i += 2;
            }
            if (resp.size() >= i + 2) {
                uint16_t current = *(uint16_t*)&resp[i];
                msg.current = current / 100.0f;
                i += 2;
            }
            if (resp.size() >= i + 1) {
                uint8_t bat_state = resp[i++];
                msg.power_supply_status = bat_state;
            }
            if (resp.size() >= i + 2) {
                uint16_t cell_voltage = *(uint16_t*)&resp[i];
                if (msg.cell_voltage.size() > 0) {
                    msg.cell_voltage[0] = cell_voltage / 100.0f;
                }
                i += 2;
            }
            
            msg.present = true;
            bat_pub_->publish(msg);
        }
        
        // Temperature
        if (enable_temp_ && client_->request(MSP_TEMPERATURE, {}, resp, 0.1)) {
            if (resp.size() >= 2) {
                int16_t temp_raw = *(int16_t*)&resp[0];
                
                auto msg = sensor_msgs::msg::Temperature();
                msg.header.stamp = stamp;
                msg.temperature = temp_raw / 10.0;
                msg.variance = 0.0;
                temp_pub_->publish(msg);
            }
        }
    }
    
    // Low priority polling (1Hz): GPS, ESC, arming flags, servos
    void tickLow() {
        std::vector<uint8_t> resp;
        const auto stamp = now();
        
        // GPS
        if (enable_gps_ && client_->request(MSP_RAW_GPS, {}, resp, 0.2)) {
            if (resp.size() >= 16) {
                size_t i = 0;
                uint8_t fix = resp[i++];
                // uint8_t sats = resp[i++];
                int32_t lat = *(int32_t*)&resp[i]; i += 4;
                int32_t lon = *(int32_t*)&resp[i]; i += 4;
                uint16_t altitude = *(uint16_t*)&resp[i]; i += 2;
                uint16_t speed = *(uint16_t*)&resp[i]; i += 2;
                uint16_t ground_course = *(uint16_t*)&resp[i]; i += 2;
                
                // GPS Fix
                auto fix_msg = sensor_msgs::msg::NavSatFix();
                fix_msg.header.stamp = stamp;
                fix_msg.header.frame_id = "gps";
                fix_msg.latitude = lat / 10000000.0;
                fix_msg.longitude = lon / 10000000.0;
                fix_msg.altitude = altitude;
                
                if (fix == 0) {
                    fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                } else if (fix == 1) {
                    fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                } else {
                    fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                }
                
                gps_fix_pub_->publish(fix_msg);
                
                // GPS Velocity
                auto vel_msg = geometry_msgs::msg::TwistStamped();
                vel_msg.header.stamp = stamp;
                vel_msg.header.frame_id = "gps";
                
                double speed_ms = speed / 100.0;
                double course_rad = ground_course * M_PI / 180.0;
                vel_msg.twist.linear.x = speed_ms * cos(course_rad);
                vel_msg.twist.linear.y = speed_ms * sin(course_rad);
                vel_msg.twist.linear.z = 0.0;
                
                gps_vel_pub_->publish(vel_msg);
            }
        }
        
        // ESC Telemetry
        if (enable_esc_ && client_->request(MSP_ESC_SENSOR_DATA, {}, resp, 0.2)) {
            std::stringstream esc_data;
            size_t i = 0;
            int esc_num = 0;
            
            while (i + 10 <= resp.size()) {
                uint8_t temp = resp[i];
                uint16_t rpm = *(uint16_t*)&resp[i + 1];
                
                esc_data << "ESC" << esc_num++ << ": temp=" << (int)temp 
                         << "C rpm=" << rpm << " | ";
                i += 10;
            }
            
            auto msg = std_msgs::msg::String();
            msg.data = esc_data.str();
            esc_pub_->publish(msg);
        }
        
        // Arming Disable Flags
        if (enable_arming_ && client_->request(MSP_ARMING_DISABLE_FLAGS, {}, resp, 0.2)) {
            if (resp.size() >= 4) {
                uint32_t flags = *(uint32_t*)&resp[0];
                
                auto msg = std_msgs::msg::UInt32();
                msg.data = flags;
                arming_pub_->publish(msg);
            }
        }
        
        // Servos
        if (enable_srv_ && client_->request(MSP_SERVO, {}, resp, 0.2)) {
            auto msg = std_msgs::msg::UInt16MultiArray();
            msg.data.resize(resp.size() / 2);
            for (size_t i = 0; i + 1 < resp.size(); i += 2) {
                msg.data[i / 2] = (uint16_t)(resp[i] | (resp[i + 1] << 8));
            }
            servo_pub_->publish(msg);
        }
        
        // Sensor Status
        // if (client_->request(MSP_SENSOR_STATUS, {}, resp, 0.2)) {
        //     if (resp.size() >= 2) {
        //         uint16_t sensor_status = *(uint16_t*)&resp[0];
                
        //         auto msg = std_msgs::msg::UInt16();
        //         msg.data = sensor_status;
        //         sensor_status_pub_->publish(msg);
        //     }
        // }
    }
    
    diagnostic_msgs::msg::KeyValue makeKeyValue(const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        return kv;
    }

    // Serial client
    std::unique_ptr<MSPClient> client_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr timer_high_;
    rclcpp::TimerBase::SharedPtr timer_medium_;
    rclcpp::TimerBase::SharedPtr timer_low_;
    
    // Parameters
    std::string port_;
    int baud_;
    bool debug_;
    double poll_hz_high_;
    double poll_hz_medium_;
    double poll_hz_low_;
    
    // Enable flags
    bool enable_att_, enable_imu_, enable_alt_, enable_bat_;
    bool enable_mot_, enable_srv_, enable_rc_, enable_gps_;
    bool enable_status_, enable_temp_, enable_esc_, enable_arming_;
    
    // Publishers - High Priority
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr att_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr rc_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr alt_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vario_pub_;
    
    // Publishers - Medium Priority
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr bat_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    
    // Publishers - Low Priority
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr gps_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr esc_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr arming_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr servo_pub_;
    // rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr sensor_status_pub_;
    
    // Publishers - One-time
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fc_info_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<MSPReaderNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("msp_reader"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}