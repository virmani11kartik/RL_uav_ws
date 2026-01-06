#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "uav_msp_bridge/MSPClient.hpp"
#include <vector>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <mutex>

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
    MSP_ATTITUDE = 108,
    MSP_ALTITUDE = 109,
    MSP_ANALOG = 110,
    
    MSP_VOLTAGE_METERS = 128,
    MSP_CURRENT_METERS = 129,
    MSP_BATTERY_STATE = 130,
    MSP_TEMPERATURE = 132,
    MSP_ESC_SENSOR_DATA = 134,
    
    MSP_ARMING_DISABLE_FLAGS = 150,
    MSP_SET_ARMING = 151,
    MSP_SET_MOTOR = 214,

    MSP_SET_RAW_RC=200,
};

struct ArmingFlag {
    uint32_t bit;
    const char* name;
};

static const std::vector<ArmingFlag> ARMING_FLAGS = {
    {1 << 0,  "ARM_SWITCH"},
    {1 << 1,  "FAILSAFE"},
    {1 << 2,  "THROTTLE_HIGH"},
    {1 << 3,  "ANGLE_MODE"},
    {1 << 4,  "BOOT_GRACE_TIME"},
    {1 << 5,  "NO_GYRO"},
    {1 << 6,  "RX_FAILSAFE / NO_RX"},
    {1 << 7,  "MSP_OVERRIDE"},
    {1 << 8,  "CLI_ACTIVE"},
    {1 << 9,  "PREARM"},
    {1 << 10, "NO_ACC_CAL"},
    {1 << 11, "MOTOR_PROTOCOL"},
    {1 << 12, "AIRMODE"},
    // you can extend this later if needed
};

class LaunchNode : public rclcpp::Node {
public:
    LaunchNode() : Node("launch_node") {
        // Declare parameters
        port_ = declare_parameter<std::string>("device_path", "/dev/ttyACM0");
        baud_ = declare_parameter<int>("baud", 115200);
        debug_ = declare_parameter<bool>("debug", false);
        
        // Motor control parameters
        num_motors_ = declare_parameter<int>("num_motors", 4);
        min_throttle_ = declare_parameter<int>("min_throttle", 800);
        max_throttle_ = declare_parameter<int>("max_throttle", 2000);
        safety_enabled_ = declare_parameter<bool>("safety_enabled", true);
        command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 1000);
        auto_disarm_on_shutdown_ = declare_parameter<bool>("auto_disarm_on_shutdown", true);
        
        // Telemetry parameters - multiple poll rates
        poll_hz_high_ = declare_parameter<double>("poll_hz_high", 10.0);
        poll_hz_medium_ = declare_parameter<double>("poll_hz_medium", 2.0);
        poll_hz_low_ = declare_parameter<double>("poll_hz_low", 0.5);
        
        // Enable flags for each data category
        enable_att_ = declare_parameter<bool>("enable_attitude", true);
        enable_imu_ = declare_parameter<bool>("enable_imu", true);
        enable_alt_ = declare_parameter<bool>("enable_altitude", true);
        enable_bat_ = declare_parameter<bool>("enable_battery", true);
        enable_mot_ = declare_parameter<bool>("enable_motors", true);
        enable_srv_ = declare_parameter<bool>("enable_servos", false);
        enable_rc_ = declare_parameter<bool>("enable_rc", true);
        enable_gps_ = declare_parameter<bool>("enable_gps", false);
        enable_status_ = declare_parameter<bool>("enable_status", true);
        enable_temp_ = declare_parameter<bool>("enable_temperature", false);
        enable_esc_ = declare_parameter<bool>("enable_esc", false);
        enable_arming_ = declare_parameter<bool>("enable_arming_flags", true);
        
        RCLCPP_INFO(get_logger(), "=================================================");
        RCLCPP_INFO(get_logger(), "     UNIFIED FC CONTROL & TELEMETRY NODE");
        RCLCPP_INFO(get_logger(), "=================================================");
        RCLCPP_INFO(get_logger(), "Configuration:");
        RCLCPP_INFO(get_logger(), "  Port: %s @ %d baud", port_.c_str(), baud_);
        RCLCPP_INFO(get_logger(), "  Motors: %d (range %d-%d)", num_motors_, min_throttle_, max_throttle_);
        RCLCPP_INFO(get_logger(), "  Safety: %s (timeout: %d ms)", 
                safety_enabled_ ? "enabled" : "disabled", command_timeout_ms_);
        RCLCPP_INFO(get_logger(), "  Poll rates: High=%.1f Hz, Medium=%.1f Hz, Low=%.1f Hz", 
                poll_hz_high_, poll_hz_medium_, poll_hz_low_);
        
        current_motor_values_.resize(num_motors_, min_throttle_);
        last_command_time_ = now();
        
        // Open serial connection
        client_ = std::make_unique<MSPClient>(port_, baud_);
        if (!client_->open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", port_.c_str());
            throw std::runtime_error("Failed to open serial");
        }
        
        RCLCPP_INFO(get_logger(), "✓ Connected to FC on %s @ %d baud", port_.c_str(), baud_);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // ========== CREATE PUBLISHERS FIRST ==========
        // Create all publishers BEFORE fetching FC info or creating timers
        RCLCPP_INFO(get_logger(), "Creating publishers...");
        createPublishers();
        RCLCPP_INFO(get_logger(), "✓ Publishers created");
        
        // ========== CREATE SUBSCRIBERS ==========
        RCLCPP_INFO(get_logger(), "Creating subscribers...");
        createSubscribers();
        RCLCPP_INFO(get_logger(), "✓ Subscribers created");
        
        // ========== FETCH FC INFO (safely after publishers exist) ==========
        try {
            fetchFCInfo();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to fetch FC info: %s", e.what());
        }
        
        // ========== CREATE TIMERS LAST ==========
        // Create timers last to prevent race conditions
        RCLCPP_INFO(get_logger(), "Starting telemetry timers...");
        createTimers();
        RCLCPP_INFO(get_logger(), "✓ Timers started");
        
        RCLCPP_INFO(get_logger(), "=================================================");
        RCLCPP_INFO(get_logger(), "Motor Control Topics:");
        RCLCPP_INFO(get_logger(), "  Sub: /motor_control/command (UInt16MultiArray)");
        RCLCPP_INFO(get_logger(), "  Sub: /motor_control/all_motors (UInt16)");
        RCLCPP_INFO(get_logger(), "  Sub: /motor_control/motor_1-%d (UInt16)", num_motors_);
        RCLCPP_INFO(get_logger(), "  Sub: /motor_control/emergency_stop (Bool)");
        RCLCPP_INFO(get_logger(), "  Pub: /motor_control/state, /motor_control/status");
        RCLCPP_INFO(get_logger(), "-------------------------------------------------");
        RCLCPP_INFO(get_logger(), "Arming Topics:");
        RCLCPP_INFO(get_logger(), "  Sub: /arming/arm_command, /arming/disarm_command (Bool)");
        RCLCPP_INFO(get_logger(), "  Pub: /arming/armed_state, /arming/disable_flags, /arming/status");
        RCLCPP_INFO(get_logger(), "-------------------------------------------------");
        RCLCPP_INFO(get_logger(), "MSP Telemetry Topics:");
        RCLCPP_INFO(get_logger(), "  Pub: /msp/imu_raw, /msp/attitude, /msp/altitude, /msp/vario");
        RCLCPP_INFO(get_logger(), "  Pub: /msp/rc_raw, /msp/motors, /msp/servos");
        RCLCPP_INFO(get_logger(), "  Pub: /msp/battery, /msp/temperature, /msp/status");
        RCLCPP_INFO(get_logger(), "  Pub: /msp/gps/fix, /msp/gps/velocity");
        RCLCPP_INFO(get_logger(), "  Pub: /msp/esc_telemetry, /msp/arming_flags, /msp/fc_info");
        RCLCPP_INFO(get_logger(), "=================================================");
        RCLCPP_INFO(get_logger(), "✓ Unified FC Node Ready");
        RCLCPP_INFO(get_logger(), "=================================================");
    }
    
    ~LaunchNode() {
        RCLCPP_INFO(get_logger(), "Shutting down Unified FC Node...");
        
        if (auto_disarm_on_shutdown_ && isArmed()) {
            RCLCPP_WARN(get_logger(), "Auto-disarming flight controller");
            sendDisarmCommand();
        }
        
        RCLCPP_INFO(get_logger(), "Stopping all motors");
        stopAllMotors();
        
        if (client_) {
            client_->close();
        }
        
        RCLCPP_INFO(get_logger(), "✓ Shutdown complete");
    }

    std::string decodeArmingFlags(uint32_t flags)
    {
        if (flags == 0) {
            return "NONE (ARMING ALLOWED)";
        }

        std::ostringstream oss;
        bool first = true;

        for (const auto& f : ARMING_FLAGS) {
            if (flags & f.bit) {
                if (!first) oss << ", ";
                oss << f.name;
                first = false;
            }
        }

        return oss.str();
    }


private:
    void createSubscribers() {
        // Motor control subscribers
        motor_cmd_sub_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/motor_control/command", 10,
            std::bind(&LaunchNode::motorCommandCallback, this, std::placeholders::_1));
        
        all_motors_sub_ = create_subscription<std_msgs::msg::UInt16>(
            "/motor_control/all_motors", 10,
            std::bind(&LaunchNode::allMotorsCallback, this, std::placeholders::_1));
        
        for (int i = 0; i < num_motors_; i++) {
            std::string topic = "/motor_control/motor_" + std::to_string(i + 1);
            auto sub = create_subscription<std_msgs::msg::UInt16>(
                topic, 10,
                [this, i](const std_msgs::msg::UInt16::SharedPtr msg) {
                    this->individualMotorCallback(msg, i);
                });
            individual_motor_subs_.push_back(sub);
        }
        
        emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/motor_control/emergency_stop", 10,
            std::bind(&LaunchNode::emergencyStopCallback, this, std::placeholders::_1));
        
        // Arming subscribers
        arm_cmd_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/arming/arm_command", 10,
            std::bind(&LaunchNode::armCommandCallback, this, std::placeholders::_1));
        
        disarm_cmd_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/arming/disarm_command", 10,
            std::bind(&LaunchNode::disarmCommandCallback, this, std::placeholders::_1));

        // RC Suscriber
        rc_cmd_sub_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/rc_control/command", 10,
            std::bind(&LaunchNode::rcCommandCallback, this, std::placeholders::_1));

    }
    
    void createPublishers() {
        // Motor control publishers
        motor_state_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>(
            "/motor_control/state", 10);
        motor_status_pub_ = create_publisher<std_msgs::msg::String>(
            "/motor_control/status", 10);
        
        // Arming publishers
        armed_state_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/arming/armed_state", 10);
        arming_flags_pub_ = create_publisher<std_msgs::msg::UInt32>(
            "/arming/disable_flags", 10);
        arming_status_pub_ = create_publisher<std_msgs::msg::String>(
            "/arming/status", 10);
        
        // MSP Telemetry publishers - High priority
        msp_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "/msp/imu_raw", rclcpp::SensorDataQoS());
        msp_attitude_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/msp/attitude", rclcpp::SensorDataQoS());
        msp_altitude_pub_ = create_publisher<std_msgs::msg::Float32>("/msp/altitude", 10);
        msp_vario_pub_ = create_publisher<std_msgs::msg::Float32>("/msp/vario", 10);
        msp_rc_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>(
            "/msp/rc_raw", rclcpp::SensorDataQoS());
        msp_motors_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>(
            "/msp/motors", rclcpp::SensorDataQoS());
        
        // MSP Telemetry publishers - Medium priority
        msp_status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
            "/msp/status", 10);
        msp_battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/msp/battery", 10);
        msp_temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("/msp/temperature", 10);
        
        // MSP Telemetry publishers - Low priority
        msp_gps_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/msp/gps/fix", 10);
        msp_gps_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/msp/gps/velocity", 10);
        msp_esc_pub_ = create_publisher<std_msgs::msg::String>("/msp/esc_telemetry", 10);
        msp_arming_pub_ = create_publisher<std_msgs::msg::UInt32>("/msp/arming_flags", 10);
        msp_arming_text_pub_ = create_publisher<std_msgs::msg::String>("/msp/arming_flags_text", 10);
        msp_servo_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>("/msp/servos", 10);
        msp_fc_info_pub_ = create_publisher<std_msgs::msg::String>("/msp/fc_info", 10);
    }
    
    void createTimers() {
        // High priority: IMU, attitude, RC, motors, altitude (20Hz default)
        int high_ms = static_cast<int>(1000.0 / poll_hz_high_);
        timer_high_ = create_wall_timer(
            std::chrono::milliseconds(high_ms),
            std::bind(&LaunchNode::tickHigh, this));
        
        // Medium priority: status, battery, temperature (5Hz default)
        int medium_ms = static_cast<int>(1000.0 / poll_hz_medium_);
        timer_medium_ = create_wall_timer(
            std::chrono::milliseconds(medium_ms),
            std::bind(&LaunchNode::tickMedium, this));
        
        // Low priority: GPS, ESC, arming flags, servos (1Hz default)
        int low_ms = static_cast<int>(1000.0 / poll_hz_low_);
        timer_low_ = create_wall_timer(
            std::chrono::milliseconds(low_ms),
            std::bind(&LaunchNode::tickLow, this));
        
        // Motor state publishing (10Hz)
        motor_state_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LaunchNode::publishMotorState, this));
        
        // Arming state publishing (coupled with medium priority)
        arming_timer_ = create_wall_timer(
            std::chrono::milliseconds(medium_ms),
            std::bind(&LaunchNode::publishArmingState, this));
        
        // Watchdog
        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(command_timeout_ms_),
            std::bind(&LaunchNode::watchdogCallback, this));
    }
    
    void debugLog(const std::string& msg) {
        if (debug_) {
            RCLCPP_INFO(get_logger(), "[DEBUG] %s", msg.c_str());
        }
    }

    bool requestMSP(uint8_t cmd, const std::vector<uint8_t>& payload, 
                    std::vector<uint8_t>& response, double timeout = 0.1) {
        std::lock_guard<std::mutex> lock(msp_mutex_);
        return client_->request(cmd, payload, response, timeout);
    }
    
    void fetchFCInfo() {
        std::vector<uint8_t> resp;
        std::stringstream info;
        
        RCLCPP_INFO(get_logger(), "Fetching flight controller information...");
        
        // API Version
        if (requestMSP(MSP_API_VERSION, {}, resp, 0.5) && resp.size() >= 3) {
            info << "API: " << (int)resp[0] << "." << (int)resp[1] << "." << (int)resp[2] << " | ";
            RCLCPP_INFO(get_logger(), "  API Version: %d.%d.%d", resp[0], resp[1], resp[2]);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // FC Variant
        if (requestMSP(MSP_FC_VARIANT, {}, resp, 0.5) && !resp.empty()) {
            // Safely extract string - limit to response size
            std::string variant;
            variant.reserve(resp.size());
            for (size_t i = 0; i < resp.size() && resp[i] != '\0'; ++i) {
                if (std::isprint(resp[i])) {
                    variant += static_cast<char>(resp[i]);
                }
            }
            if (!variant.empty()) {
                info << "Variant: " << variant << " | ";
                RCLCPP_INFO(get_logger(), "  FC Variant: %s", variant.c_str());
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // FC Version
        if (requestMSP(MSP_FC_VERSION, {}, resp, 0.5) && resp.size() >= 3) {
            info << "Version: " << (int)resp[0] << "." << (int)resp[1] << "." << (int)resp[2] << " | ";
            RCLCPP_INFO(get_logger(), "  FC Version: %d.%d.%d", resp[0], resp[1], resp[2]);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Board Info - often contains binary data, handle carefully
        if (requestMSP(MSP_BOARD_INFO, {}, resp, 0.5) && !resp.empty()) {
            // Extract board identifier (first 4 bytes are typically the board ID as string)
            std::string board;
            board.reserve(std::min(resp.size(), size_t(32))); // Reasonable limit
            for (size_t i = 0; i < resp.size() && i < 32 && resp[i] != '\0'; ++i) {
                if (std::isprint(resp[i])) {
                    board += static_cast<char>(resp[i]);
                } else {
                    break; // Stop at first non-printable character
                }
            }
            if (!board.empty()) {
                info << "Board: " << board << " | ";
                RCLCPP_INFO(get_logger(), "  Board: %s", board.c_str());
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Craft Name
        if (requestMSP(MSP_NAME, {}, resp, 0.5) && !resp.empty()) {
            // Safely extract craft name
            std::string name;
            name.reserve(std::min(resp.size(), size_t(16))); // Names are typically short
            for (size_t i = 0; i < resp.size() && i < 16 && resp[i] != '\0'; ++i) {
                if (std::isprint(resp[i])) {
                    name += static_cast<char>(resp[i]);
                }
            }
            if (!name.empty()) {
                info << "Name: " << name;
                RCLCPP_INFO(get_logger(), "  Craft Name: %s", name.c_str());
            }
        }
        
        std::string info_str = info.str();
        if (!info_str.empty()) {
            RCLCPP_INFO(get_logger(), "  Summary: %s", info_str.c_str());
            
            // Only publish if publisher exists and is valid
            if (msp_fc_info_pub_ && msp_fc_info_pub_->get_subscription_count() > 0) {
                auto msg = std_msgs::msg::String();
                msg.data = info_str;
                msp_fc_info_pub_->publish(msg);
            }
        }
    }
    
    // ========== MOTOR CONTROL CALLBACKS ==========
    void motorCommandCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        if (msg->data.size() != (size_t)num_motors_) {
            RCLCPP_ERROR(get_logger(), "Motor command size mismatch: got %zu, expected %d",
                        msg->data.size(), num_motors_);
            return;
        }
        
        std::vector<uint16_t> motor_values;
        for (size_t i = 0; i < msg->data.size(); i++) {
            uint16_t value = std::clamp((int)msg->data[i], min_throttle_, max_throttle_);
            motor_values.push_back(value);
        }
        
        if (setMotors(motor_values)) {
            current_motor_values_ = motor_values;
            last_command_time_ = now();
        }
    }
    
    void allMotorsCallback(const std_msgs::msg::UInt16::SharedPtr msg) {
        uint16_t value = std::clamp((int)msg->data, min_throttle_, max_throttle_);
        std::vector<uint16_t> motor_values(num_motors_, value);
        
        if (setMotors(motor_values)) {
            current_motor_values_ = motor_values;
            last_command_time_ = now();
        }
    }
    
    void individualMotorCallback(const std_msgs::msg::UInt16::SharedPtr msg, int motor_index) {
        uint16_t value = std::clamp((int)msg->data, min_throttle_, max_throttle_);
        current_motor_values_[motor_index] = value;
        
        if (setMotors(current_motor_values_)) {
            last_command_time_ = now();
        }
    }
    
    void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_WARN(get_logger(), "⚠ EMERGENCY STOP TRIGGERED!");
            stopAllMotors();
            publishMotorStatus("EMERGENCY STOP");
        }
    }

     void rcCommandCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4) {
            RCLCPP_ERROR(get_logger(), "RC command needs at least 4 channels (roll,pitch,yaw,throttle)");
            return;
        }

        std::array<uint16_t, 8> rc = {
            (uint16_t)msg->data[0], // roll
            (uint16_t)msg->data[1], // pitch
            (uint16_t)msg->data[2], // yaw
            (uint16_t)msg->data[3], // throttle
            (msg->data.size() > 4 ? (uint16_t)msg->data[4] : 1000),
            (msg->data.size() > 5 ? (uint16_t)msg->data[5] : 1000),
            (msg->data.size() > 6 ? (uint16_t)msg->data[6] : 1000),
            (msg->data.size() > 7 ? (uint16_t)msg->data[7] : 1000),
        };

        // clamp the first 4 channels to 1000-2000 for safety
        for (int i = 0; i < 4; i++) rc[i] = std::clamp((int)rc[i], 800, 2000);

        if (setRawRC(rc)) {
            last_command_time_ = now();
        }
    }
    
    // ========== ARMING CALLBACKS ==========
    void armCommandCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(get_logger(), "ARM command received");
            
            uint32_t flags = getArmingDisableFlags();
            if (flags != 0) {
                RCLCPP_ERROR(get_logger(), "Cannot ARM - disable flags active: 0x%08X", flags);
                publishArmingStatus("ARM FAILED: Disable flags active");
                return;
            }
            
            if (sendArmCommand()) {
                RCLCPP_INFO(get_logger(), "✓ ARM command sent successfully");
                publishArmingStatus("ARMED");
            } else {
                RCLCPP_ERROR(get_logger(), "✗ Failed to send ARM command");
                publishArmingStatus("ARM FAILED");
            }
        }
    }
    
    void disarmCommandCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(get_logger(), "DISARM command received");
            
            if (sendDisarmCommand()) {
                RCLCPP_INFO(get_logger(), "✓ DISARM command sent successfully");
                publishArmingStatus("DISARMED");
            } else {
                RCLCPP_ERROR(get_logger(), "✗ Failed to send DISARM command");
                publishArmingStatus("DISARM FAILED");
            }
        }
    }

    
    // ========== HIGH PRIORITY TELEMETRY (20Hz default) ==========
    void tickHigh() {
        std::vector<uint8_t> resp;
        const auto stamp = now();
        
        // IMU
        if (enable_imu_) {
            if (requestMSP(MSP_RAW_IMU, {}, resp, 0.05) && resp.size() >= 18) {
                int16_t* vals = (int16_t*)resp.data();
                
                auto msg = sensor_msgs::msg::Imu();
                msg.header.stamp = stamp;
                msg.header.frame_id = "imu_link";
                
                msg.linear_acceleration.x = vals[0];
                msg.linear_acceleration.y = vals[1];
                msg.linear_acceleration.z = vals[2];
                msg.angular_velocity.x = vals[3];
                msg.angular_velocity.y = vals[4];
                msg.angular_velocity.z = vals[5];
                
                msg.orientation_covariance[0] = -1;
                
                msp_imu_pub_->publish(msg);
            }
        }
        
        // Attitude
        if (enable_att_) {
            if (requestMSP(MSP_ATTITUDE, {}, resp, 0.05) && resp.size() >= 6) {
                int16_t roll_raw = *(int16_t*)&resp[0];
                int16_t pitch_raw = *(int16_t*)&resp[2];
                int16_t yaw_raw = *(int16_t*)&resp[4];
                
                auto msg = geometry_msgs::msg::Vector3Stamped();
                msg.header.stamp = stamp;
                msg.header.frame_id = "base_link";
                msg.vector.x = roll_raw / 10.0;
                msg.vector.y = pitch_raw / 10.0;
                msg.vector.z = (double)yaw_raw;
                
                msp_attitude_pub_->publish(msg);
            }
        }
        
        // Altitude
        if (enable_alt_) {
            if (requestMSP(MSP_ALTITUDE, {}, resp, 0.05) && resp.size() >= 6) {
                int32_t alt_cm = *(int32_t*)&resp[0];
                int16_t vario_cms = *(int16_t*)&resp[4];
                
                auto alt_msg = std_msgs::msg::Float32();
                alt_msg.data = alt_cm / 100.0f;
                msp_altitude_pub_->publish(alt_msg);
                
                auto vario_msg = std_msgs::msg::Float32();
                vario_msg.data = vario_cms / 100.0f;
                msp_vario_pub_->publish(vario_msg);
            }
        }
        
        // RC Channels
        if (enable_rc_) {
            if (requestMSP(MSP_RC, {}, resp, 0.05)) {
                auto msg = std_msgs::msg::UInt16MultiArray();
                for (size_t i = 0; i + 1 < resp.size(); i += 2) {
                    msg.data.push_back((uint16_t)(resp[i] | (resp[i + 1] << 8)));
                }
                msp_rc_pub_->publish(msg);
            }
        }
        
        // Motors
        if (enable_mot_) {
            if (requestMSP(MSP_MOTOR, {}, resp, 0.05)) {
                auto msg = std_msgs::msg::UInt16MultiArray();
                for (size_t i = 0; i + 1 < resp.size(); i += 2) {
                    msg.data.push_back((uint16_t)(resp[i] | (resp[i + 1] << 8)));
                }
                msp_motors_pub_->publish(msg);
            }
        }
    }
    
    // ========== MEDIUM PRIORITY TELEMETRY (5Hz default) ==========
    void tickMedium() {
        std::vector<uint8_t> resp;
        const auto stamp = now();
        
        // Status
        if (enable_status_ && requestMSP(MSP_STATUS, {}, resp, 0.1)) {
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
            
            msp_status_pub_->publish(msg);
        }
        
        // Battery State
        if (enable_bat_ && requestMSP(MSP_BATTERY_STATE, {}, resp, 0.1)) {
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
            msp_battery_pub_->publish(msg);
        }
        
        // Temperature
        if (enable_temp_ && requestMSP(MSP_TEMPERATURE, {}, resp, 0.1)) {
            if (resp.size() >= 2) {
                int16_t temp_raw = *(int16_t*)&resp[0];
                
                auto msg = sensor_msgs::msg::Temperature();
                msg.header.stamp = stamp;
                msg.temperature = temp_raw / 10.0;
                msg.variance = 0.0;
                msp_temp_pub_->publish(msg);
            }
        }
    }
    
    // ========== LOW PRIORITY TELEMETRY (1Hz default) ==========
    void tickLow() {
        std::vector<uint8_t> resp;
        const auto stamp = now();
        
        // GPS
        if (enable_gps_ && requestMSP(MSP_RAW_GPS, {}, resp, 0.2)) {
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
                
                msp_gps_fix_pub_->publish(fix_msg);
                
                // GPS Velocity
                auto vel_msg = geometry_msgs::msg::TwistStamped();
                vel_msg.header.stamp = stamp;
                vel_msg.header.frame_id = "gps";
                
                double speed_ms = speed / 100.0;
                double course_rad = ground_course * M_PI / 180.0;
                vel_msg.twist.linear.x = speed_ms * cos(course_rad);
                vel_msg.twist.linear.y = speed_ms * sin(course_rad);
                vel_msg.twist.linear.z = 0.0;
                
                msp_gps_vel_pub_->publish(vel_msg);
            }
        }
        
        // ESC Telemetry
        if (enable_esc_ && requestMSP(MSP_ESC_SENSOR_DATA, {}, resp, 0.2)) {
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
            msp_esc_pub_->publish(msg);
        }
        
        // // Arming Disable Flags (MSP topic)
        // if (enable_arming_ && requestMSP(MSP_ARMING_DISABLE_FLAGS, {}, resp, 0.2)) {
        //     if (resp.size() >= 4) {
        //         uint32_t flags = *(uint32_t*)&resp[0];
                
        //         auto msg = std_msgs::msg::UInt32();
        //         msg.data = flags;
        //         msp_arming_pub_->publish(msg);
        //     }
        // }

        // Arming Disable Flags (MSP topic)
        if (enable_arming_ && requestMSP(MSP_ARMING_DISABLE_FLAGS, {}, resp, 0.2)) {
            if (resp.size() >= 4) {

                uint32_t flags =
                    (uint32_t(resp[0])      ) |
                    (uint32_t(resp[1]) <<  8) |
                    (uint32_t(resp[2]) << 16) |
                    (uint32_t(resp[3]) << 24);

                // Publish raw value (unchanged)
                auto msg = std_msgs::msg::UInt32();
                msg.data = flags;
                msp_arming_pub_->publish(msg);

                auto smsg = std_msgs::msg::String();
                smsg.data = decodeArmingFlags(flags);
                msp_arming_text_pub_->publish(smsg);

                // RCLCPP_INFO_THROTTLE(
                //     get_logger(),
                //     *get_clock(),
                //     1000,  // once per second
                //     "[ARMING FLAGS] raw=%u -> %s",
                //     flags,
                //     decodeArmingFlags(flags).c_str()
                // );
            }
        }
        
        // Servos
        if (enable_srv_ && requestMSP(MSP_SERVO, {}, resp, 0.2)) {
            auto msg = std_msgs::msg::UInt16MultiArray();
            for (size_t i = 0; i + 1 < resp.size(); i += 2) {
                msg.data.push_back((uint16_t)(resp[i] | (resp[i + 1] << 8)));
            }
            msp_servo_pub_->publish(msg);
        }
    }
    
    // ========== MOTOR STATE PUBLISHING ==========
    void publishMotorState() {
        std::vector<uint8_t> response;
        if (requestMSP(MSP_MOTOR, {}, response, 0.05)) {
            auto msg = std_msgs::msg::UInt16MultiArray();
            for (size_t i = 0; i + 1 < response.size(); i += 2) {
                uint16_t value = response[i] | (response[i + 1] << 8);
                msg.data.push_back(value);
            }
            motor_state_pub_->publish(msg);
        }
    }
    
    // ========== ARMING STATE PUBLISHING ==========
    void publishArmingState() {
        // Publish armed state
        bool armed = isArmed();
        auto armed_msg = std_msgs::msg::Bool();
        armed_msg.data = armed;
        armed_state_pub_->publish(armed_msg);
        
        // Publish arming disable flags
        uint32_t flags = getArmingDisableFlags();
        auto flags_msg = std_msgs::msg::UInt32();
        flags_msg.data = flags;
        arming_flags_pub_->publish(flags_msg);
    }
    
    // ========== WATCHDOG ==========
    void watchdogCallback() {
        if (!safety_enabled_) return;
        
        auto time_since_command = (now() - last_command_time_).seconds();
        
        if (time_since_command > (command_timeout_ms_ / 1000.0)) {
            bool motors_active = false;
            for (auto val : current_motor_values_) {
                if (val > min_throttle_) {
                    motors_active = true;
                    break;
                }
            }
            
            if (motors_active) {
                RCLCPP_WARN(get_logger(), 
                           "⚠ Command timeout (%.1f s) - stopping motors for safety",
                           time_since_command);
                stopAllMotors();
                publishMotorStatus("WATCHDOG: Command timeout - motors stopped");
            }
        }
    }
    
    // ========== MSP COMMANDS ==========
    bool setMotors(const std::vector<uint16_t>& motor_values) {
        std::vector<uint8_t> payload;
        payload.reserve(motor_values.size() * 2);
        
        for (auto value : motor_values) {
            payload.push_back(value & 0xFF);
            payload.push_back((value >> 8) & 0xFF);
        }
        
        std::vector<uint8_t> response;
        return requestMSP(MSP_SET_MOTOR, payload, response, 0.1);
    }

    bool setRawRC(const std::array<uint16_t, 8>& rc)
    {
        std::vector<uint8_t> payload;
        payload.reserve(16);

        for (auto v : rc) {
            payload.push_back(v & 0xFF);
            payload.push_back((v >> 8) & 0xFF);
        }

        std::vector<uint8_t> response;
        return requestMSP(MSP_SET_RAW_RC, payload, response, 0.1);
    }

    
    void stopAllMotors() {
        std::vector<uint16_t> stop_values(num_motors_, min_throttle_);
        if (setMotors(stop_values)) {
            current_motor_values_ = stop_values;
            RCLCPP_INFO(get_logger(), "✓ All motors stopped");
        }
    }
    
    bool isArmed() {
        std::vector<uint8_t> response;
        if (!requestMSP(MSP_STATUS, {}, response, 0.1) || response.size() < 10) {
            return false;
        }
        uint32_t flags = response[6] | (response[7] << 8) | 
                       (response[8] << 16) | (response[9] << 24);
        return (flags & 0x01) != 0;
    }
    
    uint32_t getArmingDisableFlags() {
        std::vector<uint8_t> response;
        if (!requestMSP(MSP_ARMING_DISABLE_FLAGS, {}, response, 0.1) || response.size() < 4) {
            return 0xFFFFFFFF;
        }
        return response[0] | (response[1] << 8) | (response[2] << 16) | (response[3] << 24);
    }
    
    bool sendArmCommand() {
        std::vector<uint8_t> payload = {0x01};
        std::vector<uint8_t> response;
        return requestMSP(MSP_SET_ARMING, payload, response, 0.2);
    }
    
    bool sendDisarmCommand() {
        std::vector<uint8_t> payload = {0x00};
        std::vector<uint8_t> response;
        return requestMSP(MSP_SET_ARMING, payload, response, 0.2);
    }
    
    // ========== UTILITIES ==========
    void publishMotorStatus(const std::string& status) {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        motor_status_pub_->publish(msg);
    }
    
    void publishArmingStatus(const std::string& status) {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        arming_status_pub_->publish(msg);
    }
    
    diagnostic_msgs::msg::KeyValue makeKeyValue(const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        return kv;
    }
    
    // ========== MEMBER VARIABLES ==========
    std::unique_ptr<MSPClient> client_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr all_motors_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr> individual_motor_subs_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disarm_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr rc_cmd_sub_;

    
    // Publishers - Motor Control
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_status_pub_;
    
    // Publishers - Arming
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_state_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr arming_flags_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arming_status_pub_;
    
    // Publishers - MSP Telemetry (High Priority)
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr msp_imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr msp_attitude_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr msp_altitude_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr msp_vario_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr msp_rc_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr msp_motors_pub_;
    
    // Publishers - MSP Telemetry (Medium Priority)
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr msp_status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr msp_battery_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr msp_temp_pub_;
    
    // Publishers - MSP Telemetry (Low Priority)
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr msp_gps_fix_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr msp_gps_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msp_esc_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr msp_arming_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msp_arming_text_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr msp_servo_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msp_fc_info_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr timer_high_;
    rclcpp::TimerBase::SharedPtr timer_medium_;
    rclcpp::TimerBase::SharedPtr timer_low_;
    rclcpp::TimerBase::SharedPtr motor_state_timer_;
    rclcpp::TimerBase::SharedPtr arming_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    
    // Parameters
    std::string port_;
    std::mutex msp_mutex_; 
    int baud_;
    bool debug_;
    int num_motors_;
    int min_throttle_;
    int max_throttle_;
    bool safety_enabled_;
    int command_timeout_ms_;
    bool auto_disarm_on_shutdown_;
    double poll_hz_high_;
    double poll_hz_medium_;
    double poll_hz_low_;
    
    // Enable flags
    bool enable_att_, enable_imu_, enable_alt_, enable_bat_;
    bool enable_mot_, enable_srv_, enable_rc_, enable_gps_;
    bool enable_status_, enable_temp_, enable_esc_, enable_arming_;
    
    // State
    std::vector<uint16_t> current_motor_values_;
    rclcpp::Time last_command_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<LaunchNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_node"), 
                    "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}