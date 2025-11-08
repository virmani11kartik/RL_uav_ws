#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "uav_msp_bridge/MSPClient.hpp"
#include <vector>
#include <cstring>

using uav_msp_bridge::MSPClient;

// MSP Command IDs
enum : uint8_t {
    MSP_API_VERSION = 1,
    MSP_FC_VARIANT = 2,
    MSP_FC_VERSION = 3,
    MSP_MOTOR = 104,
    MSP_RC = 105,
    MSP_SET_MOTOR = 214,
    MSP_SET_RAW_RC = 200,
    MSP_ARMING_DISABLE_FLAGS = 150,
};

class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode() : Node("motor_control") {
        // Declare parameters
        port_ = declare_parameter<std::string>("device_path", "/dev/ttyACM0");
        baud_ = declare_parameter<int>("baud", 115200);
        num_motors_ = declare_parameter<int>("num_motors", 4);
        num_rc_channels_ = declare_parameter<int>("num_rc_channels", 8);
        min_throttle_ = declare_parameter<int>("min_throttle", 1000);
        max_throttle_ = declare_parameter<int>("max_throttle", 2000);
        min_rc_ = declare_parameter<int>("min_rc", 988);
        max_rc_ = declare_parameter<int>("max_rc", 2012);
        safety_enabled_ = declare_parameter<bool>("safety_enabled", true);
        command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 1000);
        
        RCLCPP_INFO(get_logger(), "Motor Control Node Configuration:");
        RCLCPP_INFO(get_logger(), "  Port: %s", port_.c_str());
        RCLCPP_INFO(get_logger(), "  Baud: %d", baud_);
        RCLCPP_INFO(get_logger(), "  Number of motors: %d", num_motors_);
        RCLCPP_INFO(get_logger(), "  Number of RC channels: %d", num_rc_channels_);
        RCLCPP_INFO(get_logger(), "  Throttle range: %d-%d", min_throttle_, max_throttle_);
        RCLCPP_INFO(get_logger(), "  RC range: %d-%d", min_rc_, max_rc_);
        RCLCPP_INFO(get_logger(), "  Safety enabled: %s", safety_enabled_ ? "true" : "false");
        
        current_motor_values_.resize(num_motors_, min_throttle_);
        current_rc_values_.resize(num_rc_channels_, 1500); // Initialize to center
        
        client_ = std::make_unique<MSPClient>(port_, baud_);
        if (!client_->open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", port_.c_str());
            throw std::runtime_error("Failed to open serial");
        }
        
        RCLCPP_INFO(get_logger(), "Connected to FC on %s @ %d baud", port_.c_str(), baud_);
        
        // Initialize last command times to current time
        last_command_time_ = now();
        last_rc_command_time_ = now();
        
        fetchFCInfo();
        
        // Motor control subscriptions
        motor_cmd_sub_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/motor_control/command",
            10,
            std::bind(&MotorControlNode::motorCommandCallback, this, std::placeholders::_1));
        
        all_motors_sub_ = create_subscription<std_msgs::msg::UInt16>(
            "/motor_control/all_motors",
            10,
            std::bind(&MotorControlNode::allMotorsCallback, this, std::placeholders::_1));
        
        for (int i = 0; i < num_motors_; i++) {
            std::string topic = "/motor_control/motor_" + std::to_string(i + 1);
            auto sub = create_subscription<std_msgs::msg::UInt16>(
                topic,
                10,
                [this, i](const std_msgs::msg::UInt16::SharedPtr msg) {
                    this->individualMotorCallback(msg, i);
                });
            individual_motor_subs_.push_back(sub);
        }
        
        // RC control subscriptions
        rc_cmd_sub_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/rc_control/command",
            10,
            std::bind(&MotorControlNode::rcCommandCallback, this, std::placeholders::_1));
        
        // Individual RC channel topics (rc_1, rc_2, etc.)
        for (int i = 0; i < num_rc_channels_; i++) {
            std::string topic = "/rc_control/channel_" + std::to_string(i + 1);
            auto sub = create_subscription<std_msgs::msg::UInt16>(
                topic,
                10,
                [this, i](const std_msgs::msg::UInt16::SharedPtr msg) {
                    this->individualRCCallback(msg, i);
                });
            individual_rc_subs_.push_back(sub);
        }
        
        emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/motor_control/emergency_stop",
            10,
            std::bind(&MotorControlNode::emergencyStopCallback, this, std::placeholders::_1));
        
        // Create publishers
        motor_state_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>(
            "/motor_control/state", 10);
        
        rc_state_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>(
            "/rc_control/state", 10);
        
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/motor_control/status", 10);
        
        // Create timer for reading motor and RC state
        state_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotorControlNode::publishState, this));
        
        // Create timer for continuously sending RC commands to maintain control
        rc_send_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),  // Send at 20Hz to maintain control
            std::bind(&MotorControlNode::continuousRCSend, this));
        
        // Create watchdog timer
        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(command_timeout_ms_),
            std::bind(&MotorControlNode::watchdogCallback, this));
        
        RCLCPP_INFO(get_logger(), "Motor control node initialized");
        RCLCPP_INFO(get_logger(), "Available ROS topics:");
        RCLCPP_INFO(get_logger(), "Motor Control:");
        RCLCPP_INFO(get_logger(), "  /motor_control/command - UInt16MultiArray [m1, m2, m3, m4]");
        RCLCPP_INFO(get_logger(), "  /motor_control/all_motors - UInt16 (set all motors to same value)");
        for (int i = 0; i < num_motors_; i++) {
            RCLCPP_INFO(get_logger(), "  /motor_control/motor_%d - UInt16 (individual motor)", i + 1);
        }
        RCLCPP_INFO(get_logger(), "  /motor_control/emergency_stop - Bool (stop all motors)");
        RCLCPP_INFO(get_logger(), "  /motor_control/state - UInt16MultiArray (current motor values)");
        
        RCLCPP_INFO(get_logger(), "RC Control:");
        RCLCPP_INFO(get_logger(), "  /rc_control/command - UInt16MultiArray [ch1, ch2, ..., ch%d]", num_rc_channels_);
        for (int i = 0; i < num_rc_channels_; i++) {
            RCLCPP_INFO(get_logger(), "  /rc_control/channel_%d - UInt16 (individual channel)", i + 1);
        }
        RCLCPP_INFO(get_logger(), "  /rc_control/state - UInt16MultiArray (current RC values)");
        
        if (safety_enabled_) {
            RCLCPP_WARN(get_logger(), "SAFETY MODE ENABLED: Motors will stop if no commands received for %d ms", 
                       command_timeout_ms_);
        }
    }
    
    ~MotorControlNode() {
        // Safety: stop all motors on shutdown
        RCLCPP_INFO(get_logger(), "Shutting down - stopping all motors");
        stopAllMotors();
        if (client_) {
            client_->close();
        }
    }

private:
    void fetchFCInfo() {
        std::vector<uint8_t> resp;
        
        // API Version
        if (client_->request(MSP_API_VERSION, {}, resp, 0.5) && resp.size() >= 3) {
            RCLCPP_INFO(get_logger(), "FC API Version: %d.%d.%d", 
                       resp[0], resp[1], resp[2]);
        }
        
        // FC Variant
        if (client_->request(MSP_FC_VARIANT, {}, resp, 0.5)) {
            std::string variant(resp.begin(), resp.end());
            variant.erase(std::remove(variant.begin(), variant.end(), '\0'), variant.end());
            if (!variant.empty()) {
                RCLCPP_INFO(get_logger(), "FC Variant: %s", variant.c_str());
            }
        }
        
        // FC Version
        if (client_->request(MSP_FC_VERSION, {}, resp, 0.5) && resp.size() >= 3) {
            RCLCPP_INFO(get_logger(), "FC Version: %d.%d.%d", 
                       resp[0], resp[1], resp[2]);
        }
    }
    
    void motorCommandCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        if (msg->data.size() != (size_t)num_motors_) {
            RCLCPP_ERROR(get_logger(), 
                        "Motor command has %zu values, expected %d", 
                        msg->data.size(), num_motors_);
            return;
        }
        
        std::vector<uint16_t> motor_values;
        
        for (size_t i = 0; i < msg->data.size(); i++) {
            uint16_t value = msg->data[i];
            
            if (value < min_throttle_) {
                RCLCPP_WARN(get_logger(), "Motor %zu: value %d below minimum %d, clamping", 
                           i, value, min_throttle_);
                value = min_throttle_;
            }
            if (value > max_throttle_) {
                RCLCPP_WARN(get_logger(), "Motor %zu: value %d above maximum %d, clamping", 
                           i, value, max_throttle_);
                value = max_throttle_;
            }
            
            motor_values.push_back(value);
        }
        
        if (setMotors(motor_values)) {
            current_motor_values_ = motor_values;
            last_command_time_ = now();
            
            static int command_count = 0;
            command_count++;
            if (command_count <= 5 || command_count % 50 == 0) {
                std::string values_str;
                for (auto v : motor_values) {
                    values_str += std::to_string(v) + " ";
                }
                RCLCPP_INFO(get_logger(), "Motors set to: %s", values_str.c_str());
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to set motor values");
            publishStatus("ERROR: Failed to set motor values");
        }
    }
    
    void allMotorsCallback(const std_msgs::msg::UInt16::SharedPtr msg) {
        uint16_t value = msg->data;
        
        if (value < min_throttle_) {
            RCLCPP_WARN(get_logger(), "Value %d below minimum %d, clamping", 
                       value, min_throttle_);
            value = min_throttle_;
        }
        if (value > max_throttle_) {
            RCLCPP_WARN(get_logger(), "Value %d above maximum %d, clamping", 
                       value, max_throttle_);
            value = max_throttle_;
        }
        
        std::vector<uint16_t> motor_values(num_motors_, value);
        
        if (setMotors(motor_values)) {
            current_motor_values_ = motor_values;
            last_command_time_ = now();
            RCLCPP_INFO(get_logger(), "All motors set to: %d", value);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to set motor values");
            publishStatus("ERROR: Failed to set all motors");
        }
    }
    
    void individualMotorCallback(const std_msgs::msg::UInt16::SharedPtr msg, int motor_index) {
        uint16_t value = msg->data;
        
        if (value < min_throttle_) {
            RCLCPP_WARN(get_logger(), "Motor %d: value %d below minimum %d, clamping", 
                       motor_index + 1, value, min_throttle_);
            value = min_throttle_;
        }
        if (value > max_throttle_) {
            RCLCPP_WARN(get_logger(), "Motor %d: value %d above maximum %d, clamping", 
                       motor_index + 1, value, max_throttle_);
            value = max_throttle_;
        }
        
        current_motor_values_[motor_index] = value;
        
        if (setMotors(current_motor_values_)) {
            last_command_time_ = now();
            RCLCPP_INFO(get_logger(), "Motor %d set to: %d", motor_index + 1, value);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to set motor %d", motor_index + 1);
            publishStatus("ERROR: Failed to set motor " + std::to_string(motor_index + 1));
        }
    }
    
    void rcCommandCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        if (msg->data.size() != (size_t)num_rc_channels_) {
            RCLCPP_ERROR(get_logger(), 
                        "RC command has %zu values, expected %d", 
                        msg->data.size(), num_rc_channels_);
            return;
        }
        
        std::vector<uint16_t> rc_values;
        
        for (size_t i = 0; i < msg->data.size(); i++) {
            uint16_t value = msg->data[i];
            
            if (value < min_rc_) {
                RCLCPP_WARN(get_logger(), "RC channel %zu: value %d below minimum %d, clamping", 
                           i + 1, value, min_rc_);
                value = min_rc_;
            }
            if (value > max_rc_) {
                RCLCPP_WARN(get_logger(), "RC channel %zu: value %d above maximum %d, clamping", 
                           i + 1, value, max_rc_);
                value = max_rc_;
            }
            
            rc_values.push_back(value);
        }
        
        if (setRawRC(rc_values)) {
            current_rc_values_ = rc_values;
            last_rc_command_time_ = now();
            
            static int rc_command_count = 0;
            rc_command_count++;
            if (rc_command_count <= 5 || rc_command_count % 50 == 0) {
                std::string values_str;
                for (auto v : rc_values) {
                    values_str += std::to_string(v) + " ";
                }
                RCLCPP_INFO(get_logger(), "RC channels set to: %s", values_str.c_str());
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to set RC values");
            publishStatus("ERROR: Failed to set RC values");
        }
    }
    
    void individualRCCallback(const std_msgs::msg::UInt16::SharedPtr msg, int channel_index) {
        uint16_t value = msg->data;
        
        if (value < min_rc_) {
            RCLCPP_WARN(get_logger(), "RC channel %d: value %d below minimum %d, clamping", 
                       channel_index + 1, value, min_rc_);
            value = min_rc_;
        }
        if (value > max_rc_) {
            RCLCPP_WARN(get_logger(), "RC channel %d: value %d above maximum %d, clamping", 
                       channel_index + 1, value, max_rc_);
            value = max_rc_;
        }
        
        current_rc_values_[channel_index] = value;
        
        if (setRawRC(current_rc_values_)) {
            last_rc_command_time_ = now();
            RCLCPP_INFO(get_logger(), "RC channel %d set to: %d", channel_index + 1, value);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to set RC channel %d", channel_index + 1);
            publishStatus("ERROR: Failed to set RC channel " + std::to_string(channel_index + 1));
        }
    }
    
    void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_WARN(get_logger(), "EMERGENCY STOP TRIGGERED!");
            stopAllMotors();
            publishStatus("EMERGENCY STOP");
        }
    }
    
    void watchdogCallback() {
        if (!safety_enabled_) {
            return;
        }
        
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
                           "Command timeout (%.1f s) - stopping motors for safety", 
                           time_since_command);
                stopAllMotors();
                publishStatus("WATCHDOG: Command timeout - motors stopped");
            }
        }
    }
    
    bool setMotors(const std::vector<uint16_t>& motor_values) {
        std::vector<uint8_t> payload;
        payload.reserve(motor_values.size() * 2);
        
        for (auto value : motor_values) {
            payload.push_back(value & 0xFF);        
            payload.push_back((value >> 8) & 0xFF); 
        }
        
        std::vector<uint8_t> response;
        return client_->request(MSP_SET_MOTOR, payload, response, 0.1);
    }
    
    bool setRawRC(const std::vector<uint16_t>& rc_values) {
        std::vector<uint8_t> payload;
        payload.reserve(rc_values.size() * 2);
        
        // MSP_SET_RAW_RC expects channel values in little-endian format
        for (auto value : rc_values) {
            payload.push_back(value & 0xFF);        // Low byte
            payload.push_back((value >> 8) & 0xFF); // High byte
        }
        
        std::vector<uint8_t> response;
        bool success = client_->request(MSP_SET_RAW_RC, payload, response, 0.1);
        
        if (success) {
            rc_command_active_ = true;
        }
        
        return success;
    }
    
    void continuousRCSend() {
        // Continuously send RC commands to maintain control
        // This prevents the FC from reverting to other RC sources
        if (rc_command_active_) {
            auto time_since_rc_command = (now() - last_rc_command_time_).seconds();
            
            // If we haven't received a new command in 2 seconds, stop sending
            if (time_since_rc_command > 2.0) {
                rc_command_active_ = false;
                RCLCPP_INFO(get_logger(), "RC command stream stopped (no new commands)");
                return;
            }
            
            // Resend the last RC values
            std::vector<uint8_t> payload;
            payload.reserve(current_rc_values_.size() * 2);
            
            for (auto value : current_rc_values_) {
                payload.push_back(value & 0xFF);
                payload.push_back((value >> 8) & 0xFF);
            }
            
            std::vector<uint8_t> response;
            client_->request(MSP_SET_RAW_RC, payload, response, 0.05);
        }
    }
    
    void stopAllMotors() {
        std::vector<uint16_t> stop_values(num_motors_, min_throttle_);
        if (setMotors(stop_values)) {
            current_motor_values_ = stop_values;
            RCLCPP_INFO(get_logger(), "All motors stopped");
            publishStatus("All motors stopped");
        }
    }
    
    void publishState() {
        // Publish motor state
        std::vector<uint8_t> motor_response;
        if (client_->request(MSP_MOTOR, {}, motor_response, 0.05)) {
            auto msg = std_msgs::msg::UInt16MultiArray();
            
            for (size_t i = 0; i + 1 < motor_response.size(); i += 2) {
                uint16_t value = motor_response[i] | (motor_response[i + 1] << 8);
                msg.data.push_back(value);
            }
            
            motor_state_pub_->publish(msg);
        }
        
        // Publish RC state
        std::vector<uint8_t> rc_response;
        if (client_->request(MSP_RC, {}, rc_response, 0.05)) {
            auto msg = std_msgs::msg::UInt16MultiArray();
            
            for (size_t i = 0; i + 1 < rc_response.size(); i += 2) {
                uint16_t value = rc_response[i] | (rc_response[i + 1] << 8);
                msg.data.push_back(value);
            }
            
            rc_state_pub_->publish(msg);
        }
    }
    
    void publishStatus(const std::string& status) {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        status_pub_->publish(msg);
    }
    
    std::unique_ptr<MSPClient> client_;
    
    // Motor control subscribers
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr all_motors_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr> individual_motor_subs_;
    
    // RC control subscribers
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr rc_cmd_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr> individual_rc_subs_;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_state_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr rc_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr rc_send_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    
    std::string port_;
    int baud_;
    int num_motors_;
    int num_rc_channels_;
    int min_throttle_;
    int max_throttle_;
    int min_rc_;
    int max_rc_;
    bool safety_enabled_;
    int command_timeout_ms_;
    
    std::vector<uint16_t> current_motor_values_;
    std::vector<uint16_t> current_rc_values_;
    rclcpp::Time last_command_time_;
    rclcpp::Time last_rc_command_time_;
    bool rc_command_active_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<MotorControlNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}