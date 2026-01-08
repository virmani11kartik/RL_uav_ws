#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>

class ArmOnce : public rclcpp::Node {
public:
    ArmOnce() : Node("arm_once") {
        this->declare_parameter("rc_out_topic", "/rc_aetr_aux");
        this->declare_parameter("aux1_arm_us", 1100);
        this->declare_parameter("center_us", 1500);
        this->declare_parameter("throttle_us", 988);
        this->declare_parameter("aux_defaults_us", std::vector<int64_t>{1500, 1500, 1500});

        std::string out = this->get_parameter("rc_out_topic").as_string();
        pub_ = this->create_publisher<std_msgs::msg::String>(out, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ArmOnce::tick, this)
        );
    }

private:
    void tick() {
        int A = this->get_parameter("center_us").as_int();
        int E = A;
        int R = A;
        int T = this->get_parameter("throttle_us").as_int();
        int aux1 = this->get_parameter("aux1_arm_us").as_int();
        
        std::vector<int64_t> aux_defaults = 
            this->get_parameter("aux_defaults_us").as_integer_array();
        int aux2 = static_cast<int>(aux_defaults[0]);
        int aux3 = static_cast<int>(aux_defaults[1]);
        int aux4 = static_cast<int>(aux_defaults[2]);

        std::ostringstream oss;
        oss << A << " " << E << " " << T << " " << R << " " 
            << aux1 << " " << aux2 << " " << aux3 << " " << aux4;

        auto msg = std_msgs::msg::String();
        msg.data = oss.str();
        pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Sent ARM once: %s", msg.data.c_str());
        rclcpp::shutdown();
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmOnce>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}