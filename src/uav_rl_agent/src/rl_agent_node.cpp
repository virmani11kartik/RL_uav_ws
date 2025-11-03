#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

class RLAgentNodeCpp : public rclcpp::Node {
public:
  RLAgentNodeCpp() : Node("rl_agent_node_cpp") {
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, std::bind(&RLAgentNodeCpp::cb, this, std::placeholders::_1));
    pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>("/motor_control/command", 10);
    RCLCPP_INFO(get_logger(), "uav_rl_agent (C++) ready.");
  }

private:
  void cb(const sensor_msgs::msg::Imu::SharedPtr) {
    // TODO: run your C++ policy (LibTorch/TensorRT) here
    std_msgs::msg::UInt16MultiArray out;
    out.data = {1150, 1150, 1150, 1150};
    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RLAgentNodeCpp>());
  rclcpp::shutdown();
  return 0;
}
