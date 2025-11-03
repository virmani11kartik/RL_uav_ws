#include "rclcpp/rclcpp.hpp"

class RLAgentNodeCpp : public rclcpp::Node {
public:
  RLAgentNodeCpp() : Node("rl_agent_node_cpp") {
    RCLCPP_INFO(this->get_logger(), "rl_agent_node_cpp up and running");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RLAgentNodeCpp>());
  rclcpp::shutdown();
  return 0;
}
